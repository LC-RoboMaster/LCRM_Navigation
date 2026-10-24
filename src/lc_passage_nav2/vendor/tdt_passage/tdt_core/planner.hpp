#pragma once
#include "image_io.hpp"

// tdt-nav-kit 的二维规划封装。所有对外位置均为 world/map frame 的米制坐标。
// 内部栅格统一为 row=0 对应地图底部；仅显示时做上下翻转。
#include "MinimumSnapOsqp/minimumSnap.hpp"
#include "YAstar/yastar.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#ifndef OPENCV_ALL_HPP
#define OPENCV_ALL_HPP
#endif
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tdt_nav
{
    using Vec2 = Eigen::Vector2d;
    using Path = std::vector<Vec2>;
    using Clock = std::chrono::steady_clock;

    inline double elapsedMs(Clock::time_point start)
    {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }

    enum class RowOrder { BottomUp, TopDown };

    struct MapInfo
    {
        double resolution = 0.05;
        Vec2 origin = Vec2::Zero(); // 栅格 (0,0) 的左下角在 world 中的位置
        double yaw = 0.; // 地图局部坐标轴相对 world 的旋转
        std::string frame_id = "map";

        Vec2 toLocal(const Vec2& world) const
        {
            Vec2 d = world - origin;
            double c = std::cos(yaw), s = std::sin(yaw);
            return {c * d.x() + s * d.y(), -s * d.x() + c * d.y()};
        }

        Vec2 rotateToWorld(const Vec2& v) const
        {
            double c = std::cos(yaw), s = std::sin(yaw);
            return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
        }

        Vec2 toWorld(const Vec2& local) const
        {
            return origin + rotateToWorld(local);
        }

        void validate() const
        {
            if (!std::isfinite(resolution) || resolution <= 0 || !origin.allFinite() ||
                !std::isfinite(yaw))
                throw std::invalid_argument("invalid resolution/origin/yaw");
        }
    };

    struct GridMap
    {
        MapInfo info;
        cv::Mat free; // CV_8UC1: 255=可通行，0=障碍，row 0 为地图底部
        cv::Mat unknown; // CV_8UC1: 255=未知（即使允许通行也保留）
        cv::Mat soft; // CV_32FC1: 非负附加代价，保留外部 costmap 的软代价
        void validate() const
        {
            info.validate();
            if (free.empty() || free.type() != CV_8UC1 || unknown.type() != CV_8UC1 ||
                soft.type() != CV_32FC1 || free.size() != unknown.size() || free.size() != soft.size())
                throw std::invalid_argument("invalid grid matrices");
            if (!cv::checkRange(soft, true, nullptr, 0, std::numeric_limits<float>::max()))
                throw std::invalid_argument("soft costs must be finite and nonnegative");
        }

        bool worldToCell(const Vec2& p, cv::Point& cell) const
        {
            Vec2 q = info.toLocal(p) / info.resolution;
            if (!q.allFinite() || q.x() < 0 || q.y() < 0 || q.x() >= free.cols || q.y() >= free.rows)
                return false;
            cell = {static_cast<int>(std::floor(q.x())), static_cast<int>(std::floor(q.y()))};
            return true;
        }

        bool worldToImage(const Vec2& p, cv::Point& pixel) const
        {
            if (!worldToCell(p, pixel))
                return false;
            pixel.y = free.rows - 1 - pixel.y;
            return true;
        }

        Vec2 cellCenterToWorld(int x, int y) const
        {
            return info.toWorld(Vec2(x + 0.5, y + 0.5) * info.resolution);
        }
    };

    // 不依赖 ROS 消息类型；ROS2 OccupancyGrid/Costmap2D 都可直接传其数组和元数据。
    class Maps
    {
        static GridMap make(int w, int h, MapInfo info)
        {
            info.validate();
            if (w <= 0 || h <= 0 || static_cast<uint64_t>(w) * h > 100000000ULL)
                throw std::invalid_argument("invalid or excessively large map");
            GridMap m;
            m.info = std::move(info);
            m.free = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));
            m.unknown = cv::Mat::zeros(h, w, CV_8UC1);
            m.soft = cv::Mat::zeros(h, w, CV_32FC1);
            return m;
        }

        static int row(int input, int h, RowOrder order)
        {
            return order == RowOrder::TopDown ? h - 1 - input : input;
        }

    public:
        static GridMap binary(const cv::Mat& image, MapInfo info = {},
                              RowOrder order = RowOrder::TopDown, bool zero_is_obstacle = true)
        {
            if (image.type() != CV_8UC1 || image.empty())
                throw std::invalid_argument("binary map must be CV_8UC1");
            auto m = make(image.cols, image.rows, std::move(info));
            for (int y = 0; y < image.rows; ++y)
                for (int x = 0; x < image.cols; ++x)
                {
                    bool blocked =
                        zero_is_obstacle ? image.at<uint8_t>(y, x) == 0 : image.at<uint8_t>(y, x) != 0;
                    m.free.at<uint8_t>(row(y, image.rows, order), x) = blocked ? 0 : 255;
                }
            return m;
        }

        static GridMap image(const cv::Mat& image, MapInfo info = {}, int occupied_max = 65,
                             int free_min = 196, bool unknown_blocked = true,
                             RowOrder order = RowOrder::TopDown)
        {
            if (image.empty() || image.depth() != CV_8U || occupied_max < 0 || free_min > 255 ||
                occupied_max >= free_min)
                throw std::invalid_argument("invalid image or thresholds");
            cv::Mat gray;
            if (image.channels() == 1)
                gray = image;
            else if (image.channels() == 3)
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            else if (image.channels() == 4)
                cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
            else
                throw std::invalid_argument("image must have 1, 3 or 4 channels");
            auto m = make(gray.cols, gray.rows, std::move(info));
            for (int y = 0; y < gray.rows; ++y)
                for (int x = 0; x < gray.cols; ++x)
                {
                    int v = gray.at<uint8_t>(y, x), r = row(y, gray.rows, order);
                    bool transparent = image.channels() == 4 && image.at<cv::Vec4b>(y, x)[3] == 0;
                    bool unknown = transparent || (v > occupied_max && v < free_min);
                    m.unknown.at<uint8_t>(r, x) = unknown ? 255 : 0;
                    m.free.at<uint8_t>(r, x) =
                        (unknown ? unknown_blocked : v <= occupied_max) ? 0 : 255;
                }
            return m;
        }

        static GridMap imageFile(const std::string& file, MapInfo info = {}, int occupied_max = 65,
                                 int free_min = 196, bool unknown_blocked = true)
        {
            auto img = tdt_image::read(file, cv::IMREAD_UNCHANGED);
            if (img.empty())
                throw std::invalid_argument("cannot read image: " + file);
            return image(img, std::move(info), occupied_max, free_min, unknown_blocked);
        }

        static GridMap occupancy(const std::vector<int8_t>& data, int w, int h, MapInfo info = {},
                                 int occupied_min = 65, bool unknown_blocked = true,
                                 RowOrder order = RowOrder::BottomUp)
        {
            auto m = make(w, h, std::move(info));
            if (data.size() != static_cast<size_t>(w) * h || occupied_min < 1 || occupied_min > 100)
                throw std::invalid_argument("invalid occupancy size/threshold");
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                {
                    int v = data[static_cast<size_t>(y) * w + x], r = row(y, h, order);
                    if (v < -1 || v > 100)
                        throw std::invalid_argument("occupancy values must be -1..100");
                    m.unknown.at<uint8_t>(r, x) = v == -1 ? 255 : 0;
                    m.free.at<uint8_t>(r, x) =
                        (v == -1 ? unknown_blocked : v >= occupied_min) ? 0 : 255;
                }
            return m;
        }

        static GridMap costmap(const std::vector<uint8_t>& data, int w, int h, MapInfo info = {},
                               int blocked_min = 253, bool unknown_blocked = true,
                               RowOrder order = RowOrder::BottomUp)
        {
            auto m = make(w, h, std::move(info));
            if (data.size() != static_cast<size_t>(w) * h || blocked_min < 1 || blocked_min > 254)
                throw std::invalid_argument("invalid costmap size/threshold");
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                {
                    int v = data[static_cast<size_t>(y) * w + x], r = row(y, h, order);
                    m.unknown.at<uint8_t>(r, x) = v == 255 ? 255 : 0;
                    m.free.at<uint8_t>(r, x) =
                        (v == 255 ? unknown_blocked : v >= blocked_min) ? 0 : 255;
                    if (v < blocked_min)
                        m.soft.at<float>(r, x) = static_cast<float>(v) / 252.f;
                }
            return m;
        }

        static GridMap floatCosts(const cv::Mat& values, MapInfo info = {}, bool unknown_blocked = true,
                                  RowOrder order = RowOrder::BottomUp)
        {
            if (values.type() != CV_32FC1 || values.empty())
                throw std::invalid_argument("costs must be CV_32FC1");
            auto m = make(values.cols, values.rows, std::move(info));
            for (int y = 0; y < values.rows; ++y)
                for (int x = 0; x < values.cols; ++x)
                {
                    float v = values.at<float>(y, x);
                    int r = row(y, values.rows, order);
                    if (std::isnan(v))
                    {
                        m.unknown.at<uint8_t>(r, x) = 255;
                        m.free.at<uint8_t>(r, x) = unknown_blocked ? 0 : 255;
                    }
                    else if (v == std::numeric_limits<float>::infinity())
                        m.free.at<uint8_t>(r, x) = 0;
                    else if (!std::isfinite(v) || v < 0)
                        throw std::invalid_argument("cost must be >=0, +inf, or NaN");
                    else
                        m.soft.at<float>(r, x) = v;
                }
            return m;
        }
    };

    enum class Backend { CorridorOSQP, FixedPathOSQP, ClosedForm };

    enum class Status
    {
        Success,
        GeometricFallback,
        NotReady,
        InvalidRequest,
        StartBlocked,
        GoalBlocked,
        NoPath,
        SearchLimit,
        OptimizationFailed,
        ValidationFailed,
        InternalError
    };

    inline const char* statusName(Status s)
    {
        switch (s)
        {
        case Status::Success:
            return "success";
        case Status::GeometricFallback:
            return "geometric_fallback";
        case Status::NotReady:
            return "not_ready";
        case Status::InvalidRequest:
            return "invalid_request";
        case Status::StartBlocked:
            return "start_blocked";
        case Status::GoalBlocked:
            return "goal_blocked";
        case Status::NoPath:
            return "no_path";
        case Status::SearchLimit:
            return "search_limit";
        case Status::OptimizationFailed:
            return "optimization_failed";
        case Status::ValidationFailed:
            return "validation_failed";
        default:
            return "internal_error";
        }
    }

    struct Config
    {
        double robot_radius = 0.15, safety_margin = 0.03; // 保守的方形膨胀，包含地图边界
        double obstacle_weight = 0.5, obstacle_decay = 0.1, source_cost_weight = 1.;
        double simplify_tolerance = 0.08;
        bool optimize = true, allow_geometric_fallback = false;
        Backend backend = Backend::CorridorOSQP;
        double corridor_range = 1.; // 单侧最大扩张距离，米
        double max_speed = 3., max_acceleration = 2., max_jerk = 0.; // 0 表示不约束 jerk
        double sample_dt = 0.05;
        int optimization_iterations = 8, curve_check_depth = 18;
        size_t max_search_expansions = 1000000, max_output_samples = 200000;
        double search_timeout_ms = 1000., optimization_timeout_ms = 1000.;
        bool collect_debug_images = true;

        void validate() const
        {
            for (double v : {
                     robot_radius, safety_margin, obstacle_weight, source_cost_weight,
                     simplify_tolerance, corridor_range, max_jerk
                 })
                if (!std::isfinite(v) || v < 0)
                    throw std::invalid_argument("invalid nonnegative planner parameter");
            for (double v : {
                     obstacle_decay, max_speed, max_acceleration, sample_dt, search_timeout_ms,
                     optimization_timeout_ms
                 })
                if (!std::isfinite(v) || v <= 0)
                    throw std::invalid_argument("invalid positive planner parameter");
            if (optimization_iterations < 1 || optimization_iterations > 100 || curve_check_depth < 1 ||
                curve_check_depth > 24 || max_search_expansions == 0 || max_output_samples < 2)
                throw std::invalid_argument("invalid planner limits");
        }
    };

    struct Request
    {
        Vec2 start = Vec2::Zero(), goal = Vec2::Zero();
        Vec2 start_velocity = Vec2::Zero(), goal_velocity = Vec2::Zero(); // world frame
    };

    struct Sample
    {
        double time = 0.;
        Vec2 position = Vec2::Zero(), velocity = Vec2::Zero(), acceleration = Vec2::Zero(),
             jerk = Vec2::Zero();
    };

    struct PolynomialSegment
    {
        double start_time = 0., duration = 0.;
        // world 坐标，归一化时间 u=(t-start_time)/duration，系数按升幂排列。
        Eigen::Matrix<double, 2, 6> coefficients = Eigen::Matrix<double, 2, 6>::Zero();

        Sample evaluate(double local_time) const
        {
            double u = std::clamp(local_time / duration, 0., 1.);
            Sample out;
            out.time = start_time + u * duration;
            for (int derivative = 0; derivative <= 3; ++derivative)
            {
                Vec2 value = Vec2::Zero();
                for (int j = 5; j >= derivative; --j)
                {
                    double factor = 1.;
                    for (int k = 0; k < derivative; ++k)
                        factor *= j - k;
                    value = value * u + factor * coefficients.col(j);
                }
                value /= std::pow(duration, derivative);
                if (derivative == 0)
                    out.position = value;
                else if (derivative == 1)
                    out.velocity = value;
                else if (derivative == 2)
                    out.acceleration = value;
                else
                    out.jerk = value;
            }
            return out;
        }
    };

    struct Corridor
    {
        std::array<Vec2, 4> corners;
    }; // world，多边形可直接绘制
    struct Timing
    {
        double map_ms = 0., search_ms = 0., simplify_ms = 0., optimize_ms = 0., debug_ms = 0.,
               total_ms = 0.;
    };

    struct DebugImage
    {
        std::string name;
        cv::Mat image;
    };

    struct Attempt
    {
        int iteration = 0;
        bool solver_ok = false, collision_free = false;
        double duration = 0., speed_bound = 0., acceleration_bound = 0., jerk_bound = 0.;
    };

    struct Result
    {
        Status status = Status::NotReady;
        std::string message, frame_id;
        MapInfo map_info;
        cv::Size map_size;
        Config config;
        Request request;
        uint64_t map_version = 0;
        Path raw_path, simplified_path, final_path;
        std::vector<Corridor> corridors;
        std::vector<PolynomialSegment> segments;
        std::vector<Sample> trajectory;
        std::vector<Attempt> attempts;
        std::vector<std::string> logs;
        std::vector<DebugImage> images;
        Timing timing;
        size_t search_expansions = 0;
        double duration = 0., speed_bound = 0., acceleration_bound = 0., jerk_bound = 0.;
        bool has_timed_trajectory = false;

        bool ok() const
        {
            return status == Status::Success || status == Status::GeometricFallback;
        }
    };

    namespace detail
    {
        // 复用 YAstar 代价场；不可变 (f,g,index) 入堆，避免原比较器引用可变代价破坏堆序。
        class Astar : public YAstar
        {
        public:
            cv::Mat costs(const cv::Mat& free, const cv::Mat& soft, double resolution, const Config& cfg)
            {
                setMapping(static_cast<float>(resolution));
                setOriginPos(0.f, 0.f);
                setMap(free.cols, free.rows, const_cast<uint8_t*>(free.ptr<uint8_t>()));
                setCostField(0.f, [&](float d)
                {
                    return static_cast<float>(cfg.obstacle_weight / (cfg.obstacle_decay + d));
                });
                initCostMap(true);
                cv::Mat result(free.size(), CV_32FC1);
                for (int y = 0; y < free.rows; ++y)
                    for (int x = 0; x < free.cols; ++x)
                    {
                        float v = costMap(y, x);
                        result.at<float>(y, x) =
                            free.at<uint8_t>(y, x)
                                ? v + static_cast<float>(cfg.source_cost_weight) * soft.at<float>(y, x)
                                : std::numeric_limits<float>::infinity();
                    }
                return result;
            }
        };

        class Optimizer : public MinimumSnap
        {
        public:
            std::pair<MatXd, MatXd> once(const std::vector<double>& times,
                                         const std::vector<SolveInput::Item>& items,
                                         const std::vector<PointPair>& boxes, tdt_nav::Backend backend)
            {
                setOrder(6);
                setMaxDx(3);
                if (backend == tdt_nav::Backend::ClosedForm)
                    return closeSolver(times, items, true);
                // 先消去位置/速度/连续性等式，再在较小的自由变量空间交给 OSQP。
                // 避免长短分段混合时，原始等式约束 QP 在很严的容差下反复迭代。
                if (times.size() + 1 != items.size() || boxes.size() != items.size())
                    return {};
                MatXd A = generateA(times, true);
                MatXd Q = generateQ(times, true).selfadjointView<Eigen::Upper>();
                Q /= std::max(Q.cwiseAbs().maxCoeff(), 1e-12);
                auto areas = postProcess(boxes);
                MatXd solution(A.cols(), 2);
                for (int axis = 0; axis < 2; ++axis)
                {
                    const auto& area = axis == 0 ? areas.first : areas.second;
                    VecXd low = generateLow(area), up = generateUp(area);
                    const int n = static_cast<int>(items.size());
                    const int interiorRow = static_cast<int>(A.rows()) - (n - 2);
                    for (int i = 0; i < n; ++i)
                        if (items[i].useVxvy)
                        {
                            low(i + 1) = up(i + 1) = items[i].xy(axis);
                            int row = i == 0 ? 0 : (i == n - 1 ? n + 1 : interiorRow + i - 1);
                            low(row) = up(row) = items[i].vxvy(axis);
                        }
                    std::vector<int> equal, inequal;
                    for (int row = 0; row < A.rows(); ++row)
                    {
                        if (std::isfinite(low(row)) && std::isfinite(up(row)) && low(row) == up(row))
                            equal.push_back(row);
                        else if (std::isfinite(low(row)) || std::isfinite(up(row)))
                            inequal.push_back(row);
                    }
                    MatXd E(equal.size(), A.cols());
                    VecXd b(equal.size());
                    for (size_t row = 0; row < equal.size(); ++row)
                    {
                        double scale = A.row(equal[row]).norm();
                        if (scale <= 0)
                            return {};
                        E.row(row) = A.row(equal[row]) / scale;
                        b(row) = low(equal[row]) / scale;
                    }
                    Eigen::FullPivLU<MatXd> lu(E);
                    VecXd x0 = lu.solve(b);
                    if (!x0.allFinite() || (E * x0 - b).cwiseAbs().maxCoeff() > 1e-7)
                        return {};
                    MatXd N = lu.kernel();
                    if (lu.dimensionOfKernel() > 0)
                    {
                        Eigen::HouseholderQR<MatXd> qr(N);
                        N = qr.householderQ() * MatXd::Identity(N.rows(), N.cols());
                        MatXd H = N.transpose() * Q * N;
                        H = (0.5 * (H + H.transpose())).eval();
                        VecXd gradient = N.transpose() * Q * x0;
                        double scale = std::max(H.cwiseAbs().maxCoeff(), 1e-12);
                        H /= scale;
                        gradient /= scale;
                        H.diagonal().array() += 1e-10;
                        MatXd C(inequal.size(), N.cols());
                        VecXd lo(inequal.size()), hi(inequal.size());
                        for (size_t row = 0; row < inequal.size(); ++row)
                        {
                            int k = inequal[row];
                            C.row(row) = A.row(k) * N;
                            double shift = A.row(k).dot(x0);
                            double norm = std::max(C.row(row).norm(), 1e-12);
                            C.row(row) /= norm;
                            lo(row) = (low(k) - shift) / norm;
                            hi(row) = (up(k) - shift) / norm;
                        }
                        Eigen::SparseMatrix<double> sparseH = H.sparseView(), sparseC = C.sparseView();
                        OsqpEigen::Solver solver;
                        solver.settings()->setVerbosity(false);
                        solver.settings()->setPolish(true);
                        solver.settings()->setAbsoluteTolerance(1e-7);
                        solver.settings()->setRelativeTolerance(1e-7);
                        solver.settings()->setMaxIteration(20000);
                        solver.data()->setNumberOfVariables(H.cols());
                        solver.data()->setNumberOfConstraints(C.rows());
                        if (!solver.data()->setHessianMatrix(sparseH) ||
                            !solver.data()->setGradient(gradient) ||
                            !solver.data()->setLinearConstraintsMatrix(sparseC) ||
                            !solver.data()->setLowerBound(lo) || !solver.data()->setUpperBound(hi) ||
                            !solver.initSolver())
                            return {};
                        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError ||
                            (solver.getStatus() != OsqpEigen::Status::Solved &&
                                solver.getStatus() != OsqpEigen::Status::SolvedInaccurate))
                            return {};
                        x0 += N * solver.getSolution();
                    }
                    VecXd values = A * x0;
                    if (!x0.allFinite() || (low - values).maxCoeff() > 1e-5 ||
                        (values - up).maxCoeff() > 1e-5)
                        return {};
                    solution.col(axis) = x0;
                }
                MatXd x(times.size(), 6), y(times.size(), 6);
                for (int i = 0; i < x.rows(); ++i)
                    for (int j = 0; j < 6; ++j)
                    {
                        x(i, 5 - j) = solution(i * 6 + j, 0);
                        y(i, 5 - j) = solution(i * 6 + j, 1);
                    }
                return {x, y};
            }
        };

        inline double binomial(int n, int k)
        {
            double v = 1;
            for (int i = 1; i <= k; ++i)
                v = v * (n - i + 1) / i;
            return v;
        }

        inline std::vector<Vec2> bernstein(const Eigen::Matrix<double, 2, 6>& a, int derivative = 0,
                                           double duration = 1.)
        {
            int n = 5 - derivative;
            std::vector<Vec2> power(n + 1, Vec2::Zero()), b(n + 1, Vec2::Zero());
            for (int j = derivative; j <= 5; ++j)
            {
                double f = 1;
                for (int d = 0; d < derivative; ++d)
                    f *= j - d;
                power[j - derivative] = a.col(j) * f / std::pow(duration, derivative);
            }
            for (int k = 0; k <= n; ++k)
                for (int j = 0; j <= k; ++j)
                    b[k] += power[j] * (binomial(k, j) / binomial(n, j));
            return b;
        }

        inline double derivativeBound(const Eigen::Matrix<double, 2, 6>& a, int d, double t)
        {
            double bound = 0.;
            for (const auto& p : bernstein(a, d, t))
                bound = std::max(bound, p.norm());
            return bound;
        }

        inline double length(const Path& p)
        {
            double n = 0;
            for (size_t i = 1; i < p.size(); ++i)
                n += (p[i] - p[i - 1]).norm();
            return n;
        }
    } // namespace detail

    class Planner
    {
    public:
        explicit Planner(Config cfg = {}) : cfg_(std::move(cfg))
        {
            cfg_.validate();
        }

        // forbidden is a constraint on the robot centre (e.g. a special-road
        // rectangle), not physical occupancy. Apply it AFTER robot inflation:
        // inflating that artificial boundary can otherwise close its own A/B.
        void setMap(const GridMap& map, const cv::Mat& forbidden = {})
        {
            map.validate();
            if (!forbidden.empty() &&
                (forbidden.type() != CV_8UC1 || forbidden.size() != map.free.size()))
                throw std::invalid_argument("forbidden mask must be CV_8UC1 and match map");
            std::lock_guard<std::mutex> lock(mutex_);
            auto begin = Clock::now();
            GridMap copy = map;
            copy.free = map.free.clone();
            copy.unknown = map.unknown.clone();
            copy.soft = map.soft.clone();
            cv::Mat blocked;
            cv::compare(copy.free, 0, blocked, cv::CMP_EQ);
            // Circular robot: Euclidean configuration-space inflation (not a square kernel).
            const double radius = cfg_.robot_radius + cfg_.safety_margin;
            const int border = static_cast<int>(std::ceil(radius / copy.info.resolution)) + 2;
            cv::Mat padded_free, distance, inflated;
            cv::copyMakeBorder(copy.free, padded_free, border, border, border, border,
                               cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::distanceTransform(padded_free, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
            const double threshold = radius > 0 ? radius / copy.info.resolution + 0.7071067811865476 : 0.;
            cv::compare(distance(cv::Rect(border,border,copy.free.cols,copy.free.rows)),
                        threshold, inflated, cv::CMP_GT);
            if (!forbidden.empty())
                inflated.setTo(0, forbidden);
            // 距离场还要把地图外部视为障碍，因此加一圈边界；搜索时取出内部区域。
            cv::Mat field_free, padded_soft;
            cv::copyMakeBorder(inflated, field_free, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::copyMakeBorder(copy.soft, padded_soft, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
            auto padded_cost = astar_.costs(field_free, padded_soft, copy.info.resolution, cfg_);
            auto cost = padded_cost(cv::Rect(1, 1, inflated.cols, inflated.rows)).clone();
            cv::Mat mask;
            cv::compare(inflated, 0, mask, cv::CMP_EQ);
            mask /= 255;
            cv::Mat integral;
            cv::integral(mask, integral, CV_64F);
            map_ = std::move(copy);
            free_ = inflated;
            cost_ = cost;
            integral_ = integral;
            ready_ = true;
            ++version_;
            map_ms_ = elapsedMs(begin);
        }

        cv::Mat inflatedFree() const { return free_.clone(); }
        cv::Mat costField() const { return cost_.clone(); }

        Result plan(const Request& request)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto begin = Clock::now();
            Result out;
            out.map_version = version_;
            out.timing.map_ms = map_ms_;
            out.config = cfg_;
            out.request = request;
            if (ready_)
            {
                out.map_info = map_.info;
                out.map_size = map_.free.size();
            }
            auto finish = [&](Status status, const std::string& message)
            {
                out.status = status;
                out.message = message;
                out.logs.push_back(std::string(statusName(status)) + ": " + message);
                if (ready_)
                    out.frame_id = map_.info.frame_id;
                if (ready_ && cfg_.collect_debug_images)
                {
                    auto t = Clock::now();
                    makeDebug(out, request);
                    out.timing.debug_ms = elapsedMs(t);
                }
                out.timing.total_ms = elapsedMs(begin);
                return out;
            };
            try
            {
                if (!ready_)
                    return finish(Status::NotReady, "call setMap() before plan()");
                if (!request.start.allFinite() || !request.goal.allFinite() ||
                    !request.start_velocity.allFinite() || !request.goal_velocity.allFinite() ||
                    request.start_velocity.norm() > cfg_.max_speed + 1e-8 ||
                    request.goal_velocity.norm() > cfg_.max_speed + 1e-8)
                    return finish(Status::InvalidRequest,
                                  "invalid position/velocity or endpoint velocity exceeds limit");
                cv::Point sc, gc;
                if (!map_.worldToCell(request.start, sc) || !free_.at<uint8_t>(sc))
                    return finish(Status::StartBlocked, "start outside map or blocked after inflation");
                if (!map_.worldToCell(request.goal, gc) || !free_.at<uint8_t>(gc))
                    return finish(Status::GoalBlocked, "goal outside map or blocked after inflation");
                Vec2 start = map_.info.toLocal(request.start), goal = map_.info.toLocal(request.goal);
                Path raw;
                auto t = Clock::now();
                Status search_status = search(sc, gc, raw, out.search_expansions);
                out.timing.search_ms = elapsedMs(t);
                if (search_status != Status::Success)
                    return finish(search_status,
                                  "A* could not find a path within the configured budget");
                // 路径中保留精确起终点；栅格节点使用格心，避免沿栅格角落运动。
                raw.insert(raw.begin(), start);
                raw.push_back(goal);
                removeDuplicates(raw);
                if ((goal - start).norm() < 1e-9)
                    raw = {start};
                if (!pathSafe(raw))
                    return finish(Status::ValidationFailed,
                                  "endpoint connector intersects an occupied cell");
                out.raw_path = worldPath(raw);
                t = Clock::now();
                Path simple = simplify(raw);
                out.simplified_path = worldPath(simple);
                out.timing.simplify_ms = elapsedMs(t);
                out.logs.push_back("A*: expanded=" + std::to_string(out.search_expansions) +
                    ", raw_points=" + std::to_string(raw.size()) +
                    ", simplified_points=" + std::to_string(simple.size()));
                if (!cfg_.optimize)
                {
                    out.final_path = out.simplified_path;
                    return finish(Status::Success, "geometric path only; optimization disabled");
                }
                if ((goal - start).norm() < 1e-9)
                {
                    if (request.start_velocity.norm() > 1e-9 || request.goal_velocity.norm() > 1e-9)
                        return finish(Status::InvalidRequest,
                                      "stationary request requires zero endpoint velocities");
                    out.final_path = {request.start};
                    out.trajectory.push_back(
                        {0., request.start, Vec2::Zero(), Vec2::Zero(), Vec2::Zero()});
                    out.has_timed_trajectory = true;
                    return finish(Status::Success, "already at goal");
                }
                t = Clock::now();
                bool optimized = optimize(simple, request, out);
                out.timing.optimize_ms = elapsedMs(t);
                if (optimized)
                    return finish(Status::Success,
                                  "optimized trajectory validated against grid and derivative bounds");
                if (cfg_.allow_geometric_fallback)
                {
                    out.final_path = out.simplified_path;
                    return finish(
                        Status::GeometricFallback,
                        "optimization failed; returned validated geometric path without timing");
                }
                return finish(Status::OptimizationFailed,
                              "no validated optimized trajectory; see attempts and logs");
            }
            catch (const std::exception& e)
            {
                out.final_path.clear();
                out.trajectory.clear();
                out.segments.clear();
                out.has_timed_trajectory = false;
                return finish(Status::InternalError, e.what());
            }
        }

        static std::string summary(const Result& r)
        {
            std::ostringstream s;
            s << std::fixed << std::setprecision(3) << statusName(r.status) << ": " << r.message
                << "\nframe=" << r.frame_id << ", map_version=" << r.map_version
                << ", points(raw/simple/final)=" << r.raw_path.size() << "/" << r.simplified_path.size()
                << "/" << r.final_path.size() << ", expansions=" << r.search_expansions
                << "\ntotal_ms=" << r.timing.total_ms << ", search_ms=" << r.timing.search_ms
                << ", optimize_ms=" << r.timing.optimize_ms << ", trajectory_duration_s=" << r.duration
                << "\ncertified upper bounds: speed=" << r.speed_bound
                << ", acceleration=" << r.acceleration_bound << ", jerk=" << r.jerk_bound;
            return s.str();
        }

        static void saveDebug(const Result& r, const std::filesystem::path& directory)
        {
            std::filesystem::create_directories(directory);
            for (const auto& im : r.images)
                if (!im.image.empty() &&
                    !tdt_image::write((directory / (im.name + ".png")).string(), im.image))
                    throw std::runtime_error("cannot write debug image: " + im.name);
            cv::FileStorage fs((directory / "report.json").string(),
                               cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
            if (!fs.isOpened())
                throw std::runtime_error("cannot open debug report");
            fs << "status" << statusName(r.status) << "message" << r.message << "frame_id" << r.frame_id
                << "map_version" << std::to_string(r.map_version) << "has_timed_trajectory"
                << r.has_timed_trajectory << "duration_s" << r.duration << "length_m"
                << detail::length(r.final_path) << "speed_bound" << r.speed_bound << "acceleration_bound"
                << r.acceleration_bound << "jerk_bound" << r.jerk_bound;
            fs << "map" << "{" << "width" << r.map_size.width << "height" << r.map_size.height
                << "resolution" << r.map_info.resolution << "origin_x" << r.map_info.origin.x()
                << "origin_y" << r.map_info.origin.y() << "origin_yaw" << r.map_info.yaw
                << "internal_row_order" << "bottom_up" << "display_row_order" << "top_down" << "}";
            fs << "parameters" << "{" << "robot_radius" << r.config.robot_radius << "safety_margin"
                << r.config.safety_margin << "obstacle_weight" << r.config.obstacle_weight
                << "obstacle_decay" << r.config.obstacle_decay << "source_cost_weight"
                << r.config.source_cost_weight << "simplify_tolerance" << r.config.simplify_tolerance
                << "corridor_range" << r.config.corridor_range << "max_speed" << r.config.max_speed
                << "max_acceleration" << r.config.max_acceleration << "max_jerk" << r.config.max_jerk
                << "sample_dt" << r.config.sample_dt << "optimization_iterations"
                << r.config.optimization_iterations << "backend" << static_cast<int>(r.config.backend)
                << "}";
            fs << "timing_ms" << "{" << "map_preparation" << r.timing.map_ms << "search"
                << r.timing.search_ms << "simplify" << r.timing.simplify_ms << "optimize"
                << r.timing.optimize_ms << "debug" << r.timing.debug_ms << "total_plan"
                << r.timing.total_ms << "}";
            for (auto item : std::vector<std::pair<std::string, const Path*>>{
                     {"raw_path", &r.raw_path},
                     {"simplified_path", &r.simplified_path},
                     {"final_path", &r.final_path}
                 })
            {
                fs << item.first << "[";
                for (const auto& p : *item.second)
                    fs << "{" << "x" << p.x() << "y" << p.y() << "}";
                fs << "]";
            }
            fs << "trajectory" << "[";
            for (const auto& p : r.trajectory)
                fs << "{" << "t" << p.time << "x" << p.position.x() << "y" << p.position.y() << "vx"
                    << p.velocity.x() << "vy" << p.velocity.y() << "ax" << p.acceleration.x() << "ay"
                    << p.acceleration.y() << "jx" << p.jerk.x() << "jy" << p.jerk.y() << "}";
            fs << "]";
            fs << "segments" << "[";
            for (const auto& p : r.segments)
            {
                fs << "{" << "start_time" << p.start_time << "duration" << p.duration
                    << "coefficients_ascending" << "[";
                for (int j = 0; j < 6; ++j)
                    fs << "{" << "x" << p.coefficients(0, j) << "y" << p.coefficients(1, j) << "}";
                fs << "]" << "}";
            }
            fs << "]";
            fs << "corridors" << "[";
            for (const auto& box : r.corridors)
            {
                fs << "[";
                for (const auto& p : box.corners)
                    fs << "{" << "x" << p.x() << "y" << p.y() << "}";
                fs << "]";
            }
            fs << "]";
            fs << "attempts" << "[";
            for (const auto& a : r.attempts)
                fs << "{" << "iteration" << a.iteration << "solver_ok" << a.solver_ok
                    << "collision_free" << a.collision_free << "duration" << a.duration << "speed_bound"
                    << a.speed_bound << "acceleration_bound" << a.acceleration_bound << "jerk_bound"
                    << a.jerk_bound << "}";
            fs << "]";
            fs << "logs" << "[";
            for (const auto& line : r.logs)
                fs << line;
            fs << "]";
            // CSV 可直接用 Python、PlotJuggler 或电子表格画速度/加速度曲线。
            std::ofstream csv(directory / "trajectory.csv");
            if (!csv)
                throw std::runtime_error("cannot open trajectory CSV");
            csv << std::setprecision(17) << "time,x,y,vx,vy,ax,ay,jx,jy\n";
            for (const auto& p : r.trajectory)
                csv << p.time << ',' << p.position.x() << ',' << p.position.y() << ',' << p.velocity.x()
                    << ',' << p.velocity.y() << ',' << p.acceleration.x() << ',' << p.acceleration.y()
                    << ',' << p.jerk.x() << ',' << p.jerk.y() << '\n';
            if (!csv)
                throw std::runtime_error("failed to write trajectory CSV");
        }

    private:
        Config cfg_;
        GridMap map_;
        cv::Mat free_, cost_, integral_;
        bool ready_ = false;
        uint64_t version_ = 0;
        double map_ms_ = 0.;
        detail::Astar astar_;
        detail::Optimizer optimizer_;
        std::mutex mutex_;

        Vec2 cellCenter(int x, int y) const
        {
            return Vec2(x + 0.5, y + 0.5) * map_.info.resolution;
        }

        Path worldPath(const Path& input) const
        {
            Path p;
            p.reserve(input.size());
            for (const auto& q : input)
                p.push_back(map_.info.toWorld(q));
            return p;
        }

        static void removeDuplicates(Path& p)
        {
            p.erase(std::unique(p.begin(), p.end(),
                                [](const Vec2& a, const Vec2& b) { return (a - b).norm() < 1e-9; }),
                    p.end());
        }

        bool boxFree(const Vec2& lo, const Vec2& hi) const
        {
            if (!lo.allFinite() || !hi.allFinite())
                return false;
            double r = map_.info.resolution;
            if (lo.x() < 0 || lo.y() < 0 || hi.x() >= free_.cols * r || hi.y() >= free_.rows * r)
                return false;
            int x0 = static_cast<int>(std::floor(lo.x() / r)),
                y0 = static_cast<int>(std::floor(lo.y() / r));
            int x1 = static_cast<int>(std::floor(hi.x() / r)) + 1,
                y1 = static_cast<int>(std::floor(hi.y() / r)) + 1;
            return integral_.at<double>(y1, x1) - integral_.at<double>(y0, x1) -
                integral_.at<double>(y1, x0) + integral_.at<double>(y0, x0) ==
                0.;
        }

        // Bezier 凸包包含曲线；递归细分后，每个包围盒都不碰障碍才通过。
        // 深度耗尽时保守拒绝，不把离散采样误当成连续安全证明。
        bool curveSafe(const std::vector<Vec2>& b, int depth = 0) const
        {
            Vec2 lo = b.front(), hi = b.front();
            for (const auto& p : b)
            {
                lo = lo.cwiseMin(p);
                hi = hi.cwiseMax(p);
            }
            if (boxFree(lo, hi))
                return true;
            if (depth >= cfg_.curve_check_depth || (hi - lo).norm() < map_.info.resolution * 1e-6)
                return false;
            auto t = b;
            std::vector<Vec2> left(b.size()), right(b.size());
            left.front() = t.front();
            right.back() = t.back();
            for (size_t k = 1; k < b.size(); ++k)
            {
                for (size_t j = 0; j < b.size() - k; ++j)
                    t[j] = (t[j] + t[j + 1]) * 0.5;
                left[k] = t.front();
                right[b.size() - 1 - k] = t[b.size() - 1 - k];
            }
            return curveSafe(left, depth + 1) && curveSafe(right, depth + 1);
        }

        bool lineSafe(const Vec2& a, const Vec2& b) const
        {
            return curveSafe({a, b});
        }

        bool pathSafe(const Path& p) const
        {
            for (size_t i = 1; i < p.size(); ++i)
                if (!lineSafe(p[i - 1], p[i]))
                    return false;
            return !p.empty();
        }

        Status search(cv::Point start, cv::Point goal, Path& path, size_t& expansions) const
        {
            const int w = free_.cols, h = free_.rows;
            size_t n = static_cast<size_t>(w) * h;
            struct Entry
            {
                double f, g;
                int at;

                bool operator>(const Entry& b) const
                {
                    return f == b.f ? g < b.g : f > b.f;
                }
            };
            std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
            std::vector<double> g(n, std::numeric_limits<double>::infinity());
            std::vector<int> parent(n, -1);
            std::vector<uint8_t> closed(n, 0);
            int first = start.y * w + start.x, last = goal.y * w + goal.x;
            g[first] = 0.;
            auto heuristic = [&](int x, int y) { return std::hypot(x - goal.x, y - goal.y); };
            open.push({heuristic(start.x, start.y), 0., first});
            auto begin = Clock::now();
            while (!open.empty())
            {
                auto e = open.top();
                open.pop();
                if (closed[e.at] || e.g > g[e.at])
                    continue;
                if (++expansions > cfg_.max_search_expansions ||
                    elapsedMs(begin) > cfg_.search_timeout_ms)
                    return Status::SearchLimit;
                if (e.at == last)
                {
                    for (int i = last; i != -1; i = parent[i])
                        path.push_back(cellCenter(i % w, i / w));
                    std::reverse(path.begin(), path.end());
                    return Status::Success;
                }
                closed[e.at] = 1;
                int x = e.at % w, y = e.at / w;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || nx >= w || ny < 0 || ny >= h || !free_.at<uint8_t>(ny, nx))
                            continue;
                        if (dx && dy && (!free_.at<uint8_t>(y, nx) || !free_.at<uint8_t>(ny, x)))
                            continue;
                        int next = ny * w + nx;
                        if (closed[next])
                            continue;
                        double ng = e.g + cost_.at<float>(ny, nx) * ((dx && dy) ? std::sqrt(2.) : 1.);
                        if (ng < g[next])
                        {
                            g[next] = ng;
                            parent[next] = e.at;
                            open.push({ng + heuristic(nx, ny), ng, next});
                        }
                    }
            }
            return Status::NoPath;
        }

        double weightedLineCost(const Vec2& a,const Vec2& b) const
        {
            const double dist=(b-a).norm();
            const int count=std::max(1,int(std::ceil(dist/(map_.info.resolution*.5))));
            double sum=0.;
            for(int k=0;k<count;++k) {
                Vec2 q=(a+(b-a)*((k+.5)/count))/map_.info.resolution;
                int x=int(std::floor(q.x())),y=int(std::floor(q.y()));
                if(x<0||y<0||x>=cost_.cols||y>=cost_.rows)return 1e100;
                sum+=cost_.at<float>(y,x);
            }
            return dist*sum/count;
        }
        double weightedSubpathCost(const Path& p,size_t a,size_t b) const
        {
            double sum=0.;for(size_t i=a+1;i<=b;++i)sum+=weightedLineCost(p[i-1],p[i]);return sum;
        }

        Path simplify(const Path& path) const
        {
            if (path.size() < 3)
                return path;
            std::vector<uint8_t> keep(path.size(), 0);
            keep.front() = keep.back() = 1;
            std::vector<std::pair<size_t, size_t>> stack{{0, path.size() - 1}};
            while (!stack.empty())
            {
                auto [a, b] = stack.back();
                stack.pop_back();
                if (b <= a + 1)
                    continue;
                Vec2 ab = path[b] - path[a];
                double maxd = -1;
                size_t middle = (a + b) / 2;
                for (size_t i = a + 1; i < b; ++i)
                {
                    double u = ab.squaredNorm() > 0
                                   ? std::clamp((path[i] - path[a]).dot(ab) / ab.squaredNorm(), 0., 1.)
                                   : 0.;
                    double d = (path[i] - (path[a] + u * ab)).norm();
                    if (d > maxd)
                    {
                        maxd = d;
                        middle = i;
                    }
                }
                if (maxd > cfg_.simplify_tolerance || !lineSafe(path[a], path[b]) ||
                    weightedLineCost(path[a],path[b]) > weightedSubpathCost(path,a,b)*1.03 + 1e-8)
                {
                    keep[middle] = 1;
                    stack.push_back({a, middle});
                    stack.push_back({middle, b});
                }
            }
            Path p;
            for (size_t i = 0; i < path.size(); ++i)
                if (keep[i])
                    p.push_back(path[i]);
            return p;
        }

        MinimumSnap::PointPair corridor(const Vec2& p, double range) const
        {
            const double r = map_.info.resolution;
            int x = static_cast<int>(std::floor(p.x() / r)),
                y = static_cast<int>(std::floor(p.y() / r));
            int l = x, rr = x, b = y, t = y;
            int steps = static_cast<int>(
                std::min(std::ceil(range / r), static_cast<double>(std::max(free_.cols, free_.rows))));
            auto clear = [&](int x0, int y0, int x1, int y1)
            {
                if (x0 < 0 || y0 < 0 || x1 >= free_.cols || y1 >= free_.rows)
                    return false;
                return integral_.at<double>(y1 + 1, x1 + 1) - integral_.at<double>(y0, x1 + 1) -
                    integral_.at<double>(y1 + 1, x0) + integral_.at<double>(y0, x0) ==
                    0.;
            };
            for (int i = 0; i < steps; ++i)
            {
                if (clear(l - 1, b, rr, t))
                    --l;
                if (clear(l, b, rr + 1, t))
                    ++rr;
                if (clear(l, b - 1, rr, t))
                    --b;
                if (clear(l, b, rr, t + 1))
                    ++t;
            }
            double eps = r * 1e-4;
            return {
                static_cast<float>(l * r + eps), static_cast<float>(b * r + eps),
                static_cast<float>((rr + 1) * r - eps), static_cast<float>((t + 1) * r - eps)
            };
        }

        bool optimize(Path path, const Request& request, Result& out)
        {
            auto begin = Clock::now();
            std::vector<double> times;
            for (size_t i = 1; i < path.size(); ++i)
            {
                double d = (path[i] - path[i - 1]).norm();
                times.push_back(
                    std::max({0.05, 2 * d / cfg_.max_speed, 2 * std::sqrt(d / cfg_.max_acceleration)}));
            }
            std::vector<MinimumSnap::PointPair> boxes;
            auto buildBoxes = [&]()
            {
                boxes.clear();
                for (size_t i = 0; i < path.size(); ++i)
                {
                    const auto& p = path[i];
                    if (i == 0 || i + 1 == path.size() || cfg_.backend != Backend::CorridorOSQP)
                        boxes.push_back({
                            static_cast<float>(p.x()), static_cast<float>(p.y()),
                            static_cast<float>(p.x()), static_cast<float>(p.y())
                        });
                    else
                        boxes.push_back(corridor(p, cfg_.corridor_range));
                }
            };
            buildBoxes();
            for (int iteration = 0; iteration < cfg_.optimization_iterations; ++iteration)
            {
                if (elapsedMs(begin) > cfg_.optimization_timeout_ms)
                {
                    out.logs.push_back("optimization wall-clock budget exhausted between solver calls");
                    break;
                }
                std::vector<MinimumSnap::SolveInput::Item> items(path.size());
                for (size_t i = 0; i < path.size(); ++i)
                    items[i].xy = path[i].cast<float>();
                items.front().useVxvy = items.back().useVxvy = true;
                // 原点不影响向量；R^T 将 world 速度转换为地图局部速度。
                items.front().vxvy =
                    map_.info.toLocal(map_.info.origin + request.start_velocity).cast<float>();
                items.back().vxvy =
                    map_.info.toLocal(map_.info.origin + request.goal_velocity).cast<float>();
                Attempt attempt;
                attempt.iteration = iteration + 1;
                for (double v : times)
                    attempt.duration += v;
                auto [sx, sy] = optimizer_.once(times, items, boxes, cfg_.backend);
                attempt.solver_ok = sx.rows() == static_cast<int>(times.size()) &&
                    sy.rows() == sx.rows() && sx.cols() == 6 && sy.cols() == 6 &&
                    sx.allFinite() && sy.allFinite();
                std::vector<Eigen::Matrix<double, 2, 6>> coeff;
                std::vector<size_t> bad;
                if (attempt.solver_ok)
                {
                    for (int i = 0; i < sx.rows(); ++i)
                    {
                        Eigen::Matrix<double, 2, 6> c;
                        for (int j = 0; j < 6; ++j)
                            c.col(j) = Vec2(sx(i, 5 - j), sy(i, 5 - j));
                        coeff.push_back(c);
                        if (!curveSafe(detail::bernstein(c)))
                            bad.push_back(static_cast<size_t>(i));
                        attempt.speed_bound =
                            std::max(attempt.speed_bound, detail::derivativeBound(c, 1, times[i]));
                        attempt.acceleration_bound = std::max(attempt.acceleration_bound,
                                                              detail::derivativeBound(c, 2, times[i]));
                        attempt.jerk_bound =
                            std::max(attempt.jerk_bound, detail::derivativeBound(c, 3, times[i]));
                    }
                    attempt.collision_free = bad.empty();
                }
                out.attempts.push_back(attempt);
                out.logs.push_back("optimization attempt " + std::to_string(iteration + 1) +
                    ": solver=" + std::to_string(attempt.solver_ok) +
                    ", collision_free=" + std::to_string(attempt.collision_free));
                out.corridors.clear();
                for (const auto& b : boxes)
                {
                    Corridor c;
                    c.corners = {
                        map_.info.toWorld({b[0], b[1]}), map_.info.toWorld({b[2], b[1]}),
                        map_.info.toWorld({b[2], b[3]}), map_.info.toWorld({b[0], b[3]})
                    };
                    out.corridors.push_back(c);
                }
                double scale = std::max(attempt.speed_bound / cfg_.max_speed,
                                        std::sqrt(attempt.acceleration_bound / cfg_.max_acceleration));
                if (cfg_.max_jerk > 0)
                    scale = std::max(scale, std::cbrt(attempt.jerk_bound / cfg_.max_jerk));
                if (attempt.solver_ok && attempt.collision_free && scale <= 1. + 1e-6)
                {
                    // 明确验证端点位置/速度和段间 p/v/a 连续性，防止求解器残差被当成成功。
                    if (!checkEqualities(coeff, times, request))
                    {
                        out.logs.push_back("endpoint/continuity residual too large");
                        return false;
                    }
                    double offset = 0.;
                    std::vector<PolynomialSegment> segments;
                    for (size_t i = 0; i < coeff.size(); ++i)
                    {
                        PolynomialSegment s;
                        s.start_time = offset;
                        s.duration = times[i];
                        for (int j = 0; j < 6; ++j)
                            s.coefficients.col(j) = map_.info.rotateToWorld(coeff[i].col(j));
                        s.coefficients.col(0) += map_.info.origin;
                        segments.push_back(s);
                        offset += times[i];
                    }
                    std::vector<Sample> samples;
                    for (size_t i = 0; i < segments.size(); ++i)
                    {
                        double countd = std::ceil(segments[i].duration / cfg_.sample_dt);
                        if (!std::isfinite(countd) || countd > cfg_.max_output_samples ||
                            samples.size() + countd + 1 > cfg_.max_output_samples)
                        {
                            out.logs.push_back("output sample limit exceeded");
                            return false;
                        }
                        size_t n = std::max<size_t>(1, static_cast<size_t>(countd));
                        for (size_t k = (i ? 1 : 0); k <= n; ++k)
                            samples.push_back(segments[i].evaluate(segments[i].duration * k / n));
                    }
                    out.segments = std::move(segments);
                    out.trajectory = std::move(samples);
                    out.final_path.clear();
                    for (const auto& s : out.trajectory)
                        out.final_path.push_back(s.position);
                    out.duration = offset;
                    out.speed_bound = attempt.speed_bound;
                    out.acceleration_bound = attempt.acceleration_bound;
                    out.jerk_bound = attempt.jerk_bound;
                    out.has_timed_trajectory = true;
                    return true;
                }
                if (!attempt.solver_ok)
                {
                    for (auto& v : times)
                        v *= 1.5;
                    continue;
                }
                if (!bad.empty())
                {
                    // 对碰撞段插入原前端线段的中点；原线段已经过碰撞检查。
                    for (auto it = bad.rbegin(); it != bad.rend(); ++it)
                    {
                        size_t i = *it;
                        path.insert(path.begin() + i + 1, (path[i] + path[i + 1]) * 0.5);
                        double half = times[i] * 0.5;
                        times[i] = half;
                        times.insert(times.begin() + i + 1, half);
                    }
                    buildBoxes();
                    // 保守收紧到线段骨架，后续碰撞仍会细分，而不是无限放宽。
                    for (size_t i = 1; i + 1 < boxes.size(); ++i)
                    {
                        auto& b = boxes[i];
                        float px = static_cast<float>(path[i].x()),
                              py = static_cast<float>(path[i].y());
                        b = {
                            0.5f * (b[0] + px), 0.5f * (b[1] + py), 0.5f * (b[2] + px),
                            0.5f * (b[3] + py)
                        };
                    }
                }
                if (scale > 1.)
                    for (auto& v : times)
                        v *= std::min(10., scale * 1.05);
                if (path.size() > 2000)
                {
                    out.logs.push_back("too many optimization control points");
                    break;
                }
            }
            return false;
        }

        bool checkEqualities(const std::vector<Eigen::Matrix<double, 2, 6>>& c,
                             const std::vector<double>& t, const Request& request) const
        {
            std::vector<PolynomialSegment> p(c.size());
            for (size_t i = 0; i < c.size(); ++i)
            {
                p[i].coefficients = c[i];
                p[i].duration = t[i];
            }
            const auto first = p.front().evaluate(0), last = p.back().evaluate(t.back());
            Vec2 sv = map_.info.toLocal(map_.info.origin + request.start_velocity),
                 gv = map_.info.toLocal(map_.info.origin + request.goal_velocity);
            if ((first.position - map_.info.toLocal(request.start)).norm() > 1e-4 ||
                (last.position - map_.info.toLocal(request.goal)).norm() > 1e-4 ||
                (first.velocity - sv).norm() > 1e-4 || (last.velocity - gv).norm() > 1e-4)
                return false;
            for (size_t i = 1; i < p.size(); ++i)
            {
                auto a = p[i - 1].evaluate(t[i - 1]), b = p[i].evaluate(0);
                if ((a.position - b.position).norm() > 1e-4 ||
                    (a.velocity - b.velocity).norm() > 1e-4 ||
                    (a.acceleration - b.acceleration).norm() > 1e-3)
                    return false;
            }
            return true;
        }

        void makeDebug(Result& out, const Request& request) const
        {
            cv::Mat base;
            cv::cvtColor(map_.free, base, cv::COLOR_GRAY2BGR);
            base.setTo(cv::Scalar(110, 110, 110), map_.unknown);
            cv::Mat inflated;
            cv::cvtColor(free_, inflated, cv::COLOR_GRAY2BGR);
            cv::Mat soft = cost_.clone();
            double hi = 1.;
            for (int y = 0; y < soft.rows; ++y)
                for (int x = 0; x < soft.cols; ++x)
                    if (std::isfinite(soft.at<float>(y, x)))
                        hi = std::max(hi, static_cast<double>(soft.at<float>(y, x)));
            cv::Mat normalized(soft.size(), CV_8UC1);
            for (int y = 0; y < soft.rows; ++y)
                for (int x = 0; x < soft.cols; ++x)
                    normalized.at<uint8_t>(y, x) =
                        std::isfinite(soft.at<float>(y, x))
                            ? static_cast<uint8_t>(255 * soft.at<float>(y, x) / hi)
                            : 255;
            cv::Mat heat;
            cv::applyColorMap(normalized, heat, cv::COLORMAP_TURBO);
            heat.setTo(cv::Scalar(0, 0, 0), free_ == 0);
            // 之后的绘制在 canonical row 上进行，最后统一 flip，避免各阶段独立翻转。
            auto pixel = [&](const Vec2& p)
            {
                Vec2 q = map_.info.toLocal(p) / map_.info.resolution;
                if (!q.allFinite() || q.cwiseAbs().maxCoeff() > 1e7)
                    return cv::Point(-1, -1);
                return cv::Point(static_cast<int>(std::floor(q.x())),
                                 static_cast<int>(std::floor(q.y())));
            };
            auto draw = [&](cv::Mat& image, const Path& path, cv::Scalar color, int width = 1)
            {
                for (size_t i = 1; i < path.size(); ++i)
                    cv::line(image, pixel(path[i - 1]), pixel(path[i]), color, width, cv::LINE_AA);
            };
            auto endpoints = [&](cv::Mat& im)
            {
                cv::Point p;
                if (map_.worldToCell(request.start, p))
                    cv::circle(im, p, 3, {0, 220, 0}, -1);
                if (map_.worldToCell(request.goal, p))
                    cv::circle(im, p, 3, {220, 0, 220}, -1);
            };
            auto push = [&](const std::string& name, cv::Mat im)
            {
                endpoints(im);
                cv::flip(im, im, 0);
                out.images.push_back({name, im});
            };
            push("00_input_map", base.clone());
            push("01_inflated_map", inflated.clone());
            push("02_costmap", heat);
            auto raw = inflated.clone();
            draw(raw, out.raw_path, {0, 140, 255});
            push("03_astar_raw", raw);
            auto simple = inflated.clone();
            draw(simple, out.simplified_path, {0, 180, 0}, 2);
            for (const auto& p : out.simplified_path)
                cv::circle(simple, pixel(p), 2, {0, 100, 0}, -1);
            push("04_simplified", simple);
            auto corridors = inflated.clone();
            for (const auto& c : out.corridors)
                for (int i = 0; i < 4; ++i)
                    cv::line(corridors, pixel(c.corners[i]), pixel(c.corners[(i + 1) % 4]),
                             {200, 130, 0}, 1);
            draw(corridors, out.simplified_path, {0, 180, 0});
            push("05_corridors", corridors.clone());
            auto final = inflated.clone();
            draw(final, out.final_path, {0, 0, 230}, 2);
            push("06_final", final);
            auto overlay = corridors;
            draw(overlay, out.raw_path, {0, 140, 255});
            draw(overlay, out.simplified_path, {0, 180, 0});
            draw(overlay, out.final_path, {0, 0, 230}, 2);
            push("07_overlay", overlay);
            int panel_w = std::max(320, map_.free.cols), panel_h = std::max(150, map_.free.rows);
            cv::Mat montage(panel_h * 2, panel_w * 4, CV_8UC3, cv::Scalar(35, 35, 35));
            for (size_t i = 0; i < out.images.size(); ++i)
            {
                int ox = static_cast<int>(i % 4) * panel_w, oy = static_cast<int>(i / 4) * panel_h;
                double scale = std::min(static_cast<double>(panel_w) / out.images[i].image.cols,
                                        static_cast<double>(panel_h - 28) / out.images[i].image.rows);
                cv::Mat resized;
                cv::resize(out.images[i].image, resized, {}, scale, scale, cv::INTER_NEAREST);
                resized.copyTo(montage(cv::Rect(ox, oy + 28, resized.cols, resized.rows)));
                cv::putText(montage, out.images[i].name, {ox + 5, oy + 19}, cv::FONT_HERSHEY_SIMPLEX,
                            0.45, {240, 240, 240}, 1, cv::LINE_AA);
            }
            out.images.push_back({"08_montage", montage});
        }
    };
} // namespace tdt_nav

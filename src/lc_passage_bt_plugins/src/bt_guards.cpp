#include <atomic>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <lc_passage_msgs/msg/passage_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lc_passage_bt_plugins
{
    class GuardSubscription
    {
    protected:
        std::atomic<bool> locked_{false};
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<lc_passage_msgs::msg::PassageState>::SharedPtr sub_;
        rclcpp::CallbackGroup::SharedPtr group_;
        rclcpp::executors::SingleThreadedExecutor executor_;

        void init(const BT::NodeConfiguration& config)
        {
            node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
            group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive, false);
            executor_.add_callback_group(group_, node_->get_node_base_interface());
            rclcpp::SubscriptionOptions options;
            options.callback_group = group_;
            sub_ = node_->create_subscription<lc_passage_msgs::msg::PassageState>(
                "passage/state", rclcpp::QoS(1).reliable().transient_local(),
                [this](lc_passage_msgs::msg::PassageState::ConstSharedPtr s)
                {
                    locked_ = s->locked;
                },
                options);
        }
    };

    class PassageLocked : public BT::ConditionNode, private GuardSubscription
    {
    public:
        PassageLocked(const std::string& name, const BT::NodeConfiguration& config)
            : BT::ConditionNode(name, config)
        {
            init(config);
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<nav_msgs::msg::Path>("path")};
        }

        BT::NodeStatus tick() override
        {
            executor_.spin_some();
            nav_msgs::msg::Path p;
            return locked_ && getInput("path", p) && !p.poses.empty()
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        }
    };

    class PassageSafeRecovery : public BT::DecoratorNode,
                                private GuardSubscription
    {
    public:
        PassageSafeRecovery(const std::string& name,
                            const BT::NodeConfiguration& config)
            : BT::DecoratorNode(name, config)
        {
            init(config);
        }

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override
        {
            executor_.spin_some();
            if (locked_)
            {
                if (child_node_->status() == BT::NodeStatus::RUNNING)
                    child_node_->halt();
                return BT::NodeStatus::RUNNING;
            }
            auto status = child_node_->executeTick();
            if (status != BT::NodeStatus::RUNNING)
                resetChild();
            return status;
        }
    };
} // namespace lc_passage_bt_plugins
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<lc_passage_bt_plugins::PassageLocked>("PassageLocked");
    factory.registerNodeType<lc_passage_bt_plugins::PassageSafeRecovery>(
        "PassageSafeRecovery");
}

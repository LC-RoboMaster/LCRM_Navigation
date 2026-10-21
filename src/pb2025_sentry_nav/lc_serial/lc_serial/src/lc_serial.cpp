#include "../include/lc_serial/lc_serial.h"
//订阅nav2的cmd_vel话题

//处理nav2的数据

//cjson打包

//打开串口

//串口发送数据

Serial::Serial():Node("serial_node"){
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Serial::sendData, this, std::placeholders::_1));

    device_name_ = this->declare_parameter("device_name", "/dev/ttyACM0");
    drivers::serial_driver::SerialPortConfig device_config_(
        115200,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);    

    try{
        io_context_ = std::make_shared<drivers::common::IoContext>(1);
        serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_context_);
        serial_driver_->init_port(device_name_, device_config_);
        if(!serial_driver_->port()->is_open()){
            serial_driver_->port()->open();
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Serial node has been failed to start.");
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        reopenPort();  
        throw e;
    }
    
    RCLCPP_INFO(this->get_logger(), "Serial node has been started.");
}

Serial::~Serial(){
    
}

void Serial::sendData(const geometry_msgs::msg::Twist::SharedPtr msg){
    RCLCPP_WARN(this->get_logger(), "Twist linear_x: {%f} linear_y: {%f} angular_z: {%f}", msg->linear.x, msg->linear.y, msg->angular.z);
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;
    double angular_z = msg->angular.z;

    char* str = NULL;
    //"date":[x,y];
    cJSON* cjson_date = cJSON_CreateArray();
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(linear_x));
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(linear_y));
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(angular_z));

    // dat
    cJSON* cjson_dat = cJSON_CreateObject();
    cJSON_AddItemToObject(cjson_dat, "date", cjson_date);
    cJSON_AddStringToObject(cjson_dat, "mode", "visual");

    // send
    cJSON* cjson_send = cJSON_CreateObject();
    cJSON_AddStringToObject(cjson_send, "cmd", "ctr_mode");

    cJSON_AddItemToObject(cjson_send, "dat", cjson_dat);

    // 转化为待发送数据结构
    str = cJSON_PrintUnformatted(cjson_send);
    cJSON_Delete(cjson_send);
    int str_len = std::strlen(str);
    
    std::vector<uint8_t> data(str, str + str_len);
    data.push_back('\n');
    // data.push_back('\0');
    for(int i = 0; i < str_len; i++){
        std::cout << str[i];
    }

    serial_driver_->port()->send(data);

}

void Serial::reopenPort(){
    RCLCPP_ERROR(this->get_logger(), "Try to reopen the serial port...");
    try{
        if(serial_driver_->port()->is_open()){
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(this->get_logger(), "Success to reopen the port!");
    }catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "ERROR: %s", e.what());
        if(rclcpp::ok()){
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Serial>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class robotNewsStationNode : public rclcpp::Node
{
public:
    robotNewsStationNode() : Node("robot_news_station")
    {
        this->declare_parameter("RobotName", "R2D2");
        robot_name_ = this->get_parameter("RobotName").as_string();
        
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&robotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "robot_news_station has been started");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("hi, this is ") + robot_name_ + std::string(" from the robot news station");
        publisher_->publish(msg);
    }
    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"

class turtleControllNode : public rclcpp::Node 
{
public:
    turtleControllNode() : Node("turtle_controller") 
    {
        RCLCPP_INFO(this->get_logger(), "turtle_controller has been started");
    }

private:
};
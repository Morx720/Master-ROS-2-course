#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class batteryNode : public rclcpp::Node 
{
public:
    batteryNode() : Node("battery") 
    {
        RCLCPP_INFO(this->get_logger(), "battery has been started");
    }

private:
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<batteryNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), leds(3, false)
        {
        auto pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);
        auto service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                                                                               std::bind(&LedPanelNode::callbackSetLed, this));
        RCLCPP_INFO(this->get_logger(), "led_panel has been started");
    }

private:
    void publishLedState()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_state = leds;

    }

    void callbackSetLed() {}

    std::vector<bool> leds;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

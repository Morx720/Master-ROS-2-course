#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), leds(3, false)
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);

        service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                                                                          std::bind(&LedPanelNode::callbackSetLed,
                                                                                    this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "led_panel has been started");
    }

private:
    void publishLedState()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_state = leds;
        pub_->publish(msg);
    }

    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr resopnse)
    {
        int led = request->led_number;
        bool state = request->state;
        if (!(led > 0 and led < 4))
        {
            resopnse->success = false;
            RCLCPP_ERROR(this->get_logger(), "Service call denied. got invaled led number:%d. allowed number 1-3" , led);
            return;
            
        }

        RCLCPP_INFO(this->get_logger(), "led %d has been set to %s", led, state ? "on" : "off");
        leds[led - 1] = state;
        this->publishLedState();
        resopnse->success = true;
    }

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

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class batteryNode : public rclcpp::Node
{
public:
    batteryNode() : Node("battery"), battery_empty(true)
    {
        this->setLed(3, true);
        battery_last_state_change = this->get_clock()->now().seconds();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&batteryNode::simulateBattery, this));
        RCLCPP_INFO(this->get_logger(), "battery has been started");
    }

    void setLed(int led, bool state)
    {
        threads_.push_back(std::thread(std::bind(&batteryNode::callSetLedServer, this, led, state)));
    }

    void callSetLedServer(int led, bool state)
    {
        auto client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server ....");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed_Request>();
        request->led_number = led;
        request->state = state;

        auto future = client_->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "led %d has been set to %s", led, state ? "on" : "off");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Servce call failed");
        }
    }

    void simulateBattery()
    {
        double now = this->get_clock()->now().seconds();
        if (battery_empty)
        {
            if (now - battery_last_state_change > 6.0)
            {
                battery_last_state_change = now;
                battery_empty = false;
                this->setLed(3, false);
                RCLCPP_INFO(this->get_logger(), "Battery full");
            }
        }
        else
        {
            if (now - battery_last_state_change > 4.0)
            {
                battery_last_state_change = now;
                battery_empty = true;
                this->setLed(3, true);
                RCLCPP_INFO(this->get_logger(), "Battery empty");
            }
        }
    }

private:
    double battery_last_state_change;
    bool battery_empty;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<batteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
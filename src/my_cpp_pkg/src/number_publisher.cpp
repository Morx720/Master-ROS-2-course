#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPubNode : public rclcpp::Node
{
public:
    NumberPubNode() : Node("number_publisher")
    {
        this->declare_parameter("number_to_publish", 2);
        this->declare_parameter("publish_rate", 1.0);

        number_ = this->get_parameter("number_to_publish").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / publish_rate)),
                                         std::bind(&NumberPubNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "number_publisher has been started");
    }

private:
    void timerCallback()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }

    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), sum_(0)
    {
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                                                                                  std::bind(&NumberCounterNode::subCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "number_counter has been started");
    }

private:
    void subCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        sum_ += msg->data;
        auto pubMsg = example_interfaces::msg::Int64();
        pubMsg.data = sum_;
        pub_->publish(pubMsg);
        RCLCPP_INFO(this->get_logger(), "Sum:%ld", pubMsg.data);
    };
    std::int64_t sum_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
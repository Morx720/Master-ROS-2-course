#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), sum_(0)
    {
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                                                                                  std::bind(&NumberCounterNode::subCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter", std::bind(&NumberCounterNode::callbackResetCounter,this, _1, _2));

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

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data == true)
        {
            sum_ = 0;
            response->success = true;
            response->message = "Counter has been reseted";
        }
        else
        {
            response->success = false;
            response->message = "failed to reset counter";
        };
    };

    std::int64_t sum_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pub_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
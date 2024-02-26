#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberPubNode : public rclcpp::Node 
{
public:
    NumberPubNode() : Node("number_publisher") 
    {
        publisher_ = this -> create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(500),std::bind(&NumberPubNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "number_publisher has been started");
    }

private:

    void timerCallback(){
        auto msg = example_interfaces::msg::Int64();
        msg.data = 2;
        publisher_->publish(msg);
    };
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
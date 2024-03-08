#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class ResetCounterClientNode : public rclcpp::Node 
{
public:
    ResetCounterClientNode() : Node("reset_counter_client") 
    {
        RCLCPP_INFO(this->get_logger(), "reset_counter_client has been started");
        threads_.push_back(std::thread(std::bind(&ResetCounterClientNode::callResetCounterServer, this)));
    }

    void callResetCounterServer(){
        auto client = this->create_client<example_interfaces::srv::SetBool>("reset_counter");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server ....");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool_Request>();
        request->data = true;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"%s", response->message.c_str());
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Servce call failed");
        }
        

        
    }
private:
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<ResetCounterClientNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
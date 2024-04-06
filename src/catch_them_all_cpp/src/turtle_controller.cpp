#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/alive_turtles.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class turtleControllNode : public rclcpp::Node
{
public:
    turtleControllNode() : Node("turtle_controller"), name_("turtle1"), turtlesim_up_(false)
    {
        this->declare_parameter("catch_closest_turtle_first", true);
        catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();

        cmd_Vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            name_ + "/cmd_vel", 10);
        pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            name_ + "/pose", 10, std::bind(&turtleControllNode::callbackPosse, this, std::placeholders::_1));
        alive_turtles_sub = this->create_subscription<my_robot_interfaces::msg::AliveTurtles>(
            "alive_turtles", 10, std::bind(&turtleControllNode::callbackTurtleToCatch, this, std::placeholders::_1));

        control_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&turtleControllNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "turtle_controller has been started");
    }

private:
    void callbackPosse(const turtlesim::msg::Pose::SharedPtr pose)
    {
        pose_ = *pose.get();
        turtlesim_up_ = true;
    }

    double getDistanceFromCurrentPose(my_robot_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;

        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callbackTurtleToCatch(const my_robot_interfaces::msg::AliveTurtles::SharedPtr msg)
    {
        if (!msg->trutles.empty())
        {
            if (catch_closest_turtle_first_)
            {
                my_robot_interfaces::msg::Turtle closest_turtle = msg->trutles.at(0);
                double closest__turtle_diistance = getDistanceFromCurrentPose(closest_turtle);

                for (int i = 1; i < (int)msg->trutles.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->trutles.at(i));
                    if (distance < closest__turtle_diistance)
                    {
                        closest_turtle = msg->trutles.at(i);
                        closest__turtle_diistance = distance;
                    }
                }

                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->trutles.at(0);
            }
        }
    }

    void publlisshVmdVel(double x, double theta)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.angular.z = theta;
        cmd_Vel_pub->publish(msg);
    }

    void controlLoop()
    {
        if (!turtlesim_up_ || turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            msg.linear.x = 2 * distance;

            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            catch_turtle_thread_.push_back(std::make_shared<std::thread>(std::bind(&turtleControllNode::callCatchTurtleService, this, turtle_to_catch_.name)));
            turtle_to_catch_.name = "";
        }
        cmd_Vel_pub->publish(msg);
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for catch_turtle Server to be up...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle_Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            auto resopnse = future.get();
            if (!resopnse->success)
            {
                RCLCPP_ERROR(this->get_logger(), "failed to catch turtle");
                
            }
            
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed. ");
            
        }
        
        
    }


    std::string name_;
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;
    bool catch_closest_turtle_first_;
    my_robot_interfaces::msg::Turtle turtle_to_catch_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_Vel_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Subscription<my_robot_interfaces::msg::AliveTurtles>::SharedPtr alive_turtles_sub;
    rclcpp::TimerBase::SharedPtr control_timer;

    std::vector<std::shared_ptr<std::thread>> catch_turtle_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleControllNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
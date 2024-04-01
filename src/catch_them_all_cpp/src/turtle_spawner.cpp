#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/alive_turtles.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class turtleSpawnerNode : public rclcpp::Node
{
public:
    turtleSpawnerNode() : Node("turttle_spawner"), turtle_count_(1)
    {
        this->declare_parameter("spawn_frequency", 1.0);
        spawnRate_ = this->get_parameter("spawn_frequency").as_double();

        this->declare_parameter("turtle_name_prefix", "Turtle");
        name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();

        alive_turtle_pup_ = this->create_publisher<my_robot_interfaces::msg::AliveTurtles>("alive_turtles", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0)),
                                         std::bind(&turtleSpawnerNode::spawnRandomTurtle, this));

        catch_turtle_server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>("catch_turtle",
                                                                                           std::bind(&turtleSpawnerNode::catchTurtleCallback, this));

        RCLCPP_INFO(this->get_logger(), "turttle_spawner has been started");
    }

private:
    void publishAliveTurtle()
    {
        auto msg = my_robot_interfaces::msg::AliveTurtles();
        msg.trutles = alive_turtle;
        alive_turtle_pup_->publish(msg);
    }

    void catchTurtleCallback(const my_robot_interfaces::srv::CatchTurtle::Request request,
                             const my_robot_interfaces::srv::CatchTurtle::Response response)
    {
        
    }

    void spawnRandomTurtle() {}

    void killTurtle(std::string turtle_name){}

    int turtle_count_;
    double spawnRate_;
    std::string name_prefix_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtle;

    rclcpp::Publisher<my_robot_interfaces::msg::AliveTurtles>::SharedPtr alive_turtle_pup_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
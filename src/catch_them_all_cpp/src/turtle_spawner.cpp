#include <iostream>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/alive_turtles.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

class turtleSpawnerNode : public rclcpp::Node
{
public:
    turtleSpawnerNode() : Node("turtle_spawner"), turtle_count_(1)
    {
        this->declare_parameter("spawn_frequency", 1.0);
        spawnRate_ = this->get_parameter("spawn_frequency").as_double();

        this->declare_parameter("turtle_name_prefix", "Turtle");
        name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();

        alive_turtle_pup_ = this->create_publisher<my_robot_interfaces::msg::AliveTurtles>("alive_turtles", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0)),
                                         std::bind(&turtleSpawnerNode::spawnRandomTurtle, this));

        catch_turtle_server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>("catch_turtle",
                                                                                           std::bind(&turtleSpawnerNode::catchTurtleCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "turttle_spawner has been started");
    }

private:
    float randomFloat(float min, float max)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(min, max);
        return dis(gen);
    }
    void publishAliveTurtle()
    {
        auto msg = my_robot_interfaces::msg::AliveTurtles();
        msg.trutles = alive_turtle;
        alive_turtle_pup_->publish(msg);
    }

    void catchTurtleCallback(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&turtleSpawnerNode::killTurtle, this, request->name)));

        response->success = true;
    }

    void spawnRandomTurtle()
    {
        spawn_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&turtleSpawnerNode::callSpawnTurtleServer, this)));
    }

    void callSpawnTurtleServer()
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for spawn server ....");
        }
        auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
        request->x = randomFloat(1.0, 10.0);
        request->y = randomFloat(1.0, 10.0);
        request->theta = randomFloat(0.0, 2 * M_PI);
        request->name = name_prefix_ + std::to_string(turtle_count_);
        turtle_count_ += 1;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (response->name != "")
            {
                auto new_turtle = my_robot_interfaces::msg::Turtle();
                new_turtle.name = response->name;
                new_turtle.x = request->x;
                new_turtle.y = request->y;
                new_turtle.theta = request->theta;
                alive_turtle.push_back(new_turtle);
                publishAliveTurtle();

                RCLCPP_INFO(this->get_logger(), "turtlesim has spawned %s", response->name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Servce call failed");
        }
    }

    void killTurtle(std::string turtle_name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill server ....");
        }
        auto request = std::make_shared<turtlesim::srv::Kill_Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            future.get();
            for (int i = 0; i < (int)alive_turtle.size(); i++)
            {
                if (alive_turtle.at(i).name == turtle_name)
                {
                    alive_turtle.erase(alive_turtle.begin() + i);
                    publishAliveTurtle();
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Servce call failed");
        }
    }

    int turtle_count_;
    double spawnRate_;
    std::string name_prefix_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtle;

    rclcpp::Publisher<my_robot_interfaces::msg::AliveTurtles>::SharedPtr alive_turtle_pup_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_server_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
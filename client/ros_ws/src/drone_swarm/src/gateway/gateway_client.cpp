#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/drone_request.hpp"
#include "drone_msgs/msg/drone_response.hpp"
#include "gateway/swarm_client_http.h"

std::string get_ip() {
    const char *env_ip = std::getenv("SERVER_IP");
    std::string ip = env_ip ? std::string(env_ip) : "localhost";

    return "http://" + ip + ":2020/drone_position";
}

class GatewayClientNode : public rclcpp::Node
{
public:
    GatewayClientNode()
            : Node("gateway_client_node"),
              http_client_(get_ip())
    {
        using std::placeholders::_1;

        // Buffer size of 1 to always send & get the latest data
        subscription_ = this->create_subscription<drone_msgs::msg::DroneRequest>(
                "drone_position", 1,
                std::bind(&GatewayClientNode::handle_request, this, _1));

        publisher_ = this->create_publisher<drone_msgs::msg::DroneResponse>(
                "swarm_position", 1);

        RCLCPP_INFO(this->get_logger(), "GatewayClientNode initialized");
    }

private:
    void handle_request(const drone_msgs::msg::DroneRequest::SharedPtr msg)
    {
        try {
            auto swarm = http_client_.sendPositionAndGetSwarm(*msg);
            publisher_->publish(swarm);
            RCLCPP_INFO(this->get_logger(), "Published swarm data for drone_id: %s", msg->drone_id.c_str());
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "HTTP error: %s", e.what());
        }
    }

    SwarmClientHttp http_client_;
    rclcpp::Subscription<drone_msgs::msg::DroneRequest>::SharedPtr subscription_;
    rclcpp::Publisher<drone_msgs::msg::DroneResponse>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GatewayClientNode>());
    rclcpp::shutdown();
    return 0;
}

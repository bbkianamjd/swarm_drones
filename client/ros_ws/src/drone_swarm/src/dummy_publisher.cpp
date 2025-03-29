#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/drone_request.hpp"
#include "drone_msgs/msg/drone_response.hpp"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

struct DroneState {
    float x, y, z;
    std::string timestamp;
};

class DummyPublisherNode : public rclcpp::Node {
public:
    DummyPublisherNode()
            : Node("dummy_position_publisher"), sequence_(0) {

        // Size of 1 to always send and get the latest message
        publisher_ = this->create_publisher<drone_msgs::msg::DroneRequest>("drone_position", 1);
        subscription_ = this->create_subscription<drone_msgs::msg::DroneResponse>(
                "swarm_position", 1,
                std::bind(&DummyPublisherNode::swarm_callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
                1000ms, std::bind(&DummyPublisherNode::publish_position, this));

        const char *env_drone_id = std::getenv("DRONE_ID");
        drone_id_ = env_drone_id ? std::string(env_drone_id) : "default_drone";

        RCLCPP_INFO(this->get_logger(), "DummyPublisherNode started");
    }


private:
    void publish_position() {
        auto msg = drone_msgs::msg::DroneRequest();
        msg.sequence_number = sequence_++;
        msg.drone_id = drone_id_;
        msg.positionx = 1.0f;
        msg.positiony = 2.0f;
        msg.positionz = 3.0f;

        // Generate a simple timestamp string
        auto now = this->get_clock()->now();
        msg.timestamp = std::to_string(now.seconds());

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published dummy drone position #%d", msg.sequence_number);
    }

    void swarm_callback(const drone_msgs::msg::DroneResponse::SharedPtr msg) {
        std::lock_guard <std::mutex> lock(cache_mutex_);
        swarm_cache_.clear();
        for (const auto &drone : msg->swarm) {
            swarm_cache_[drone.drone_id] = {
                    drone.positionx,
                    drone.positiony,
                    drone.positionz,
                    drone.timestamp
            };
        }
        RCLCPP_INFO(this->get_logger(), "Swarm cache updated with %zu drones", swarm_cache_.size());
    }

    rclcpp::Publisher<drone_msgs::msg::DroneRequest>::SharedPtr publisher_;
    rclcpp::Subscription<drone_msgs::msg::DroneResponse>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    int32_t sequence_;

    std::unordered_map <std::string, DroneState> swarm_cache_;
    std::mutex cache_mutex_;
    std::string drone_id_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyPublisherNode>());
    rclcpp::shutdown();
    return 0;
}


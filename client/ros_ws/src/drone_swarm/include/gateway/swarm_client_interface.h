#ifndef DRONE_SWARM_GATEWAY_INTERFACE_H
#define DRONE_SWARM_GATEWAY_INTERFACE_H

#include "drone_msgs/msg/drone_request.hpp"
#include "drone_msgs/msg/drone_response.hpp"

/// An interface class for sending our drone position to a server
/// and retrieving the entire swarm's positions.
class SwarmClientInterface
{
public:
    virtual ~SwarmClientInterface() = default;

    /// Post the local drone's position to the server and return
    /// the updated swarm positions as a DroneResponse message.
    virtual drone_msgs::msg::DroneResponse sendPositionAndGetSwarm(
            const drone_msgs::msg::DroneRequest & request) = 0;
};

#endif // DRONE_SWARM_GATEWAY_INTERFACE_H

#ifndef SWARM_CLIENT_HTTP_H
#define SWARM_CLIENT_HTTP_H

#include "swarm_client_interface.h"
#include "drone_msgs/msg/drone_request.hpp"
#include "drone_msgs/msg/drone_response.hpp"

#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

class SwarmClientHttp : public SwarmClientInterface
{
public:
    explicit SwarmClientHttp(const std::string& server_url)
            : server_url_(server_url) {}

    drone_msgs::msg::DroneResponse sendPositionAndGetSwarm(
            const drone_msgs::msg::DroneRequest& request) override;

private:
    std::string server_url_;

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, std::string *s);
    nlohmann::json serializeRequest(const drone_msgs::msg::DroneRequest& request);
    drone_msgs::msg::DroneResponse parseResponse(const std::string& body);
};

#endif // SWARM_CLIENT_HTTP_H

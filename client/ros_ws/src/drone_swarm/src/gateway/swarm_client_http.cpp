#include "gateway/swarm_client_http.h"

using json = nlohmann::json;

size_t SwarmClientHttp::writeCallback(void *contents, size_t size, size_t nmemb, std::string *s)
{
    size_t totalSize = size * nmemb;
    s->append(static_cast<char *>(contents), totalSize);
    return totalSize;
}

json SwarmClientHttp::serializeRequest(const drone_msgs::msg::DroneRequest & req)
{
    return json{
            {"sequence_number", req.sequence_number},
            {"drone_id", req.drone_id},
            {"positionx", req.positionx},
            {"positiony", req.positiony},
            {"positionz", req.positionz},
            {"timestamp", req.timestamp}
    };
}

drone_msgs::msg::DroneResponse SwarmClientHttp::parseResponse(const std::string & body)
{
    drone_msgs::msg::DroneResponse response;
    auto swarm_json = json::parse(body).at("swarm");

    for (const auto & j : swarm_json)
    {
        drone_msgs::msg::DroneRequest r;
        r.sequence_number = j.at("sequence_number").get<int>();
        r.drone_id = j.at("drone_id").get<std::string>();
        r.positionx = j.at("positionx").get<float>();
        r.positiony = j.at("positiony").get<float>();
        r.positionz = j.at("positionz").get<float>();
        r.timestamp = j.at("timestamp").get<std::string>();
        response.swarm.push_back(r);
    }

    return response;
}

drone_msgs::msg::DroneResponse SwarmClientHttp::sendPositionAndGetSwarm(
        const drone_msgs::msg::DroneRequest & request)
{
    CURL *curl = curl_easy_init();
    std::string response_string;

    if (!curl) {
        throw std::runtime_error("Failed to initialize CURL");
    }

    json request_json = serializeRequest(request);
    std::string request_body = request_json.dump();

    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, server_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        throw std::runtime_error("CURL request failed: " + std::string(curl_easy_strerror(res)));
    }

    return parseResponse(response_string);
}

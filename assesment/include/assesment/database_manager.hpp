#ifndef DATABASE_MANAGER_HPP
#define DATABASE_MANAGER_HPP

#include <libpq-fe.h>
#include <string>
#include <vector>

struct WormholeData {
    std::string map1_name;
    std::string map2_name;
    double map1_x, map1_y, map1_oz, map1_ow;
    double map2_x, map2_y, map2_oz, map2_ow;
    // geometry_msgs::msg::Pose source_pose;
    // geometry_msgs::msg::Pose target_pose;
};

class DatabaseManager {
public:
    DatabaseManager(const std::string& connection_string);
    ~DatabaseManager();

    bool connect();
    void disconnect();

    std::vector<WormholeData> getWormholes(const std::string& from_map, const std::string& to_map);
    bool insertWormhole(const WormholeData& wormhole);

private:
    PGconn* connection_;
    std::string connection_string_;
};

#endif  // DATABASE_MANAGER_HPP
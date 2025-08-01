#include "assesment/database_manager.hpp"
#include <iostream>
#include <sstream>
// #include <queue>
// #include <map>
#include <libpq-fe.h>

DatabaseManager::DatabaseManager(const std::string& connection_string)
    : connection_(nullptr), connection_string_(connection_string) {}

DatabaseManager::~DatabaseManager() {
    disconnect();
}

bool DatabaseManager::connect() {
    std::cout << "Connecting to database..." << std::endl;
    connection_ = PQconnectdb(connection_string_.c_str());
    if (PQstatus(connection_) != CONNECTION_OK) {
        std::cerr << "Database connection failed: " << PQerrorMessage(connection_) << std::endl;
        return false;
    }
    std::cout << "Database connected successfully." << std::endl;
    return true;
}

void DatabaseManager::disconnect() {
    if (connection_) {
        PQfinish(connection_);
        connection_ = nullptr;
    }
}

std::vector<WormholeData> DatabaseManager::getWormholes(const std::string& from_map, const std::string& to_map) {
    std::vector<WormholeData> wormholes;
    
    if (!connection_) {
        std::cerr << "No database connection." << std::endl;
        return wormholes;
    }

    std::stringstream query;
    query << "SELECT map1_name, map2_name, map1_position_x, map1_position_y, map1_orientation_z, map1_orientation_w, "
          << "map2_position_x, map2_position_y, map2_orientation_z, map2_orientation_w FROM wormholes "
          << "WHERE (map1_name = '" << from_map << "' AND map2_name = '" << to_map << "') "
          << "OR (map1_name = '" << to_map << "' AND map2_name = '" << from_map << "')";
    
    PGresult* res = PQexec(connection_, query.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Failed to fetch wormholes: " << PQerrorMessage(connection_) << std::endl;
        PQclear(res);
        return wormholes;
    }

    int rows = PQntuples(res);
    for (int i = 0; i < rows; i++) {
        WormholeData wh;
        wh.map1_name = PQgetvalue(res, i, 0);
        wh.map2_name = PQgetvalue(res, i, 1);
        wh.map1_x = std::stod(PQgetvalue(res, i, 2));
        wh.map1_y = std::stod(PQgetvalue(res, i, 3));
        wh.map1_oz = std::stod(PQgetvalue(res, i, 4));
        wh.map1_ow = std::stod(PQgetvalue(res, i, 5));
        wh.map2_x = std::stod(PQgetvalue(res, i, 6));
        wh.map2_y = std::stod(PQgetvalue(res, i, 7));
        wh.map2_oz = std::stod(PQgetvalue(res, i, 8));
        wh.map2_ow = std::stod(PQgetvalue(res, i, 9));
        wormholes.push_back(wh);
    }
    PQclear(res);
    return wormholes;
}

bool DatabaseManager::insertWormhole(const WormholeData& wormhole) {
    if (!connection_) {
        std::cerr << "No database connection." << std::endl;
        return false;
    }
    
    const char* paramValues[] = {
        wormhole.map1_name.c_str(),
        wormhole.map2_name.c_str(),
        std::to_string(wormhole.map1_x).c_str(),
        std::to_string(wormhole.map1_y).c_str(),
        std::to_string(wormhole.map1_oz).c_str(),
        std::to_string(wormhole.map1_ow).c_str(),
        std::to_string(wormhole.map2_x).c_str(),
        std::to_string(wormhole.map2_y).c_str(),
        std::to_string(wormhole.map2_oz).c_str(),
        std::to_string(wormhole.map2_ow).c_str()
    };

    PGresult* res = PQexecParams(connection_,
        "INSERT INTO wormholes (map1_name, map2_name, map1_position_x, map1_position_y, map1_orientation_z, map1_orientation_w,"
        "map2_position_x, map2_position_y, map2_orientation_z, map2_orientation_w) "
        "VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)",
        10, nullptr, paramValues, nullptr, nullptr, 0);

    if (PQresultStatus(res) != PGRES_COMMAND_OK) {
        std::cerr << "Failed to insert wormhole: " << PQerrorMessage(connection_) << std::endl;
        PQclear(res);
        return false;
    }
    PQclear(res);
    return true;
}

#include "calibration.h"

#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

YAML::Node loadYamlLenient(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open YAML file: " + path);

    // Kalibr writes "%YAML:1.0", which is not a valid YAML directive
    // ("%YAML 1.0" would be). Drop directive lines before parsing.
    std::stringstream ss;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line[0] == '%') continue;
        ss << line << '\n';
    }
    return YAML::Load(ss.str());
}

Eigen::Matrix4d parseMatrix4d(const YAML::Node& node, const std::string& key) {
    if (!node[key])
        throw std::runtime_error("Missing transform '" + key + "' in calibration file");

    const YAML::Node& m = node[key];
    if (!m.IsSequence() || m.size() != 4)
        throw std::runtime_error("Transform '" + key + "' must be 4 rows, got " +
                                 std::to_string(m.size()));

    Eigen::Matrix4d T;
    for (int r = 0; r < 4; ++r) {
        if (!m[r].IsSequence() || m[r].size() != 4)
            throw std::runtime_error("Transform '" + key + "' row " + std::to_string(r) +
                                     " must have 4 entries");
        for (int c = 0; c < 4; ++c)
            T(r, c) = m[r][c].as<double>();
    }
    return T;
}

std::string expandRosPath(const std::string& path) {
    const std::string prefix = "$(find ";
    size_t start = path.find(prefix);
    if (start == std::string::npos) return path;

    size_t close = path.find(')', start);
    if (close == std::string::npos) return path;

    std::string pkg = path.substr(start + prefix.size(),
                                  close - start - prefix.size());
    std::string resolved = ros::package::getPath(pkg);
    if (resolved.empty())
        throw std::runtime_error("Cannot resolve ROS package '" + pkg + "' in path: " + path);

    std::string out = path;
    out.replace(start, close - start + 1, resolved);
    return out;
}

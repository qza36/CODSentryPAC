#include "trajectory_generation/MapLoader.hpp"
#include <iostream>

cv::Mat MapLoader::loadOccMap(const std::string& path) {
    cv::Mat occ_map = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (occ_map.empty()) {
        std::cerr << "[MapLoader] Failed to load occ_map: " << path << std::endl;
    }
    return occ_map;
}

cv::Mat MapLoader::loadBevMap(const std::string& path) {
    cv::Mat bev_map_color = cv::imread(path, cv::IMREAD_COLOR);
    if (bev_map_color.empty()) {
        std::cerr << "[MapLoader] Failed to load bev_map: " << path << std::endl;
        return cv::Mat();
    }
    std::vector<cv::Mat> channels;
    cv::split(bev_map_color, channels);
    return channels.at(0);
}

cv::Mat MapLoader::loadTopoMap(const std::string& path) {
    cv::Mat topo_map = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (topo_map.empty()) {
        std::cerr << "[MapLoader] Failed to load topo_map: " << path << std::endl;
    }
    return topo_map;
}

cv::Mat MapLoader::swellOccMap(const cv::Mat& occ_map, double radius, double resolution) {
    int swell_num = static_cast<int>(radius / resolution);
    if (swell_num <= 0) {
        return occ_map.clone();
    }

    cv::Mat mask;
    cv::threshold(occ_map, mask, 10, 255, cv::THRESH_BINARY);

    int k_size = 2 * swell_num + 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k_size, k_size));

    cv::Mat occ;
    cv::dilate(mask, occ, element);

    std::cout << "[MapLoader] Map swell done, kernel size: " << k_size << std::endl;
    return occ;
}

map_utils::MapData MapLoader::matToMapData(const cv::Mat& mat) {
    map_utils::MapData data;
    if (mat.empty()) return data;
    data.width = mat.cols;
    data.height = mat.rows;
    data.pixels.assign(mat.data, mat.data + mat.total());
    return data;
}

map_utils::MapData MapLoader::loadOccMapData(const std::string& path) {
    return matToMapData(loadOccMap(path));
}

map_utils::MapData MapLoader::loadTopoMapData(const std::string& path) {
    return matToMapData(loadTopoMap(path));
}

map_utils::MapData MapLoader::loadBevMapData(const std::string& path) {
    return matToMapData(loadBevMap(path));
}

map_utils::MapData MapLoader::loadAndSwellOccMapData(const std::string& path, double radius, double resolution) {
    cv::Mat occ = loadOccMap(path);
    cv::Mat swelled = swellOccMap(occ, radius, resolution);
    return matToMapData(swelled);
}

#ifndef TRAJECTORY_GENERATION_MAPLOADER_H
#define TRAJECTORY_GENERATION_MAPLOADER_H

#include "opencv2/opencv.hpp"
#include "trajectory_generation/MapData.hpp"
#include <string>

/**
 * @brief 静态图像加载与预处理工具类
 * 职责：纯粹的图像I/O与预处理（膨胀、通道分离）
 * 不持有任何状态，所有方法均为静态
 */
class MapLoader {
public:
    // 返回 MapData 的接口（GridMap 使用）
    static map_utils::MapData loadOccMapData(const std::string& path);
    static map_utils::MapData loadTopoMapData(const std::string& path);
    static map_utils::MapData loadBevMapData(const std::string& path);
    static map_utils::MapData loadAndSwellOccMapData(const std::string& path, double radius, double resolution);

    // 原有 cv::Mat 接口（内部使用）
    static cv::Mat loadOccMap(const std::string& path);
    static cv::Mat loadBevMap(const std::string& path);
    static cv::Mat loadTopoMap(const std::string& path);
    static cv::Mat swellOccMap(const cv::Mat& occ_map, double radius, double resolution);

private:
    MapLoader() = delete;
    static map_utils::MapData matToMapData(const cv::Mat& mat);
};

#endif //TRAJECTORY_GENERATION_MAPLOADER_H

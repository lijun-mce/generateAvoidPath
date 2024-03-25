/**
 * @brief 基于bug2原理的路径生成算法
 * @author lijun 20240319
 * 包含两个功能  ：第一个为碰撞检测 、 第二个为轨迹生成
 */
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <vector>
using namespace std;

class GeneratePath {
public:
    GeneratePath(){};
    ~GeneratePath(){};

    /**
     * @brief 主函数，包含两个功能：第一个为碰撞检测 、 第二个为轨迹生成
     * @param map 传入地图数据
     * @param path 出入的正在跟踪的路径
     * @param line_start 初始跟踪线的起点
     * @param line_end 初始跟踪线的终点
     * @param robot_pose 飞机的坐标点
     * @return vector<cv::Point>  输出绕障路径
     */
    vector<cv::Point> generateNewPath(const cv::Mat &map, const vector<cv::Point> &path, cv::Point line_start, cv::Point line_end,
                                      cv::Point robot_pose = cv::Point(-1, -1));

private:
    // 裁剪路径
    vector<cv::Point> prunePath(cv::Point robot_pose, const vector<cv::Point> &path);
    // 寻找边界路径
    bool findEdgePath(cv::Point robot_pose, const cv::Mat &map, cv::Point start, cv::Point end);
    // 计算最近点位
    int findMinDistIndex(cv::Point target_point, const vector<cv::Point> &contour);
    // 碰撞检测
    bool linePassable(const cv::Mat &map, const cv::Point &pt1, const cv::Point &pt2);
    // 点在直线的哪边
    int pointRelativeToVector(const cv::Point &point, const cv::Point &start, const cv::Point &end);

private:
    std::vector<cv::Point> new_path_;
    std::vector<cv::Point> old_path_;
};
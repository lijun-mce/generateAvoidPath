
/**
 * @brief 基于bug2原理的路径生成算法
 * @author lijun 20240319
 * 包含两个功能  ：第一个为碰撞检测 、 第二个为轨迹生成
 */
#include "../include/generate_path.hpp"
using namespace std;

// 主函数
vector<cv::Point> GeneratePath::generateNewPath(const cv::Mat &map, const vector<cv::Point> &path, cv::Point line_start, cv::Point line_end,
                                                cv::Point robot_pose) {
    if (map.empty()) {
        std::cout << "map is empty, error !" << endl;
        return path;
    }
    if (path.size() < 2) {
        std::cout << "path size < 2 , error !" << endl;
        return path;
    }

    // 路径裁剪
    old_path_ = prunePath(robot_pose, path);

    for (int i = 0; i < old_path_.size() - 1; ++i) {
        bool sign = linePassable(map, old_path_[i], old_path_[i + 1]);
        if (!sign) {
            cout << "遇到障碍物，需要重新生成轨迹 !" << endl;
            findEdgePath(robot_pose, map, line_start, line_end);
            return new_path_;
        }
    }
    return old_path_;
}

// 裁剪路径
vector<cv::Point> GeneratePath::prunePath(cv::Point robot_pose, const vector<cv::Point> &path) {
    vector<cv::Point> old_path = path;
    float min_dist = 1e6;
    int min_index = 0;
    float x = robot_pose.x;
    float y = robot_pose.y;
    for (size_t i = 0; i < old_path.size(); i++) {
        float dist = (old_path[i].x - x) * (old_path[i].x - x) + (old_path[i].y - y) * (old_path[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    old_path.erase(old_path.begin(), old_path.begin() + min_index);
    return old_path;
}

// 寻找边界路径
bool GeneratePath::findEdgePath(cv::Point robot_pose, const cv::Mat &map, cv::Point start, cv::Point end) {
    cv::Mat temp_map = map.clone();
    cv::Mat temp_map2(map.rows, map.cols, CV_8UC1, cv::Scalar(0));

    // 在地图上画一条直线，直线颜色为白色（255）
    cv::line(temp_map, start, end, cv::Scalar(255), 1, cv::LINE_AA);

    // 遍历图像中的每个像素
    for (int y = 0; y < map.rows; ++y) {
        for (int x = 0; x < map.cols; ++x) {
            // 获取像素的坐标
            cv::Point point(x, y);

            // 右绕障，左边为可行区域
            cv::Point closestPoint;
            double dist = pointRelativeToVector(point, start, end);
            if (dist < 0) {
                temp_map.at<uchar>(y, x) = 255;
            } else {
                temp_map.at<uchar>(y, x) = 0;
            }
        }
    }
    temp_map.setTo(0, map == 0);
    cv::imwrite("../data/process_map-1.png", temp_map);

    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(temp_map, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cout << "contour.size():" << all_contours.size() << endl;

    // 寻找最大的内轮廓 ？ TODO:需要验证
    int max_index = 0, max_size = 0;
    for (int i = 0; i < all_contours.size(); ++i) {
        if (all_contours[i].size() > max_size) {
            max_index = i;
            max_size = all_contours[i].size();
        }
    }

    // 轮廓顺序默认为逆时针，需要注意
    int start_index;
    if (robot_pose.x == -1) {
        start_index = findMinDistIndex(start, all_contours[max_index]);
    } else {
        start_index = findMinDistIndex(robot_pose, all_contours[max_index]);
    }
    int end_index = findMinDistIndex(end, all_contours[max_index]);
    cout << "start_index:" << start_index << ", end_index:" << end_index << endl;
    std::vector<cv::Point> new_path;
    if (end_index < start_index) {
        new_path.insert(new_path.end(), all_contours[max_index].begin() + start_index, all_contours[max_index].end());
        new_path.insert(new_path.end(), all_contours[max_index].begin(), all_contours[max_index].begin() + end_index);
    } else {
        new_path.insert(new_path.end(), all_contours[max_index].begin() + start_index, all_contours[max_index].begin() + end_index);
    }
    for (size_t i = 0; i < new_path.size(); ++i) {
        if (i + 1 < new_path.size()) cv::line(temp_map2, new_path[i], new_path[i + 1], cv::Scalar(255), 2);
    }
    new_path_ = new_path;
    cv::imwrite("../data/process_map-2.png", temp_map2);
    return true;
}

// 判断点在直线的左侧或者右侧
int GeneratePath::pointRelativeToVector(const cv::Point &point, const cv::Point &start, const cv::Point &end) {
    // 计算叉积
    double cross = (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x);
    // 根据叉积的符号判断点的位置
    if (cross < 0) {
        return -1; // 左侧
    } else if (cross > 0) {
        return 1; // 右侧
    } else {
        return 0; // 共线
    }
}

// 寻找最近的点
int GeneratePath::findMinDistIndex(cv::Point target_point, const vector<cv::Point> &contour) {
    float min_dist = 1e6;
    int min_index = 0;
    float x = target_point.x;
    float y = target_point.y;
    for (size_t i = 0; i < contour.size(); i++) {
        float dist = (contour[i].x - x) * (contour[i].x - x) + (contour[i].y - y) * (contour[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    return min_index;
}

// 碰撞检测
bool GeneratePath::linePassable(const cv::Mat &map, const cv::Point &pt1, const cv::Point &pt2) {
    cv::LineIterator it(map, pt1, pt2);
    for (int i = 0; i < it.count; i++, ++it) {
        int val = map.at<uchar>(it.pos());
        if (val < 1) {
            return false;
        }
    }
    return true;
}
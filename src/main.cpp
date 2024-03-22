
/**
 * @brief 用于基于bug2的路径生成算法
 * @author lijun 20240319
 * 包含两个功能  ：第一个为碰撞检测 、 第二个为轨迹生成
 */
#include <iostream>
#include <string>

#include "../include/generate_path.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;
using namespace cv;

#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat test_map(400, 400, CV_8UC1, cv::Scalar(255));
    cv::rectangle(test_map, cv::Point(150, 100), cv::Point(200, 250), cv::Scalar(0), -1);
    cv::circle(test_map, cv::Point(250, 250), 60, cv::Scalar(0), -1);
    cv::imwrite("../data/test_map.png", test_map);

    cv::Mat example = test_map.clone();
    cv::Point pt_start(350, 350);
    cv::Point pt_end(50, 50);

    cv::line(example, pt_start, pt_end, cv::Scalar(200), 2);
    cv::imwrite("../data/example_map.png", example);

    // 起始点路径
    std::vector<cv::Point> path;
    path.push_back(pt_start);
    path.push_back(pt_end);
    // 机器人位置
    cv::Point robot_pose(320, 320);
    GeneratePath generate_path_test;
    std::vector<cv::Point> new_path = generate_path_test.generateNewPath(test_map, path, pt_start, pt_end, robot_pose);

    cv::cvtColor(example, example, CV_GRAY2BGR);
    for (size_t i = 0; i < new_path.size(); ++i) {
        if (i + 1 < new_path.size()) cv::line(example, new_path[i], new_path[i + 1], cv::Scalar(0, 0, 200), 2);
    }
    cv::imwrite("../data/new_path.png", example);
    return 0;
}
#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "lastar.hpp"
#include "minisnap.hpp"


void path_service();
class grid_map{
    public:
    grid_map();
    //加工原始图像
    void process_raw_image(const std::string & filename = "obstacle.png");
    //将原始图像转换为栅格地图
    void raw2grid(cv::Mat raw_image);
    //可视化栅格地图
    void visualize_grid_map(int cols,int rows);
    //创建基于人工势场的代价地图
    void create_potential_cost_map();
    //交互界面
    void interactive_interface();
    //重置交互操作
    void reset_image();
    //处理鼠标事件
    void handle_mouse_event(int event,int x,int y);

    private:
    lastar_planner lp;//A星算法
    int block_size = 1;//栅格尺寸
    int grid_width;//栅格地图的宽
    int grid_height;//栅格地图的高
    int potential_cost_influential_range;//人工势场代价影响的考虑范围
    Eigen::MatrixXi real_grid_map;//栅格地图
    Eigen::MatrixXi potential_cost_map;//基于人工势场的代价地图
    cv::Mat visualized_map;//可视化处理后的地图
    cv::Mat final_image;//最终显示给用户的画面
    bool start_exist = false;//起点是否存在
    bool goal_exist = false;//目标点是否存在
    std::vector<std::pair<int,int>> path;//路径
    double white_ratio = 0.1;//栅格的白色处理权重
    int report_count = 0;//与DMP的发布有关
    int start_x = -1;//起点x坐标
    int start_y = -1;//起点y坐标
    int goal_x = -1;//目标点x坐标
    int goal_y = -1;//目标点y坐标
    float mapping = 0.05f;
    std::vector<std::pair<float,float>> real_path;
    minisnap ms;
    std::vector<std::pair<float,float>> final_path;
    int prolong = 2;
};

#endif
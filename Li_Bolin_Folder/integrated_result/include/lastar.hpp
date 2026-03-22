#ifndef LASTAR_HPP
#define LASTAR_HPP

enum DebugLevel{
    DEBUG_NONE = 0,
    DEBUG_ERROR = 1,
    DEBUG_WARN = 2,
    DEBUG_INFO = 3,
    DEBUG_VERBOSE = 4
};

#define CURRENT_DEBUG_LEVEL DEBUG_INFO

#include <unordered_map>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <utility>
#include <unordered_set>
#include <queue>
#include <functional>
#include <string>
#include <cmath>

class lastar_planner{
    public:
    //debug框架
    static void log_error(const std::string & msg){
        #if CURRENT_DEBUG_LEVEL >= DEBUG_ERROR
        std::cout << "[ERROR]" << msg << std::endl;
        #endif
    }
    static void log_warn(const std::string & msg){
        #if CURRENT_DEBUG_LEVEL >= DEBUG_WARN
        std::cout << "[WARN]" << msg << std::endl;
        #endif
    }
    static void log_info(const std::string & msg){
        #if CURRENT_DEBUG_LEVEL >= DEBUG_INFO
        std::cout << "[INFO]" << msg << std::endl;
        #endif
    }
    static void log_verbose(const std::string & msg){
        #if CURRENT_DEBUG_LEVEL >= DEBUG_VERBOSE
        std::cout << "[VERBOSE]" << msg << std::endl;
        #endif
    }
    lastar_planner();
    //计算索引
    int get_index(int x,int y);
    //计算h(n)
    double calculate_h(int from_x,int from_y,int to_x,int to_y);
    //寻找路径
    std::vector<std::pair<int,int>> find_path(int start_x,int start_y,int goal_x,int goal_y);
    //寻找节点的有效邻居
    std::vector<std::pair<int,int>> find_neighbors(int x,int y);
    //接收地图信息
    void get_map_infomation(int grid_width,int grid_height,Eigen::MatrixXi received_cost_map,float received_mapping);
    //计算移动代价
    double get_move_cost(int from_x,int from_y,int to_x,int to_y);
    //回溯路径
    std::vector<std::pair<int,int>> reconstruct_path(const std::unordered_map<int,std::pair<int,int>> & came_from,int goal_x,int goal_y);
    //简化路径点
    std::vector<std::pair<float,float>> douglas_peucker(std::vector<std::pair<float,float>> path,int start_idx,int end_idx);
    //计算点到线的距离
    float distance_from_line(float a_x,float a_y,float b_x,float b_y,float c_x,float c_y);

    bool line_in_obstacle(std::pair<float,float> st,std::pair<float,float> end);
    //节点结构体
    struct node{
        int x,y;
        double g,h,f;
        node(int x,int y,double g,double h):x(x),y(y),g(g),h(h),f(g+h){}
        bool operator >(const node & other)const{
            return f>other.f;
        }
    };

    private:
    int width;//地图的宽
    int height;//地图的高
    Eigen::MatrixXi cost_map;//代价地图
    float threshold = 0.1f;
    float mapping;
};

#endif
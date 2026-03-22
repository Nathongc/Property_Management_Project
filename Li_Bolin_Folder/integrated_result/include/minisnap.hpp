#ifndef MINISNAP_HPP
#define MINISNAP_HPP

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

class minisnap{
    public:
    minisnap();
    //接收基础信息
    void get_information(Eigen::MatrixXi received_map,int received_width,int received_height,float received_mapping);
    //接收原始路径
    void get_path(std::vector<std::pair<float,float>> received_path);
    //阶乘
    float factorial(int a);
    //下降阶乘
    float falling_factorial(int a,int n);
    //分配时间
    void allocate_time();
    //幂
    float power(float a,int n);
    //主加工函数
    std::vector<std::pair<float,float>> main_process();
    //构建Q
    void construct_Q();
    //初始化x
    void construct_x();
    //构建A
    void construct_A();
    //导数矩阵
    float derivative_matrix(int i,int j,float t);
    //构建low和up
    void construct_limits();
    //求解qp问题
    void solve_QP();
    //生成路径
    std::vector<std::pair<float,float>> generate_trajectory();
    //寻找碰撞路段
    std::vector<int> find_collision_segments();

    private:
    std::vector<std::pair<float,float>> path;
    float max_vec = 5.0f;
    float max_accel = 2.0f;
    int derivative = 4;
    std::vector<float> times;
    int s_count;
    Eigen::MatrixXf x_Q;
    Eigen::MatrixXf y_Q;
    Eigen::VectorXf x_x;
    Eigen::VectorXf y_x;
    Eigen::MatrixXf x_A;
    Eigen::MatrixXf y_A;
    Eigen::VectorXf x_limit;
    Eigen::VectorXf y_limit;
    Eigen::VectorXf x_solution;
    Eigen::VectorXf y_solution;
    int points_per_segment = 30;
    float mapping;
    float width;
    float height;
    Eigen::MatrixXi map;
    int max_iterations = 5;
};

#endif
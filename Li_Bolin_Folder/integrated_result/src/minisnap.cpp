#include "minisnap.hpp"

minisnap::minisnap(){};

void minisnap::get_information(Eigen::MatrixXi received_map,int received_width,int received_height,float received_mapping){
    map = received_map;
    width = received_width;
    height = received_height;
    mapping = received_mapping;
}

void minisnap::get_path(std::vector<std::pair<float,float>> received_path){
    path = received_path;
}

float minisnap::factorial(int a){
    if(a==0){
        return 1.0f;
    }
    return falling_factorial(a,a);
}

float minisnap::falling_factorial(int a,int n){
    float result = 1.0f;
    float a_ = static_cast<float>(a);
    for(int i = 0;i<n;i++){
        result = result * static_cast<float>(a_);
        a_ = a_ - 1.0f;
    }
    return result;
}

float minisnap::power(float a,int n){
    if(a==0.0f){
        return 0.0f;
    }else if(n==0){
        return 1.0f;
    }else{
        float result = 1.0f;
        for(int i = 0;i<n;i++){
            result = result * a;
        }
        return result;
    }
}

void minisnap::allocate_time(){
    times.clear();
    float accel_t = max_vec / max_accel;
    float accel_s = max_accel * accel_t * accel_t / 2.0f;
    //分配时间
    for(int i = 0;i<path.size()-1;i++){
        float temp_time;
        float dx = path[i+1].first - path[i].first;
        float dy = path[i+1].second - path[i].second;
        float s = sqrtf(dx*dx+dy*dy);
        if(s < 2.0f * accel_s){
            temp_time = sqrtf(4.0f * s / max_accel);
        }else{
            temp_time = (s-2.0f*accel_s)/max_vec + 2.0f * accel_t;
        }
        times.push_back(temp_time);
    }
}

void minisnap::construct_Q(){
    //构建x轴的Q
    x_Q = Eigen::MatrixXf::Zero(2*derivative*s_count,2*derivative*s_count);
    for(int s = 0;s<s_count;s++){
        Eigen::MatrixXf sub_Q= Eigen::MatrixXf::Zero(2*derivative,2*derivative);
        for(int i = derivative;i < 2 * derivative;i++){
            for(int j = derivative;j<2*derivative;j++){
                sub_Q(i,j) = falling_factorial(i,derivative) * falling_factorial(j,derivative) * power(times[s],i+j-2*derivative+1)/static_cast<float>(i+j-2*derivative+1);
            }
        }
        x_Q.block(2*derivative*s,2*derivative*s,2*derivative,2*derivative) = sub_Q;
    }
    //构建y轴的Q
    y_Q = Eigen::MatrixXf::Zero(2*derivative*s_count,2*derivative*s_count);
    for(int s = 0;s<s_count;s++){
        Eigen::MatrixXf sub_Q= Eigen::MatrixXf::Zero(2*derivative,2*derivative);
        for(int i = derivative;i < 2 * derivative;i++){
            for(int j = derivative;j<2*derivative;j++){
                sub_Q(i,j) = falling_factorial(i,derivative) * falling_factorial(j,derivative) * power(times[s],i+j-2*derivative+1)/static_cast<float>(i+j-2*derivative+1);
            }
        }
        y_Q.block(2*derivative*s,2*derivative*s,2*derivative,2*derivative) = sub_Q;
    }
}

void minisnap::construct_x(){
    x_x = Eigen::VectorXf();
    y_x = Eigen::VectorXf();
    //构建x轴的x向量
    std::vector<float> temp_x;
    for(int i = 0;i<s_count;i++){
        float w2;
        if(times[i]==0.0f){
            w2 = 0.0f;
        }else{
            w2 = (path[i+1].first-path[i].first)/times[i];
        }
        temp_x.push_back(path[i].first);
        temp_x.push_back(w2);
        for(int j = 2;j<2*derivative;j++){
            temp_x.push_back(0.0f);
        }
    }
    x_x = Eigen::Map<Eigen::VectorXf>(temp_x.data(),temp_x.size());
    //构建y轴的x向量
    std::vector<float> temp_y;
    for(int i = 0;i<s_count;i++){
        float w2;
        if(times[i]==0.0f){
            w2 = 0.0f;
        }else{
            w2 = (path[i+1].second-path[i].second)/times[i];
        }
        temp_y.push_back(path[i].second);
        temp_y.push_back(w2);
        for(int j = 2;j<2*derivative;j++){
            temp_y.push_back(0.0f);
        }
    }
    y_x = Eigen::Map<Eigen::VectorXf>(temp_y.data(),temp_y.size());
}

void minisnap::construct_A(){
    x_A = Eigen::MatrixXf::Zero(s_count+1+derivative*(s_count-1),2*derivative*s_count);
    x_A(0,0) = 1.0f;
    y_A = Eigen::MatrixXf::Zero(s_count+1+derivative*(s_count-1),2*derivative*s_count);
    y_A(0,0) = 1.0f;
    //x轴的A的点约束
    for(int i = 1;i<s_count+1;i++){
        for(int j = 0;j<2*derivative;j++){
            if(j==0){
                x_A(i,(i-1)*2*derivative+j) = 1.0f;
            }else{
                x_A(i,(i-1)*2*derivative+j) = power(times[i-1],j);
            }
        }
    }
    //x轴的A的连续性约束
    for(int s = 0;s<s_count-1;s++){
        Eigen::MatrixXf left = Eigen::MatrixXf::Zero(derivative,2*derivative);
        Eigen::MatrixXf right = Eigen::MatrixXf::Zero(derivative,2*derivative);
        for(int i = 0;i<derivative;i++){
            for(int j = 0;j<2*derivative;j++){
                right(i,i) = -1.0f * factorial(i);
                left(i,j) = derivative_matrix(i,j,times[s]);
            }
        }
        x_A.block(s_count+1+s*derivative,s*2*derivative,derivative,2*derivative) = left;
        x_A.block(s_count+1+s*derivative,(s+1)*2*derivative,derivative,2*derivative) = right;
    }
    //y轴的A的点约束
    for(int i = 1;i<s_count+1;i++){
        for(int j = 0;j<2*derivative;j++){
            if(j==0){
                y_A(i,(i-1)*2*derivative+j) = 1.0f;
            }else{
                y_A(i,(i-1)*2*derivative+j) = power(times[i-1],j);
            }
        }
    }
    //y轴的A的连续性约束
    for(int s = 0;s<s_count-1;s++){
        Eigen::MatrixXf left = Eigen::MatrixXf::Zero(derivative,2*derivative);
        Eigen::MatrixXf right = Eigen::MatrixXf::Zero(derivative,2*derivative);
        for(int i = 0;i<derivative;i++){
            for(int j = 0;j<2*derivative;j++){
                right(i,i) = -1.0f * factorial(i);
                left(i,j) = derivative_matrix(i,j,times[s]);
            }
        }
        y_A.block(s_count+1+s*derivative,s*2*derivative,derivative,2*derivative) = left;
        y_A.block(s_count+1+s*derivative,(s+1)*2*derivative,derivative,2*derivative) = right;
    }
}

float minisnap::derivative_matrix(int i,int j,float t){
    if(i<j){
        if(t==0.0f){
            return 0.0f;
        }else{
            return (factorial(j)/(factorial(j-i)))*power(t,j-i);
        }
    }else if(i==j){
        return factorial(i);
    }else{
        return 0.0f;
    }
}

void minisnap::construct_limits(){
    x_limit = Eigen::VectorXf();
    y_limit = Eigen::VectorXf();
    std::vector<float> x_temp;
    std::vector<float> y_temp;
    for(int i = 0;i<s_count+1+derivative*(s_count-1);i++){
        if(i<s_count+1){
            x_temp.push_back(path[i].first);
            y_temp.push_back(path[i].second);
        }else{
            x_temp.push_back(0.0f);
            y_temp.push_back(0.0f);
        }
    }
    x_limit = Eigen::Map<Eigen::VectorXf>(x_temp.data(),x_temp.size());
    y_limit = Eigen::Map<Eigen::VectorXf>(y_temp.data(),y_temp.size());
}

void minisnap::solve_QP(){
    x_solution = Eigen::VectorXf();
    y_solution = Eigen::VectorXf();
    OsqpEigen::Solver x_solver;
    OsqpEigen::Solver y_solver;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2*derivative*s_count);
    //求解x
    Eigen::MatrixXd x_Q_double = x_Q.cast<double>();
    Eigen::MatrixXd x_A_double = x_A.cast<double>();
    Eigen::VectorXd x_x_double = x_x.cast<double>();
    Eigen::VectorXd x_limit_double = x_limit.cast<double>();
    Eigen::SparseMatrix<double> x_Q_sparse = x_Q_double.sparseView();
    Eigen::SparseMatrix<double> x_A_sparse = x_A_double.sparseView();
    x_solver.settings()->setVerbosity(false);
    x_solver.settings()->setWarmStart(true);
    x_solver.settings()->setAbsoluteTolerance(1e-2);
    x_solver.settings()->setRelativeTolerance(2e-3);
    x_solver.settings()->setRho(1e-1);
    x_solver.settings()->setPolish(true);
    x_solver.settings()->setMaxIteration(4000);
    x_solver.data()->setNumberOfVariables(2*derivative*s_count);
    x_solver.data()->setNumberOfConstraints(s_count+1+derivative*(s_count-1));
    x_solver.data()->setHessianMatrix(x_Q_sparse);
    x_solver.data()->setGradient(q);
    x_solver.data()->setLinearConstraintsMatrix(x_A_sparse);
    x_solver.data()->setLowerBound(x_limit_double);
    x_solver.data()->setUpperBound(x_limit_double);
    x_solver.initSolver();
    x_solver.setPrimalVariable(x_x_double);
    x_solver.solveProblem();
    x_solution = x_solver.getSolution().cast<float>();
    x_solver.clearSolver();
    //求解y
    Eigen::MatrixXd y_Q_double = y_Q.cast<double>();
    Eigen::MatrixXd y_A_double = y_A.cast<double>();
    Eigen::VectorXd y_x_double = y_x.cast<double>();
    Eigen::VectorXd y_limit_double = y_limit.cast<double>();
    Eigen::SparseMatrix<double> y_Q_sparse = y_Q_double.sparseView();
    Eigen::SparseMatrix<double> y_A_sparse = y_A_double.sparseView();
    y_solver.settings()->setVerbosity(false);
    y_solver.settings()->setWarmStart(true);
    y_solver.settings()->setAbsoluteTolerance(1e-2);
    y_solver.settings()->setRelativeTolerance(2e-3);
    y_solver.settings()->setRho(1e-2);
    y_solver.settings()->setPolish(true);
    y_solver.settings()->setMaxIteration(4000);
    y_solver.data()->setNumberOfVariables(2*derivative*s_count);
    y_solver.data()->setNumberOfConstraints(s_count+1+derivative*(s_count-1));
    y_solver.data()->setHessianMatrix(y_Q_sparse);
    y_solver.data()->setGradient(q);
    y_solver.data()->setLinearConstraintsMatrix(y_A_sparse);
    y_solver.data()->setLowerBound(y_limit_double);
    y_solver.data()->setUpperBound(y_limit_double);
    y_solver.initSolver();
    y_solver.setPrimalVariable(y_x_double);
    y_solver.solveProblem();
    y_solution = y_solver.getSolution().cast<float>();
    y_solver.clearSolver();
}

std::vector<std::pair<float,float>> minisnap::generate_trajectory(){
    std::vector<std::pair<float,float>> trajectory;
    for(int seg = 0;seg<s_count;seg++){
        int start_idx = seg * 2 * derivative;
        Eigen::VectorXf x_coeffs = x_solution.segment(start_idx,2*derivative);
        Eigen::VectorXf y_coeffs = y_solution.segment(start_idx,2*derivative);
        float seg_time = times[seg];
        for(int i = 0;i<=points_per_segment;i++){
            float t = static_cast<float>(i) * seg_time / static_cast<float>(points_per_segment);
            float x_point = 0.0f;
            float y_point = 0.0f;
            for(int j = 0;j<2*derivative;j++){
                if(t==0.0f){
                    x_point = path[seg].first;
                    y_point = path[seg].second;
                    break;
                }else{
                    x_point = x_point + x_coeffs[j]*power(t,j);
                    y_point = y_point + y_coeffs[j]*power(t,j);
                }
            }
            trajectory.emplace_back(x_point,y_point);
        }
    }
    return trajectory;
}

std::vector<int> minisnap::find_collision_segments(){
    std::vector<int> collision_segments;
    int samples = 30;
    for(int seg = 0;seg<s_count;seg++){
        int start_idx = seg * 2 * derivative;
        Eigen::VectorXf x_coeffs = x_solution.segment(start_idx,2*derivative);
        Eigen::VectorXf y_coeffs = y_solution.segment(start_idx,2*derivative);
        float seg_time = times[seg];
        for(int i = 0;i<=samples;i++){
            float t = static_cast<float>(i) * seg_time / static_cast<float>(samples);
            float x_point = 0.0f;
            float y_point = 0.0f;
            for(int j = 0;j<2*derivative;j++){
                if(t==0.0f){
                    x_point = path[seg].first;
                    y_point = path[seg].second;
                    break;
                }else{
                    x_point = x_point + x_coeffs[j]*power(t,j);
                    y_point = y_point + y_coeffs[j]*power(t,j);
                }
            }
            int x = static_cast<int>(x_point/mapping);
            int y = static_cast<int>(y_point/mapping);
            if(x < 0 || y < 0 || x >= width || y >=height){
                collision_segments.push_back(seg);
                break;
            }else if(map(y,x)>1000){
                collision_segments.push_back(seg);
                break;
            }
        }
    }
    return collision_segments;
}

std::vector<std::pair<float,float>> minisnap::main_process(){
    if(path.size()<2){
        return path;
    }
    s_count = static_cast<int>(path.size()-1);
    allocate_time();
    construct_Q();
    construct_x();
    construct_A();
    construct_limits();
    solve_QP();
    for(int i = 0;i<max_iterations;i++){
        std::vector<int> collision_segments = find_collision_segments();
        if(collision_segments.empty()){
            break;
        }
        for(const auto & cs : collision_segments){
            times[cs] = times[cs] * 0.92f;
        }
        construct_Q();
        construct_x();
        construct_A();
        solve_QP();
    }
    return generate_trajectory();
}
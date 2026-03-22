#include "lastar.hpp"

lastar_planner::lastar_planner(){}

std::vector<std::pair<int,int>> lastar_planner::find_path(int start_x,int start_y,int goal_x,int goal_y){
    std::unordered_set<int> closed;
    std::unordered_map<int,double> g_save;
    std::unordered_map<int,std::pair<int,int>> came_from;
    std::priority_queue<node,std::vector<node>,std::greater<node>> open_set;
    int start_idx = get_index(start_x,start_y);
    //初始化起始节点
    g_save[start_idx] = 0.0;
    double start_h = calculate_h(start_x,start_y,goal_x,goal_y);
    node start_node(start_x,start_y,0.0,start_h);
    open_set.push(start_node);
    came_from[start_idx] = {-1,-1};
    //进入A星算法循环
    while(!open_set.empty()){
        node current = open_set.top();
        open_set.pop();
        int current_idx = get_index(current.x,current.y);
        //不再处理已搜索的节点
        if(closed.find(current_idx)!=closed.end()){
            continue;
        }
        closed.insert(current_idx);
        if(current.x == goal_x && current.y == goal_y){
            return reconstruct_path(came_from,goal_x,goal_y);
        }
        //寻找邻居
        for(const auto & neighbor : find_neighbors(current.x,current.y)){
            int nx = neighbor.first;
            int ny = neighbor.second;
            int n_idx = get_index(nx,ny);
            if(closed.find(n_idx)!=closed.end()){
                continue;
            }
            double g = current.g + get_move_cost(current.x,current.y,nx,ny);
            if(g_save.find(n_idx) == g_save.end() || g < g_save[n_idx]){
                g_save[n_idx] = g;
                came_from[n_idx] = {current.x,current.y};
                node n_node(nx,ny,g,calculate_h(nx,ny,goal_x,goal_y));
                open_set.push(n_node);
            }
        }
    }
    return {};
}

std::vector<std::pair<int,int>> lastar_planner::find_neighbors(int x, int y){
    std::vector<std::pair<int,int>> directions = {{0,1},{0,-1},{1,0},{-1,0},{1,1},{-1,1},{-1,-1},{1,-1}};
    std::vector<std::pair<int,int>> neighbors;
    for(const auto & dir : directions){
        int new_x = x + dir.first;
        int new_y = y + dir.second;
        if(new_x>=0&&new_y>=0&&new_x<width&&new_y<height){
            neighbors.push_back({new_x,new_y});
        }
    }
    return neighbors;
}

void lastar_planner::get_map_infomation(int grid_width,int grid_height,Eigen::MatrixXi received_cost_map,float received_mapping){
    width = grid_width;
    height = grid_height;
    cost_map = received_cost_map;
    mapping = received_mapping;
}

double lastar_planner::get_move_cost(int from_x,int from_y,int to_x,int to_y){
    double move_const;
    if(std::hypotf(from_x-to_x,from_y-to_y)>1){
        move_const = 1.414;
    }else{
        move_const = 1.0;
    }
    return static_cast<double>(cost_map(to_y,to_x)) * move_const;
}

int lastar_planner::get_index(int x,int y){
    return y * width + x;
}

double lastar_planner::calculate_h(int from_x,int from_y,int to_x,int to_y){
    double dx = static_cast<double>(abs(from_x-to_x));
    double dy = static_cast<double>(abs(from_y-to_y));
    double big,small;
    if(dx>=dy){
        big = dx;
        small = dy;
    }else{
        big = dy;
        small = dx;
    }
    double h = small * 14.14 + (big-small)*10;
    return h;
}

std::vector<std::pair<int,int>> lastar_planner::reconstruct_path(const std::unordered_map<int,std::pair<int,int>> & came_from,int goal_x,int goal_y){
    std::vector<std::pair<int,int>> path;
    std::pair<int,int> current = {goal_x,goal_y};
    while(came_from.find(get_index(current.first,current.second))!=came_from.end()){
        path.push_back(current);
        current = came_from.at(get_index(current.first,current.second));
    }
    path.push_back(current);
    std::reverse(path.begin(),path.end());
    return path;
}

std::vector<std::pair<float,float>> lastar_planner::douglas_peucker(std::vector<std::pair<float,float>> path,int start_idx,int end_idx){
    //基线条件
    if(end_idx-start_idx<2){
        std::vector<std::pair<float,float>> result;
        result.push_back(path[start_idx]);
        if(start_idx!=end_idx){
            result.push_back(path[end_idx]);
        }
        return result;
    }
    std::vector<float> distance(end_idx-start_idx-1);
    //计算距离
    std::transform(path.begin()+start_idx+1,path.begin()+end_idx,distance.begin(),[&](auto & p){
        return distance_from_line(path[start_idx].first,path[start_idx].second,path[end_idx].first,path[end_idx].second,p.first,p.second);
    });
    int max_idx = static_cast<int>(std::max_element(distance.begin(),distance.end())-distance.begin()) + start_idx + 1;
    float max_dis = distance[max_idx-start_idx-1];
    if(max_dis>threshold || line_in_obstacle(path[start_idx],path[end_idx])==true){
        //两段递归
        auto path1 = douglas_peucker(path,start_idx,max_idx);
        auto path2 = douglas_peucker(path,max_idx,end_idx);
        path1.insert(path1.end(),path2.begin()+1,path2.end());
        return path1;
    }else{
        //只返回端点
        std::vector<std::pair<float,float>> processed_path;
        processed_path.push_back(path[start_idx]);
        processed_path.push_back(path[end_idx]);
        return processed_path;
    }
}

float lastar_planner::distance_from_line(float a_x,float a_y,float b_x,float b_y,float c_x,float c_y){
    float ab_x = b_x - a_x;
    float ab_y = b_y - a_y;
    float ac_x = c_x - a_x;
    float ac_y = c_y - a_y;
    float cross = fabsf(ac_x*ab_y-ac_y*ab_x);
    return cross/(sqrtf(ab_x*ab_x+ab_y*ab_y+0.00001f));
}

bool lastar_planner::line_in_obstacle(std::pair<float,float> st,std::pair<float,float> end){
    int st_x = static_cast<int>(st.first/mapping);
    int st_y = static_cast<int>(st.second/mapping);
    int end_x = static_cast<int>(end.first/mapping);
    int end_y = static_cast<int>(end.second/mapping);
    if(st_x<0 || st_y <0 || end_x <0 || end_y < 0 || st_x >= width || st_y >= height || end_x >= width || end_y >= height){
        return true;
    }
    int dx = abs(st_x-end_x);
    int dy = abs(st_y-end_y);
    int step_x,step_y;
    if(end_x-st_x>0){
        step_x = 1;
    }else{
        step_x = -1;
    }
    if(end_y-st_y>0){
        step_y = 1;
    }else{
        step_y = -1;
    }
    int error = dx - dy;
    while(true){
        if(cost_map(st_y,st_x)>1000){
            return true;
        }
        if(st_x==end_x&&st_y==end_y){
            return false;
        }
        int e = 2 * error;
        if(e>-dy){
            error = error - dy;
            st_x = st_x + step_x;
        }
        if(e<dx){
            error = error + dx;
            st_y = st_y + step_y;
        }
    }
}
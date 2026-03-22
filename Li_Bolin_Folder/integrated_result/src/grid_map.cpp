#include "grid_map.hpp"

grid_map::grid_map(){}

void grid_map::process_raw_image(const std::string & filename){
    cv::Mat gray_image = cv::imread(filename,cv::IMREAD_GRAYSCALE);
    int a = gray_image.cols;
    int b = gray_image.rows;
    grid_width = gray_image.cols / block_size;
    grid_height = gray_image.rows / block_size;
    raw2grid(gray_image);
    create_potential_cost_map();
    visualize_grid_map(a,b);
    interactive_interface();
    return;
}

void grid_map::raw2grid(cv::Mat raw_image){
    //将原始地图转为栅格地图
    real_grid_map = Eigen::MatrixXi::Constant(grid_height,grid_width,-1);
    for(int grid_x = 0;grid_x<grid_width;grid_x++){
        for(int grid_y = 0;grid_y<grid_height;grid_y++){
            int free_count = 0;
            int total_count = block_size * block_size;
            for(int x = 0;x<block_size;x++){
                for(int y = 0;y<block_size;y++){
                    int image_x = grid_x * block_size + x;
                    int image_y = grid_y * block_size + y;
                    uchar pixel = raw_image.at<uchar>(image_y,image_x);
                    if(pixel>200){
                        free_count = free_count + 1;
                    }
                }
            }
            if(static_cast<double>(free_count)/static_cast<double>(total_count)>white_ratio){
                real_grid_map(grid_y,grid_x) = 0;
            }else{
                real_grid_map(grid_y,grid_x) = 100;
            }
        }
    }
}

void grid_map::visualize_grid_map(int cols,int rows){
    //将栅格地图可视化
    visualized_map = cv::Mat(prolong*rows,prolong*cols,CV_8UC3,cv::Scalar(255,255,255));
    for(int gx = 0;gx<grid_width;gx++){
        for(int gy = 0;gy<grid_height;gy++){
            if(real_grid_map(gy,gx)==100){
                cv::rectangle(visualized_map,cv::Rect(gx*prolong*block_size,gy*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(0,0,0),cv::FILLED);
            }else if(real_grid_map(gy,gx)==0){
                cv::rectangle(visualized_map,cv::Rect(gx*prolong*block_size,gy*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(255,255,255),cv::FILLED);
            }
        }
    }
}

static void on_mouse(int event,int x,int y,int flags,void* userdata){
    grid_map* map = static_cast<grid_map*>(userdata);
    map->handle_mouse_event(event,x,y);
}

void grid_map::create_potential_cost_map(){
    potential_cost_map = Eigen::MatrixXi::Constant(grid_height,grid_width,10);
    //将障碍物处的代价设为极大
    for(int x = 0;x < grid_width;x++){
        for(int y = 0;y < grid_height;y++){
            if(real_grid_map(y,x)==100){
                potential_cost_map(y,x) = 9999;
            }
        }
    }
    //计算出适宜的人工势场的代价影响范围半径
    double temp = static_cast<double>(grid_width)/50;
    potential_cost_influential_range = std::round(temp);
    if(potential_cost_influential_range<1){
        potential_cost_influential_range = 1;
    }
    //计算代价
    for(int x = 0;x < grid_width;x++){
        for(int y = 0;y < grid_height;y++){
            //不考虑计算障碍物处的代价
            if(real_grid_map(y,x)==100){
                continue;
            }
            int max_cost = 0;
            for(int predict_x = x - potential_cost_influential_range;predict_x<=x+potential_cost_influential_range;predict_x++){
                for(int predict_y = y - potential_cost_influential_range;predict_y<=y + potential_cost_influential_range;predict_y++){
                    if(predict_x < 0 || predict_x >= grid_width || predict_y < 0 || predict_y >= grid_height){
                        continue;
                    }
                    //根据欧式距离计算人工势场代价
                    if(real_grid_map(predict_y,predict_x)==100){
                        int distance = std::round(std::hypotf(predict_x - x,predict_y - y));
                        if(distance==0){
                            continue;
                        }
                        if(std::round(potential_cost_influential_range/distance)>max_cost){
                            max_cost = std::round(potential_cost_influential_range/distance);
                        }
                    }
                }
            }
            potential_cost_map(y,x) = potential_cost_map(y,x) + max_cost;
        }
    }
}

void grid_map::interactive_interface(){
    cv::namedWindow("GUI display",cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("GUI display",on_mouse,this);
    final_image = visualized_map.clone();
    lp.get_map_infomation(grid_width,grid_height,potential_cost_map,mapping);
    ms.get_information(potential_cost_map,grid_width,grid_height,mapping);
    while(true){
        cv::imshow("GUI display",final_image);
        int key = cv::waitKey(1);
        //按空格键清除操作
        if(key==32){
            reset_image();
        }else if(key == 27){
            break;
        }
    }
    cv::destroyWindow("GUI display");
}

void grid_map::reset_image(){
    start_exist = false;
    goal_exist = false;
    start_x = -1;
    start_y = -1;
    goal_x = -1;
    goal_y = -1;
    report_count = 0;
    final_image = visualized_map.clone();
}

void grid_map::handle_mouse_event(int event,int x,int y){
    int click_grid_x = x/(block_size*prolong);
    int click_grid_y = y/(block_size*prolong);
    bool valid_click = false;
    if(real_grid_map(click_grid_y,click_grid_x)==0){
        valid_click = true;
    }
    if(valid_click==true){
        if(event == cv::EVENT_LBUTTONDOWN && start_exist == false){
            start_exist = true;
            start_x = click_grid_x;
            start_y = click_grid_y;
            cv::rectangle(final_image,cv::Rect(click_grid_x*prolong*block_size,click_grid_y*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(0,255,0),cv::FILLED);
        }else if(event == cv::EVENT_RBUTTONDOWN && goal_exist == false){
            goal_exist = true;
            goal_x = click_grid_x;
            goal_y = click_grid_y;
            cv::rectangle(final_image,cv::Rect(click_grid_x*prolong*block_size,click_grid_y*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(255,0,0),cv::FILLED);
        }
    }
    if(start_exist == true && goal_exist == true && report_count == 0){
        auto start_time = std::chrono::high_resolution_clock::now();
        path = lp.find_path(start_x,start_y,goal_x,goal_y);
        auto end_time = std::chrono::high_resolution_clock::now();
        //计算路径规划耗时
        double time_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
        time_duration = time_duration / 1000.0;
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3) << time_duration;
        if(report_count==0){
            lp.log_info("Time: " + stream.str() + "ms");
            report_count = report_count + 1;
        }
        //将路径转化为现实中的路径
        std::vector<std::pair<float,float>> tf_path;
        for(const auto & p : path){
            tf_path.emplace_back(static_cast<float>(p.first)*mapping,static_cast<float>(p.second)*mapping);
        }
        //进行Douglas_Peucker简化
        int end_idx = static_cast<int>(tf_path.size())-1;
        real_path = lp.douglas_peucker(tf_path,0,end_idx);
        real_path.erase(real_path.begin());
        //用minimum snap处理简化后的路径
        ms.get_path(real_path);
        final_path = ms.main_process();
        //绘制minimum snap处理后的轨迹
        for(const auto & p : final_path){
            cv::rectangle(final_image,cv::Rect(p.first*prolong*block_size/mapping,p.second*prolong*block_size/mapping,prolong*block_size,prolong*block_size),cv::Scalar(0,0,255),cv::FILLED);
        };
        cv::rectangle(final_image,cv::Rect(start_x*prolong*block_size,start_y*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(0,255,0),cv::FILLED);
        cv::rectangle(final_image,cv::Rect(goal_x*prolong*block_size,goal_y*prolong*block_size,prolong*block_size,prolong*block_size),cv::Scalar(255,0,0),cv::FILLED);
    }
}

void path_service(){
    grid_map grid;
    grid.process_raw_image("community_obstacle_map.png");
    return;
}
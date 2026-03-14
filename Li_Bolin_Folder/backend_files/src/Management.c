#include "Management.h"
#include <microhttpd.h>
#include <limits.h>  // 用于 PATH_MAX
#include <time.h>

Imfor* imfor = NULL;
Person* head = NULL;

// 辅助函数：从 URL 获取参数
char* get_param(struct MHD_Connection* connection, const char* key) {
    const char* param = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, key);
    return (char*)param;
}

// 辅助函数：发送 HTTP 响应
int send_response(struct MHD_Connection* connection, const char* content, int status_code) {
    struct MHD_Response* response = MHD_create_response_from_buffer(strlen(content), (void*)content, MHD_RESPMEM_PERSISTENT);
    MHD_add_response_header(response, "Content-Type", "text/plain; charset=utf-8");
    int ret = MHD_queue_response(connection, status_code, response);
    MHD_destroy_response(response);
    return ret;
}

// 辅助函数：查找人员（复用你的逻辑）
Person* find_person_by_name(const char* name) {
    Person* node = head;
    while (node != NULL) {
        if (strcmp(node->M_name, name) == 0) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

// 辅助函数：查找人员（按电话）
Person* find_person_by_phone(long long phone) {
    Person* node = head;
    while (node != NULL) {
        if (node->M_phone_num == phone) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

// API 接口：查询人员信息
int handle_query_user(struct MHD_Connection* connection) {
    char* name = get_param(connection, "name");
    char* phone_str = get_param(connection, "phone");
    
    Person* found_person = NULL;
    long long phone = 0;
    
    if (phone_str) {
        phone = atoll(phone_str);
        found_person = find_person_by_phone(phone);
    }
    
    if (name && !found_person) {
        found_person = find_person_by_name(name);
    }
    
    if (found_person) {
        // 构建返回格式：职位,姓名,年龄,性别,房间号,密码,停车位,工作区域,缴费状态,电话
        char response[2048];
        snprintf(response, sizeof(response), "%s,%s,%d,%s,%d,%lld,%d,%d,%s,%lld",
            found_person->Career,
            found_person->M_name,
            found_person->M_age,
            found_person->M_sex,
            found_person->M_area,
            found_person->password,
            found_person->parking_imfor,
            found_person->Area[0], // 工作区域取第一个，实际可能需要多个
            found_person->Count_charge > 0 ? "已缴费" : "未缴费", // 简化缴费状态
            found_person->M_phone_num
        );
        return send_response(connection, response, MHD_HTTP_OK);
    }
    
    return send_response(connection, "", MHD_HTTP_NOT_FOUND);
}

// API 接口：获取所有人员信息
int handle_get_all_users(struct MHD_Connection* connection) {
    if (head == NULL) {
        return send_response(connection, "", MHD_HTTP_OK);
    }
    
    char* response = malloc(100000); // 足够大的缓冲区
    response[0] = '\0';
    
    Person* node = head;
    while (node != NULL) {
        char line[2048];
        snprintf(line, sizeof(line), "%s,%s,%d,%s,%d,%lld,%d,%d,%s,%lld\n",
            node->Career,
            node->M_name,
            node->M_age,
            node->M_sex,
            node->M_area,
            node->password,
            node->parking_imfor,
            node->Area[0],
            node->Count_charge > 0 ? "已缴费" : "未缴费",
            node->M_phone_num
        );
        strcat(response, line);
        node = node->next;
    }
    
    // 移除最后的换行符
    int len = strlen(response);
    if (len > 0 && response[len-1] == '\n') {
        response[len-1] = '\0';
    }
    
    int ret = send_response(connection, response, MHD_HTTP_OK);
    free(response);
    return ret;
}

// API 接口：新增人员（POST 请求）
int handle_add_user(struct MHD_Connection* connection, const char* user_data) {
    // 解析 user_data: "业主,张三,30,男,101,123456,0,0,已缴费,13800138000"
    Person* person = (Person*)malloc(sizeof(Person));
    if (person == NULL) {
        return send_response(connection, "内存分配失败", MHD_HTTP_INTERNAL_SERVER_ERROR);
    }
    
    Init_Person(person);
    
    char data_copy[2048];
    strncpy(data_copy, user_data, sizeof(data_copy) - 1);
    data_copy[sizeof(data_copy) - 1] = '\0';
    
    char* tokens[10];
    char* token = strtok(data_copy, ",");
    int i = 0;
    while (token && i < 10) {
        tokens[i] = token;
        token = strtok(NULL, ",");
        i++;
    }
    
    if (i < 10) {
        free(person);
        return send_response(connection, "数据格式错误", MHD_HTTP_BAD_REQUEST);
    }
    
    strncpy(person->Career, tokens[0], MAX - 1);
    strncpy(person->M_name, tokens[1], MAX - 1);
    person->M_age = atoi(tokens[2]);
    strncpy(person->M_sex, tokens[3], MAX - 1);
    person->M_area = atoi(tokens[4]);
    person->password = atoll(tokens[5]);
    person->parking_imfor = atoi(tokens[6]);
    person->Area[0] = atoi(tokens[7]);
    // 简化处理缴费状态，实际可能需要更复杂的逻辑
    person->Count_charge = (strcmp(tokens[8], "已缴费") == 0) ? 1 : 0;
    person->M_phone_num = atoll(tokens[9]);
    
    head = ADD_TO_LIST(head, person);
    Save(head);
    
    return send_response(connection, "添加成功", MHD_HTTP_OK);
}

// API 接口：删除人员（POST 请求，接收完整行数据）
int handle_delete_user(struct MHD_Connection* connection, const char* user_data) {
    char data_copy[2048];
    strncpy(data_copy, user_data, sizeof(data_copy) - 1);
    data_copy[sizeof(data_copy) - 1] = '\0';
    
    char* tokens[10];
    char* token = strtok(data_copy, ",");
    int i = 0;
    while (token && i < 10) {
        tokens[i] = token;
        token = strtok(NULL, ",");
        i++;
    }
    
    if (i < 10) {
        return send_response(connection, "数据格式错误", MHD_HTTP_BAD_REQUEST);
    }
    
    // 按姓名查找并删除
    char* name = tokens[1];
    Person* current = head;
    Person* prev = NULL;
    
    while (current != NULL) {
        if (strcmp(current->M_name, name) == 0) {
            if (prev == NULL) {
                head = current->next;
            } else {
                prev->next = current->next;
            }
            free(current);
            Save(head);
            return send_response(connection, "删除成功", MHD_HTTP_OK);
        }
        prev = current;
        current = current->next;
    }
    
    return send_response(connection, "未找到该用户", MHD_HTTP_NOT_FOUND);
}

// API 接口：修改人员信息
int handle_update_user(struct MHD_Connection* connection, const char* request_body) {
    // 简化实现：直接按姓名匹配并更新
    char data_copy[2048];
    strncpy(data_copy, request_body, sizeof(data_copy) - 1);
    data_copy[sizeof(data_copy) - 1] = '\0';
    
    char* tokens[10];
    char* token = strtok(data_copy, ",");
    int i = 0;
    while (token && i < 10) {
        tokens[i] = token;
        token = strtok(NULL, ",");
        i++;
    }
    
    if (i < 10) {
        return send_response(connection, "数据格式错误", MHD_HTTP_BAD_REQUEST);
    }
    
    // 按姓名查找要修改的人员
    char* name = tokens[1];
    Person* current = head;
    
    while (current != NULL) {
        if (strcmp(current->M_name, name) == 0) {
            // 更新信息
            strncpy(current->Career, tokens[0], MAX - 1);
            strncpy(current->M_name, tokens[1], MAX - 1);
            current->M_age = atoi(tokens[2]);
            strncpy(current->M_sex, tokens[3], MAX - 1);
            current->M_area = atoi(tokens[4]);
            current->password = atoll(tokens[5]);
            current->parking_imfor = atoi(tokens[6]);
            current->Area[0] = atoi(tokens[7]);
            current->Count_charge = (strcmp(tokens[8], "已缴费") == 0) ? 1 : 0;
            current->M_phone_num = atoll(tokens[9]);
            
            Save(head);
            return send_response(connection, "修改成功", MHD_HTTP_OK);
        }
        current = current->next;
    }
    
    return send_response(connection, "未找到要修改的用户", MHD_HTTP_NOT_FOUND);
}


// 获取当前可执行文件的绝对路径
char* get_exe_path() {
    char path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path)-1);
    if (len == -1) return NULL;
    path[len] = '\0';
    
    // 获取目录部分
    char* last_slash = strrchr(path, '/');
    if (last_slash) {
        *last_slash = '\0';
    }
    return strdup(path);
}

char* map_url_to_file(const char* url) {
    printf("DEBUG: 请求 URL: '%s'\n", url);
    
    char* exe_dir = get_exe_path();
    if (!exe_dir) {
        printf("DEBUG: 获取执行目录失败\n");
        return NULL;
    }
    
    printf("DEBUG: 执行目录: '%s'\n", exe_dir);
    
    // 从 build/ 回退到 Li_Bolin_Folder/ 项目根目录
    char full_path[2048];
    snprintf(full_path, sizeof(full_path), "%s/../..%s", exe_dir, url);
    printf("DEBUG: 尝试项目根路径: '%s'\n", full_path);
    
    FILE* test_fp = fopen(full_path, "r");
    if (test_fp) {
        printf("DEBUG: 项目根路径找到文件！\n");
        fclose(test_fp);
        free(exe_dir);
        return strdup(full_path);
    }
    
    // 如果在根目录没找到，尝试 manager_files/
    snprintf(full_path, sizeof(full_path), "%s/../../manager_files%s", exe_dir, url);
    printf("DEBUG: 尝试 manager_files 路径: '%s'\n", full_path);
    
    test_fp = fopen(full_path, "r");
    if (test_fp) {
        printf("DEBUG: manager_files 路径找到文件！\n");
        fclose(test_fp);
        free(exe_dir);
        return strdup(full_path);
    }
    
    // 尝试 owner_files/
    snprintf(full_path, sizeof(full_path), "%s/../../owner_files%s", exe_dir, url);
    printf("DEBUG: 尝试 owner_files 路径: '%s'\n", full_path);
    
    test_fp = fopen(full_path, "r");
    if (test_fp) {
        printf("DEBUG: owner_files 路径找到文件！\n");
        fclose(test_fp);
        free(exe_dir);
        return strdup(full_path);
    }
    
    printf("DEBUG: 所有路径都找不到文件\n");
    free(exe_dir);
    return NULL;
}

int request_handler(void* cls,
                   struct MHD_Connection* connection,
                   const char* url,
                   const char* method,
                   const char* version,
                   const char* upload,
                   size_t* upload_data_size,
                   void** ptr) {
    
    // 只处理 GET 和 POST 的完整数据
    if (strcmp(method, "GET") == 0) {
        if (strcmp(url, "/api/all_info") == 0) {
            return handle_get_all_users(connection);
        } else if (strncmp(url, "/api/user", 9) == 0) {
            return handle_query_user(connection);
        }
        // 静态文件略（你已有）
    }
    
    // POST 处理：假设数据已完整（小数据场景）
    if (strcmp(method, "POST") == 0 && *upload_data_size > 0) {
        if (strcmp(url, "/api/users") == 0) {
            return handle_add_user(connection, upload_data);
        } else if (strcmp(url, "/api/delete") == 0) {
            return handle_delete_user(connection, upload_data);
        } else if (strcmp(url, "/api/update") == 0) {
            return handle_update_user(connection, upload_data);
        }
    }
    
    return send_response(connection, "Not Found", MHD_HTTP_NOT_FOUND);
}

// 初始化函数（复用你的逻辑）
void initialize_system() {
    printf("正在初始化系统...\n");
    imfor = Init_imfor(imfor);
    Imfor_Read(); // 从文件加载数据
    printf("系统初始化完成！\n");
}

// 主函数
int main() {
    initialize_system();
    
    struct MHD_Daemon* daemon = MHD_start_daemon(
        MHD_USE_SELECT_INTERNALLY,
        8080,
        NULL,
        NULL,
        &request_handler,
        NULL,
        MHD_OPTION_END
    );
    
    if (daemon == NULL) {
        printf("启动服务器失败！\n");
        return 1;
    }
    
    printf("服务器已启动，访问 http://localhost:8080\n");
    printf("按 Enter 键停止服务器...\n");
    
    getchar();
    
    MHD_stop_daemon(daemon);
    printf("服务器已停止。\n");
    
    return 0;
}

// 以下是你的原始函数实现（保持不变）
void Show_Imfor(Person* head, Imfor* imfor) {
    if (head == NULL || imfor == NULL) {
        printf("信息展示出现问题！\n");
        return;
    }
    printf("当前停车位数目：%d\n楼宇数目：%d\n收费周期：%d\n单次收费金额：%d\n", imfor->Num_parking, imfor->Num_Building,
        imfor->charging_date, imfor->charging_fee);
    Person* node = head;
    while (node != NULL) {
        printf("姓名：%s 年龄：%d 性别：%s 电话号：%lld\n", node->M_name, node->M_age, node->M_sex, node->M_phone_num);
        node = node->next;
    }
}

Imfor* Init_imfor(Imfor* imfor) {
    if (imfor == NULL) {
        imfor = malloc(sizeof(Imfor));
    }
    for (int i = 0; i < 10; i++) {
        imfor->parking[i] = 0;
    }
    for (int i = 10; i < MAX; i++) {
        imfor->parking[i] = -1;
    }
    imfor->Num_Building = 40;
    imfor->Num_parking = 10;
    imfor->charging_fee = 50;
    imfor->charging_date = 6;
    return imfor;
}

Person* Init_Person(Person* person) {
    person->password = 11111111;
    person->Count_charge = 0;
    person->Date_charge[0][0] = 0;
    person->Date_charge[0][1] = 0;
    person->Date_charge[0][2] = 0;
    person->parking_imfor = 0;
    return person;
}

void Save(Person* head) {
    FILE* fp;
    fp = fopen(FILENAME, "w");
    if (fp == NULL) {
        printf("文件打开遇到错误！\n");
        return;
    }
    fprintf(fp, "%d %d %d\n", imfor->Num_Building, imfor->charging_date, imfor->charging_fee);
    for (int i = 0; i < imfor->Num_parking; i++) {
        if (i == 0) {
            fprintf(fp, "%d", imfor->parking[i]);
        }
        else {
            fprintf(fp, " %d", imfor->parking[i]);
        }
    }
    fprintf(fp, "\n");
    Person* node = head;
    while (node != NULL) {
        fprintf(fp, "%s %s %d %d %lld %lld %d %s\n", node->M_name, node->M_sex, node->M_age, node->M_area, node->M_phone_num, node->password,
            node->parking_imfor, node->Career);
        if (strcmp(node->Career, "业主") == 0) {
            fprintf(fp, "0\n");
        }
        else {
            for (int i = 0; i < node->Area_count; i++) {
                if (i == 0) {
                    fprintf(fp, "%d", node->Area[i]);
                }
                else {
                    fprintf(fp, " %d", node->Area[i]);
                }
            }
            fprintf(fp, "\n");
        }
        fprintf(fp, "%d\n", node->Count_charge);
        for (int i = 0; i < node->Count_charge; i++) {
            if (i == 0) {
                fprintf(fp, "%d %d %d", node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
            }
            else {
                fprintf(fp, " %d %d %d", node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
            }
        }
        fprintf(fp, "\n");
        node = node->next;
    }
    fclose(fp);
    fp = NULL;
    return;
}

void Imfor_Read() {
    Imfor* temp_imfor = malloc(sizeof(Imfor));
    Init_imfor(temp_imfor);
    FILE* fp;
    fp = fopen(FILENAME, "r");
    if (fp == NULL) {
        printf("文件打开遇到错误！\n");
        free(temp_imfor);
        temp_imfor = NULL;
        return;
    }
    
    // 读取基本信息
    if (fscanf(fp, "%d %d %d", &temp_imfor->Num_Building, &temp_imfor->charging_date, &temp_imfor->charging_fee) != 3) {
        printf("文件读取失败！\n");
        fclose(fp);
        free(temp_imfor);
        temp_imfor = NULL;
        return;
    }
    
    // 读取停车位信息
    int parking_count = 0;
    while (fscanf(fp, "%d", &temp_imfor->parking[parking_count]) == 1) {
        parking_count++;
        if (parking_count >= MAX) break;  // 防止溢出
        
        char c = fgetc(fp);
        if (c == '\n' || c == EOF) {
            break;
        }
        if (c == ' ') {
            ungetc(c, fp);
        }
    }
    temp_imfor->Num_parking = parking_count;
    
    if (imfor == NULL) {
        imfor = malloc(sizeof(Imfor));
        if (imfor == NULL) {
            printf("全局imfor内存分配失败！\n");
            fclose(fp);
            free(temp_imfor);
            temp_imfor = NULL;
            return;
        }
    }
    *imfor = *temp_imfor;
    free(temp_imfor);
    
    // 修复：添加人员数据读取的最大数量限制，防止死循环
    int person_count = 0;
    const int MAX_PERSONS = 1000;  // 限制最大人数
    
    while (person_count < MAX_PERSONS) {  // ❌ 原来的 while(1) 改为有限循环
        int c = fgetc(fp);
        if (c == EOF) {
            break;  // 文件结束
        }
        ungetc(c, fp);  // 把字符放回去
        
        Person* temp_person = (Person*)malloc(sizeof(Person));
        if (temp_person == NULL) {
            printf("初始化异常！\n");
            fclose(fp);
            return;
        }
        Init_Person(temp_person);
        
        // 尝试读取人员数据
        if (fscanf(fp, "%s %s %d %d %lld %lld %d %s",
            temp_person->M_name,
            temp_person->M_sex,
            &temp_person->M_age,
            &temp_person->M_area,
            &temp_person->M_phone_num,
            &temp_person->password,
            &temp_person->parking_imfor,
            temp_person->Career) != 8) {
            printf("数据读取异常! 可能是文件格式错误或到达文件末尾\n");
            free(temp_person);
            break;  // 退出循环
        }
        
        // 读取工作区域信息
        if (strcmp(temp_person->Career, "业主") == 0) {
            fscanf(fp, "%d", &temp_person->Area_count);
            temp_person->Area[0] = 0;
        }
        else {
            int temp_Area_count = 0;
            while (temp_Area_count < MAX) {
                int area_val;
                if (fscanf(fp, "%d", &area_val) == 1) {
                    temp_person->Area[temp_Area_count] = area_val;
                    temp_Area_count++;
                } else {
                    break;  // 读取失败
                }
                
                char sep = fgetc(fp);
                if (sep == '\n' || sep == EOF) {
                    break;  // 换行符或文件结束
                }
                if (sep != ' ') {
                    ungetc(sep, fp);  // 不是空格，放回去
                    break;
                }
            }
            temp_person->Area_count = temp_Area_count;
        }
        
        // 读取缴费次数
        fscanf(fp, "%d", &temp_person->Count_charge);
        
        // 读取缴费日期
        int temp_Charge_count = 0;
        while (temp_Charge_count < MAX && temp_Charge_count < temp_person->Count_charge) {
            if (fscanf(fp, "%d %d %d", 
                &temp_person->Date_charge[temp_Charge_count][0],
                &temp_person->Date_charge[temp_Charge_count][1],
                &temp_person->Date_charge[temp_Charge_count][2]) == 3) {
                temp_Charge_count++;
            } else {
                break;  // 读取失败
            }
            
            char sep = fgetc(fp);
            if (sep == '\n' || sep == EOF) {
                break;  // 换行符或文件结束
            }
            if (sep != ' ') {
                ungetc(sep, fp);  // 不是空格，放回去
                break;
            }
        }
        temp_person->Count_charge = temp_Charge_count;
        
        head = ADD_TO_LIST(head, temp_person);
        person_count++;  // 计数器递增
        
        // 检查是否到达文件末尾
        if (feof(fp)) {
            break;
        }
    }
    
    fclose(fp);
}

Person* ADD_TO_LIST(Person* head, Person* person) {
    if (person == NULL) {
        printf("系统错误！\n");
        return head;
    }
    if (head == NULL) {
        head = person;
        person->next = NULL;
        return head;
    }
    Person* node = head;
    while (node->next != NULL) {
        node = node->next;
    }
    node->next = person;
    person->next = NULL;
    return head;
}

Person* Delimfor(Person* head) {
    if (head == NULL) {
        printf("系统错误！\n");
        return head;
    }
    printf("请输入你要删除的对象姓名：\n");
    char delname[MAX];
    scanf("%s", delname);
    Person* node = head;
    if (strcmp(node->M_name, delname) == 0 && node->next != NULL) {
        head = node->next;
        free(node);
        printf("删除成功！\n");
        Save(head);
        return head;
    }
    if (node->next == NULL) {
        if (strcmp(node->M_name, delname) != 0) {
            printf("找不到对象！\n");
            return head;
        }
        else {
            free(node);
            node = NULL;
            head = NULL;
            printf("删除成功！\n");
            Save(head);
            return head;
        }
    }
    while (node->next != NULL) {
        if (strcmp(node->next->M_name, delname) == 0 && node->next->next != NULL) {
            Person* tempnode = node->next;
            node->next = node->next->next;
            free(tempnode);
            printf("删除成功！\n");
            Save(head);
            return head;
        }
        if (strcmp(node->next->M_name, delname) == 0 && node->next->next == NULL) {
            Person* tempnode = node->next;
            node->next = NULL;
            free(tempnode);
            printf("删除成功！\n");
            Save(head);
            return head;
        }
        node = node->next;
    }
    printf("找不到对象！\n");
    return head;
}

Person* Mod_Person(Person* person) {
    Person* new_person = (Person*)malloc(sizeof(Person));
    *new_person = *person;
    printf("请输入你要修改的信息：\n1.姓名 2.年龄 3.电话号 4.职业 5.家庭住址 6.工作区域");
    int mod_Person_choice;
    scanf("%d", &mod_Person_choice);
    switch (mod_Person_choice) {
    case 1:
        printf("请输入新名称：\n");
        char newname[MAX];
        scanf("%s", newname);
        strcpy(new_person->M_name, newname);
        printf("修改成功！\n");
        break;
    case 2:
        printf("请输入年龄：\n");
        int newage;
        scanf("%d", &newage);
        new_person->M_age = newage;
        printf("修改成功！\n");
        break;
    case 3:
        printf("请输入电话号：\n");
        long long newphonenumber;
        scanf("%lld", &newphonenumber);
        new_person->M_phone_num = newphonenumber;
        printf("修改成功！\n");
        break;
    case 4:
        printf("请输入新职业：\n");
        char newcareer[MAX];
        scanf("%s", newcareer);
        strcpy(new_person->Career, newcareer);
        printf("修改成功！\n");
        break;
    case 5:
        printf("请输入家庭住址：\n");
        int newarea;
        scanf("%d", &newarea);
        new_person->M_area = newarea;
        printf("修改成功！\n");
        break;
    case 6:
        printf("请输入ta的工作区域(输入-1停止输入)\n");
        int num;
        int M_count = 0;
        while (1) {
            scanf("%d", &num);
            if (num == -1) {
                break;
            }
            if (M_count >= MAX) {
                printf("输入过多\n");
                break;
            }
            new_person->Area[M_count++] = num;
            new_person->Area_count = M_count;
        }
        printf("修改成功！\n");
        break;
    default:
        break;
    }
    free(person);
    return new_person;
}

Person* ModImfor(Person* head, Imfor* imfor) {
    printf("请输入你要修改的对象类型：\n1.人员信息 2.公共服务\n");
    int choice;
    scanf("%d", &choice);
    switch (choice) {
    case 1:
        printf("请输入你要更改的人员姓名：\n");
        char mod_person[MAX];
        scanf("%s", mod_person);
        Person* node = head;
        if (head == NULL) {
            printf("链表为空！\n");
            return head;
        }
        if (node->next == NULL) {
            if (strcmp(node->M_name, mod_person) != 0) {
                printf("找不到对象！\n");
                return head;
            }
            else {
                Person* newperson = Mod_Person(node);
                head = newperson;
                newperson->next = NULL;
                printf("修改成功！\n");
                Save(head);
                return head;
            }
        }
        if (node->next != NULL && strcmp(node->M_name, mod_person) == 0) {
            Person* nextnode = head->next;
            Person* newperson = Mod_Person(node);
            head = newperson;
            newperson->next = nextnode;
            printf("修改成功！\n");
            Save(head);
            return head;
        }
        while (node->next != NULL) {
            if (node->next->next != NULL && strcmp(node->next->M_name, mod_person) == 0) {
                Person* newperson = Mod_Person(node->next);
                Person* nextnode = node->next->next;
                Person* freenode = node->next;
                node->next = newperson;
                newperson->next = nextnode;
                printf("修改成功！\n");
                Save(head);
                return head;
            }
            if (node->next->next == NULL && strcmp(node->next->M_name, mod_person) == 0) {
                Person* newperson = Mod_Person(node->next);
                Person* freenode = node->next;
                node->next = newperson;
                newperson->next = NULL;
                printf("修改成功！\n");
                Save(head);
                return head;
            }
            node = node->next;
        }
        printf("找不到对象！\n");
        break;
    case 2:
        printf("请输入你要更改的服务信息：\n1.停车位 2.收费细则 3.楼房数目\n");
        int serve_choice;
        scanf("%d", &serve_choice);
        switch (serve_choice) {
        case 1:
            printf("当前停车位数目:%d\n请输入你的操作：\n1.新增 2.更改\n", imfor->Num_parking);
            int park_choice;
            scanf("%d", &park_choice);
            switch (park_choice) {
            case 1:
                printf("请输入新增的停车位数目：\n");
                int new_add_parking;
                scanf("%d", &new_add_parking);
                if (new_add_parking > 0) {
                    for (int i = 0; i < new_add_parking; i++) {
                        imfor->parking[i + imfor->Num_parking] = 0;
                    }
                    imfor->Num_parking += new_add_parking;
                    printf("操作成功！\n");
                }
                else {
                    printf("输入必须是正数！\n");
                }
                break;
            case 2:
                printf("请输入你要更改状态的停车位：\n");
                int mod_parking;
                scanf("%d", &mod_parking);
                if (imfor->parking[mod_parking] == 0) {
                    printf("是否关闭？\n1.yes 2.no\n");
                    int of;
                    scanf("%d", &of);
                    switch (of) {
                    case 1:
                        imfor->parking[mod_parking] = -1;
                        printf("已关闭\n");
                        break;
                    case 2:
                        printf("已取消操作\n");
                        break;
                    default:
                        printf("输入错误，已退出\n");
                        break;
                    }
                }
                else if (imfor->parking[mod_parking] == -1) {
                    printf("是否开放？\n1.yes 2.no\n");
                    int of;
                    scanf("%d", &of);
                    switch (of) {
                    case 1:
                        imfor->parking[mod_parking] = 0;
                        printf("已开放\n");
                        break;
                    case 2:
                        printf("已取消操作\n");
                        break;
                    default:
                        printf("输入错误，已退出\n");
                        break;
                    }
                }
                else {
                    printf("无法操作！该停车位被占用！\n");
                }
                break;
            default:
                printf("输入错误，已退出\n");
                break;
            }
            break;
        case 2:
            printf("请输入要更改的收费规则\n1.收费周期 2.单次收费金额\n");
            int charge_choice;
            scanf("%d", &charge_choice);
            switch (charge_choice) {
            case 1:
                printf("请输入新的收费周期：\n");
                int new_charge_date;
                scanf("%d", &new_charge_date);
                if (new_charge_date <= 0) {
                    printf("数据不合法！已退出\n");
                    break;
                }
                else {
                    imfor->charging_date = new_charge_date;
                }
                printf("修改成功！\n");
                break;
            case 2:
                printf("请输入新的单次收费金额：\n");
                int new_charge_fee;
                scanf("%d", &new_charge_fee);
                if (new_charge_fee <= 0) {
                    printf("数据不合法！已退出\n");
                    break;
                }
                else {
                    imfor->charging_fee = new_charge_fee;
                }
                printf("修改成功！\n");
                break;
            default:
                printf("输入错误，已退出\n");
                break;
            }
            break;
        case 3:
            printf("当前楼宇数目：%d\n请输入更改数据(增加的楼宇数目)：\n", imfor->Num_Building);
            int mod_building;
            scanf("%d", &mod_building);
            if (mod_building <= 0) {
                printf("数据不合法！已退出\n");
                break;
            }
            printf("您真的要更改吗？\n1.yes 2.no\n");
            int choice;
            scanf("%d", &choice);
            switch (choice) {
            case 1:
                imfor->Num_Building += mod_building;
                printf("更改成功！\n");
                break;
            case 2:
                printf("已退出\n");
                break;
            default:
                printf("输入错误，已退出\n");
                break;
            }
            break;
        default:
            break;
        }
        break;
    default:
        printf("输入错误！已退出\n");
        break;
    }
    Save(head);
    return head;
}
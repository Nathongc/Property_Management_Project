#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#define MAX_STAFF 100
#define MAX_OWNER 10000
#define MAX_RECORD 1000
#define STAFF_FILE "staff.txt"
#define OWNER_FILE "owner.txt"
#define RECORD_FILE "record.txt"

#include "../include/server.h"

typedef enum
{
    PROPERTY_MANAGER,//物业经理
    CUSTOMER_SERVICE_SPECIALIST,//客服专员
    SECURITY_OFFICER,//保安
    CLEANER,//清洁工
    BUILDING_BULTER//楼栋管家
}PositionType;


// struct Property_Manager//物业经理
// {

// };

// struct Customer_Service_Specialist//客服专员
// {

// };

// struct Security_Officer//保安
// {
    
// };

// struct Cleaner//清洁工
// {
    
// };

// struct Building_Bulter//楼栋管家
// {
    
// };

struct Staff//物业服务人员
{
    char name[20];
    int id;
    char password[9];
    char area[50];
    char phonenumber[15];
    PositionType position;
    // union{
    //     struct Property_Manager pm;
    //     struct Customer_Service_Specialist css;
    //     struct Security_Officer so;
    //     struct Cleaner cln;
    //     struct Building_Bulter bb;
    // }data;
};

struct PaymentRecord
{
    int payment_year;//缴费年份
    int payment_status;//0_未缴费，1_已缴费
};

struct Owner
{
    char name[20];
    int id;
    char address[50];
    struct PaymentRecord pr[20];
    int payment_count;
};

struct ServiceRecord//服务记录
{
    int staff_id;//服务人员id
    int owner_id;//业主id
    char service_content[100];
    char date[20];
};

typedef struct StaffNode{
    struct Staff data;
    struct StaffNode* next;
}StaffNode;

typedef struct OwnerNode{
    struct Owner data;
    struct OwnerNode* next;
}OwnerNode;

typedef struct RecordNode{
    struct ServiceRecord data;
    struct RecordNode* next;
}RecordNode;

StaffNode* staff_head=NULL;
OwnerNode* owner_head=NULL;
RecordNode* record_head=NULL;
int staff_count=0;
int owner_count=0;
int record_count=0;

StaffNode* createStaffNode(struct Staff s){
    StaffNode* node=(StaffNode*)malloc(sizeof(StaffNode));
    node->data=s;
    node->next=NULL;
    return node;
}

OwnerNode* createOwnerNode(struct Owner o){
    OwnerNode* node=(OwnerNode*)malloc(sizeof(OwnerNode));
    node->data=o;
    node->next=NULL;
    return node;
}

RecordNode* createRecordNode(struct ServiceRecord r){
    RecordNode* node=(RecordNode*)malloc(sizeof(RecordNode));
    node->data=r;
    node->next=NULL;
    return node;
}

void addStaffTail(struct Staff s){
    StaffNode* node=createStaffNode(s);
    if(staff_head==NULL){
        staff_head=node;
    }else{
        StaffNode* p=staff_head;
        while(p->next!=NULL){
            p=p->next;
        }
        p->next=node;
    }
    staff_count++;
}

void addOwnerTail(struct Owner o){
    OwnerNode* node=createOwnerNode(o);
    if(owner_head==NULL){
        owner_head=node;
    }else{
        OwnerNode* p=owner_head;
        while(p->next!=NULL){
            p=p->next;
        }
        p->next=node;
    }
    owner_count++;
}

void addRecordTail(struct ServiceRecord r){
    RecordNode* node=createRecordNode(r);
    if(record_head==NULL){
        record_head=node;
    }else{
        RecordNode* p=record_head;
        while(p->next!=NULL){
            p=p->next;
        }
        p->next=node;
    }
    record_count++;
}

void freeStaffList(){
    StaffNode* p=staff_head;
    while(p!=NULL){
        StaffNode* temp=p;
        p=p->next;
        free(temp);
    }
    staff_head=NULL;
    staff_count=0;
}

void freeOwnerList(){
    OwnerNode* p=owner_head;
    while(p!=NULL){
        OwnerNode* temp=p;
        p=p->next;
        free(temp);
    }
    owner_head = NULL;
    owner_count = 0;
}

void freeRecordList(){
    RecordNode* p = record_head;
    while(p != NULL)
    {
        RecordNode* temp = p;
        p = p->next;
        free(temp);
    }
    record_head = NULL;
    record_count = 0;
}

void free_all(){
    freeStaffList();
    freeOwnerList();
    freeRecordList();
}

void saveStaffText(){
    FILE* fp = fopen(STAFF_FILE, "w");
    if(fp == NULL){
        printf("打开文件夹失败");
        return;
    }
    fprintf(fp, "%d\n", staff_count);
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        fprintf(fp, "%s %d %s %s %s %d\n",
                p->data.name,
                p->data.id,
                p->data.password,
                p->data.area,
                p->data.phonenumber,
                p->data.position);
    }
    fclose(fp);
}

void loadStaffText(){
    FILE* fp = fopen(STAFF_FILE, "r");
    if(fp == NULL){
        // 文件不存在，这是允许的，不报错
        return;
    }
    // 读取预期的员工总数
    int expected_count;
    if(fscanf(fp, "%d", &expected_count) != 1){
        fprintf(stderr, "❌ staff.txt: 读取员工总数失败\n");
        fclose(fp);
        return;
    }

    // 验证员工数量是否合理
    if(expected_count < 0 || expected_count > MAX_STAFF){
        fprintf(stderr, "❌ staff.txt: 员工数量不合理 (%d)\n", expected_count);
        fclose(fp);
        return;
    }

    // 循环读取，直到达到预期数量或文件结束
    for(int i = 0; i < expected_count; i++){
        struct Staff s;

        // 尝试读取一个员工的信息
        int ret = fscanf(fp, "%s %d %s %s %s %d",
                         s.name, &s.id, s.password, s.area, s.phonenumber, &s.position);

        // 如果返回值不是 6，说明读取失败或到达文件末尾
        if(ret != 6){
            // 如果是意外的读取失败（比如格式错误）
            if(ret != EOF){
                fprintf(stderr, "❌ staff.txt: 读取第 %d 个员工错误 (ret=%d)\n", i+1, ret);
            }
            // 如果是到达文件末尾，但预期数量还没读完，给出提示
            if(ret == EOF){
                fprintf(stderr, "⚠️  staff.txt: 文件中的员工数量少于预期 (%d < %d)，仅加载已找到的员工。\n", i, expected_count);
            }
            // 无论哪种情况，都停止读取
            break;
        }

        // 验证读取的数据是否有效（可选）
        if(s.id <= 0){
            fprintf(stderr, "❌ staff.txt: 第 %d 个员工ID无效 (%d)\n", i+1, s.id);
            // 可以选择是跳过这个员工还是中断加载
            // 这里选择跳过
            i--; // 因的无效，所以索引不应该增加
            continue;
        }

        // 将读取成功的员工添加到链表
        addStaffTail(s);
    }
    // 更新全局计数
    staff_count = 0; // 重新计算，因为可能有无效ID被跳过了
    StaffNode* current = staff_head;
    while(current != NULL){
        staff_count++;
        current = current->next;
    }

    fclose(fp);
    printf("✅ 成功加载 %d 名物业服务人员\n", staff_count);
}

void saveOwnerText(){
    FILE* fp = fopen(OWNER_FILE, "w");
    if(fp == NULL){
        perror("打开文件夹失败");
        return;
    }
    fprintf(fp, "%d\n", owner_count);
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        fprintf(fp, "%s %d %s %d\n",
                p->data.name,
                p->data.id,
                p->data.address,
                p->data.payment_count);
        for(int j = 0; j < p->data.payment_count; j++){
            fprintf(fp, "%d %d\n",
                    p->data.pr[j].payment_year,
                    p->data.pr[j].payment_status);
        }
    }
    fclose(fp);
}

void loadOwnerText(){
    FILE* fp = fopen(OWNER_FILE, "r");
    if(fp == NULL){
        // 文件不存在，这是允许的，不报错
        return;
    }
    // 读取预期的业主总数
    int expected_count;
    if(fscanf(fp, "%d", &expected_count) != 1){
        fprintf(stderr, "❌ owner.txt: 读取业主总数失败\n");
        fclose(fp);
        return;
    }

    // 验证业主数量是否合理
    if(expected_count < 0 || expected_count > MAX_OWNER){
        fprintf(stderr, "❌ owner.txt: 业主数量不合理 (%d)\n", expected_count);
        fclose(fp);
        return;
    }

    for(int i = 0; i < expected_count; i++){
        struct Owner o;

        // 尝试读取业主基本信息
        int ret = fscanf(fp, "%s %d %s %d",
                         o.name, &o.id, o.address, &o.payment_count);

        if(ret != 4){
            if(ret != EOF){
                fprintf(stderr, "❌ owner.txt: 读取第 %d 个业主基本信息失败 (ret=%d)\n", i+1, ret);
            }else{
                fprintf(stderr, "⚠️  owner.txt: 文件中的业主数量少于预期 (%d < %d)，仅加载已找到的业主。\n", i, expected_count);
            }
            break; // 停止读取
        }

        // 验证读取的数据是否有效
        if(o.id <= 0 || o.payment_count < 0 || o.payment_count > 20){ // 假设缴费记录数不超过20{
            fprintf(stderr, "❌ owner.txt: 第 %d 个业主数据无效 (ID=%d, Count=%d)\n", i+1, o.id, o.payment_count);
            // 可以选择跳过此业主或中断加载，这里选择跳过
            continue;
        }

        // 读取缴费记录
        int j;
        for(j = 0; j < o.payment_count; j++){
            int year, status;
            int ret_pr = fscanf(fp, "%d %d", &year, &status);
            if(ret_pr != 2){
                fprintf(stderr, "❌ owner.txt: 读取第 %d 个业主的第 %d 条缴费记录失败 (ret=%d)\n", i+1, j+1, ret_pr);
                // 读取缴费记录失败，可以选择跳过此业主或仅加载部分记录
                // 这里选择跳过此业主
                break; // 跳出缴费记录循环
            }
            // 将读取的记录存入数组
            o.pr[j].payment_year = year;
            o.pr[j].payment_status = status;
        }

        // 检查是否因为读取失败而跳出缴费记录循环
        if(j != o.payment_count){
            fprintf(stderr, "⚠️  owner.txt: 未能完整读取第 %d 个业主的缴费记录，跳过该业主。\n", i+1);
            continue; // 跳过此业主
        }

        // 将读取成功的业主添加到链表
        addOwnerTail(o);
    }

    // 更新全局计数（可选，与staff类似）
    // owner_count = 0; // 如果需要严格基于链表长度，可以重新计算
    // OwnerNode* current = owner_head;
    // while(current != NULL) {
    //     owner_count++;
    //     current = current->next;
    // }

    fclose(fp);
    printf("✅ 成功加载 %d 名业主\n", owner_count); // 这里的 owner_count 是基于 addOwnerTail 累加的
}

void saveRecordText(){
    FILE* fp = fopen(RECORD_FILE, "w");
    if(fp == NULL){
        perror("打开文件夹失败");
        return;
    }
    fprintf(fp, "%d\n", record_count);
    for(RecordNode* p = record_head; p != NULL; p = p->next){
        fprintf(fp, "%d %d %s %s\n",
                p->data.staff_id,
                p->data.owner_id,
                p->data.service_content,
                p->data.date);
    }
    fclose(fp);
}

void loadRecordText(){
    FILE* fp = fopen(RECORD_FILE, "r");
    if(fp == NULL){
        // 文件不存在，这是允许的，不报错
        return;
    }
    // 读取预期的记录总数
    int expected_count;
    if(fscanf(fp, "%d", &expected_count) != 1){
        fprintf(stderr, "❌ record.txt: 读取记录总数失败\n");
        fclose(fp);
        return;
    }

    // 验证记录数量是否合理
    if(expected_count < 0 || expected_count > MAX_RECORD){
        fprintf(stderr, "❌ record.txt: 记录数量不合理 (%d)\n", expected_count);
        fclose(fp);
        return;
    }

    for(int i = 0; i < expected_count; i++){
        struct ServiceRecord r;

        // 尝试读取一条服务记录
        int ret = fscanf(fp, "%d %d %s %s",
                         &r.staff_id, &r.owner_id, r.service_content, r.date);

        if(ret != 4){
            if(ret != EOF){
                fprintf(stderr, "❌ record.txt: 读取第 %d 条记录失败 (ret=%d)\n", i+1, ret);
            }else{
                fprintf(stderr, "⚠️  record.txt: 文件中的记录数量少于预期 (%d < %d)，仅加载已找到的记录。\n", i, expected_count);
            }
            break; // 停止读取
        }

        // 验证读取的数据是否有效（可选）
        if(r.staff_id <= 0 || r.owner_id <= 0){
            fprintf(stderr, "❌ record.txt: 第 %d 条记录数据无效 (staff_id=%d, owner_id=%d)\n", i+1, r.staff_id, r.owner_id);
            continue; // 跳过无效记录
        }

        // 将读取成功的记录添加到链表
        addRecordTail(r);
    }

    // 更新全局计数（可选，与staff类似）
    // record_count = 0; // 如果需要严格基于链表长度，可以重新计算
    // RecordNode* current = record_head;
    // while(current != NULL) {
    //     record_count++;
    //     current = current->next;
    // }

    fclose(fp);
    printf("✅ 成功加载 %d 条服务记录\n", record_count); // 这里的 record_count 是基于 addRecordTail 累加的
}

void save_all_text(){
    saveStaffText();
    saveOwnerText();
    saveRecordText();
}

void load_all_text(){
    loadStaffText();
    loadOwnerText();
    loadRecordText();
}

void show_staff_menu(){
    printf("******************************\n");
    printf(" 物业服务人员菜单 \n");
    printf("****** 1.我的         **** \n");
    printf("****** 2.信息查询     **** \n");
    printf("****** 3.信息排序     **** \n");
    printf("****** 4.信息统计     **** \n");
    printf("****** 0.退出         **** \n");
    printf("\n");
}

void show_mine_menu(){
    printf("******************************\n");
    printf("********我的           ********\n");
    printf("****** 1.查看个人信息  **** \n");
    printf("****** 2.添加服务记录  **** \n");
    printf("****** 3.我的服务记录  **** \n");
    printf("****** 4.修改密码     **** \n");
    printf("****** 0.返回菜单     **** \n");
    printf("\n");
}

//我的
void mine(int index,int id){
    int choice;
    while(1){
        show_mine_menu();
        printf("请选择:");
        scanf("%d",&choice);
        switch(choice){
            case 1:
            show_my_info(index);
            break;
            case 2:
            add_record(id);
            break;
            case 3:
            show_my_records(id);
            break;
            case 4:{
                char new_password[9];
                int is_too_long=0;
                int cycle_count=0;
                while(1){
                    is_too_long=0;
                    printf("请输入8位的新密码(超过8位自动取前8位):");
                    if(!cycle_count){
                        while(getchar()!='\n'){

                        }
                    }
                    cycle_count++;
                    fgets(new_password,9,stdin);
                    int len=strlen(new_password);
                    if(len>0&&new_password[len-1]=='\n'){
                        new_password[len-1]='\0';
                    }else{
                        is_too_long=1;
                        while(getchar()!='\n'){

                        }
                    }
                    len=strlen(new_password);
                    if(len==8){
                        if(is_too_long){
                            printf("提示:已自动截取前8位作为您的新密码!\n");
                        }
                        break;
                    }else{
                        printf("密码长度不正确!\n");
                    }
                }
                change_password(index,new_password);
                break;
            }
            case 0:
            printf("返回菜单...\n");
            return;
            default:
            printf("输入错误!\n");
        }
    }
}

void show_query_menu(){
    printf("******************************\n");
    printf("********信息查询       ********\n");
    printf("********1.我的负责区域  ********\n");
    printf("********2.按姓名查业主  ********\n");
    printf("********3.查业主所有缴费********\n");
    printf("********4.查业主某年缴费********\n");
    printf("********5.查某年未缴费业主********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
}

void query(int index,int id){
    int choice;
    while(1){
        show_query_menu();
        printf("请选择:");
        scanf("%d",&choice);
        switch(choice){
            case 1:
                query_my_area(index);
                break;
            case 2:{
                char name[20]={0};
                printf("请输入业主姓名(支持模糊查询):");
                while(getchar()!='\n');
                fgets(name,20,stdin);
                int len=strlen(name);
                if(len>0&&name[len-1]=='\n'){
                    name[len-1]='\0';
                }
                query_owner_by_name(name);
                break;
            }
            case 3:{
                int owner_id;
                printf("请输入业主ID:");
                scanf("%d",&owner_id);
                query_all_payment(owner_id);
                break;
            }
            case 4:{
                int owner_id, year;
                printf("请输入业主ID:");
                scanf("%d",&owner_id);
                printf("请输入年份:");
                scanf("%d",&year);
                query_payment_by_year(owner_id, year);
                break;
            }
            case 5:{
                int year;
                printf("请输入年份:");
                scanf("%d",&year);
                query_unpaid_by_year(year, index);
                break;
            }
            case 0:
                printf("返回菜单...\n");
                return;
            default:
                printf("输入错误!\n");
        }
    }
}

void show_sort_menu(){
    printf("******************************\n");
    printf("********信息排序       ********\n");
    printf("********1.按姓名升序排序********\n");
    printf("********2.按姓名降序排序********\n");
    printf("********3.按ID升序排序  ********\n");
    printf("********4.按ID降序排序  ********\n");
    printf("********5.按欠费数量排序********\n");
    printf("********6.多属性组合排序********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
}

int get_unpaidcount(struct Owner* o){
    int count=0;
    for(int i=0;i<o->payment_count;i++){
        if(o->pr[i].payment_status==0){
            count++;
        }
    }
    return count;
}

int ascend_sort_by_name(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    return strcmp(o1->name,o2->name);
}

int descend_sort_by_name(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    return strcmp(o2->name,o1->name);
}

int ascend_sort_by_id(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    return o1->id-o2->id;
}

int descend_sort_by_id(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    return o2->id-o1->id;
}

int descend_sort_by_unpaidcount(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    int count1=get_unpaidcount(o1);
    int count2=get_unpaidcount(o2);
    return count2-count1;
}

int multi_sort(const void* a,const void* b){
    struct Owner* o1=(struct Owner*)a;
    struct Owner* o2=(struct Owner*)b;
    int has_unpaid1=get_unpaidcount(o1)>0?1:0;
    int has_unpaid2=get_unpaidcount(o2)>0?1:0;
    if(has_unpaid1!=has_unpaid2){
        return has_unpaid2-has_unpaid1;
    }
    return strcmp(o1->name,o2->name);
}

void sort(int index){
    int i=0;
    struct Staff* mine=NULL;
    for(StaffNode* p=staff_head;p!=NULL;p=p->next){
        if(i==index){
            mine=&p->data;
            break;
        }
        i++;
    }
    if(mine==NULL) return;
    char* my_area=mine->area;
    static struct Owner owner_arr[MAX_OWNER];
    int owner_num=0;
    for(OwnerNode* p=owner_head;p!=NULL;p=p->next){
        if(strstr(p->data.address,my_area)!=NULL){
            owner_arr[owner_num++]=p->data;
            if(owner_num>=MAX_OWNER) break;
        }
    }
    if(owner_num==0){
        printf("您的区域暂无业主，无法排序!\n");
        return;
    }
    int choice;
    while(1){
        show_sort_menu();
        printf("请选择:");
        scanf("%d",&choice);
        switch(choice){
            case 1:
            qsort(owner_arr,owner_num,sizeof(struct Owner),ascend_sort_by_name);
            break;
            case 2:
            qsort(owner_arr,owner_num,sizeof(struct Owner),descend_sort_by_name);
            break;
            case 3:
            qsort(owner_arr,owner_num,sizeof(struct Owner),ascend_sort_by_id);
            break;
            case 4:
            qsort(owner_arr,owner_num,sizeof(struct Owner),descend_sort_by_id);
            break;
            case 5:
            qsort(owner_arr,owner_num,sizeof(struct Owner),descend_sort_by_unpaidcount);
            break;
            case 6:
            qsort(owner_arr,owner_num,sizeof(struct Owner),multi_sort);
            break;
            case 0:
            printf("返回菜单...\n");
            return;
            default:
            printf("输入错误!\n");
            continue;
        }
        printf("已排序，结果如下:\n");
        for(int j=0;j<owner_num;j++){
            int unpaid_num=get_unpaidcount(&owner_arr[j]);
            printf("%d.姓名:%s\tID:%d\t地址:%s\t未缴费记录:%d条\n",
                    j+1,
                    owner_arr[j].name,
                    owner_arr[j].id,
                    owner_arr[j].address,
                    unpaid_num);
        }
    }
}

void show_statistics_menu(){
    printf("******************************\n");
    printf("********信息统计       ********\n");
    printf("********1.按单一属性统计********\n");
    printf("********2.按多属性统计 ********\n");
    printf("********3.预设统计     ********\n");
    printf("********4.按条件统计   ********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
}

void statistics(int index,int id){
    int choice;
    while(1){
        show_statistics_menu();
        printf("请选择:");
        scanf("%d",&choice);
        switch(choice){
            case 1:{
                int flag=1;
                while(flag){
                    printf("请选择:\n");
                    printf("1.按年份统计  2.按区域统计\n");
                    int choice1;
                    scanf("%d",&choice1);
                    switch(choice1){
                        case 1:{
                            printf("请输入年份:");
                            int year;
                            scanf("%d",&year);
                            statistics_by_year(year,index);
                            flag=0;
                            break;
                        }
                        case 2:{
                            int inside_flag=1;
                            int cycle_count=0;
                            while(inside_flag){
                                printf("请输入目标区域:");
                                char target_area[50]={0};
                                if(!cycle_count){
                                    while(getchar()!='\n'){

                                    }
                                }
                                cycle_count++;
                                fgets(target_area,50,stdin);
                                int len=strlen(target_area);
                                if(len>0&&target_area[len-1]=='\n'){
                                    target_area[len-1]='\0';
                                }
                                inside_flag=!statistics_by_area(target_area,index);
                            }
                            flag=0;
                            break;
                        }
                        default:
                        printf("输入错误!\n");
                    }
                }
            }
            break;
            case 2:{
                printf("按年份和区域统计:\n");
                printf("请输入年份:");
                int year;
                scanf("%d",&year);
                int flag=1;
                while(flag){
                    printf("请输入目标区域:");
                    char target_area[50]={0};
                    while(getchar()!='\n'){

                    }
                    fgets(target_area,50,stdin);
                    int len=strlen(target_area);
                    if(len>0&&target_area[len-1]=='\n'){
                        target_area[len-1]='\0';
                    }
                    flag=!statistics_by_year_and_area(year,target_area,index);
                }
                break;
            }
            case 3:{
                printf("统计2026年未缴费业主人数:\n");
                statistics_2026_unpaid(index);
                break;
            }
            case 4:{
                printf("统计从某年至今存在未缴费记录业主人数:\n");
                statistics_year_condition(index);
                break;
            }
            case 0:
            printf("返回菜单...\n");
            return;
            default:
            printf("输入错误!\n");
        }
    }
}

const char* get_pos_name(PositionType position){
    switch(position)
    {
    case PROPERTY_MANAGER:
        return "物业经理";
    case CUSTOMER_SERVICE_SPECIALIST:
        return "客服专员";
    case SECURITY_OFFICER:
        return "保安";
    case CLEANER:
        return "清洁工";
    case BUILDING_BULTER:
        return "楼栋管家";
    default:
        return "未知";
    }
}

//登录
int staff_login(int id, char* password){
    int index = 0;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(p->data.id == id && strcmp(p->data.password, password) == 0){
            return index;
        }
        index++;
    }
    return -1;
}

//找回密码
void find_password(){
    int id;
    char name[20] = {0};
    char phonenumber[15] = {0};
    printf("请输入您的姓名:");
    while(getchar() != '\n'){}
    fgets(name, 20, stdin);
    int len1 = strlen(name);
    if(len1 > 0 && name[len1-1] == '\n'){
        name[len1-1] = '\0';
    }
    printf("请输入您的ID: ");
    scanf("%d", &id);
    printf("请输入您的联系电话: ");
    while(getchar() != '\n'){}
    fgets(phonenumber, 15, stdin);
    int len2 = strlen(phonenumber);
    if(len2 > 0 && phonenumber[len2-1] == '\n'){
        phonenumber[len2-1] = '\0';
    }
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(p->data.id == id &&
                strcmp(p->data.name, name) == 0 &&
                strcmp(p->data.phonenumber, phonenumber) == 0){
            printf("验证成功!\n");
            printf("您的密码是【%s】\n", p->data.password);
            return;
        }
    }
    printf("验证失败!请检查后重试。\n");
}

//查看个人信息
void show_my_info(int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return;
    printf("姓名:%s\n", s->name);
    printf("ID:%d\n", s->id);
    printf("负责区域:%s\n", s->area);
    printf("联系电话:%s\n", s->phonenumber);
    printf("职位:%s\n", get_pos_name(s->position));
}

//添加服务记录
void add_record(int staff_id){
    if(record_count >= MAX_RECORD){
        printf("记录已满！\n");
        return;
    }
    struct ServiceRecord r;
    r.staff_id = staff_id;
    printf("请输入业主ID:");
    scanf("%d", &r.owner_id);
    printf("请输入服务内容:");
    while(getchar() != '\n'){}
    fgets(r.service_content, 100, stdin);
    int len1 = strlen(r.service_content);
    if(len1 > 0 && r.service_content[len1-1] == '\n'){
        r.service_content[len1-1] = '\0';
    }
    printf("请输入日期(如2026-3-8):");
    fgets(r.date, 20, stdin);
    int len2 = strlen(r.date);
    if(len2 > 0 && r.date[len2-1] == '\n'){
        r.date[len2-1] = '\0';
    }
    addRecordTail(r);
    saveRecordText();
    printf("服务记录添加成功！\n");
}

//查看我的服务记录
void show_my_records(int staff_id){
    int found_count = 0;
    for(RecordNode* p = record_head; p != NULL; p = p->next){
        if(p->data.staff_id == staff_id)
        {
            found_count++;
            printf("%d.业主ID:%d\t服务内容:%s\t日期:%s\n",
                   found_count,
                   p->data.owner_id,
                   p->data.service_content,
                   p->data.date);
        }
    }
    if(!found_count){
        printf("暂无记录\n");
    }
}

//修改密码
void change_password(int index, char* new_password){
    if(strlen(new_password)!=8){
        printf("密码修改失败，请确保密码为8位\n");
        return;
    }
    int i = 0;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            strcpy(p->data.password, new_password);
            break;
        }
        i++;
    }
    saveStaffText();
    printf("密码修改成功！\n");
}

//查询负责区域
void query_my_area(int index){
    int i = 0;
    struct Staff* mine = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            mine = &p->data;
        }
        i++;
    }
    if(mine == NULL) return;
    printf("----------我的负责区域----------\n");
    printf("区域：%s\n", mine->area);
    int found_count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, mine->area) != NULL){
            if(!found_count) printf("该区域业主如下：\n");
            found_count++;
            printf("%d.姓名:%s\tID:%d\t地址:%s\n ",
                   found_count,
                   p->data.name,
                   p->data.id,
                   p->data.address);
        }
    }
    if(!found_count) printf("该区域暂无业主\n");
    printf("------------------------------\n");
}

//查询业主信息
void query_owner_by_name(char* name){
    printf("------------查询结果------------\n");
    int found_count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.name, name) != NULL){
            found_count++;
            printf("%d.姓名:%s\tID:%d\t地址:%s\n",
                   found_count,
                   p->data.name,
                   p->data.id,
                   p->data.address);
        }
    }
    if(!found_count) printf("暂无符合条件的业主\n");
    printf("------------------------------\n");
}

//查询某业主某年是否缴费
void query_payment_by_year(int owner_id, int year){
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(p->data.id == owner_id){
            for(int j = 0; j < p->data.payment_count; j++){
                if(p->data.pr[j].payment_year == year){
                    printf("业主%s\n%d年:%s\n", p->data.name, year, p->data.pr[j].payment_status ? "已缴费" : "未缴费");
                    return;
                }
            }
            printf("未找到该年份缴费记录\n");
            return;
        }
    }
    printf("未找到该业主\n");
}

//查询某业主所有缴费情况
void query_all_payment(int owner_id){
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(p->data.id == owner_id){
            printf("业主%s:\n", p->data.name);
            if(p->data.payment_count == 0){
                printf("无缴费记录\n");
                return;
            }
            for(int j = 0; j < p->data.payment_count; j++){
                printf("%d年:%s\n", p->data.pr[j].payment_year, p->data.pr[j].payment_status ? "已缴费" : "未缴费");
            }
            return;
        }
    }
    printf("未找到该业主\n");
}

//查询某年未缴费业主
void query_unpaid_by_year(int year, int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return;
    char* area = s->area;
    int found_count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_year == year && p->data.pr[j].payment_status == 0){
                found_count++;
                printf("%d.姓名:%s\tID:%d\t地址:%s\n ",
                       found_count,
                       p->data.name,
                       p->data.id,
                       p->data.address);
            }
        }
    }
    if(!found_count) printf("%d年无未缴费业主\n", year);
}

//信息统计
//按单一属性统计
//按年份统计
void statistics_by_year(int year, int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index)
        {
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return;
    char* area = s->area;
    int count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_year == year && p->data.pr[j].payment_status == 0){
                count++;
                break;
            }
        }
    }
    printf("%d年未缴费业主共%d人\n", year, count);
}

//按区域统计
int statistics_by_area(char* area, int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return 0;
    char* my_area = s->area;
    if(strstr(area, my_area) == NULL){
        printf("错误:目标区域必须在【%s】区域内!\n", my_area);
        return 0;
    }
    int count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        int payment = 1;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_status == 0){
                payment = 0;
                break;
            }
        }
        if(!payment) count++;
    }
    printf("%s未缴费业主共%d人\n", area, count);
    return 1;
}

//按多属性统计
int statistics_by_year_and_area(int year, char* area, int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return 0;
    char* my_area = s->area;
    if(strstr(area, my_area) == NULL){
        printf("错误:目标区域必须在【%s】区域内!\n", my_area);
        return 0;
    }
    int count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_year == year && p->data.pr[j].payment_status == 0){
                count++;
                break;
            }
        }
    }
    printf("%s%d年未缴费业主共%d人\n", area, year, count);
    return 1;
}

//预设统计
//2026年未缴费统计
void statistics_2026_unpaid(int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return;
    char* area = s->area;
    int year = 2026;
    int count = 0;
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_year == year && p->data.pr[j].payment_status == 0){
                count++;
                break;
            }
        }
    }
    printf("2026年未缴费业主共%d人\n", count);
}

//2026我的区域未缴费统计
// void statistics_my_area_unpaid(int index){
//     char* area=staff_list[index].area;
//     int year=2026;
//     int count=0;
//     for(int i=0;i<owner_count;i++){
//         if(strstr(owner_list[i].address,area)!=NULL){
//             for(int j=0;j<owner_list[i].payment_count;j++){
//                 if(owner_list[i].pr[j].payment_year==year&&owner_list[i].pr[j].payment_status==0){
//                     count++;
//                     break;
//                 }
//             }
//         }
//     }
//     printf("你负责的区域(%s)2026年未缴费业主共%d人\n",area,count);
// }

//按条件统计
//%d年及以后存在未缴费
void statistics_year_condition(int index){
    int i = 0;
    struct Staff* s = NULL;
    for(StaffNode* p = staff_head; p != NULL; p = p->next){
        if(i == index){
            s = &p->data;
            break;
        }
        i++;
    }
    if(s == NULL) return;
    char* area = s->area;
    int start_year;
    int count = 0;
    printf("请输入起始年份: ");
    scanf("%d", &start_year);
    for(OwnerNode* p = owner_head; p != NULL; p = p->next){
        if(strstr(p->data.address, area) == NULL) continue;
        int payment = 1;
        for(int j = 0; j < p->data.payment_count; j++){
            if(p->data.pr[j].payment_year >= start_year && p->data.pr[j].payment_status == 0){
                payment = 0;
                break;
            }
        }
        if(!payment) count++;
    }
    printf("%d年及以后存在未缴费记录的业主共%d人\n", start_year, count);
}

// void test01(){
//     struct Staff s;
//     strcpy(s.name,"zhangsan");
//     s.id=1;
//     strcpy(s.password,"12345678");
//     strcpy(s.phonenumber,"12345678910");
//     strcpy(s.area,"A区");
//     s.position=CUSTOMER_SERVICE_SPECIALIST;
//     addStaffTail(s);
// }

// server.c 文件中的 server_system 函数替换为以下内容：
void server_system(){
    // printf("程序运行成功\n");
    load_all_text();
    // test01();
    int index = 0;
    int id;
    int flag = 1;
    while(flag){
        int choice;
        printf("----------登录----------\n");
        printf("1.登录  2.找回密码  0.退出\n");
        printf("请选择: ");
        scanf("%d", &choice);
        switch(choice){
        case 1:
        {
            while(1){
                char password[9] = {0};
                printf("请输入ID: ");
                scanf("%d", &id);
                while(getchar() != '\n'){}
                printf("请输入密码: ");
                fgets(password, 9, stdin);
                int len = strlen(password);
                if(len > 0 && password[len-1] == '\n'){
                    password[len-1] = '\0';
                }
                if((index = staff_login(id, password)) >= 0){
                    printf("登录成功!\n");
                    flag = 0;
                    break;
                }else{
                    printf("ID或密码错误，请重新输入!\n");
                    continue;
                }
            }
            break;
        }
        case 2:
            find_password();
            break;
        case 0:
            printf("退出成功，感谢您的使用!\n");
            return;
        default:
            printf("输入错误!\n");
        }
    }
    int choice;
    while(1){
        show_staff_menu();
        printf("请选择: ");
        scanf("%d", &choice);
        switch(choice){
        case 1:
            mine(index, id);
            break;
        case 2:
            query(index, id);
            break;
        case 3:
            sort(index);
            break;
        case 4:
            statistics(index, id);
            break;
        case 0:
            save_all_text();
            free_all();
            printf("退出成功，感谢您的使用!\n");
            return;
        default:
            printf("输入错误!\n");
        }
    }
    return;
}
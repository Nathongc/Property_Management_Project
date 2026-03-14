#include<stdio.h>
#include<string.h>
#define MAX_STAFF 100
#define MAX_OWNER 10000
#define MAX_RECORD 1000

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
    char password[8];
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

struct Staff staff_list[MAX_STAFF];
struct Owner owner_list[MAX_OWNER];
struct ServiceRecord record_list[MAX_RECORD];
int staff_count=0;
int owner_count=0;
int record_count=0;

void show_staff_menu(){
    printf("******************************\n");
    printf("********物业服务人员菜单********\n");
    printf("********1.我的         ********\n");
    printf("********2.信息查询     ********\n");
    printf("********3.信息排序     ********\n");
    printf("********4.信息统计     ********\n");
    printf("********0.退出         ********\n");
    printf("******************************\n");
}

void show_mine_menu(){
    printf("******************************\n");
    printf("********我的           ********\n");
    printf("********1.查看个人信息  ********\n");
    printf("********2.添加服务记录  ********\n");
    printf("********3.我的服务记录  ********\n");
    printf("********4.修改密码     ********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
}

void show_query_menu(){
    printf("******************************\n");
    printf("********信息查询       ********\n");
    printf("********1.我的负责区域  ********\n");
    printf("********2.业主缴费信息  ********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
}

void show_sort_menu(){
    printf("******************************\n");
    printf("********信息排序       ********\n");
    printf("********0.返回菜单     ********\n");
    printf("******************************\n");
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

const char* get_pos_name(PositionType position){
    switch(position){
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
int staff_login(int id,char* password){
    for(int i=0;i<staff_count;i++){
        if(staff_list[i].id==id&&strcmp(staff_list[i].password,password)==0){
            return i;
        }
    }
    return -1;
}

//查看个人信息
void show_my_info(int index){
    struct Staff* s=&staff_list[index];
    printf("姓名:%s\n",s->name);
    printf("ID:%d\n",s->id);
    printf("负责区域:%s\n",s->area);
    printf("联系电话:%s\n",s->phonenumber);
    printf("职位:%s\n",get_pos_name(s->position));
}

//添加服务记录
void add_record(int staff_id){
    if(record_count>=MAX_RECORD){
        printf("记录已满！\n");
        return;
    }
    struct ServiceRecord r;
    r.staff_id=staff_id;
    printf("请输入业主ID:");
    scanf("%d",&r.owner_id);
    printf("请输入服务内容:");
    fgets(r.service_content,100,stdin);
    int len1=strlen(r.service_content);
    if(len1>0&&r.service_content[len1-1]=='\n'){
        r.service_content[len1-1]='\0';
    }
    printf("请输入日期(如2026-3-8):");
    fgets(r.date,20,stdin);
    int len2=strlen(r.date);
    if(len2>0&&r.date[len2-1]=='\n'){
        r.date[len2-1]='\0';
    }
    record_list[record_count++]=r;
    printf("服务记录添加成功！\n");
}

//查看我的服务记录
void show_my_records(int staff_id){
    int found_count=0;
    for(int i=0;i<record_count;i++){
        if(record_list[i].staff_id==staff_id){
            found_count++;
            printf("%d.\t业主ID:%d\t服务内容:%s\t日期:%s\n",found_count,record_list[i].owner_id,record_list[i].service_content,record_list[i].date);
        }
    }
    if(!found_count){
        printf("暂无记录\n");
    }
}

//修改密码
void change_password(int index,char* new_password){
    strcpy(staff_list[index].password,new_password);
    printf("密码修改成功！\n");
}

//查询负责区域
void query_my_area(int index){
    printf("==========我的负责区域==========");
    printf("区域：%s\n",staff_list[index].area);
    int found_count=0;
    for(int i=0;i<owner_count;i++){
        if(strstr(owner_list[i].address,staff_list[index].area)!=NULL){
            if(!found_count) printf("该区域野猪如下：\n");
            found_count++;
            printf("%d.\t姓名:%s\tID:%d\t地址:%s\n",found_count,owner_list[i].name,owner_list[i].id,owner_list[i].address);
        }
    }
    if(!found_count) printf("该区域暂无业主\n");
}

//查询业主信息
void query_owner_by_name(char* name){
    printf("===========查询结果===========");
    int found_count=0;
    for(int i=0;i<owner_count;i++){
        if(strstr(owner_list[i].name,name)!=NULL){
            found_count++;
            printf("%d.\t姓名:%s\tID:%d\t地址:%s\n",found_count,owner_list[i].name,owner_list[i].id,owner_list[i].address);
        }
    }
    if(!found_count) printf("暂无符合条件的业主");
}

//查询某业主某年是否缴费
void query_payment_by_year(int owner_id,int year){
    for(int i=0;i<owner_count;i++){
        if(owner_list[i].id==owner_id){
            for(int j=0;j<owner_list[i].payment_count;j++){
                if(owner_list[i].pr[j].payment_year==year){
                    printf("业主%s\n%d年:%s\n",owner_list[i].name,year,owner_list[i].pr[j].payment_status?"已缴费":"未缴费");
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
    for(int i=0;i<owner_count;i++){
        if(owner_list[i].id==owner_id){
            printf("业主%s:\n",owner_list[i].name);
            if(owner_list[i].payment_count==0){
                printf("无缴费记录\n");
                return;
            }
            for(int j=0;j<owner_list[i].payment_count;j++){
                printf("%d年:%s\n",owner_list[i].pr[j].payment_year,owner_list[i].pr[j].payment_status?"已缴费":"未缴费");
            }
            return;
        }
    }
    printf("未找到该业主\n");
}

//查询某年未缴费业主
void query_unpaid_by_year(int year){
    int found_count=0;
    for(int i=0;i<owner_count;i++){
        for(int j=0;j<owner_list[i].payment_count;j++){
            if(owner_list[i].pr[j].payment_year==year&&owner_list[i].pr[j].payment_status==0){
                found_count++;
                printf("%d.\t姓名:%s\tID:%d\t地址:%s\n",found_count,owner_list[i].name,owner_list[i].id,owner_list[i].address);
            }
        }
    }
    if(!found_count) printf("%d年无未缴费业主\n",year);
}

//信息统计
//按单一属性统计
void statistics_by_year(int year){
    int count=0;
    for(int i=0;i<owner_count;i++){
        for(int j=0;j<owner_list[i].payment_count;j++){
            if(owner_list[i].pr[j].payment_year==year&&owner_list[i].pr[j].payment_status==0){
                count++;
                break;
            }
        }
    }
    printf("%d年未缴费业主共%d人\n",year,count);
}

//按多属性统计
void statistics_by_year_and_area(int year,char* area){
    int count=0;
    for(int i=0;i<owner_count;i++){
        if(strstr(owner_list[i].address,area)==NULL) continue;
        for(int j=0;j<owner_list[i].payment_count;j++){
            if(owner_list[i].pr[j].payment_year==year&&owner_list[i].pr[j].payment_status==0){
                count++;
                break;
            }
        }
    }
    printf("%s%d年未缴费业主共%d人\n",area,year,count);
}

//预设统计


//按条件统计

int main()
{

    return 0;
}
#include<stdio.h>
#include<string.h>
#define MAX_STAFF 100
#define MAX_RECORD 1000

typedef enum
{
    PROPERTY_MANAGER,
    CUSTOMER_SERVICE_SPECIALIST,
    SECURITY_OFFICER,
    CLEANER,
    BUILDING_BULTER
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
    int phonenumber;
    PositionType position;
    // union{
    //     struct Property_Manager pm;
    //     struct Customer_Service_Specialist css;
    //     struct Security_Officer so;
    //     struct Cleaner cln;
    //     struct Building_Bulter bb;
    // }data;
};

struct ServiceRecord//服务记录
{
    int staff_id;//服务人员id
    int owner_id;//业主id
    char service_content[100];
    char date[20];
};

struct Staff staff_list[MAX_STAFF];
struct ServiceRecord record_list[MAX_RECORD];
int staff_count=0;
int record_count=0;

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

int staff_login(int id,char* password){
    for(int i=0;i<staff_count;i++){
        if(staff_list[i].id==id&&strcmp(staff_list[i].password,password)==0){
            return 1;
        }
    }
    return -1;
}//登录

void change_password(int index,char* new_password){
    strcpy(staff_list[index].password,new_password);
    printf("密码修改成功！");
}//修改密码

// void select_role()
// {
//     printf("****************************************");
//     printf("*****请选择你的身份：                *****");
//     printf("*****1.物业经理                     *****");
//     printf("*****2.客服专员                     *****");
//     printf("*****3.保安                         *****");
//     printf("*****4.清洁工                       *****");
//     printf("*****5.楼栋管家                     *****");
//     printf("*****0.取消                         *****");
//     printf("****************************************");
// }

void add_record(){

}

int main()
{

    return 0;
}
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#define MAX 1000
#define FILENAME "Manager.txt"
typedef struct Imformation Imfor;
typedef struct Owner Owner;
typedef struct Server Server;
typedef struct Person Person;
typedef struct Server_List* S_LIST;
typedef struct Owner_List* O_LIST;
typedef struct ChargeImfor ChargeImfor;
extern S_LIST server_head;
extern O_LIST owner_head;
extern Imfor* imfor;
// 链表头指针

void ShowMenu();
struct Person{//通用
    long long password;//密码
    int M_area;//家庭住址
    char M_name[MAX];//姓名
    char M_sex[MAX];//性别
    int M_age;//年龄
    char Position[MAX];//职位
    int parking_imfor;//停车位占用情况
};
struct ChargeImfor{//业主缴费信息
    int Charge_num;//次数
    int Date[MAX][3];//日期:年月日
};
struct Owner{//业主
    Person base;//基本信息
    ChargeImfor charge;
};
struct Owner_List{
    O_LIST next;
    Owner owner;
};
struct Server{
    Person base;//继承
    char Career[MAX];
    int Area[MAX];
    int Areacount;//负责楼房数目
};
struct Server_List{
    S_LIST next;
    Server server;
};
struct Imformation{
    int Building[MAX];//楼宇
    int Num_parking;//停车位数量
    int parking[MAX];//停车位，0为空闲，1为占用,-1为关闭
    int charging_fee;//收费金额
    int charging_date;//收费周期
};
void Save(S_LIST server_head,O_LIST owner_head);//存入文件、
void Read(S_LIST server_head,O_LIST owner_head);//读取文件
void AddImfor();//添加人员信xi
Imfor ModImfor(O_LIST owner_head,S_LIST server_head);//更改人员信息
S_LIST SERVER_LISTADD(Server* server,S_LIST server_head);//把服务人员信息加入链表
O_LIST OWNER_ADDLIST(Owner* M_owner,O_LIST owner_head);//把业主信息加入链表
S_LIST SERVER_DelImfor(S_LIST server_head);//删除物业管理人员信息
O_LIST OWNER_DelImfor(O_LIST owner_head);//删除业主信息

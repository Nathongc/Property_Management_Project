#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#define MAX 100
typedef struct Imformation Imfor;
typedef struct Owner Owner;
typedef struct Server Server;
typedef struct Person Person;
typedef struct Server_List* S_LIST;
typedef struct Owner_List* O_LIST;
// 链表头指针
S_LIST server_head = NULL;
O_LIST owner_head = NULL;
void ShowMenu();
struct Person{
    int M_area;//家庭住址
    char M_name[MAX];//姓名
    char M_sex[MAX];//性别
    int M_age;//年龄
    char Position[MAX];//职位
};
struct Owner{
    Person base;
    //缴费信息
    //是否缴费
};
struct Owner_List{
    O_LIST next;
    Owner owner;
};
struct Server{
    Person base;//继承
    char Career[MAX];
    int Area[MAX];
};
struct Server_List{
    S_LIST next;
    Server server;
};
struct Imformation{
    int Building;
    int House;
    S_LIST S_head;
    O_LIST O_head;
};
void AddImfor(Person* person,Server* server,Owner* owner);//添加人员信息
void DelImfor();//删除人员信息
void ModImfor();//更改人员信息
S_LIST SERVER_LISTADD(Server* server,S_LIST server_head);//把服务人员信息加入链表
O_LIST OWNER_ADDLIST(Owner* M_owner,O_LIST owner_head);//把业主信息加入链表
void ShowServer_Imfor(S_LIST server_head){
    S_LIST node=server_head;
    while(node->next!=NULL){
        printf("%s %d",node->server.base.M_name,node->server.base.M_age);
        node=node->next;
    }
}
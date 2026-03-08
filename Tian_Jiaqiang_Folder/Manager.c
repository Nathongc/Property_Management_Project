#include "Manager.h"
void ShowMenu(){
    printf("请选择您的身份\n");
    printf("1.管理员 2.物业服务人员 3.业主\n");
}
S_LIST SERVER_LISTADD(Server* M_server,S_LIST server_head){
    S_LIST newnode=(S_LIST)malloc(sizeof(struct Server_List));//初始化新节点
    newnode->server=*M_server;
    newnode->next=NULL;
    if(server_head==NULL){//链表为空
        server_head=newnode;
        return server_head;
    }
    else{
        S_LIST node=server_head;
        while(node->next!=NULL){
            node=node->next;
        }
        node->next=newnode;
        return server_head;
    }
}
O_LIST OWNER_ADDLIST(Owner* M_owner,O_LIST owner_head){
    O_LIST newnode=(O_LIST)malloc(sizeof(struct Owner_List));//初始化新节点
    newnode->owner=*M_owner;
    newnode->next=NULL;
    if(owner_head==NULL){//链表为空
        owner_head=newnode;
        return owner_head;
    }
    else{
        O_LIST node=owner_head;
        while(node->next!=NULL){
            node=node->next;
        }
        node->next=newnode;
        return owner_head;
    }
}
void AddImfor(Person* person,Server* server,Owner* owner){
    char name[MAX];
    int age;
    int area;
    printf("请输入基本信息：\n");
    printf("姓名：\n");
    scanf("%s",name);
    printf("年龄：\n");
    scanf("%d",age);
    printf("性别：1.man 2.woman\n");
    int SEX_choice;
    A:scanf("%d",&SEX_choice);
    switch(SEX_choice){
        case 1:
            strcpy(person->M_sex,"男");
            break;
        case 2:
            strcpy(person->M_sex,"女");
            break;
        default:
            printf("请输入正确的选择！！！");
            goto A;
            break;
    }
    printf("家庭住址：\n");
    scanf("%d",&area);
    person->M_area=area;
    strcpy(person->M_name, name);
    person->M_age=age;
    printf("请输入ta的职位\n");
    printf("1.业务服务人员 2.业主");
    int choice;
    scanf("%d",&choice);
    switch(choice){
        case 1:
            strcpy(person->Position,"Server");
            server->base=*person;
            printf("请输入ta的职位：1.保洁 2.保安 3.管家 4.其他\n");
            int Position_choice;
            scanf("%d",&Position_choice);
            switch(Position_choice){
                case 1:
                    strcpy(server->Career,"保洁");
                    break;
                case 2:
                    strcpy(server->Career, "保安");
                    break;
                case 3:
                    strcpy(server->Career, "管家");
                    break;
                default:
                    strcpy(server->Career, "未知");
                    break;
            }
            printf("请输入ta负责的区域：(输入-1结束输入)");
            int count=0;
            int num;
            while(scanf("%d",&num)){
                if(num==-1){
                    break;
                }
                server->Area[count]=num;
                count++;
            }
            server_head=SERVER_LISTADD(server,server_head);
            break;
        case 2:
            strcpy(person->Position,"Owner");
            owner->base=*person;
            /*此处业主缴费信息等等待后续补充*/
            owner_head=OWNER_ADDLIST(owner,owner_head);
            break;
    }

}
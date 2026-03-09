#include "Manager.h"
S_LIST server_head = NULL;
O_LIST owner_head = NULL;
void Save(S_LIST server_head,O_LIST owner_head){
    FILE *fp;
    fp=fopen(FILENAME,"w");
    if(fp==NULL){
        printf("文件打开遇到错误！\n");
        return;
    }
    fputs("业务管理系统\n",fp);
    S_LIST S_node=server_head;
    if(server_head!=NULL){
        while(S_node!=NULL){
            fprintf(fp,"职位：业务管理人员     姓名：%s    年龄：%d    家庭住址：%d楼    密码：%lld     停车位占用情况：%d   工作：%s",
            S_node->server.base.M_name,S_node->server.base.M_age,S_node->server.base.M_area,S_node->server.base.password,S_node->server.base.parking_imfor,
            S_node->server.Career);
            for(int i=0;i<S_node->server.Areacount;i++){
                fprintf(fp,"    负责区域：%d楼 ",S_node->server.Area[i]);
            }
            S_node=S_node->next;
        }
    }
    O_LIST O_node=owner_head;
    if(owner_head!=NULL){
        while(O_node!=NULL){
            fprintf(fp,"职位：业主     姓名：%s    年龄：%d    家庭住址：%d楼    密码：%lld     停车位占用情况：%d   ",
            O_node->owner.base.M_name,O_node->owner.base.M_age,O_node->owner.base.M_area,O_node->owner.base.password,O_node->owner.base.parking_imfor);
            O_node=O_node->next;
        }
    }
    fclose(fp);
    fp=NULL;
    return;
}
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
    scanf("%d",&age);
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
    B:scanf("%d",&choice);
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
            server->Areacount=count;
            server_head=SERVER_LISTADD(server,server_head);
            break;
        case 2:
            strcpy(person->Position,"Owner");
            owner->base=*person;
            /*此处业主缴费信息等等待后续补充*/
            owner_head=OWNER_ADDLIST(owner,owner_head);
            break;
        default:
            printf("请输入正确的数字！");
            goto B;
            break;
    }
    Save(server_head,owner_head);
}
S_LIST SERVER_DelImfor(S_LIST server_head){
    if(server_head==NULL){
        printf("空链表！");
        return server_head;
    }
    printf("请输入你要删除的对象姓名：");
    char delname[MAX];
    scanf("%s",delname);
    S_LIST node=server_head;
    if(node->next==NULL&&strcmp(node->server.base.M_name,delname)!=0){//整个链表只有一个成员且不匹配
        printf("找不到该对象！\n");
        return server_head;
    }
    if(strcmp(node->server.base.M_name,delname)==0){//第一个成员要被删除
        if(node->next==NULL){//整个链表只有一个成员
            free(server_head); 
            server_head = NULL; // 头指针置空
            printf("删除%s对象成功！\n",delname);
            Save(server_head,owner_head);
            return server_head;
        }
        server_head=node->next;
        free(node);
        node=NULL;
        printf("删除%s对象成功！\n",delname);
        Save(server_head,owner_head);
        return server_head;
    }
    while(node->next!=NULL){
        if(strcmp(node->next->server.base.M_name, delname) == 0){
            S_LIST Delnode=node->next;
            node->next=node->next->next;
            free(Delnode);
            Delnode=NULL;
            printf("删除%s对象成功！\n",delname);
            Save(server_head,owner_head);
            return server_head;
        }
        if (node->next->next == NULL && strcmp(node->next->server.base.M_name, delname) != 0) {
            printf("找不到该对象！\n");
            return server_head;  
        }
        node=node->next;
    }
    printf("找不到该对象！\n");
    Save(server_head,owner_head);
    return server_head;
}
O_LIST OWNER_DelImfor(O_LIST owner_head){
    if(owner_head==NULL){
        printf("空链表！");
        return owner_head;
    }
    printf("请输入你要删除的对象姓名：");
    char delname[MAX];
    scanf("%s",delname);
    O_LIST node=owner_head;
    if(node->next==NULL&&strcmp(node->owner.base.M_name,delname)!=0){//整个链表只有一个成员且不匹配
        printf("找不到该对象！\n");
        return owner_head;
    }
    if(strcmp(node->owner.base.M_name,delname)==0){//第一个成员要被删除
        if(node->next==NULL){//整个链表只有一个成员
            free(owner_head); 
            owner_head = NULL; // 头指针置空
            printf("删除%s对象成功！\n",delname);
            Save(server_head,owner_head);
            return owner_head;
        }
        owner_head=node->next;
        free(node);
        node=NULL;
        printf("删除%s对象成功！\n",delname);
        Save(server_head,owner_head);
        return owner_head;
    }
    while(node->next!=NULL){
        if(strcmp(node->next->owner.base.M_name, delname) == 0){
            O_LIST Delnode=node->next;
            node->next=node->next->next;
            free(Delnode);
            Delnode=NULL;
            printf("删除%s对象成功！\n",delname);
            Save(server_head,owner_head);
            return owner_head;
        }
        if (node->next->next == NULL && strcmp(node->next->owner.base.M_name, delname) != 0) {
            printf("找不到该对象！\n");
            return owner_head;  
        }
        node=node->next;
    }
    printf("找不到该对象！\n");
    Save(server_head,owner_head);
    return owner_head;
}
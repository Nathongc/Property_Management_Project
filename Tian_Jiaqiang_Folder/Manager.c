#include "Manager.h"
S_LIST server_head = NULL;
O_LIST owner_head = NULL;
Imfor* imfor = NULL;
void Save(S_LIST server_head, O_LIST owner_head) {
    FILE* fp;
    fp = fopen(FILENAME, "w");
    if (fp == NULL) {
        printf("文件打开遇到错误！\n");
        return;
    }
    fprintf(fp, "%d %d\n", imfor->charging_date, imfor->charging_fee);
    /*收费周期，单次收费金额*/
    for (int i = 0; i < imfor->Num_parking; i++) {
        fprintf(fp, "%d ", imfor->parking[i]);
    }
    fprintf(fp, "\n");
    S_LIST S_node = server_head;
    if (server_head != NULL) {
        while (S_node != NULL) {
            fprintf(fp, "Server %s %d %d %lld %d %s\n",
                S_node->server.base.M_name, S_node->server.base.M_age, S_node->server.base.M_area, S_node->server.base.password, S_node->server.base.parking_imfor,
                S_node->server.Career);
            /*职位：业务管理人员，姓名，年龄，家庭住址，密码停车位占用情况，工作*/
            for (int i = 0; i < S_node->server.Areacount; i++) {
                fprintf(fp, "%d ", S_node->server.Area[i]);/*负责区域*/
            }
            fprintf(fp, "\n");
            S_node = S_node->next;
        }
    }
    O_LIST O_node = owner_head;
    if (owner_head != NULL) {
        while (O_node != NULL) {
            fprintf(fp, "Owner %s %d %d %lld %d\n",
                O_node->owner.base.M_name, O_node->owner.base.M_age, O_node->owner.base.M_area, O_node->owner.base.password, O_node->owner.base.parking_imfor);
            /*职位：业主,姓名,年龄,家庭住址,密码,停车位占用情况*/
            if (O_node->owner.charge.Charge_num == 0) {
                fprintf(fp, "0\n");
            }
            else {
                for (int i = 0; i < O_node->owner.charge.Charge_num; i++) {
                    for (int j = 0; j < 3; j++) {
                        fprintf(fp, "%d ", O_node->owner.charge.Date[i][j]);
                    }
                }
                fprintf(fp, "\n");
            }
            O_node = O_node->next;
        }
    }
    fclose(fp);
    fp = NULL;
    return;
}
void Imfor_Read() {
    Imfor* temp_imfor = malloc(sizeof(Imfor));
    FILE* fp;
    fp = fopen(FILENAME, "r");
    if (fp == NULL) {
        printf("文件打开遇到错误！\n");
        free(temp_imfor);
        temp_imfor = NULL;
        return;
    }
    if (fscanf(fp, "%d %d\n", &temp_imfor->charging_date, &temp_imfor->charging_fee) != 2) {
        printf("收费信息读取失败！\n");//读取收费信息
        fclose(fp);
        free(temp_imfor);
        temp_imfor = NULL;
        return;
    }
    int parking_count = 0; // 读取到的停车位数量
    while (fscanf(fp, "%d", &temp_imfor->parking[parking_count]) == 1) {
        parking_count++;
        // 遇到换行停止
        if (fgetc(fp) == '\n') {
            break;
        }
    }
    temp_imfor->Num_parking = parking_count;
    if (imfor == NULL) { // 如果全局imfor没初始化，先分配内存
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
    fclose(fp);
}
void ShowMenu() {
    printf("请选择您的身份\n");
    printf("1.管理员 2.物业服务人员 3.业主\n");
}
S_LIST SERVER_LISTADD(Server* M_server, S_LIST server_head) {
    S_LIST newnode = (S_LIST)malloc(sizeof(struct Server_List));//初始化新节点
    newnode->server = *M_server;
    newnode->next = NULL;
    if (server_head == NULL) {//链表为空
        server_head = newnode;
        return server_head;
    }
    else {
        S_LIST node = server_head;
        while (node->next != NULL) {
            node = node->next;
        }
        node->next = newnode;
        return server_head;
    }
}
O_LIST OWNER_ADDLIST(Owner* M_owner, O_LIST owner_head) {
    O_LIST newnode = (O_LIST)malloc(sizeof(struct Owner_List));//初始化新节点
    newnode->owner = *M_owner;
    newnode->next = NULL;
    if (owner_head == NULL) {//链表为空
        owner_head = newnode;
        return owner_head;
    }
    else {
        O_LIST node = owner_head;
        while (node->next != NULL) {
            node = node->next;
        }
        node->next = newnode;
        return owner_head;
    }
}
void AddImfor() {
    printf("请输入要添加的成员类型：\n1.新的人员 2.新的停车位");
    int Newchioce;
    scanf("%d", &Newchioce);
    switch (Newchioce) {
    case 1:
    {
        Person* person = (Person*)malloc(sizeof(Person));
        Server* server = (Server*)malloc(sizeof(Server));
        Owner* owner = (Owner*)malloc(sizeof(Owner));
        if (person == NULL || server == NULL || owner == NULL) {
            printf("内存分配失败！\n");
            if (person) free(person);
            if (server) free(server);
            if (owner) free(owner);
            break;
        }
        char name[MAX];
        int age;
        int area;
        printf("请输入基本信息：\n");
        printf("姓名：\n");
        scanf("%s", name);
        printf("年龄：\n");
        scanf("%d", &age);
        printf("性别：1.man 2.woman\n");
        int SEX_choice;
    A:scanf("%d", &SEX_choice);
        switch (SEX_choice) {
        case 1:
            strcpy(person->M_sex, "男");
            break;
        case 2:
            strcpy(person->M_sex, "女");
            break;
        default:
            printf("请输入正确的选择！！！");
            goto A;
            break;
        }
        printf("家庭住址：\n");
        scanf("%d", &area);
        person->M_area = area;
        strcpy(person->M_name, name);
        person->M_age = age;
        printf("请输入ta的职位\n");
        printf("1.业务服务人员 2.业主");
        int choice;
    B:scanf("%d", &choice);
        switch (choice) {
        case 1:
        {
            strcpy(person->Position, "Server");
            server->base = *person;
            printf("请输入ta的职位：1.保洁 2.保安 3.管家 4.其他\n");
            int Position_choice;
            scanf("%d", &Position_choice);
            switch (Position_choice) {
            case 1:
                strcpy(server->Career, "保洁");
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
            int count = 0;
            int num;
            while (1) {
                scanf("%d", &num);
                if (num == -1) {
                    break;
                }
                server->Area[count] = num;
                count++;
            }
            server->Areacount = count;
            server_head = SERVER_LISTADD(server, server_head);
            free(owner);
            free(person);
            break;
        }
        case 2:
            strcpy(person->Position, "Owner");
            owner->base = *person;
            owner->charge.Charge_num = 0;
            owner_head = OWNER_ADDLIST(owner, owner_head);
            free(server);
            free(person);
            break;
        default:
            printf("请输入正确的数字！\n");
            goto B;
            break;
        }
        break;
    }
    case 2:
    {
        printf("当前停车位数量：%d\n请输入要添加的停车位数目：\n", imfor->Num_parking);
        int New_Parking_Num;
        scanf("%d", &New_Parking_Num);
        if (New_Parking_Num <= 0) {
            printf("停车位数目必须是正数！\n");
            break;
        }
        if (imfor->Num_parking + New_Parking_Num > MAX) {
            printf("停车位数量过大！\n");
            break;
        }
        for (int i = 0; i < New_Parking_Num; i++) {
            imfor->parking[i + imfor->Num_parking] = 0;
        }
        imfor->Num_parking += New_Parking_Num;
        printf("停车位增加成功！\n");
        break;
    }
    default:
        printf("请输入正确的选择！\n");
        break;
    }
    Save(server_head, owner_head);
}
S_LIST SERVER_DelImfor(S_LIST server_head) {
    if (server_head == NULL) {
        printf("空链表！");
        return server_head;
    }
    printf("请输入你要删除的对象姓名：");
    char delname[MAX];
    scanf("%s", delname);
    S_LIST node = server_head;
    if (node->next == NULL && strcmp(node->server.base.M_name, delname) != 0) {//整个链表只有一个成员且不匹配
        printf("找不到该对象！\n");
        return server_head;
    }
    if (strcmp(node->server.base.M_name, delname) == 0) {//第一个成员要被删除
        if (node->next == NULL) {//整个链表只有一个成员
            free(server_head);
            server_head = NULL; // 头指针置空
            printf("删除%s对象成功！\n", delname);
            Save(server_head, owner_head);
            return server_head;
        }
        server_head = node->next;
        free(node);
        node = NULL;
        printf("删除%s对象成功！\n", delname);
        Save(server_head, owner_head);
        return server_head;
    }
    while (node->next != NULL) {
        if (strcmp(node->next->server.base.M_name, delname) == 0) {
            S_LIST Delnode = node->next;
            node->next = node->next->next;
            free(Delnode);
            Delnode = NULL;
            printf("删除%s对象成功！\n", delname);
            Save(server_head, owner_head);
            return server_head;
        }
        if (node->next->next == NULL && strcmp(node->next->server.base.M_name, delname) != 0) {
            printf("找不到该对象！\n");
            return server_head;
        }
        node = node->next;
    }
    printf("找不到该对象！\n");
    Save(server_head, owner_head);
    return server_head;
}
O_LIST OWNER_DelImfor(O_LIST owner_head) {
    if (owner_head == NULL) {
        printf("空链表！");
        return owner_head;
    }
    printf("请输入你要删除的对象姓名：");
    char delname[MAX];
    scanf("%s", delname);
    O_LIST node = owner_head;
    if (node->next == NULL && strcmp(node->owner.base.M_name, delname) != 0) {//整个链表只有一个成员且不匹配
        printf("找不到该对象！\n");
        return owner_head;
    }
    if (strcmp(node->owner.base.M_name, delname) == 0) {//第一个成员要被删除
        if (node->next == NULL) {//整个链表只有一个成员
            free(owner_head);
            owner_head = NULL; // 头指针置空
            printf("删除%s对象成功！\n", delname);
            Save(server_head, owner_head);
            return owner_head;
        }
        owner_head = node->next;
        free(node);
        node = NULL;
        printf("删除%s对象成功！\n", delname);
        Save(server_head, owner_head);
        return owner_head;
    }
    while (node->next != NULL) {
        if (strcmp(node->next->owner.base.M_name, delname) == 0) {
            O_LIST Delnode = node->next;
            node->next = node->next->next;
            free(Delnode);
            Delnode = NULL;
            printf("删除%s对象成功！\n", delname);
            Save(server_head, owner_head);
            return owner_head;
        }
        if (node->next->next == NULL && strcmp(node->next->owner.base.M_name, delname) != 0) {
            printf("找不到该对象！\n");
            return owner_head;
        }
        node = node->next;
    }
    printf("找不到该对象！\n");
    Save(server_head, owner_head);
    return owner_head;
}
Imfor ModImfor(O_LIST owner_head, S_LIST server_head) {//更改信息
A:printf("请输入你要修改的对象类型：\n1.人员信息 2.公共服务\n");
    int choice;
    scanf("%d", &choice);
    switch (choice) {
    case 1:
        printf("请输入你要更改的人员姓名：\n");
        S_LIST S_node = server_head;
        char name[MAX];
        scanf("%s", name);
        break;
    case 2:
        printf("请输入你要更改的服务信息：\n1.停车位 2.收费细则");
        int P_choice;
        scanf("%d", &P_choice);
        switch (P_choice) {
        case 1:
        {
            printf("请输入你要操作的停车位：");
            int modparking = 0;
            scanf("%d", &modparking);
            if (imfor->parking[modparking + 1] == 0) {
                printf("是否要关闭该停车位？1.yes 2.no");
                int mod_parking = 0;
                scanf("%d", &mod_parking);
                switch (mod_parking) {
                case 1:
                    imfor->parking[modparking + 1] = -1;
                    printf("已成功关闭！");
                    break;
                case 2:
                    printf("已取消操作");
                    break;
                default:
                    printf("输入错误，已退出");
                    break;
                }
            }
            else if (imfor->parking[modparking + 1] == 1) {
                printf("无法操作！该停车位正在被占用！");
            }
            else {
                printf("是否要开启该停车位？1.yes 2.no");
                int mod_parking = 0;
                scanf("%d", &mod_parking);
                switch (mod_parking) {
                case 1:
                    imfor->parking[modparking + 1] = 0;
                    printf("已成功开放！");
                    break;
                case 2:
                    printf("已取消操作");
                    break;
                default:
                    printf("输入错误，已退出");
                    break;
                }
            }
        }
        case 2:
            printf("请输入你要更改的收费信息：\n1.收费周期 2.单次收费金额");
            int chioce;
            switch (choice) {
            case 1:
                printf("请输入更改后的收费周期：\n");
                int newdate;
                scanf("%d", &newdate);
                imfor->charging_date = newdate;
                printf("修改成功！");
                break;
            case 2:
                printf("请输入更改后的单次收费金额：\n");
                int newfee;
                scanf("%d", &newfee);
                imfor->charging_fee = newfee;
                printf("修改成功！");
                break;
            default:
                printf("输入错误，已退出");
                break;
            }
        }
        break;
    default:
        printf("请输入正确的选择！");
        goto A;
        break;
    }
}
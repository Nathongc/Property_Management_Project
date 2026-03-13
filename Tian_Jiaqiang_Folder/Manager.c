#include "Manager.h"
Imfor* imfor = NULL;
Person* head = NULL;
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
Imfor* Init_imfor(Imfor* imfor) {//初始化imfor对象
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
Person* Init_Person(Person* person) {//初始化人
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
    /*楼宇数量，收费周期，单次收费金额*/
    for (int i = 0; i < imfor->Num_parking; i++) {
        if (i == 0) {
            fprintf(fp, "%d", imfor->parking[i]);
        }
        else {
            fprintf(fp, " %d", imfor->parking[i]);//停车位情况
        }
    }
    fprintf(fp, "\n");
    Person* node = head;
    while (node != NULL) {
        fprintf(fp, "%s %s %d %d %lld %lld %d %s\n", node->M_name, node->M_sex, node->M_age, node->M_area, node->M_phone_num, node->password,
            node->parking_imfor, node->Career);
        /*姓名，性别，年龄，住址，电话号，密码，停车位占用，工作*/
        if (strcmp(node->Career, "业主") == 0) {//工作区域
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
    if (fscanf(fp, "%d %d %d", &temp_imfor->Num_Building, &temp_imfor->charging_date, &temp_imfor->charging_fee) != 3) {
        printf("文件读取失败！\n");//读取楼宇数量，收费周期，单次收费金额
        fclose(fp);
        free(temp_imfor);
        temp_imfor = NULL;
        return;
    }
    int parking_count = 0; // 读取到的停车位数量
    while (fscanf(fp, "%d", &temp_imfor->parking[parking_count]) == 1) {
        parking_count++;
        // 遇到换行停止
        char c = fgetc(fp);
        if (c == '\n' || c == EOF) {
            break;
        }
        if (c == ' ') {
            ungetc(c, fp);
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
    /*接下来读取人员信息*/
    while (1) {
        char c = fgetc(fp);
        if (c == EOF) {
            break;
        }
        else {
            ungetc(c, fp);
        }
        Person* temp_person = (Person*)malloc(sizeof(Person));
        if (temp_person == NULL) {
            printf("初始化异常！\n");
            fclose(fp);
            return;
        }
        Init_Person(temp_person);/*姓名，性别，年龄，住址，电话号，密码，停车位占用，工作*/
        if (fscanf(fp, "%s %s %d %d %lld %lld %d %s",
            temp_person->M_name,
            temp_person->M_sex,
            &temp_person->M_age,
            &temp_person->M_area,
            &temp_person->M_phone_num,
            &temp_person->password,
            &temp_person->parking_imfor,
            temp_person->Career) != 8) {
            printf("数据读取异常!\n");
            free(temp_person);
            fclose(fp);
            return;
        }
        if (strcmp(temp_person->Career, "业主") == 0) {
            fscanf(fp, "%d", &temp_person->Area_count);
            temp_person->Area[0] = 0;
        }
        else {
            int temp_Area_count = 0;
            while (1) {
                fscanf(fp, "%d", &temp_person->Area[temp_Area_count]);
                temp_Area_count++;
                char c = fgetc(fp);
                if (c == '\n' || c == EOF) {
                    break;
                }
                else {
                    ungetc(c, fp);
                }
            }
            temp_person->Area_count = temp_Area_count;
        }
        fscanf(fp, "%d", &temp_person->Count_charge);
        int temp_Charge_count = 0;
        while (1) {
            fscanf(fp, "%d %d %d", &temp_person->Date_charge[temp_Charge_count][0],
                &temp_person->Date_charge[temp_Charge_count][1],
                &temp_person->Date_charge[temp_Charge_count][2]);
            temp_Charge_count++;
            char c = fgetc(fp);
            if (c == '\n' || c == EOF) {
                break;
            }
            else {
                ungetc(c, fp);
            }
        }
        temp_person->Count_charge = temp_Charge_count;
        head = ADD_TO_LIST(head, temp_person);
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
void AddImfor() {//添加信息
    Person* person = (Person*)malloc(sizeof(Person));
    if (person == NULL) {
        printf("系统异常！");
        return;
    }
    person = Init_Person(person);//初始化
    printf("请输入你要添加的人员姓名：\n");
    char name[MAX];
    scanf("%s", name);
    strcpy(person->M_name, name);
    printf("请输入ta的性别：1.男2.女\n");
    int sex_choice;
    scanf("%d", &sex_choice);
    switch (sex_choice)
    {
    case 1:
        strcpy(person->M_sex, "男");
        break;
    case 2:
        strcpy(person->M_sex, "女");
        break;
    default:
        printf("输入错误！\n");
        free(person);
        break;
    }
    printf("请输入ta的年龄:\n");
    int age;
    scanf("%d", &age);
    person->M_age = age;
    printf("请输入ta的电话号:\n");
    long long phone_num;
    scanf("%lld", &phone_num);
    person->M_phone_num = phone_num;
    printf("请输入ta的家庭住址:\n");
    int area;
    scanf("%d", &area);
    person->M_area = area;
    printf("请输入ta的工作:1.保安2.保洁3.管家4.业主\n");
    int career_choice;
    scanf("%d", &career_choice);
    switch (career_choice)
    {
    case 1:
        strcpy(person->Career, "保安");
        break;
    case 2:
        strcpy(person->Career, "保洁");
        break;
    case 3:
        strcpy(person->Career, "管家");
        break;
    case 4:
        strcpy(person->Career, "业主");
        break;
    default:
        printf("输入错误！已退出\n");
        free(person);
        return;
        break;
    }
    if (strcmp(person->Career, "业主") != 0) {
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
            person->Area[M_count++] = num;
            person->Area_count = M_count;
        }
    }
    head = ADD_TO_LIST(head, person);
    printf("添加成功！\n");
    Save(head);
}
Person* Delimfor(Person* head) {//删除人员
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
    if (node->next == NULL) {//链表仅1成员
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
Person* ModImfor(Person* head, Imfor* imfor) {//更改信息
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
        if (node->next == NULL) {//链表仅一个成员
            if (strcmp(node->M_name, mod_person) != 0) {//不匹配
                printf("找不到对象！\n");
                return head;
            }
            else {//匹配
                Person* newperson = Mod_Person(node);
                head = newperson;
                newperson->next = NULL;
                
                printf("修改成功！\n");
                Save(head);
                return head;
            }
        }
        if (node->next != NULL && strcmp(node->M_name, mod_person) == 0) {//头节点匹配
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
        switch (serve_choice)
        {
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
            switch (charge_choice)
            {
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
            switch (choice)
            {
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
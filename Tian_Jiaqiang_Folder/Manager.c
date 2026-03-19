#include "Manager.h"
Imfor* imfor = NULL;
Person* head = NULL;
year = 0;
month = 0;
day = 0;
void Authorize() {//管理员授权
    printf("请输入管理员ID:\n");
    long long ManagerID;
    while (1) {
        scanf("%lld", &ManagerID);
        if (ManagerID == MANAGER_PASSWORD) {
            printf("授权成功！\n");
            break;
        }
        else {
            printf("输入错误！\n");
        }
    }
}
void Get_time() {//获取时间
    time_t current = time(NULL);
    if (current == NULL) {
        printf("当前时间获取失败！\n");
        exit(1);
    }
    struct tm* local_time = localtime(&current);
    if (local_time == NULL) {
        printf("当前时间获取失败！\n");
        exit(1);
    }
    year = local_time->tm_year + 1900;
    month = local_time->tm_mon + 1;
    day = local_time->tm_mday;
}
void Show_Imfor(Person* head, Imfor* imfor) {//展示信息
    if (head == NULL || imfor == NULL) {
        printf("信息展示出现问题！\n");
        return;
    }
    printf("当前停车位数目：%d\n楼宇数目：%d\n收费周期：%d\n单次收费金额：%d\n", imfor->Num_parking, imfor->Num_Building,
        imfor->charging_date, imfor->charging_fee);
    Person* node = head;
    while (node != NULL) {
        printf("姓名：%-8s 年龄：%-3d 性别：%-8s 电话号：%-13lld 工作：%-8s 住址：%-3d楼\n", node->M_name, node->M_age, node->M_sex, node->M_phone_num,node->Career,node->M_area);
        node = node->next;
    }
}
Imfor* Init_imfor(Imfor* imfor) {//初始化imfor对象
    if (imfor == NULL) {
        imfor = malloc(sizeof(Imfor));
        if (imfor == NULL) {
            printf("异常！\n");
            return imfor;
        }
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
    person->M_age = 0;
    person->M_area = 0;
    person->M_phone_num = 0;
    person->Area_count = 0;
    person->Area[0] = 0;
    strcpy(person->M_name, "默认名字");
    strcpy(person->M_sex, "男");
    strcpy(person->Career, "业主");
    person->next = NULL;
    return person;
}
void Save(Person* head) {//保存到文件
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
        for (int i = 0; i <= node->Count_charge; i++) {
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
void Imfor_Read() {//读文件
    FILE* fp;
    fp = fopen(FILENAME, "r");
    if (fp == NULL) {
        printf("文件打开遇到错误！本次操作将会新建文档\n");
        return;
    }
    if (fscanf(fp, "%d %d %d", &imfor->Num_Building, &imfor->charging_date, &imfor->charging_fee) != 3) {
        printf("文件读取失败！\n");//读取楼宇数量，收费周期，单次收费金额
        fclose(fp);
        return;
    }
    int parking_count = 0; // 读取到的停车位数量
    while (fscanf(fp, "%d", &imfor->parking[parking_count]) == 1) {
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
    imfor->Num_parking = parking_count;
    if (imfor == NULL) { // 如果全局imfor没初始化，先分配内存
        imfor = malloc(sizeof(Imfor));
        if (imfor == NULL) {
            printf("全局imfor内存分配失败！\n");
            fclose(fp);
            return;
        }
    }
    int occupy = 0;
    for (int i = 0; i < imfor->Num_parking; i++) {
        if (imfor->parking[i] == 1 || imfor->parking[i] == -1) {
            occupy++;
        }
    }
    if ((occupy / imfor->Num_parking) > 0.9) {
        printf("90%的停车位被占用或关闭！！！\n");
    }
    /*接下来读取人员信息*/
    while (1) {
        char c = fgetc(fp);
        if (c == EOF) {
            return;
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
        //printf("\n1");
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
        //printf("\n2");
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
        int temp_Charge_count = -1;
        //printf("\n2.5");
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
        //printf("\n3");
    }
    fclose(fp);
}
Person* ADD_TO_LIST(Person* head, Person* person) {//链表节点添加
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
Person* Mod_Person(Person* person) {//更改人员
    Person* new_person = (Person*)malloc(sizeof(Person));
    new_person = Init_Person(new_person);
    *new_person = *person;
    printf("请输入你要修改的信息：\n1.姓名 2.年龄 3.电话号 4.职业 5.家庭住址 6.工作区域\n");
    int mod_Person_choice;
    scanf("%d", &mod_Person_choice);
    switch (mod_Person_choice) {
    case 1:
        printf("请输入新名称：\n");
        char newname[MAX];
        scanf("%s", newname);
        strcpy(new_person->M_name, newname);
        break;
    case 2:
        printf("请输入年龄：\n");
        int newage;
        scanf("%d", &newage);
        new_person->M_age = newage;
        break;
    case 3:
        printf("请输入电话号：\n");
        long long newphonenumber;
        scanf("%lld", &newphonenumber);
        new_person->M_phone_num = newphonenumber;
        break;
    case 4:
    {
        char newcareer[MAX];
        printf("请输入新职业：1.业主 2.管家 3.保安 4.保洁\n");
        int new_career = 0;
        scanf("%d", &new_career);
        switch (new_career) {
        case 1:
            strcpy(newcareer, "业主");
            break;
        case 2:
            strcpy(newcareer, "管家");
            break;
        case 3:
            strcpy(newcareer, "保安");
            break;
        case 4:
            strcpy(newcareer, "保洁");
            break;
        default:
            printf("输入错误，已退出\n");
            free(new_person);
            return NULL;
        }
        strcpy(new_person->Career, newcareer);
        break;
    }
    case 5:
        printf("请输入家庭住址：\n");
        int newarea;
        scanf("%d", &newarea);
        new_person->M_area = newarea;
        break;
    case 6:
        if (strcmp(person->Career, "业主") == 0) {
            printf("业主无工作区域！\n");
            break;
        }
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
                if (newperson == NULL) {
                    return head;
                }
                head = newperson;
                newperson->next = NULL;

                printf("修改成功！\n");
                //Save(head);
                return head;
            }
        }
        if (node->next != NULL && strcmp(node->M_name, mod_person) == 0) {//头节点匹配
            Person* nextnode = head->next;
            Person* newperson = Mod_Person(node);
            if (newperson == NULL) {
                return head;
            }
            head = newperson;
            newperson->next = nextnode;

            printf("修改成功！\n");
            //Save(head);
            return head;
        }
        while (node->next != NULL) {
            if (node->next->next != NULL && strcmp(node->next->M_name, mod_person) == 0) {
                Person* newperson = Mod_Person(node->next);
                if (newperson == NULL) {
                    return head;
                }
                Person* nextnode = node->next->next;
                Person* freenode = node->next;
                node->next = newperson;
                newperson->next = nextnode;

                printf("修改成功！\n");
                //Save(head);
                return head;
            }
            if (node->next->next == NULL && strcmp(node->next->M_name, mod_person) == 0) {
                Person* newperson = Mod_Person(node->next);
                if (newperson == NULL) {
                    return head;
                }
                Person* freenode = node->next;
                node->next = newperson;
                newperson->next = NULL;

                printf("修改成功！\n");
                //Save(head);
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
void FindPerson() {//查询人员信息
    printf("请输入查询方式：\n1.按姓名 2.按电话\n");
    int choice;
    scanf("%d", &choice);
    switch (choice) {
    case 1:
        printf("请输入ta的姓名:\n");
        char find_name[MAX];
        scanf("%s", find_name);
        Person* node = head;
        if (node == NULL) {
            printf("无人员！\n");
            return;
        }
        while (node != NULL) {
            if (strcmp(node->M_name, find_name) == 0) {
                printf("姓名：%s 年龄:%d 性别：%s 电话号：%lld 住址：%d 工作：%s 停车位占用:%d\n",
                    node->M_name, node->M_age, node->M_sex, node->M_phone_num, node->M_area, node->Career, node->parking_imfor);
                if (strcmp(node->Career, "业主") != 0) {
                    printf("工作区域：");
                    for (int i = 0; i < node->Area_count; i++) {
                        if (i == 0) {
                            printf("%d", node->Area[i]);
                        }
                        else {
                            printf(" %d", node->Area[i]);
                        }
                    }
                    printf("楼\n");
                }
                printf("缴费记录：");
                if (node->Count_charge == 0) {
                    printf("无缴费记录\n");
                }
                else if (strcmp(node->Career, "业主") != 0) {
                    printf("服务人员无需缴费\n");
                }
                else {
                    for (int i = 1; i < node->Count_charge; i++) {
                        printf("第%d次，%d年%d月%d日", i, node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
                    }
                }
                return;
            }
            node = node->next;
        }
        printf("找不到对象！\n");
        break;
    case 2:
    {
        printf("请输入ta的电话号码：\n");
        long long find_phonenum;
        scanf("%lld", &find_phonenum);
        Person* node = head;
        if (node == NULL) {
            printf("无人员！\n");
            return;
        }
        while (node != NULL) {
            if (node->M_phone_num == find_phonenum) {
                printf("姓名：%s 年龄:%d 性别：%s 电话号：%lld 住址：%d 工作：%s 停车位占用:%d\n",
                    node->M_name, node->M_age, node->M_sex, node->M_phone_num, node->M_area, node->Career, node->parking_imfor);
                if (strcmp(node->Career, "业主") != 0) {
                    printf("工作区域：");
                    for (int i = 0; i < node->Area_count; i++) {
                        if (i == 0) {
                            printf("%d", node->Area[i]);
                        }
                        else {
                            printf(" %d", node->Area[i]);
                        }
                    }
                    printf("楼\n");
                }
                printf("缴费记录：");
                if (node->Count_charge == 0) {
                    printf("无缴费记录\n");
                }
                else if (strcmp(node->Career, "业主") != 0) {
                    printf("服务人员无需缴费\n");
                }
                else {
                    for (int i = 1; i < node->Count_charge; i++) {
                        printf("第%d次，%d年%d月%d日", i, node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
                    }
                }
                return;
            }
            node = node->next;
        }
        printf("找不到对象！\n");
        break;
    }
    default:
        printf("输入错误！已退出\n");
        break;
    }
}
void Sta_Imfor(Person* head) {//统计人员信息
    if (head == NULL) {
        printf("无法统计！\n");
        return;
    }
    printf("请输入你要统计的维度:\n1.职业 2.年龄 3.性别 4.住址 5.本年度已缴费 6.本年度未缴费\n");
    int choice;
    Person* node = head;
    int sum = 0;
    scanf("%d", &choice);
    switch (choice) {
    case 1:
        printf("请输入你要统计的职业：\n1.业主 2.管家 3.保安 4.保洁\n");
        char sta_career[MAX];
        int career_choice;
        scanf("%d", &career_choice);
        switch (career_choice) {
        case 1:
            strcpy(sta_career, "业主");
            break;
        case 2:
            strcpy(sta_career, "管家");
            break;
        case 3:
            strcpy(sta_career, "保安");
            break;
        case 4:
            strcpy(sta_career, "保洁");
            break;
        default:
            printf("输入错误！已退出\n");
            break;
        }
        if (strcmp(sta_career, "业主") == 0) {
            while (node != NULL) {
                if (strcmp(node->Career, sta_career) == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
        }
        if (strcmp(sta_career, "管家") == 0) {
            while (node != NULL) {
                if (strcmp(node->Career, sta_career) == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
        }
        if (strcmp(sta_career, "保安") == 0) {
            while (node != NULL) {
                if (strcmp(node->Career, sta_career) == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
        }
        if (strcmp(sta_career, "保洁") == 0) {
            while (node != NULL) {
                if (strcmp(node->Career, sta_career) == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
        }
        break;
    case 2:
        printf("请输入查找年龄的下限:\n");
        int min_age;
        scanf("%d", &min_age);
        printf("请输入查找年龄的上限:\n");
        int max_age;
        scanf("%d", &max_age);
        if (max_age < min_age) {
            printf("下限大于上限!\n");

        }
        while (node != NULL) {
            if (node->M_age >= min_age && node->M_age <= max_age) {
                printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                sum++;
            }
            node = node->next;
        }
        printf("统计结束，共%d人\n", sum);
        break;
    case 3:
        printf("请输入要统计的性别：1.男 2.女\n");
        int sta_sex_choice;
        scanf("%d", &sta_sex_choice);
        switch (sta_sex_choice)
        {
        case 1:
            while (node != NULL) {
                if (strcmp(node->M_sex, "男") == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
            break;
        case 2:
            while (node != NULL) {
                if (strcmp(node->M_sex, "女") == 0) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
                node = node->next;
            }
            printf("统计结束，共%d人\n", sum);
            break;
        default:
            printf("输入错误!已退出\n");
            break;
        }
        break;
    case 4:
        printf("请输入要统计的楼宇：\n");
        int sta_building;
        scanf("%d", &sta_building);
        while (node != NULL) {
            if (node->M_area == sta_building) {
                printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                sum++;
            }
            node = node->next;
        }
        printf("统计结束，共%d人\n", sum);
        break;
    case 5:
        printf("本年度未缴费的成员如下：\n");
        while (node != NULL) {
            if (strcmp(node->Career, "业主") == 0) {
                if ((year - node->Date_charge[node->Count_charge][0]) * 12 + (node->Date_charge[node->Count_charge][1] - month) > imfor->charging_date) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
            }
            node = node->next;
        }
        printf("统计结束，共%d人\n", sum);
        break;
    case 6:
        printf("本年度已缴费的成员如下：\n");
        while (node != NULL) {
            if (strcmp(node->Career, "业主") == 0) {
                if ((year - node->Date_charge[node->Count_charge][0]) * 12 + (node->Date_charge[node->Count_charge][1] - month) <= imfor->charging_date) {
                    printf("姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career);
                    sum++;
                }
            }
            node = node->next;
        }
        printf("统计结束，共%d人\n", sum);
        break;
    default:
        printf("输入错误！已退出\n");
        break;
    }
    return;
}
void maintain_password(Person* head) {//密码维护
    Person* node = head;
    while (node != NULL) {
        printf("姓名：%s 电话：%lld 密码：%lld\n", node->M_name, node->M_phone_num, node->password);
        node = node->next;
    }
    printf("请输入你要操作的对象姓名：\n");
    char doname[MAX];
    scanf("%s", doname);
    Person* newnode = head;
    while (newnode != NULL) {
        if (strcmp(newnode->M_name, doname) == 0) {
            printf("姓名：%s 电话：%lld 密码：%lld\n", newnode->M_name, newnode->M_phone_num, newnode->password);
            printf("1.修改 2.重置");
            int choice;
            scanf("%d", &choice);
            switch (choice) {
            case 1:
                printf("请输入更改后的密码：\n");
                long long newpassword;
                scanf("%lld", &newpassword);
                printf("请确认新密码:\n");
                long long rel_newpassword;
                scanf("%lld", &rel_newpassword);
                if (newpassword == rel_newpassword) {
                    newnode->password = newpassword;
                    printf("修改成功！\n");
                }
                else {
                    printf("修改失败！\n");
                }
                break;
            case 2:
                printf("是否重置？1.yes 2.no\n");
                int M_choice;
                scanf("%d", &M_choice);
                switch (M_choice)
                {
                case 1:
                    newnode->password = 11111111;
                    printf("重置成功！\n");
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
                printf("输入错误，已退出\n");
                break;
            }
            Save(head);
            return;
        }
        newnode = newnode->next;
    }
    printf("找不到对象！\n");
    return;
}
void Generate() {//生成账单(新增）
    int M_count = 0;
    Person* fir_node = head;
    while (fir_node != NULL) {
        if (strcmp(fir_node->Career, "业主") == 0 && fir_node->Count_charge != 0) {
            M_count++;
        }
        fir_node = fir_node->next;
    }
    if (!M_count) {
        printf("文件无可写内容！\n");
        return;
    }
    if (head == NULL) {
        printf("账单生成失败!\n");
        return;
    }
    FILE* fp;
    fp = fopen(BILL, "w");
    if (fp == NULL) {
        printf("文件生成失败!\n");
        return;
    }
    Person* node = head;
    while (node != NULL) {
        if (strcmp(node->Career, "业主") == 0 && node->Count_charge != 0) {
            fprintf(fp, "业主:%s\n", node->M_name);
            for (int i = node->Count_charge; i >= 1; i--) {
                fprintf(fp, "缴费次数:%d 时间：%d/%d/%d\n", i, node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
            }
        }
        node = node->next;
    }
    printf("生成账单到本地成功！\n");
    fclose(fp);
}
Person* Sort_Person(Person* head) {//排序
    int node_count = 0;
    Person* node = head;
    Person* array[MAX];
    if (head == NULL) {
        printf("链表为空！无法排序!\n");
        return head;
    }
    while (node != NULL) {
        array[node_count] = node;
        node_count++;
        node = node->next;
    }
    printf("请输入排序依据:\n1.年龄2.姓名3.职业+年龄4.职业+姓名5.家庭住址(升序)\n");
    int sort_choice;
    scanf("%d", &sort_choice);
    switch (sort_choice) {
    case 1:
    {
        printf("请输入排序方式：\n1.升序2.降序\n");
        int measure;
        scanf("%d", &measure);
        switch (measure) {
        case 1:
            for (int i = 0; i < node_count - 1; i++) {
                for (int j = 0; j < node_count - i - 1; j++) {
                    if (array[j]->M_age > array[j + 1]->M_age) {
                        Person* temp = array[j];
                        array[j] = array[j + 1];
                        array[j + 1] = temp;
                    }
                }
            }
            break;
        case 2:
            for (int i = 0; i < node_count - 1; i++) {
                for (int j = 0; j < node_count - i - 1; j++) {
                    if (array[j]->M_age < array[j + 1]->M_age) {
                        Person* temp = array[j];
                        array[j] = array[j + 1];
                        array[j + 1] = temp;
                    }
                }
            }
            break;
        default:
            printf("输入错误！已退出\n");
            return head;
            break;
        }
        break;
    }
    case 2:
        for (int i = 0; i < node_count - 1; i++) {
            for (int j = 0; j < node_count - i - 1; j++) {
                if (strcmp(array[j]->M_name, array[j + 1]->M_name) > 0) {
                    Person* temp = array[j];
                    array[j] = array[j + 1];
                    array[j + 1] = temp;
                }
            }
        }
        break;
    case 3:
    {
        printf("请输入排序方式：\n1.年龄升序2.年龄降序\n");
        int measure;
        scanf("%d", &measure);
        switch (measure) {
        case 1:
            for (int i = 0; i < node_count - 1; i++) {
                for (int j = 0; j < node_count - i - 1; j++) {
                    if (strcmp(array[j]->Career, array[j + 1]->Career) != 0) {
                        if (strcmp(array[j]->Career, array[j + 1]->Career) > 0) {
                            Person* temp = array[j];
                            array[j] = array[j + 1];
                            array[j + 1] = temp;
                        }
                    }
                    else {
                        if (array[j]->M_age > array[j + 1]->M_age) {
                            Person* temp = array[j];
                            array[j] = array[j + 1];
                            array[j + 1] = temp;
                        }
                    }
                }
            }
            break;
        case 2:
            for (int i = 0; i < node_count - 1; i++) {
                for (int j = 0; j < node_count - i - 1; j++) {
                    if (strcmp(array[j]->Career, array[j + 1]->Career) != 0) {
                        if (strcmp(array[j]->Career, array[j + 1]->Career) > 0) {
                            Person* temp = array[j];
                            array[j] = array[j + 1];
                            array[j + 1] = temp;
                        }
                    }
                    else {
                        if (array[j]->M_age < array[j + 1]->M_age) {
                            Person* temp = array[j];
                            array[j] = array[j + 1];
                            array[j + 1] = temp;
                        }
                    }
                }
            }
            break;
        default:
            printf("输入错误！已退出\n");
            return head;
            break;
        }
        break;
    }
    case 4:
        for (int i = 0; i < node_count - 1; i++) {
            for (int j = 0; j < node_count - i - 1; j++) {
                if (strcmp(array[j]->Career, array[j + 1]->Career) != 0) {
                    if (strcmp(array[j]->Career, array[j + 1]->Career) > 0) {
                        Person* temp = array[j];
                        array[j] = array[j + 1];
                        array[j + 1] = temp;
                    }
                }
                else {
                    if (strcmp(array[j]->M_name, array[j + 1]->M_name) > 0) {
                        Person* temp = array[j];
                        array[j] = array[j + 1];
                        array[j + 1] = temp;
                    }
                }
            }
        }
        break;
    case 5:
    {
        for (int i = 0; i < node_count - 1; i++) {
            for (int j = 0; j < node_count - i - 1; j++) {
                if (array[j]->M_area > array[j + 1]->M_area) {
                    Person* temp = array[j];
                    array[j] = array[j + 1];
                    array[j + 1] = temp;
                }
            }
        }
        break;
    }
    default:
        printf("输入错误，已退出\n");
        return head;
        break;
    }
    head = array[0];
    for (int i = 1; i < node_count; i++) {
        array[i - 1]->next = array[i];
    }
    array[node_count - 1]->next = NULL;
    Save(head);
    printf("排序成功！请回到主菜单按1查看结果\n");
    return head;
}
void Backup(Imfor* imfor,Person* head){
    FILE *fp;
    fp=fopen(BACKFILENAME,"w");
    if(imfor==NULL){
        printf("备份服务信息失败！\n");
    }
    fprintf(fp,"收费信息：\n收费周期：%d月 收费金额：%d元\n",imfor->charging_date,imfor->charging_fee);
    fprintf(fp,"楼宇数目：%d\n",imfor->Num_Building);
    int unuse=0;
    for(int i=0;i<imfor->Num_parking;i++){
        if(imfor->parking[i]==-1){
            unuse++;
        }
    }
    fprintf(fp,"停车位信息：\n停车位总数:%d\n 不可用个数:%d\n人员信息:\n",imfor->Num_parking,unuse);
    Person* node=head;
    if(head==NULL){
        printf("备份人员信息失败！\n");
    }
    while(node!=NULL){
        fprintf(fp,"姓名：%s 年龄：%d 性别：%s 住址：%d 电话号：%lld 工作：%s 密码：%lld\n", node->M_name, node->M_age, node->M_sex, node->M_area, node->M_phone_num, node->Career
        ,node->password);
        if(strcmp(node->Career,"业主")!=0){
            fprintf(fp,"工作区域：\n");
            for (int i = 0; i < node->Area_count; i++) {
                if (i == 0) {
                    fprintf(fp,"%d", node->Area[i]);
                }
                else {
                    fprintf(fp," %d", node->Area[i]);
                }
            }
            fprintf(fp,"楼\n");
        }
        fprintf(fp,"缴费记录：\n");
        if (node->Count_charge == 0) {
            fprintf(fp,"无缴费记录\n");
        }
        else if (strcmp(node->Career, "业主") != 0) {
            fprintf(fp,"服务人员无需缴费\n");
        }
        else {
            for (int i = 1; i < node->Count_charge; i++) {
                fprintf(fp,"第%d次，%d年%d月%d日", i, node->Date_charge[i][0], node->Date_charge[i][1], node->Date_charge[i][2]);
            }
            fprintf(fp,"\n");
        }
        node=node->next;
    }
    printf("备份成功!\n");
    fclose(fp);
}
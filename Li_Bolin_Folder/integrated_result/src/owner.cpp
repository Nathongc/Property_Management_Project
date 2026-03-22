#include "../include/owner.h" // 假设头文件路径正确

// 定义属于 owner.c 模块的独立全局变量
yezhu* owner_head123 = NULL;
yezhu* owner_prear123 = NULL;
fee* owner_head1123 = NULL;
fee* owner_prear1123 = NULL;
yezhu* owner_current123 = NULL;

/// 启动登录和注册
/// ////////////////////////////////////////////////////////////////
void removen(char* str) // 去换行符
{
    char* pos = strchr(str, '\n'); // 修复：使用转义字符 \n
    if (pos != NULL)
    {
        *pos = '\0';
    }
}

void freeList()
{
    yezhu* current_node = owner_head123; // 使用 owner_ 前缀的变量
    yezhu* nextnode = NULL;
    while (current_node != NULL)
    {
        nextnode = current_node->next;
        free(current_node);
        current_node = nextnode;
    }
    owner_head123 = NULL; // 使用 owner_ 前缀的变量
    owner_prear123 = NULL; // 使用 owner_ 前缀的变量
    
    // 释放缴费链表
    fee* current_fee = owner_head1123; // 使用 owner_ 前缀的变量
    fee* nextfee = NULL;
    while (current_fee != NULL)
    {
        nextfee = current_fee->next1;
        free(current_fee);
        current_fee = nextfee;
    }
    owner_head1123 = NULL; // 使用 owner_ 前缀的变量
    owner_prear1123 = NULL; // 使用 owner_ 前缀的变量
}

int loadfee() // 加载缴费到链表
{
    FILE* fp = fopen("fee.txt", "r");
    if (fp == NULL)
    {
        // 文件不存在可能是正常的（第一次运行），不打印错误以免干扰，直接返回
        return 1;
    }
    char line[200]; // 存整行数据
    while (fgets(line, 200, fp) != NULL)
    {
        removen(line);
        if (strlen(line) == 0) continue;
        fee* newnode = (fee*)malloc(sizeof(fee));
        if (newnode == NULL)
        {
            printf("内存分配失败！\n");
            fclose(fp);
            return 1;
        }
        char linecopy[200];
        strcpy(linecopy, line);
        char* add1 = strtok(linecopy, ",");
        char* add2 = strtok(NULL, ",");
        char* add3 = strtok(NULL, ",");
        char* year = strtok(NULL, ",");
        char* month = strtok(NULL, ",");
        char* day = strtok(NULL, ",");
        char* sum = strtok(NULL, ",");
        char* yn = strtok(NULL, ",");
        char* year2 = strtok(NULL, ",");
        char* month2 = strtok(NULL, ",");
        char* day2 = strtok(NULL, ",");
        if (!add1 || !add2 || !add3 || !year || !month || !day || !sum || !yn || !year2 || !month2 || !day2)
        {
            printf("跳过错误行：%s（字段缺失/格式错误）\n", line);
            free(newnode);
            continue;
        }
        strcpy(newnode->add1, add1);
        strcpy(newnode->add2, add2);
        strcpy(newnode->add3, add3);
        strcpy(newnode->sum, sum);
        strcpy(newnode->yn, yn);
        sprintf(newnode->date, "%s,%s,%s", year, month, day);
        sprintf(newnode->date2, "%s,%s,%s", year2, month2, day2);
        owner_prear1123->next1 = newnode; // 使用 owner_ 前缀的变量
        owner_prear1123 = newnode; // 使用 owner_ 前缀的变量
        owner_prear1123->next1 = NULL; // 使用 owner_ 前缀的变量
    }
    fclose(fp);
    return 0;
}

int loadusers() // 加载用户到链表
{
    FILE* fp = fopen("user.txt", "r");
    if (fp == NULL)
    {
        return 1;
    }
    char line[100]; // 存整行数据
    while (fgets(line, 100, fp) != NULL)
    {
        removen(line);
        if (strlen(line) == 0) continue;
        yezhu* newnode = (yezhu*)malloc(sizeof(yezhu));
        if (newnode == NULL)
        {
            printf("内存分配失败！\n");
            fclose(fp);
            return 1;
        }
        char linecopy[100];
        strcpy(linecopy, line);
        char* add1 = strtok(linecopy, ",");
        char* add2 = strtok(NULL, ",");
        char* add3 = strtok(NULL, ",");
        char* name = strtok(NULL, ",");
        char* pwd = strtok(NULL, ",");
        char* tel = strtok(NULL, ",");
        if (!add1 || !add2 || !add3 || !name || !pwd)
        {
            printf("跳过错误行：%s（字段缺失/格式错误）\n", line);
            free(newnode);
            continue;
        }
        strcpy(newnode->add1, add1);
        strcpy(newnode->add2, add2);
        strcpy(newnode->add3, add3);
        strcpy(newnode->name, name);
        strcpy(newnode->pwd, pwd);
        strcpy(newnode->tel, tel);
        owner_prear123->next = newnode; // 使用 owner_ 前缀的变量
        owner_prear123 = newnode; // 使用 owner_ 前缀的 variable
        owner_prear123->next = NULL; // 使用 owner_ 前缀的 variable
    }
    fclose(fp);
    return 0;
}

void qidong()
{
    printf("*************************\n");
    printf("*********请选择**********\n");
    printf("*********1__登录*********\n");
    printf("*********2__退出*********\n");
    printf("*********3__注册*********\n");
    printf("***4__找回用户名和密码***\n");
}

// 操作页面
void caozuo()
{
    printf("\n==================== 业主功能菜单 ====================\n");
    printf("1.修改信息\n");
    printf("2.缴费查询\n");
    printf("3.缴费\n");
    printf("4.显示所有缴费记录\n");
    printf("5.查询服务人员\n");
    printf("6.信息排序\n");
    printf("7.缴费统计\n");
    printf("0.退出登录\n");
    printf("======================================================\n");
    printf("请选择\n");
}

// 注册页面
int zhuce()
{
    char add1[20]; // 楼号
    char add2[20]; // 楼层
    char add3[20]; // 户号
    char tel[30];  // 电话
    char name[30];
    char pwd[30];
    int tmp = 0;
start1:
    printf("请输入用户楼号（如\"123\"，1-999 内）");
    scanf("%10s", add1);
    tmp = checkadd(add1, 4);
    if (tmp == 1)
    {
        printf("格式错误\n");
        goto start1;
    }
start2:
    printf("请输入用户楼层（如\"10\"，1-99 内）");
    scanf("%10s", add2);
    tmp = checkadd(add2, 3);
    if (tmp == 1)
    {
        printf("格式错误\n");
        goto start2;
    }
start3:
    printf("请输入用户户号（如\"123\"，1-999 内）");
    scanf("%10s", add3);
    tmp = checkadd(add3, 4);
    if (tmp == 1)
    {
        printf("格式错误\n");
        goto start3;
    }
    tmp = checkadd2(add1, add2, add3, NULL); // 传入 NULL，因为此时还未切换 current
    if (tmp == 1)
    {
        return 1;
    }
start4:
    printf("请输入用户名（1-20 字）");
    scanf("%30s", name);
    tmp = checkname2(name);
    if (tmp == 1)
    {
        goto start4;
    }
    tmp = checkname(name);
    if (tmp == 1)
    {
        goto start4;
    }
start5:
    printf("请输入密码（5-20 字）");
    scanf("%30s", pwd);
    tmp = checkpwd2(pwd);
    if (tmp == 1)
    {
        goto start5;
    }
    yezhu* newnode = (yezhu*)malloc(sizeof(yezhu));
    if (newnode == NULL)
    {
        printf("内存分配失败！\n");
        return 1;
    }
    printf("请输入联系电话电话");
    scanf("%20s", tel);
    strcpy(newnode->add1, add1);
    strcpy(newnode->add2, add2);
    strcpy(newnode->add3, add3);
    strcpy(newnode->name, name);
    strcpy(newnode->pwd, pwd);
    strcpy(newnode->tel, tel);
    newnode->next = NULL;
    owner_prear123->next = newnode; // 使用 owner_ 前缀的 variable
    owner_prear123 = newnode; // 使用 owner_ 前缀的 variable
    owner_current123 = newnode; // 切换当前业主 (owner_ 前缀)
    save(owner_prear123); // 保存新节点 (owner_ 前缀)
    printf("注册成功\n");
    return 0;
}

// 清空残留字符
void clean()
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

int checkname(const char* name) // 查重名（用户名）
{
    yezhu* cur = owner_head123->next; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if (strcmp(cur->name, name) == 0)
        {
            printf("错误：该用户名已被占用！\n");
            return 1;
        }
        cur = cur->next;
    }
    return 0; // 正常返回 0
}

int checkname2(const char* name) // 检查用户名长度 (1-20)
{
    int len = strlen(name);
    if (len < 1 || len > 20)
    {
        printf("错误：用户名长度必须在 1-20 个字符之间！\n");
        clean();
        return 1;
    }
    return 0;
}

yezhu* checkname3(const char* name) // 登录名字
{
    yezhu* cur = owner_head123->next; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if (strcmp(cur->name, name) == 0)
        {
            return cur;
        }
        cur = cur->next;
    }
    return NULL; // 正常返回 0
}

int checkadd2(const char* add1, const char* add2, const char* add3, yezhu* current_check) // 查重名（地址）
{
    yezhu* cur = owner_head123->next; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if ((strcmp(cur->add1, add1) == 0) && (strcmp(cur->add2, add2) == 0) && (strcmp(cur->add3, add3) == 0))
        {
            if (current_check == NULL) // 如果 current_check 为 NULL，说明是注册时检查
            {
                printf("错误：该地址已注册！\n");
                return 1;
            }
            else if (current_check != cur) // 如果 current_check 不为 NULL 且不等于当前找到的节点，说明是修改时检查，且地址已被他人占用
            {
                printf("错误：该地址已注册！\n");
                return 1;
            }
            // 如果 current_check 等于 cur，说明是修改自己的信息，允许
        }
        cur = cur->next;
    }
    return 0; // 正常返回 0
}

int checkadd(const char* add, int n) // 查地址格式
{
    int i = 0;
    while (add[i] != '\0')
    {
        if ((add[i] < 48) || (add[i] > 57))
        {
            return 1;
        }
        i++;
    }
    if (i > (n - 1))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// 保存业主
void save(yezhu* newnode)
{
    FILE* fp = fopen("user.txt", "a");
    if (fp == NULL)
    {
        return;
    }
    fprintf(fp, "%s,%s,%s,%s,%s,%s\n", newnode->add1, newnode->add2, newnode->add3, newnode->name, newnode->pwd, newnode->tel);
    fclose(fp);
}

// 登录
int signin()
{
    while (1)
    {
        yezhu* cur = NULL;
        int tmp = 1;
        char pwd[20];
        char name[20];
        printf("请输入用户名\n");
        scanf("%20s", name);
        printf("请输入密码\n");
        scanf("%20s", pwd);
        cur = checkname3(name);
        tmp = checkpwd(pwd, cur);
        if ((tmp == 0))
        {
            printf("登录成功\n");
            owner_current123 = cur; // 切换当前业主 (owner_ 前缀)
            return 0;
        }
        else
        {
            printf("用户名或密码错误\n");
            return 1;
        }
    }
}

int checkpwd(const char* pwd, yezhu* cur) // 核对密码
{
    if (cur == NULL)
    {
        return 1;
    }
    if (strcmp(cur->pwd, pwd) == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

int checkpwd2(const char* pwd)
{
    int len = strlen(pwd);
    if (len < 5 || len > 20)
    {
        printf("错误：密码长度必须在 5-20 个字符之间！\n");
        clean();
        return 1; // 长度不合法返回 1
    }
    return 0; // 合法返回 0
}

void zhaohui() // 密码找回
{
    char add1[20]; // 楼号
    char add2[20]; // 楼层
    char add3[20]; // 户号
    char tel[30];  // 电话
    printf("请输入用户楼号");
    scanf("%10s", add1);
    printf("请输入用户楼层");
    scanf("%10s", add2);
    printf("请输入用户户号");
    scanf("%10s", add3);
    int temp = 1;
    yezhu* cur = owner_head123->next; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if ((strcmp(cur->add1, add1) == 0) && (strcmp(cur->add2, add2) == 0) && (strcmp(cur->add3, add3) == 0))
        {
            printf("已找到该地址\n");
            temp = 0;
            break;
        }
        cur = cur->next;
    }
    if (temp == 1)
    {
        printf("未找到该地址\n");
        return;
    }
    printf("请输入电话\n");
    scanf("%20s", tel);
    if (strcmp(tel, cur->tel) == 0)
    {
        printf("成功\n");
        printf("用户名：%s\n密码：%s\n", cur->name, cur->pwd);
    }
    else
    {
        printf("电话错误\n");
    }
}

/// 功能
/// ////////////////////////////////////////////////////////////////
// 0.跳转到功能
int tiaozhuan()
{
    int tmp = 0;
    scanf("%d", &tmp);
    if (tmp == 1) // 修改信息
    {
        xiugai(owner_current123); // 传递 owner_ 前缀的 current
        return 0;
    }
    else if (tmp == 2)
    {
        queryMenu(); // 缴费查询
        return 0;
    }
    else if (tmp == 3)
    {
        jiaofei(); // 缴费
        return 0;
    }
    else if (tmp == 4)
    {
        showall(); // 显示所有缴费缴费记录
    }
    else if (tmp == 5)
    {
        showworkers(); // 显示人员
    }
    else if (tmp == 6)
    {
        paixuMenu();
    }
    else if (tmp == 7)
    {
        tongjimenu(); // 统计页面
    }
    else if (tmp == 0)
    {
        return 1;
    }
    else
    {
        printf("输入错误\n");
        return 0;
    }
    return 0;
}

/// 1. 修改信息
void xiugai(yezhu* current_to_modify)
{
    char add1[20]; // 楼号
    char add2[20]; // 楼层
    char add3[20]; // 户号
    char tel[30];  // 电话
    char name[30];
    char pwd[30];
    char b[3];
    while (1)
    {
        printf("地址要修改吗？（Y/N）\n");
        scanf("%2s", b);
        if (b[0] == 'Y')
        {
            printf("请输入用户楼号（如\"123\"，1-999 内）");
            scanf("%10s", add1);
            int tmp = checkadd(add1, 4);
            if (tmp == 1)
            {
                printf("格式错误\n");
                continue;
            }
            printf("请输入用户楼层（如\"10\"，1-99 内）");
            scanf("%10s", add2);
            tmp = checkadd(add2, 3);
            if (tmp == 1)
            {
                printf("格式错误\n");
                continue;
            }
            printf("请输入用户户号（如\"123\"，1-999 内）");
            scanf("%10s", add3);
            tmp = checkadd(add3, 4);
            if (tmp == 1)
            {
                printf("格式错误\n");
                continue;
            }
            tmp = checkadd2(add1, add2, add3, current_to_modify); // 传入当前要修改的节点指针
            if (tmp == 1)
            {
                printf("已被注册\n");
                continue;
            }
            strcpy(current_to_modify->add1, add1);
            strcpy(current_to_modify->add2, add2);
            strcpy(current_to_modify->add3, add3);
            printf("修改成功\n");
            break;
        }
        else if (b[0] == 'N')
        {
            break;
        }
        else
        {
            printf("不正确，请重新输入\n");
        }
    }
    while (1)
    {
        printf("用户名要修改吗？（Y/N）\n");
        scanf("%2s", b);
        if (b[0] == 'Y')
        {
            printf("请输入\n");
            scanf("%s", name);
            int tmp1 = checkname2(name);
            if (tmp1 == 1)
            {
                continue;
            }
            int tmp2 = checkname(name); // 检查新用户名是否与其他用户的用户名重复
            if (tmp2 == 0) // 如果不重复
            {
                strcpy(current_to_modify->name, name);
                printf("修改成功\n");
                break;
            }
            else
            {
                continue; // 重复则重新输入
            }
        }
        else if (b[0] == 'N')
        {
            break;
        }
        else
        {
            printf("不正确，请重新输入\n");
        }
    }
    while (1)
    {
        printf("密码要修改吗？（Y/N）\n");
        scanf("%2s", b);
        if (b[0] == 'Y')
        {
            printf("请输入\n");
            scanf("%s", pwd);
            strcpy(current_to_modify->pwd, pwd);
            printf("修改成功\n");
            break;
        }
        else if (b[0] == 'N')
        {
            break;
        }
        else
        {
            printf("不正确，请重新输入\n");
        }
    }
    while (1)
    {
        printf("电话要修改吗？（Y/N）\n");
        scanf("%2s", b);
        if (b[0] == 'Y')
        {
            printf("请输入\n");
            scanf("%s", tel);
            strcpy(current_to_modify->tel, tel);
            printf("修改成功\n");
            break;
        }
        else if (b[0] == 'N')
        {
            break;
        }
        else
        {
            printf("不正确，请重新输入\n");
        }
    }
    clean();
}

// 2.缴费查询（未缴费）
void check()
{
    int hasUnpaid = 0; // 是否有未缴费
    int sumall = 0;
    int i = 1;
    int tmp = 1;
    fee* cur = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if (strcmp(cur->add1, owner_current123->add1) == 0 && strcmp(cur->add2, owner_current123->add2) == 0 && strcmp(cur->add3, owner_current123->add3) == 0) // 使用 owner_ 前缀的 variable
        {
            tmp = 0;
            char year[5], month[3], day[3];
            char datecopy[20];
            strcpy(datecopy, cur->date);
            char* y = strtok(datecopy, ",");
            char* m = strtok(NULL, ",");
            char* d = strtok(NULL, ",");
            strcpy(year, y);
            strcpy(month, m);
            strcpy(day, d);
            char year2[5], month2[3], day2[3];
            char date2copy[20];
            strcpy(date2copy, cur->date2);
            char* y2 = strtok(date2copy, ",");
            char* m2 = strtok(NULL, ",");
            char* d2 = strtok(NULL, ",");
            strcpy(year2, y2);
            strcpy(month2, m2);
            strcpy(day2, d2);
            if (strcmp(cur->yn, "n") == 0)
            {
                printf("NO.%d", i);
                printf("_________________________\n");
                printf("账单时间：%s 年%s 月%s 日\n", year, month, day);
                printf("金额： %s\n", cur->sum);
                printf("是否缴费（y/n）：%s\n", cur->yn);
                printf("缴费时间：%s 年%s 月%s 日\n", year2, month2, day2);
                i++;
                hasUnpaid = 1;
                int money = atoi(cur->sum);
                sumall += money;
            }
        }
        cur = cur->next1;
    }
    if (!hasUnpaid)
    {
        printf("_______________________\n");
        printf("暂无待缴费用！\n");
        printf("_______________________\n");
        return;
    }
    printf("_______________________\n");
    printf("总欠费%d\n", sumall);
    if (tmp == 1)
    {
        printf("错误，未找到该地址\n");
    }
}

// 3.缴费
void jiaofei() // 缴费
{
    int sumall = 0;
    fee* cur = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int hasUnpaid = 0; // 是否需要缴费
    while (cur != NULL)
    {
        if (strcmp(cur->add1, owner_current123->add1) == 0 && strcmp(cur->add2, owner_current123->add2) == 0 && strcmp(cur->add3, owner_current123->add3) == 0) // 使用 owner_ 前缀的 variable
        {
            int money = atoi(cur->sum);
            sumall += money;
            hasUnpaid = 1;
        }
        cur = cur->next1;
    }
    printf("_______________________\n");
    printf("总欠费%d\n", sumall);
    printf("_______________________\n");
    if (!hasUnpaid)
    {
        printf("_______________________\n");
        printf("暂无待缴费用！\n");
        printf("_______________________\n");
        return;
    }
    while (1)
    {
        printf("确定缴费？(Y/N)\n");
        char b[3];
        scanf("%2s", b);
        clean();
        if (b[0] == 'Y')
        {
            cur = owner_head1123->next1; // 使用 owner_ 前缀的 variable
            char currentdate[20];
            // 获取当前系统日期
            getcurrentdate(currentdate);
            while (cur != NULL)
            {
                if (strcmp(cur->add1, owner_current123->add1) == 0 && strcmp(cur->add2, owner_current123->add2) == 0 && strcmp(cur->add3, owner_current123->add3) == 0) // 使用 owner_ 前缀的 variable
                {
                    if (strcmp(cur->yn, "n") == 0)
                    {
                        strcpy(cur->yn, "y");
                        strcpy(cur->date2, currentdate);
                    }
                }
                cur = cur->next1;
            }
            savefee();
            printf("缴费成功");
            return;
        }
        else if (b[0] == 'N')
        {
            printf("已取消缴费！\n");
            return;
        }
        else
        {
            printf("输入错误\n");
        }
    }
}

void savefee() // 保存缴费
{
    FILE* fp = fopen("fee.txt", "w"); // 整个重写
    if (fp == NULL)
    {
        printf("保存失败！\n");
        return;
    }
    fee* cur = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        char year[5], month[3], day[3];
        char year2[5], month2[3], day2[3];
        sscanf(cur->date, "%[^,],%[^,],%s", year, month, day); //%[^,]是按","分割字串
        sscanf(cur->date2, "%[^,],%[^,],%s", year2, month2, day2);
        fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", cur->add1, cur->add2, cur->add3, year, month, day, cur->sum, cur->yn, year2, month2, day2);
        cur = cur->next1;
    }
    fclose(fp);
    printf("缴费记录已同步到文件！\n");
}

void getcurrentdate(char* dateStr) // 获取当前系统日期
{
    time_t now = time(NULL); // 时间戳 time_t
    struct tm* t = localtime(&now); // 时间戳解析为 struct tm(年月日)
    // 将年月日复制到 char* 里
    sprintf(dateStr, "%d,%d,%d",
            t->tm_year + 1900, // 年份需 +1900
            t->tm_mon + 1,     // 月份从 0 开始，需 +1
            t->tm_mday);       // 日期
}

void showall() // 显示所有缴费记录
{
    int haspaid = 0; // 是否有未缴费
    int sumall = 0;
    int i = 1;
    int tmp = 1;
    fee* cur = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (cur != NULL)
    {
        if (strcmp(cur->add1, owner_current123->add1) == 0 && strcmp(cur->add2, owner_current123->add2) == 0 && strcmp(cur->add3, owner_current123->add3) == 0) // 使用 owner_ 前缀的 variable
        {
            tmp = 0;
            char year[5], month[3], day[3];
            char datecopy[20];
            strcpy(datecopy, cur->date);
            char* y = strtok(datecopy, ",");
            char* m = strtok(NULL, ",");
            char* d = strtok(NULL, ",");
            strcpy(year, y);
            strcpy(month, m);
            strcpy(day, d);
            char year2[5], month2[3], day2[3];
            char date2copy[20];
            strcpy(date2copy, cur->date2);
            char* y2 = strtok(date2copy, ",");
            char* m2 = strtok(NULL, ",");
            char* d2 = strtok(NULL, ",");
            strcpy(year2, y2);
            strcpy(month2, m2);
            strcpy(day2, d2);
            if (strcmp(cur->yn, "y") == 0)
            {
                printf("NO.%d", i);
                printf("_________________________\n");
                printf("账单时间：%s 年%s 月%s 日\n", year, month, day);
                printf("金额： %s\n", cur->sum);
                printf("缴费时间：%s 年%s 月%s 日\n", year2, month2, day2);
                i++;
                haspaid = 1;
                int money = atoi(cur->sum);
                sumall += money;
            }
        }
        cur = cur->next1;
    }
    if (!haspaid)
    {
        printf("_______________________\n");
        printf("暂无缴费费用！\n");
        printf("_______________________\n");
        return;
    }
    printf("_______________________\n");
    printf("总缴费%d\n", sumall);
    printf("共缴费%d 次", i);
    if (tmp == 1)
    {
        printf("错误，未找到该地址\n");
    }
}

// 5. 缴费查询总菜单（整合简单/组合/模糊查询）
int queryMenu()
{
    while (1)
    {
        int n = 0;
        printf("__________________________\n");
        printf("1.按缴费状态查询\n");
        printf("2.按时间查询\n");
        printf("3.按金额查询\n");
        printf("4.组合搜索\n");
        printf("5.退出\n");
        printf("请选择\n");
        clean();
        printf("__________________________\n");
        scanf("%d", &n);
        if (n == 1)
        {
            while (1)
            {
                n = 0;
                printf("__________________________\n");
                printf("1.查询未缴费\n");
                printf("2.查询已缴费\n");
                printf("请选择\n");
                printf("__________________________\nd", &n);
                if (n == 1)
                {
                    check();
                    break;
                }
                else if (n == 2)
                {
                    showall();
                    break;
                }
                else
                {
                    printf("输入错误，请重新输入\n");
                }
            }
        }
        else if (n == 2)
        {
            while (1)
            {
                int n = 0;
                char shijian[20];
                printf("__________________________\n");
                printf("1.简单查询\n");
                printf("2.模糊查询\n");
                printf("请选择\n");
                printf("__________________________\n");
                scanf("%d", &n);
                clean();
                printf("请输入时间\n");
                scanf("%20s", shijian);
                if (n == 1)
                {
                    shijian_jingque(shijian);
                    break;
                }
                else if (n == 2)
                {
                    shijian_mohu(shijian);
                    break;
                }
                else
                {
                    printf("输入错误，请重新输入\n");
                }
            }
        }
        else if (n == 3)
        {
            clean();
            while (1)
            {
                int n = 0;
                char jine[20];
                printf("__________________________\n");
                printf("1.简单查询\n");
                printf("2.模糊查询\n");
                printf("请选择\n");
                printf("__________________________\n");
                scanf("%d", &n);
                clean();
                printf("请输入金额\n");
                scanf("%20s", jine);
                if (n == 1)
                {
                    jine_jingque(jine);
                    break;
                }
                else if (n == 2)
                {
                    jine_mohu(jine);
                    break;
                }
                else
                {
                    printf("输入错误，请重新输入\n");
                }
            }
        }
        else if (n == 4)
        {
            int n = 0;
            char zhuangtai[20];
            char shijian[20];
            char jine[20];
            printf("__________________________\n");
            printf("1.状态 + 时间 + 金额\n");
            printf("2.状态 + 时间\n");
            printf("3.状态 + 金额\n");
            printf("4.时间 + 金额\n");
            printf("请选择\n");
            printf("__________________________\n");
            scanf("%d", &n);
            if (n == 1)
            {
                printf("请输入状态（y/n）\n");
                clean();
                scanf("%s", zhuangtai);
                clean();
                printf("请输入金额\n");
                scanf("%s", jine);
                printf("请输入时间\n");
                scanf("%s", shijian);
                cx_by_zhuangtai_shijian_jine(zhuangtai, shijian, jine);
            }
            else if (n == 2)
            {
                printf("请输入状态（y/n）\n");
                scanf("%s", zhuangtai);
                printf("请输入时间\n");
                scanf("%s", shijian);
                cx_by_zhuangtai_shijian(zhuangtai, shijian);
            }
            else if (n == 3)
            {
                printf("请输入状态（y/n）\n");
                scanf("%s", zhuangtai);
                printf("请输入金额\n");
                scanf("%s", jine);
                cx_by_zhuangtai_jine(zhuangtai, jine);
            }
            else if (n == 4)
            {
                printf("请输入金额\n");
                scanf("%s", jine);
                printf("请输入时间\n");
                scanf("%s", shijian);
                cx_by_shijian_jine(shijian, jine);
            }
            else
            {
                printf("输入错误，请重新输入\n");
            }
        }
        else if (n == 5)
        {
            return 0;
        }
        else
        {
            printf("输入错误，请重新输入\n");
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 实现查询的函数：
//
// 辅助函数 1：字符串转日期
void chartodate(const char* s, int d[3])
{
    if (s)
    {
        sscanf(s, "%d,%d,%d", &d[0], &d[1], &d[2]);
    }
}

// 辅助函数 2：比较日期
int comparedate(const int d1[3], const int d2[3])
{
    if (d1[0] == d2[0] && d1[1] == d2[1] && d1[2] == d2[2])
    {
        return 1; // 一样
    }
    else
    {
        return 0; // 不一样
    }
}

// 辅助函数 3：输出查询结果行
void print_fee(fee* p) // 改名避免冲突
{
    char tmp[10];
    strcpy(tmp, strcmp(p->yn, "y") == 0 ? "已缴费" : "未缴费");
    char pay_date[20];
    if (strcmp(p->yn, "y") == 0)
    {
        printf("账单时间：%s 金额：%s 是否缴费：%s 缴费时间：%s\n", p->date, p->sum, p->yn, p->date2);
    }
}

// 1.按时间精确查询（支持账单日期/缴费日期）
void shijian_jingque(const char* shijian) // 传入的是从在菜单时输入的日期
{
    int tmp;
    printf("1.账单日期，2.缴费日期\n");
    printf("请选择\n");
    scanf("%d", &tmp);
    int mubiao[3]; // 用户输入的日期
    int bianli[3]; // 遍历到的日期
    chartodate(shijian, mubiao); // 字符串转日期数组
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p != NULL)
    {
        // 匹配当前用户
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (tmp == 1)
            {
                chartodate(p->date, bianli); // 解析账单日期
            }
            else
            {
                if (strcmp(p->yn, "n") != 0) // 仅处理已缴费记录
                {
                    chartodate(p->date2, bianli); // 解析缴费日期
                }
                else
                {
                    p = p->next1; // 跳过未缴费
                    continue;
                }
            }
            if (comparedate(bianli, mubiao) == 1)
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 2.按时间模糊查询（支持账单日期/缴费日期）
void shijian_mohu(const char* shijian)
{
    int tmp;
    printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
    scanf("%d", &tmp);
    clean();
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        { // 找到匹配的用户
            char* compare_str = NULL; // 存储要对比的日期字符串（账单/缴费日期）
            if (tmp == 2) // 缴费日期查询记录
            {
                if (strcmp(p->yn, "n") == 0)
                {
                    p = p->next1;
                    continue; // 跳过未缴费
                }
                compare_str = p->date2; // 存进去
            }
            else
            {
                compare_str = p->date;
            }
            if (strstr(compare_str, shijian) != NULL) // 模糊匹配的核心 strstr 函数只要有重复的部分就搜到，找到了返回字符串里第一次出现位置的地址，没找到返回 NULL
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 3.按金额精确查询（和时间精确基本一样）
void jine_jingque(const char* jine) // 传入在菜单输入的金额
{
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p)
    {
        if (!strcmp(p->add1, owner_current123->add1) && !strcmp(p->add2, owner_current123->add2) && !strcmp(p->add3, owner_current123->add3)) // 使用 owner_ 前缀的 variable 匹配当前用户
        {
            if (!strcmp(p->sum, jine))
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 4.按金额模糊查询（和时间模糊基本一样）
void jine_mohu(const char* jine)
{
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (strstr(p->sum, jine) != NULL)
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// 5.6.7 都和 8 原理相同只是删减版
// 5. 状态 + 时间组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian(const char* zhuangtai, const char* rq)
{
    int query_type;
    printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
    scanf("%d", &query_type);
    clean();
    int mubiao[3], dangqian_rq[3];
    chartodate(rq, mubiao);
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p)
    {
        if (!strcmp(p->add1, owner_current123->add1) && !strcmp(p->add2, owner_current123->add2) && !strcmp(p->add3, owner_current123->add3) && !strcmp(p->yn, zhuangtai)) // 使用 owner_ 前缀的 variable
        {
            // 缴费日期查询需过滤未缴费记录
            if (query_type == 2 && strcmp(p->yn, "n") == 0)
            {
                p = p->next1;
                continue;
            }
            if (query_type == 1)
            {
                chartodate(p->date, dangqian_rq);
            }
            else
            {
                chartodate(p->date2, dangqian_rq);
            }
            if (comparedate(dangqian_rq, mubiao))
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 6. 状态 + 金额组合查询（精准匹配）
void cx_by_zhuangtai_jine(const char* zhuangtai, const char* je)
{
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p)
    {
        if (!strcmp(p->add1, owner_current123->add1) && !strcmp(p->add2, owner_current123->add2) && !strcmp(p->add3, owner_current123->add3) && !strcmp(p->yn, zhuangtai)) // 使用 owner_ 前缀的 variable
        {
            if (!strcmp(p->sum, je))
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 7. 时间 + 金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_shijian_jine(const char* rq, const char* je)
{
    int query_type;
    printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
    scanf("%d", &query_type);
    clean();
    int mubiao_rq[3], dangqian_rq[3];
    chartodate(rq, mubiao_rq);
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0;
    while (p)
    {
        if (!strcmp(p->add1, owner_current123->add1) && !strcmp(p->add2, owner_current123->add2) && !strcmp(p->add3, owner_current123->add3)) // 使用 owner_ 前缀的 variable
        {
            // 缴费日期查询过滤未缴费记录
            if (query_type == 2 && strcmp(p->yn, "n") == 0)
            {
                p = p->next1;
                continue;
            }
            if (query_type == 1)
            {
                chartodate(p->date, dangqian_rq);
            }
            else
            {
                chartodate(p->date2, dangqian_rq);
            }
            if (comparedate(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je))
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

// 8. 状态 + 时间 + 金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian_jine(const char* zhuangtai, const char* rq, const char* je)
{
    int query_type; // 时间维度（账单还是缴费）
    printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
    scanf("%d", &query_type);
    clean();
    int mubiao_rq[3], dangqian_rq[3]; // 第一个存用户输的日期，第二个存遍历的日期
    chartodate(rq, mubiao_rq);        // 字符串日期转整数数组
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    int count = 0; // 统计匹配的记录数
    while (p)
    {
        if (!strcmp(p->add1, owner_current123->add1) && !strcmp(p->add2, owner_current123->add2) && !strcmp(p->add3, owner_current123->add3) && !strcmp(p->yn, zhuangtai)) // 使用 owner_ 前缀的 variable 找到对应用户，且匹配状态
        {
            // 缴费日期查询过滤未缴费记录
            if (query_type == 2 && strcmp(p->yn, "n") == 0)
            {
                p = p->next1;
                continue;
            }
            if (query_type == 1)
            {
                chartodate(p->date, dangqian_rq);
            }
            else
            {
                chartodate(p->date2, dangqian_rq);
            }
            if (comparedate(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je))
            {
                print_fee(p);
                count++;
            }
        }
        p = p->next1;
    }
    if (count == 0)
    {
        printf("无匹配记录\n");
    }
}

void showworkers() // 5.显示人员
{
    FILE* fp = fopen("workers.txt", "r");
    if (fp == NULL)
    {
        printf("错误，无法打开文件");
        return;
    }
    char line[100]; // 存整行数据
    while (fgets(line, 100, fp) != NULL)
    {
        removen(line); // 去换行符
        printf("%s\n", line);
    }
    fclose(fp);
}

void tongjimenu() // 6.统计页面
{
    while (1)
    {
        int tmp = 0;
        printf("请选择统计方式\n");
        printf("1.按单一属性统计\n");
        printf("2.按多属性统计\n");
        printf("3.预设统计\n");
        printf("4.按条件统计\n");
        printf("5.退出\n");
        scanf("%d", &tmp);
        if (tmp == 1)
        {
            tmp = 0;
            printf("请选择统计方式\n");
            printf("1.统计已缴费\n");
            printf("2.统计未缴费\n");
            printf("3.统计所有\n");
            printf("4.按特定年份统计\n");
            scanf("%d", &tmp);
            if (tmp == 1)
            {
                yijiao();
            }
            else if (tmp == 2)
            {
                weijiao();
            }
            else if (tmp == 3)
            {
                suoyou();
            }
            else if (tmp == 4)
            {
                nianfen();
            }
        }
        else if (tmp == 2)
        {
            tmp = 0;
            printf("请选择统计方式\n");
            printf("1.统计已缴费 + 特定年份\n");
            printf("2.统计未缴费 + 特定年份\n");
            scanf("%d", &tmp);
            if (tmp == 1)
            {
                yijiao_nianfen();
            }
            else if (tmp == 2)
            {
                weijiao_nianfen();
            }
        }
        else if (tmp == 3) // 预设统计直接显示所有未缴帐单数及金额，已缴帐单数及金额
        {
            yvshe();
        }
        else if (tmp == 4)
        {
            tmp = 0;
            printf("请选择统计方式\n");
            printf("1.按年份（在哪年及之后）条件统计\n");
            printf("2.按年份（在哪年之前）条件统计\n");
            printf("3.按金额条件（大于等于多少钱）统计\n");
            printf("4.按金额条件（小于于多少钱）统计\n");
            scanf("%d", &tmp);
            if (tmp == 1)
            {
                nianfenhou();
            }
            else if (tmp == 2)
            {
                nianfenqian();
            }
            else if (tmp == 3)
            {
                jineda();
            }
            else if (tmp == 4)
            {
                jinexiao();
            }
        }
        else if (tmp == 5)
        {
            return;
        }
    }
}

// 排序菜单
void paixuMenu() {
    int choice_type = 0;
    int choice_order = 0;

    printf("--- 排序选项 ---\n");
    printf("请选择排序依据:\n");
    printf("1. 按账单日期\n");
    printf("2. 按金额\n");
    printf("请选择 (1 或 2): ");
    scanf("%d", &choice_type);

    if (choice_type != 1 && choice_type != 2) {
        printf("无效选择，返回主菜单。\n");
        clean(); // 清空输入缓冲区
        return;
    }

    printf("请选择排序顺序:\n");
    printf("1. 升序 (Ascending)\n");
    printf("2. 降序 (Descending)\n");
    printf("请选择 (1 或 2): ");
    scanf("%d", &choice_order);

    if (choice_order != 1 && choice_order != 2) {
        printf("无效选择，返回主菜单。\n");
        clean(); // 清空输入缓冲区
        return;
    }
    clean(); // 清空输入缓冲区

    // 调用具体的排序函数
    if (choice_type == 1) {
        if (choice_order == 1) {
            sortFeesByDateAsc();
        } else {
            sortFeesByDateDesc();
        }
    } else { // choice_type == 2
        if (choice_order == 1) {
            sortFeesByAmountAsc();
        } else {
            sortFeesByAmountDesc();
        }
    }

    printf("排序完成。\n");
}

// 按账单日期升序
int compareFeeDateAsc(const void *a, const void *b) {
    fee* fa = *(fee**)a;
    fee* fb = *(fee**)b;

    int date_a[3], date_b[3];
    chartodate(fa->date, date_a);
    chartodate(fb->date, date_b);

    // 比较年
    if (date_a[0] != date_b[0]) return date_a[0] - date_b[0];
    // 比较月
    if (date_a[1] != date_b[1]) return date_a[1] - date_b[1];
    // 比较日
    return date_a[2] - date_b[2];
}

// 按账单日期降序
int compareFeeDateDesc(const void *a, const void *b) {
    // 复用升序逻辑，但返回相反值
    return -compareFeeDateAsc(a, b);
}

// 按金额升序
int compareFeeAmountAsc(const void *a, const void *b) {
    fee* fa = *(fee**)a;
    fee* fb = *(fee**)b;

    int amount_a = atoi(fa->sum);
    int amount_b = atoi(fb->sum);

    return amount_a - amount_b;
}

// 按金额降序
int compareFeeAmountDesc(const void *a, const void *b) {
    // 复用升序逻辑，但返回相反值
    return -compareFeeAmountAsc(a, b);
}

// 按账单日期升序排序
void sortFeesByDateAsc() {
    sortFeesWithComparator(compareFeeDateAsc);
}

// 按账单日期降序排序
void sortFeesByDateDesc() {
    sortFeesWithComparator(compareFeeDateDesc);
}

// 按金额升序排序
void sortFeesByAmountAsc() {
    sortFeesWithComparator(compareFeeAmountAsc);
}

// 按金额降序排序
void sortFeesByAmountDesc() {
    sortFeesWithComparator(compareFeeAmountDesc);
}

// 通用排序逻辑
void sortFeesWithComparator(int (*comparator)(const void *, const void *)) {
    // 1. 遍历主链表，找出当前用户的所有缴费记录，并存入动态数组
    fee* current_fee = owner_head1123->next1; // 从第一个有效节点开始
    int count = 0;
    int capacity = 10; // 初始容量
    fee** temp_array = (fee**)malloc(capacity * sizeof(fee*));

    if (!temp_array) {
        printf("内存分配失败，无法排序。\n");
        return;
    }

    while (current_fee != NULL) {
        if (strcmp(current_fee->add1, owner_current123->add1) == 0 &&
            strcmp(current_fee->add2, owner_current123->add2) == 0 &&
            strcmp(current_fee->add3, owner_current123->add3) == 0) {

            // 如果数组满了，扩容
            if (count >= capacity) {
                capacity *= 2;
                fee** temp_realloc = (fee**)realloc(temp_array, capacity * sizeof(fee*));
                if (!temp_realloc) {
                    printf("内存重新分配失败，无法排序。\n");
                    free(temp_array);
                    return;
                }
                temp_array = temp_realloc;
            }

            temp_array[count] = current_fee;
            count++;
        }
        current_fee = current_fee->next1;
    }

    if (count <= 1) {
        // 如果记录数少于等于1，则无需排序
        free(temp_array);
        return;
    }

    // 2. 对数组进行排序
    qsort(temp_array, count, sizeof(fee*), comparator);

    // 3. 重建链表连接 (将排序后的节点重新链接起来)
    // 注意：我们只改变这些节点内部的 next1 指针，形成一个子链
    for (int i = 0; i < count - 1; i++) {
        temp_array[i]->next1 = temp_array[i+1];
    }
    temp_array[count - 1]->next1 = NULL; // 最后一个节点的 next1 设为 NULL

    // 4. 将原主链表中属于当前用户的所有节点从链上摘除
    // 需要找到这些节点的前驱和后继
    fee* prev_in_main_list = owner_head1123; // 指向主链表中某个节点的前驱
    current_fee = owner_head1123->next1;

    while (current_fee != NULL) {
        if (strcmp(current_fee->add1, owner_current123->add1) == 0 &&
            strcmp(current_fee->add2, owner_current123->add2) == 0 &&
            strcmp(current_fee->add3, owner_current123->add3) == 0) {
            
            // 找到属于当前用户的一个节点，将其从主链表中断开
            prev_in_main_list->next1 = current_fee->next1;
            // current_fee 已经在 temp_array 中，其 next1 在上一步已经被设置
            current_fee = prev_in_main_list->next1; // 移动到断开点的下一个节点继续检查
        } else {
            prev_in_main_list = current_fee;
            current_fee = current_fee->next1;
        }
    }


    // 5. 将排序后的子链（temp_array[0] 是头）重新插入到主链表的末尾
    // 首先找到主链表的最后一个节点
    fee* last_node_in_main = owner_head1123;
    while (last_node_in_main->next1 != NULL) {
        last_node_in_main = last_node_in_main->next1;
    }
    // 将排序好的子链接到主链表末尾
    last_node_in_main->next1 = temp_array[0];
    // 更新全局尾指针
    owner_prear1123 = temp_array[count - 1]; // 尾指针指向排序后子链的最后一个节点


    // 6. 释放临时数组
    free(temp_array);
}

/////////////////////////////////////////////////////////////////////////////////////
/////统计相关函数
// 统计已缴费
void yijiao()
{
    int count = 0; // 记录条数
    int total = 0; // 总金额
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        { // 找到对应用户
            if (strcmp(p->yn, "y") == 0)
            {
                count++;
                total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("已缴费记录：%d 条，总金额：%d\n", count, total);
}

// 统计未缴费（和已缴费原理相同）
void weijiao()
{
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (strcmp(p->yn, "n") == 0)
            {
                count++;
                total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("未缴费记录：%d 条，总金额：%d\n", count, total);
}

// 统计所有记录（和已缴费原理相同）
void suoyou()
{
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            count++;
            total += atoi(p->sum);
        }
        p = p->next1;
    }
    printf("所有缴费记录：%d 条，总金额：%d\n", count, total);
}

// 按特定年份统计（账单年份）
void nianfen()
{
    int year; // 用户输入的年份
    printf("请输入年份：");
    scanf("%d", &year);
    clean(); // 清空输入缓冲区
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            int y;
            sscanf(p->date, "%d,", &y); // 一个字符串中按照指定格式读取数据：就提取年份 2024 并存储，遇到",“就停止录入，逗号也不存储
            if (y == year)
            {
                count++;
                total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("%d 年的账单记录：%d 条，总金额：%d\n", year, count, total);
}

// 统计已缴费 + 特定年份（是不是已缴费不用用户输，选择这个选项就是选择已缴费了）
void yijiao_nianfen()
{
    int year;
    printf("请输入年份：");
    scanf("%d", &year);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (strcmp(p->yn, "y") == 0)
            {
                int y;
                sscanf(p->date, "%d,", &y); // 一个字符串中按照指定格式读取数据：就提取年份 2024 并存储，遇到",“就停止录入，逗号也不存储
                if (y == year)
                {
                    count++;
                    total += atoi(p->sum);
                }
            }
        }
        p = p->next1;
    }
    printf("%d 年已缴费记录：%d 条，总金额：%d\n", year, count, total);
}

// 统计未缴费 + 特定年份
void weijiao_nianfen()
{
    int year;
    printf("请输入年份：");
    scanf("%d", &year);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (strcmp(p->yn, "n") == 0)
            {
                int y;
                sscanf(p->date, "%d,", &y); // 一个字符串中按照指定格式读取数据：就提取年份 2024 并存储，遇到",“就停止录入，逗号也不存储
                if (y == year)
                {
                    count++;
                    total += atoi(p->sum);
                }
            }
        }
        p = p->next1;
    }
    printf("%d 年未缴费记录：%d 条，总金额：%d\n", year, count, total);
}

// 预设统计：显示已缴费和未缴费的总条数和总金额（就是把两个合起来了，原理不变）
void yvshe()
{
    int unpaid_count = 0, unpaid_total = 0;
    int paid_count = 0, paid_total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            if (strcmp(p->yn, "y") == 0)
            {
                paid_count++;
                paid_total += atoi(p->sum);
            }
            else
            {
                unpaid_count++;
                unpaid_total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("已缴费：%d 条，金额%d\n", paid_count, paid_total);
    printf("未缴费：%d 条，金额%d\n", unpaid_count, unpaid_total);
}

// 按年份（在哪年及之后）条件统计（后面的四个原理跟前面差不多，就是加了个大小比较）
void nianfenhou()
{
    int year;
    printf("请输入年份：");
    scanf("%d", &year);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            int y;
            sscanf(p->date, "%d,", &y);
            if (y >= year)
            {
                count++;
                total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("%d 年及之后的账单记录：%d 条，总金额：%d\n", year, count, total);
}

// 按年份（在哪年之前）条件统计
void nianfenqian()
{
    int year;
    printf("请输入年份：");
    scanf("%d", &year);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            int y;
            sscanf(p->date, "%d,", &y);
            if (y < year)
            {
                count++;
                total += atoi(p->sum);
            }
        }
        p = p->next1;
    }
    printf("%d 年之前的账单记录：%d 条，总金额：%d\n", year, count, total);
}

// 按金额条件（大于等于多少钱）统计
void jineda()
{
    int money;
    printf("请输入金额：");
    scanf("%d", &money);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            int m = atoi(p->sum);
            if (m >= money)
            {
                count++;
                total += m;
            }
        }
        p = p->next1;
    }
    printf("金额大于等于%d 的记录：%d 条，总金额：%d\n", money, count, total);
}

// 按金额条件（小于多少钱）统计
void jinexiao()
{
    int money;
    printf("请输入金额：");
    scanf("%d", &money);
    clean();
    int count = 0;
    int total = 0;
    fee* p = owner_head1123->next1; // 使用 owner_ 前缀的 variable
    while (p != NULL)
    {
        if (strcmp(p->add1, owner_current123->add1) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add2, owner_current123->add2) == 0 && // 使用 owner_ 前缀的 variable
            strcmp(p->add3, owner_current123->add3) == 0)   // 使用 owner_ 前缀的 variable
        {
            int m = atoi(p->sum);
            if (m < money)
            {
                count++;
                total += m;
            }
        }
        p = p->next1;
    }
    printf("金额小于%d 的记录：%d 条，总金额：%d\n", money, count, total);
}

// 主程序入口
void owner_system()
{
    // 初始化业主链表
    owner_head123 = (yezhu*)malloc(sizeof(yezhu)); // 创建头节点 (owner_ 前缀)
    if (owner_head123 == NULL) // 使用 owner_ 前缀的 variable
    {
        printf("内存分配失败\n");
        return; // 修改：void 函数返回
    }
    strcpy(owner_head123->add1, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head123->add2, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head123->add3, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head123->name, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head123->pwd, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head123->tel, ""); // 使用 owner_ 前缀的 variable
    owner_head123->next = NULL; // 使用 owner_ 前缀的 variable
    owner_prear123 = owner_head123; // 尾部指针 (owner_ 前缀)

    // 初始化缴费链表
    owner_head1123 = (fee*)malloc(sizeof(fee)); // 创建头节点 (owner_ 前缀)
    if (owner_head1123 == NULL) // 使用 owner_ 前缀的 variable
    {
        printf("内存分配失败\n");
        free(owner_head123); // 释放已分配的 owner_head (owner_ 前缀)
        return; // 修改：void 函数返回
    }
    strcpy(owner_head1123->add1, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->add2, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->add3, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->date, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->sum, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->yn, ""); // 使用 owner_ 前缀的 variable
    strcpy(owner_head1123->date2, ""); // 使用 owner_ 前缀的 variable
    owner_head1123->next1 = NULL; // 使用 owner_ 前缀的 variable
    owner_prear1123 = owner_head1123; // 尾部指针 (owner_ 前缀)

    ///////////////////////////////////
    loadusers();
    loadfee();
    ///////////////////////////////////

    while (1)
    {
        int tmp = 0; // 接收指令
        qidong();
        scanf("%d", &tmp);
        if (tmp == 1)
        {
            if (signin() == 0)
            {
                while (1)
                {
                    caozuo();
                    int a = tiaozhuan();
                    if (a == 1)
                    {
                        break; // 退出内层循环，回到主菜单
                    }
                }
            }
            else
            {
                printf("登录失败，返回主菜单。\n"); // 添加反馈
                continue;
            }
        }
        else if (tmp == 2)
        {
            freeList();
            printf("系统已退出。\n"); // 添加反馈
            return; // 修改：void 函数返回
        }
        else if (tmp == 3)
        {
            if (zhuce() == 0)
            {
                printf("注册成功，自动登录。\n"); // 添加反馈
                while (1)
                {
                    caozuo();
                    int a = tiaozhuan();
                    if (a == 1)
                    {
                        break; // 退出内层循环，回到主菜单
                    }
                }
            }
            else
            {
                printf("注册失败，返回主菜单。\n"); // 添加反馈
                continue;
            }
        }
        else if (tmp == 4)
        {
            zhaohui();
            continue;
        }
        else
        {
            printf("错误的指令，请重新输入\n"); // 添加换行符
            continue;
        }
    }
}
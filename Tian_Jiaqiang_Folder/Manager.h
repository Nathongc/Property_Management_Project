#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<time.h>
#define MAX 1000
#define FILENAME "Manager.txt"
#define BILL "Bill.txt"
#define MANAGER_PASSWORD 20251354
typedef struct Imformation Imfor;
typedef struct Person Person;
extern Imfor* imfor;//无机物信息
extern Person* head;//链表头指针
extern int year;
extern int month;
extern int day;
struct Person {//人
    long long password;//密码
    long long M_phone_num;//电话号
    int M_area;//家庭住址(楼)
    char M_name[MAX];//姓名
    char M_sex[MAX];//性别
    int M_age;//年龄
    int Area[MAX];//工作区域
    int Area_count;//工作区域数量
    int parking_imfor;//停车位占用情况
    char Career[MAX];//工作
    int Count_charge;//累计缴费次数
    int Date_charge[MAX][3];//缴费日期【次数】【年月日】
    Person* next;//指针
};
struct Imformation {//无机物信息
    int Num_Building;//楼宇数量
    int Num_parking;//停车位数量
    int parking[MAX];//停车位，0为空闲，1为占用,-1为关闭
    int charging_fee;//收费金额
    int charging_date;//收费周期
};
void Authorize();//授权
void Get_time();//获取时间
void Show_Imfor(Person* head, Imfor* imfor);
void Save(Person* head);//存入文件
void Imfor_Read();//读取文件中的imfor信息
void AddImfor();//添加人员信xi
void FindPerson();//查询人员信息
void Sta_Imfor(Person* head);//统计人员信息
void maintain_password(Person* head);//密码维护
void Generate();//生成账单
Imfor* Init_imfor(Imfor* imfor); //初始化imfor对象
Person* Init_Person(Person* person);//初始化人
Person* ADD_TO_LIST(Person* head, Person* person);//加入链表
Person* Delimfor(Person* head);//从链表中删除对象
Person* ModImfor(Person* head, Imfor* imfor);//更改信息
Person* Mod_Person(Person* person);//更改人员
Person* Sort_Person(Person* head);//排序
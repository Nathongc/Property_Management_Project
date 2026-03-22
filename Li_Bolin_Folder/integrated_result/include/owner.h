#ifndef OWNER_H
#define OWNER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// 结构体定义
typedef struct yezhu {
    char add1[20]; // 楼号
    char add2[20]; // 楼层
    char add3[20]; // 户号
    char name[30]; // 用户名
    char pwd[30];  // 密码
    char tel[30];  // 电话
    struct yezhu* next;
} yezhu;

typedef struct fee {
    char add1[20]; // 楼号
    char add2[20]; // 楼层
    char add3[20]; // 户号
    char date[20]; // 账单日期
    char sum[20];  // 金额
    char yn[10];   // 缴费状态 (y/n)
    char date2[20]; // 缴费日期
    struct fee* next1;
} fee;

// 全局变量声明
extern yezhu* head;
extern yezhu* prear;
extern fee* head1;
extern fee* prear1;
extern yezhu* current; // 当前业主

// 函数声明
// 主程序入口
void owner_system(void);

// 加载数据
int loadusers(void);
int loadfee(void);

// 登录注册相关
int signin(void);
int checkpwd(const char* pwd, yezhu* cur);
int checkname(const char* name);
int checkadd(const char* add, int n);
int checkadd2(const char* add1, const char* add2, const char* add3, yezhu* current);
int checkname2(const char* name);
int checkpwd2(const char* pwd);
void clean(void);
int zhuce(void);
void zhaohui(void);
void freeList(void);
yezhu* checkname3(const char* name); // 补充声明
void save(yezhu* newnode);          // 补充声明
void removen(char* str);            // 补充声明

// 功能相关
void qidong(void);
int tiaozhuan(void);
void caozuo(void);
void xiugai(yezhu* current);
void check(void);
void jiaofei(void);
void showall(void);
void savefee(void);
void getcurrentdate(char* datestr);
int queryMenu(void);
void showworkers(void);
void tongjimenu(void);

// 查询函数
void shijian_jingque(const char* rq);
void jine_jingque(const char* je);
void shijian_mohu(const char* shijian);
void jine_mohu(const char* gj);
void cx_by_zhuangtai_shijian(const char* zhuangtai, const char* rq);
void cx_by_zhuangtai_jine(const char* zhuangtai, const char* je);
void cx_by_shijian_jine(const char* rq, const char* je);
void cx_by_zhuangtai_shijian_jine(const char* zhuangtai, const char* rq, const char* je);

// 辅助函数
void chartodate(const char* s, int d[3]);
int comparedate(const int date1[3], const int date2[3]);
void print_fee(fee* p); // 改名避免冲突

// 统计函数 (补充声明)
void yijiao(void);
void weijiao(void);
void suoyou(void);
void nianfen(void);
void yijiao_nianfen(void);
void weijiao_nianfen(void);
void yvshe(void);
void nianfenhou(void);
void nianfenqian(void);
void jineda(void);
void jinexiao(void);

void paixuMenu();

void sortFeesByDateAsc();
void sortFeesByDateDesc();
void sortFeesByAmountAsc();
void sortFeesByAmountDesc();
void sortFeesWithComparator(int (*cmp)(const void*, const void*));

int compareFeeDateAsc(const void* a, const void* b);
int compareFeeDateDesc(const void* a, const void* b);
int compareFeeAmountAsc(const void* a, const void* b);
int compareFeeAmountDesc(const void* a, const void* b);

#endif // OWNER_H
//头文件
#pragma once
#define _CRT_SECURE_NO_WARNINGS 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/// 结构体
//////////////////////////////////////////////////////////////////
typedef struct yezhu//业主结构体
{
	char add1[20];//楼号
	char add2[20];//楼层
	char add3[20];//户号
	char name[20];//用户名（不重复）
	char pwd[20];//密码
	char tel[20];//电话
	struct yezhu* next;
}yezhu;
typedef struct fee//缴费结构体
{
	char add1[20];//楼号
	char add2[20];//楼层
	char add3[20];//户号
	char date[20];//创建日期
	char sum[10];//钱数
	char yn[20];//是否已交
	char date2[20];//缴费日期
	struct fee* next1;
}fee;
extern yezhu* head;//业主链表头
extern yezhu* prear;//业主链表尾
extern fee* head1;//缴费链表头
extern fee* prear1;//缴费链表尾
extern yezhu* current;//当前业主

/// 启动和注册
/// ////////////////////////////////////////////////////////////////
int loadusers();//从文件加载用户到链表
void qidong();//启动页面
int loadfee();
void caozuo();//操作页面
int zhuce();//注册页面
void save(yezhu* newnode);//保存业主
int checkname(char* name);//查重名（用户名）
int checkadd(char* add, int n);//查地址格式
int checkadd2(char* add1, char* add2, char* add3,yezhu*current);//查重名（地址）
int checkname2(char* name); // 检查用户名长度
int checkpwd2(char* pwd);   // 检查密码长度(5-20)
void clean();//清缓存
int signin();//登录
int checkpwd(char*pwd,yezhu*cur);//核对密码
void zhaohui();//找回用户名和密码
void freeList();//释放链表
///操作
//////////////////////////////////////////////////////////////////////
//0.跳转到功能
void tiaozhuan();
void xiugai(yezhu* current);/// 1. 修改信息
void check();//2.缴费查询

#include "tou.h"
/// 功能
/// ////////////////////////////////////////////////////////////////
//0.跳转到功能
void tiaozhuan()
{
	int tmp = 0;
	scanf("%d", &tmp);
	if (tmp == 1)
	{
		xiugai(current);
		return;
	}
	else if (tmp == 2)
	{
		check();
		return;
	}
}
/// 1. 修改信息
void xiugai(yezhu* current)
{
	char add1[20];//楼号
	char add2[20];//楼层
	char add3[20];//户号
	char tel[30];//电话
	char name[30];
	char pwd[30];
	char b[3];
	while (1)
	{
		printf("地址要修改吗？（Y/N）\n");
		scanf("%2s", b);
		if (b[0] == 'Y')
		{
			printf("请输入用户楼号（如“123”，1-999内）");
			scanf("%10s", add1);
			int tmp = checkadd(add1, 4);
			if (tmp == 1)
			{
				printf("格式错误\a\n");
				continue;
			}
			printf("请输入用户楼层（如“10”，1-99内）");
			scanf("%10s", add2);
			tmp = checkadd(add2, 3);
			if (tmp == 1)
			{
				printf("格式错误\a\n");
				continue;
			}
			printf("请输入用户户号（如“123”，1-999内）");
			scanf("%10s", add3);
			tmp = checkadd(add3, 4);
			if (tmp == 1)
			{
				printf("格式错误\a\n");
				continue;
			}
			tmp = checkadd2(add1, add2, add3, current);
			if (tmp == 1)
			{
				printf("错误，该地址已被注册\a\n");
				continue;
			}
			strcpy(current->add1, add1);
			strcpy(current->add2, add2);
			strcpy(current->add3, add3);
			printf("修改成功\n");
			break;
		}
		else if (b[0] == 'N')
		{
			break;
		}
		else
		{
			printf("不正确，请重新输入\a\n");
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
			int tmp2 = checkname(name);
			if (tmp2 == 0)
			{
				strcpy(current->name, name);
				printf("修改成功\n");
				break;
			}
			else
			{
				continue;
			}
		}
		else if (b[0] == 'N')
		{
			break;
		}
		else
		{
			printf("不正确，请重新输入\a\n");
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
			strcpy(current->pwd, pwd);
			printf("修改成功\n");
			break;
		}
		else if (b[0] == 'N')
		{
			break;
		}
		else
		{
			printf("不正确，请重新输入\a\n");
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
			strcpy(current->tel, tel);
			printf("修改成功\n");
			break;
		}
		else if (b[0] == 'N')
		{
			break;
		}
		else
		{
			printf("不正确，请重新输入\a\n");
		}
	}
	clean();
}
//2.缴费查询
void check()
{
	int i = 1;
	int tmp = 1;
	fee* cur = head1->next1;
	while (cur != NULL)
	{

		if (strcmp(cur->add1, current->add1) == 0 && strcmp(cur->add2, current->add2) == 0 && strcmp(cur->add3, current->add3) == 0)
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
			printf("NO.%d", i);
			printf("_________________________\n");
			printf("账单时间：%s年%s月%s日\n", year, month, day);
			printf("金额： % s\n", cur->sum);
			printf("是否缴费（y/n）：%s\n", cur->yn);
			printf("缴费时间：%s年%s月%s日\n", year2, month2, day2);
			i++;
		}
		cur = cur->next1;
	}
	if (tmp == 1)
	{
		printf("错误，未找到该地址\a\n");
	}
}

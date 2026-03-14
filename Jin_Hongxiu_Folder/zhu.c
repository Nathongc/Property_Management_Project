//主程序
#include "tou.h"
yezhu* head = NULL;
yezhu* prear = NULL;
fee* head1 = NULL;
fee* prear1 = NULL;
yezhu* current = NULL;//当前业主
int main()
{//初始化业主链表
	head = (yezhu*)malloc(sizeof(yezhu));//创建头节点
	if (head == NULL)
	{
		printf("内存分配失败\a\n");
		return 1;
	}
	strcpy(head->add1, "");
	strcpy(head->add2, "");
	strcpy(head->add3, "");
	strcpy(head->name, "");
	strcpy(head->pwd, "");
	strcpy(head->tel, "");
	head->next = NULL;
	prear = head;//尾部指针
	//初始化缴费链表
	head1 = (fee*)malloc(sizeof(fee));//创建头节点
	if (head1 == NULL)
	{
		printf("内存分配失败\a\n");
		return 1;
	}
	strcpy(head1->add1, "");
	strcpy(head1->add2, "");
	strcpy(head1->add3, "");
	strcpy(head1->date, "");
	strcpy(head1->sum, "");
	strcpy(head1->yn, "");
	strcpy(head1->date2, "");
	head1->next1 = NULL;
	prear1 = head1;//尾部指针
	///////////////////////////////////
	loadusers();
	loadfee();
	//////////////////////////////////
	while (1)
	{
		int tmp = 0;//接收指令
		qidong();
		scanf("%d", &tmp);
		if (tmp == 1)
		{
			if (signin() == 0)
			{
				caozuo();
				tiaozhuan();
				continue;
			}
			else
			{
				continue;
			}
		}
		else if (tmp == 2)
		{
			freeList();
			return 0;
		}
		else if (tmp == 3)
		{
			if (zhuce() == 0)
			{
				caozuo();
				tiaozhuan();
			}
			else
			{
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
			printf("错误的指令，请重新输入\n\a");
			continue;
		}
	}
}

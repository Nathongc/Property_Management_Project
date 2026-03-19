#include "tou.h"
/// 功能
/// ////////////////////////////////////////////////////////////////
//0.跳转到功能
int tiaozhuan()
{
	int tmp = 0;
	scanf("%d", &tmp);
	if (tmp == 1)//修改信息
	{
		xiugai(current);
		return 0;
	}
	else if (tmp == 2)
	{
		queryMenu();//缴费查询
		return 0;
	}
	else if (tmp == 3)
	{
		jiaofei();//缴费
		return 0;
	}
	else if (tmp == 4)
	{
		showall();//显示所有缴费缴费记录
	}
	else if (tmp == 5)
	{
		showworkers();//显示人员
	}
	else if (tmp == 7)
	{
		tongjimenu();//统计页面
	}
	else if (tmp == 0)
	{
		return 1;
	}
	else
	{
		printf("输入错误\a\n");
		return 0;
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
//2.缴费查询（未缴费）
void check()
{
	int hasUnpaid = 0; // 是否有未缴费
	int sumall = 0;
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
			if (strcmp(cur->yn, "n") == 0)
			{
				printf("NO.%d", i);
				printf("_________________________\n");
				printf("账单时间：%s年%s月%s日\n", year, month, day);
				printf("金额： % s\n", cur->sum);
				printf("是否缴费（y/n）：%s\n", cur->yn);
				printf("缴费时间：%s年%s月%s日\n", year2, month2, day2);
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
		printf("错误，未找到该地址\a\n");
	}
}
//3.缴费
void jiaofei()//缴费
{
	int sumall = 0;
	fee* cur = head1->next1;
	int hasUnpaid = 0;//是否需要缴费
	while (cur != NULL)
	{

		if (strcmp(cur->add1, current->add1) == 0 && strcmp(cur->add2, current->add2) == 0 && strcmp(cur->add3, current->add3) == 0)
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
			cur = head1->next1;
			char currentdate[20];
			// 获取当前系统日期
			getcurrentdate(currentdate);
			while (cur != NULL)
			{

				if (strcmp(cur->add1, current->add1) == 0 && strcmp(cur->add2, current->add2) == 0 && strcmp(cur->add3, current->add3) == 0)
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
			printf("输入错误\a\n");
		}
	}
}
void savefee()//保存缴费
{
	FILE* fp = fopen("fee.txt", "w");//整个重写
	if (fp == NULL) {
		printf("保存失败！\a\n");
		return;
	}
	fee* cur = head1->next1;
	while (cur != NULL)
	{
		char year[5], month[3], day[3];
		char year2[5], month2[3], day2[3];
		sscanf(cur->date, "%[^,],%[^,],%s", year, month, day);//%[^,]是按","分割字串
		sscanf(cur->date2, "%[^,],%[^,],%s", year2, month2, day2);
		fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", cur->add1, cur->add2, cur->add3, year, month, day, cur->sum, cur->yn, year2, month2, day2);
		cur = cur->next1;
	}

	fclose(fp);
	printf("缴费记录已同步到文件！\n");
}
void getcurrentdate(char* dateStr)// 获取当前系统日期
{
	time_t now = time(NULL); //时间戳time_t
	struct tm* t = localtime(&now);//时间戳解析为 struct tm(年月日)
	//将年月日复制到char*里
	sprintf(dateStr, "%d,%d,%d",
		t->tm_year + 1900, // 年份需+1900
		t->tm_mon + 1,     // 月份从0开始，需+1
		t->tm_mday);       // 日期
}
void showall()//显示所有缴费记录
{
	int haspaid = 0; // 是否有未缴费
	int sumall = 0;
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
			if (strcmp(cur->yn, "y") == 0)
			{
				printf("NO.%d", i);
				printf("_________________________\n");
				printf("账单时间：%s年%s月%s日\n", year, month, day);
				printf("金额： % s\n", cur->sum);
				printf("缴费时间：%s年%s月%s日\n", year2, month2, day2);
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
	printf("共缴费%d次", i);
	if (tmp == 1)
	{
		printf("错误，未找到该地址\a\n");
	}
}
//5. 缴费查询总菜单（整合简单/组合/模糊查询）
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
				printf("__________________________\n");
				scanf("%d", &n);
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
					printf("输入错误，请重新输入\a\n");
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
					printf("输入错误，请重新输入\a\n");
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
					printf("输入错误，请重新输入\a\n");
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
			printf("1.状态+时间+金额\n");
			printf("2.状态+时间\n");
			printf("3.状态+金额\n");
			printf("4.时间+金额\n");
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
				printf("输入错误，请重新输入\a\n");
			}
		}
		else if (n == 5)
		{
			return;
		}
		else
		{
			printf("输入错误，请重新输入\a\n");
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 实现查询的函数：
// 
// 辅助函数1：字符串转日期
void chartodate(char* s, int d[3])
{
	if (s) {
		sscanf(s, "%d,%d,%d", &d[0], &d[1], &d[2]);
	}
}
// 辅助函数2：比较日期
int comparedate(int d1[3], int d2[3])
{
	if (d1[0] == d2[0] && d1[1] == d2[1] && d1[2] == d2[2])
	{
		return 1;//一样
	}
	else
	{
		return 0;//不一样
	}
}
// 辅助函数3：输出查询结果行
void print(fee* p) {
	char tmp[10];
	strcpy(tmp, strcmp(p->yn, "y") == 0 ? "已缴费" : "未缴费");
	char pay_date[20];
	if (strcmp(p->yn, "y") == 0)
	{
		printf("账单时间：%s 金额：%s 是否缴费：%s 缴费时间：%s\n", p->date, p->sum, p->yn, p->date2);
	}
}
// 1.按时间精确查询（支持账单日期/缴费日期）
void shijian_jingque(char* shijian)//传入的是从在菜单时输入的日期
{
	int tmp;
	printf("1.账单日期,2.缴费日期\n");
	printf("请选择\n");
	scanf("%d", &tmp);
	int mubiao[3];//用户输入的日期
	int bianli[3];//遍历到的日期
	chartodate(shijian, mubiao);//字符串转日期数组
	fee* p = head1->next1;
	int count = 0;
	while (p != NULL)
	{
		// 匹配当前用户
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			if (tmp == 1) {
				chartodate(p->date, bianli);// 解析账单日期
			}
			else
			{
				if (strcmp(p->yn, "n") != 0) // 仅处理已缴费记录
				{
					chartodate(p->date2, bianli); // 解析缴费日期
				}
				else {
					p = p->next1; // 跳过未缴费
					continue;
				}
			}
			if (comparedate(bianli, mubiao) == 1)
			{
				print(p);
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
void shijian_mohu(const char* shijian) {
	int tmp;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &tmp);
	clean();
	fee* p = head1->next1;
	int count = 0;
	while (p != NULL) {
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)//找到匹配的用户 
		{
			char* compare_str = NULL; // 存储要对比的日期字符串（账单/缴费日期）
			if (tmp == 2) // 缴费日期查询仅匹配已缴费记录
			{
				if (strcmp(p->yn, "n") == 0) {
					p = p->next1;
					continue;//跳过未缴费
				}
				compare_str = p->date2;//存进去
			}
			else {
				compare_str = p->date;
			}
			if (strstr(compare_str, shijian) != NULL) //模糊匹配的核心strstr函数只要有重复的部分就搜到，找到了返回字符串里第一次出现位置的地址，没找到返回NULL
			{
				print(p);
				count++;
			}
		}
		p = p->next1;
	}
	if (count == 0) {
		printf("无匹配记录\n");
	}
}

// 3.按金额精确查询（和时间精确基本一样）
void jine_jingque(char* jine)//传入在菜单输入的金额
{
	fee* p = head1->next1;
	int count = 0;
	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3))	// 匹配当前用户
		{
			if (!strcmp(p->sum, jine)) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		printf("无匹配记录\n");
	}
}

// 4.按金额模糊查询（和时间模糊基本一样）
void jine_mohu(char* jine) {
	fee* p = head1->next1;
	int count = 0;
	while (p != NULL) {
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0) {

			if (strstr(p->sum, jine) != NULL) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}
	if (count == 0) {
		printf("无匹配记录\n");
	}
}
////////////////////////////////////////////////////////////////////////////////////////////
// 5.6.7都和8原理相同只是删减版
// 5. 状态+时间组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian(char* zhuangtai, char* rq) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao[3], dangqian_rq[3];
	chartodate(rq, mubiao);
	fee* p = head1->next1;
	int count = 0;
	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			// 缴费日期查询需过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				chartodate(p->date, dangqian_rq);
			}
			else {
				chartodate(p->date2, dangqian_rq);
			}

			if (comparedate(dangqian_rq, mubiao)) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		printf("无匹配记录\n");
	}
}

// 6. 状态+金额组合查询（精准匹配）
void cx_by_zhuangtai_jine(char* zhuangtai, char* je) {
	fee* p = head1->next1;
	int count = 0;
	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			if (!strcmp(p->sum, je)) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		printf("无匹配记录\n");
	}
}

// 7. 时间+金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_shijian_jine(char* rq, char* je) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao_rq[3], dangqian_rq[3];
	chartodate(rq, mubiao_rq);
	fee* p = head1->next1;
	int count = 0;
	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			// 缴费日期查询过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				chartodate(p->date, dangqian_rq);
			}
			else {
				chartodate(p->date2, dangqian_rq);
			}

			if (comparedate(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		printf("无匹配记录\n");
	}
}

// 8. 状态+时间+金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian_jine(char* zhuangtai, char* rq, char* je) {
	int query_type;//时间维度（账单还是缴费）
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();
	int mubiao_rq[3], dangqian_rq[3];//第一个存用户输的日期，第二个存遍历的日期
	chartodate(rq, mubiao_rq);// 字符串日期转整数数组
	fee* p = head1->next1;
	int count = 0;// 统计匹配的记录数
	char status[10];//储存是否缴费
	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) //找到对应用户，且匹配状态
		{
			// 缴费日期查询过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				chartodate(p->date, dangqian_rq);
			}
			else {
				chartodate(p->date2, dangqian_rq);
			}

			if (comparedate(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				print(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		printf("无匹配记录\n");
	}
}
void showworkers()//5.显示人员
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
		removen(line);//去换行符
		printf("%s\n", line);
	}
	fclose(fp);
}
void tongjimenu()//6.统计页面
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
			printf("1.统计已缴费+特定年份\n");
			printf("2.统计未缴费+特定年份\n");
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
		else if (tmp == 3)//预设统计直接显示所有未缴帐单数及金额，已缴帐单数及金额
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
/////////////////////////////////////////////////////////////////////////////////////
/////统计相关函数
// 统计已缴费
void yijiao()
{
	int count = 0;//记录条数
	int total = 0;//总金额
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)//找到对应用户
		{
			if (strcmp(p->yn, "y") == 0)
			{
				count++;
				total += atoi(p->sum);
			}
		}
		p = p->next1;
	}
	printf("已缴费记录：%d条，总金额：%d\n", count, total);
}

// 统计未缴费（和已缴费原理相同）
void weijiao()
{
	int count = 0;
	int total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			if (strcmp(p->yn, "n") == 0)
			{
				count++;
				total += atoi(p->sum);
			}
		}
		p = p->next1;
	}
	printf("未缴费记录：%d条，总金额：%d\n", count, total);
}

// 统计所有记录（和已缴费原理相同）
void suoyou()
{
	int count = 0;
	int total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			count++;
			total += atoi(p->sum);
		}
		p = p->next1;
	}
	printf("所有缴费记录：%d条，总金额：%d\n", count, total);
}

// 按特定年份统计（账单年份）
void nianfen()
{
	int year;//用户输入的年份
	printf("请输入年份：");
	scanf("%d", &year);
	clean(); // 清空输入缓冲区
	int count = 0;
	int total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			int y;
			sscanf(p->date, "%d,", &y);//一个字符串中按照指定格式读取数据：就提取年份2024并存储，遇到",“就停止录入，逗号也不存储
			if (y == year)
			{
				count++;
				total += atoi(p->sum);
			}
		}
		p = p->next1;
	}
	printf("%d年的账单记录：%d条，总金额：%d\n", year, count, total);
}

// 统计已缴费+特定年份（是不是已缴费不用用户输，选择这个选项就是选择已缴费了）
void yijiao_nianfen()
{
	int year;
	printf("请输入年份：");
	scanf("%d", &year);
	clean();
	int count = 0;
	int total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			if (strcmp(p->yn, "y") == 0)
			{
				int y;
				sscanf(p->date, "%d,", &y);//一个字符串中按照指定格式读取数据：就提取年份2024并存储，遇到",“就停止录入，逗号也不存储
				if (y == year)
				{
					count++;
					total += atoi(p->sum);
				}
			}
		}
		p = p->next1;
	}
	printf("%d年已缴费记录：%d条，总金额：%d\n", year, count, total);
}

// 统计未缴费+特定年份
void weijiao_nianfen()
{
	int year;
	printf("请输入年份：");
	scanf("%d", &year);
	clean();
	int count = 0;
	int total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
		{
			if (strcmp(p->yn, "n") == 0)
			{
				int y;
				sscanf(p->date, "%d,", &y);//一个字符串中按照指定格式读取数据：就提取年份2024并存储，遇到",“就停止录入，逗号也不存储
				if (y == year)
				{
					count++;
					total += atoi(p->sum);
				}
			}
		}
		p = p->next1;
	}
	printf("%d年未缴费记录：%d条，总金额：%d\n", year, count, total);
}

// 预设统计：显示已缴费和未缴费的总条数和总金额（就是把两个合起来了，原理不变）
void yvshe()
{
	int unpaid_count = 0, unpaid_total = 0;
	int paid_count = 0, paid_total = 0;
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
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
	printf("已缴费：%d条，金额%d\n", paid_count, paid_total);
	printf("未缴费：%d条，金额%d\n", unpaid_count, unpaid_total);
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
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
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
	printf("%d年及之后的账单记录：%d条，总金额：%d\n", year, count, total);
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
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
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
	printf("%d年之前的账单记录：%d条，总金额：%d\n", year, count, total);
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
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
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
	printf("金额大于等于%d的记录：%d条，总金额：%d\n", money, count, total);
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
	fee* p = head1->next1;
	while (p != NULL)
	{
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0)
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
	printf("金额小于%d的记录：%d条，总金额：%d\n", money, count, total);
}

#include "tou.h"
/// 功能
/// ////////////////////////////////////////////////////////////////
//0.跳转到功能
int tiaozhuan()
{
	int tmp = 0;
	scanf("%d", &tmp);
	if (tmp == 1)
	{
		xiugai(current);
		return 0;
	}
	else if (tmp == 2)
	{
		queryMenu();
		return 0;
	}
	else if (tmp == 3)
	{
		jiaofei();
		return 0;
	}
	else if (tmp == 4)
	{
		showall();
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
	time_t now = time(NULL); //时间戳
	struct tm* t = localtime(&now);//时间戳解析为 struct tm
	// 拼接为 "年,月,日" 格式（例如：2024,10,05）
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
				char shijian[20];

				printf("__________________________\n");
				printf("1.简单查询\n");
				printf("2.模糊查询\n");
				printf("请选择\n");
				printf("__________________________\n");
				scanf("%d", &n);
				printf("请输入时间\n");
				scanf("%20s", shijian);
				if (n == 1)
				{
					cx_by_shijian_jiandan(shijian);
					break;
				}
				else if (n == 2)
				{
					cx_by_shijian_mohu(shijian);
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
			while (1)
			{
				char jine[20];
				
				printf("__________________________\n");
				printf("1.简单查询\n");
				printf("2.模糊查询\n");
				printf("请选择\n");
				printf("__________________________\n");
				scanf("%d", &n);
				printf("请输入金额\n");
				scanf("%20s", jine);
				if (n == 1)
				{
					cx_by_jine_jiandan(jine);
					break;
				}
				else if (n == 2)
				{
					cx_by_jine_mohu(jine);
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
				scanf("%s", zhuangtai);
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
// 辅助函数：字符串转日期（年,月,日 → 整数数组）
static void zfc_to_rq(const char* s, int d[3]) {
	d[0] = d[1] = d[2] = 0;
	if (s) {
		char tmp[20];
		strcpy(tmp, s);
		char* p = strtok(tmp, ",");
		if (p) d[0] = atoi(p);
		p = strtok(NULL, ",");
		if (p) d[1] = atoi(p);
		p = strtok(NULL, ",");
		if (p) d[2] = atoi(p);
	}
}
// 辅助函数：比较日期（d1==d2返回1，否则0）
static int bj_rq(int d1[3], int d2[3]) {
	return (d1[0] == d2[0] && d1[1] == d2[1] && d1[2] == d2[2]) ? 1 : 0;
}

// 1——按时间简单查询（精确）
void cx_by_shijian_jiandan(const char* rq) {
	int mubiao[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao);
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			zfc_to_rq(p->date, dangqian_rq);
			if (bj_rq(dangqian_rq, mubiao)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
//2——按时间模糊查询
void cx_by_shijian_mohu(const char* gj) {
	// 遍历缴费链表
	fee* p = head1->next1;
	// 输出查询标题，提升可读性
	printf("\n===== 时间模糊查询结果（关键词：%s）=====\n", gj);

	// 计算关键词长度，用于前缀匹配
	int gj_len = strlen(gj);

	while (p != NULL) {
		// 仅查询当前登录业主的缴费记录（匹配地址）
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0) {

			// 核心模糊匹配逻辑：日期字符串前缀与关键词一致即匹配
			if (strncmp(p->date, gj, gj_len) == 0) {
				// 输出匹配到的记录（日期、金额、状态）
				printf("日期：%s | 金额：%s元 | 状态：%s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1; // 遍历下一条记录
	}
}
// 3——.按金额简单查询（精确）
void cx_by_jine_jiandan(const char* je) {
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			if (!strcmp(p->sum, je)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
// 4—— 金额模糊查询
void cx_by_jine_mohu(const char* gj) {
	fee* p = head1->next1;
	printf("\n===== 金额模糊查询结果（关键词：%s）=====\n", gj);

	while (p != NULL) {
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0) {

			// 核心：金额字符串中包含关键词即匹配
			if (strstr(p->sum, gj) != NULL) {
				printf("日期：%s | 金额：%s元 | 状态：%s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
// 5—— 状态+时间组合查询（仅精确匹配）
void cx_by_zhuangtai_shijian(const char* zhuangtai, const char* rq) {
	int mubiao[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao);
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			zfc_to_rq(p->date, dangqian_rq);
			if (bj_rq(dangqian_rq, mubiao)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
// 6—— 状态+金额组合查询（仅精确匹配）
void cx_by_zhuangtai_jine(const char* zhuangtai, const char* je) {
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			if (!strcmp(p->sum, je)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
// 7—— 时间+金额组合查询（仅精确匹配）
void cx_by_shijian_jine(const char* rq, const char* je) {
	int mubiao_rq[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao_rq);
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			zfc_to_rq(p->date, dangqian_rq);
			if (bj_rq(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
// 8—— 状态+时间+金额组合查询（仅精确匹配）
void cx_by_zhuangtai_shijian_jine(const char* zhuangtai, const char* rq, const char* je) {
	int mubiao_rq[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao_rq);
	fee* p = head1->next1;

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			zfc_to_rq(p->date, dangqian_rq);
			if (bj_rq(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				printf("%s %s %s\n", p->date, p->sum, p->yn);
			}
		}
		p = p->next1;
	}
}
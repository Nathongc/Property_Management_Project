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
// 工具函数：格式化输出查询结果行
static void print_fee_row(const fee* p) {
	char status[10];
	strcpy(status, strcmp(p->yn, "y") == 0 ? "已缴费" : "未缴费");
	char pay_date[20];
	strcpy(pay_date, strcmp(p->yn, "y") == 0 ? p->date2 : "——");
	printf("| %-15s | %-8s | %-6s | %-15s |\n",
		p->date, p->sum, status, pay_date);
}

// 工具函数：打印查询表头
static void print_query_header(const char* title) {
	printf("\n==================== %s ====================\n", title);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");
}

// 工具函数：打印无结果提示
static void print_no_result() {
	printf("| %-46s |\n", "无匹配记录");
	printf("==========================================================\n");
}

// 1.按时间精确查询（支持账单日期/缴费日期）
void cx_by_shijian_jiandan(const char* rq) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao);
	fee* p = head1->next1;
	int count = 0;
	const char* title = query_type == 1 ? "账单日期精确查询结果" : "缴费日期精确查询结果";
	print_query_header(title);

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			// 根据选择的维度解析日期
			if (query_type == 1) {
				zfc_to_rq(p->date, dangqian_rq);
			}
			else {
				// 缴费日期查询仅匹配已缴费记录
				if (strcmp(p->yn, "n") == 0) {
					p = p->next1;
					continue;
				}
				zfc_to_rq(p->date2, dangqian_rq);
			}

			if (bj_rq(dangqian_rq, mubiao)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 2.按时间模糊查询（支持账单日期/缴费日期）
void cx_by_shijian_mohu(const char* gj) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	fee* p = head1->next1;
	int count = 0;
	int gj_len = strlen(gj);
	const char* title = query_type == 1 ?
		"账单日期模糊查询结果" : "缴费日期模糊查询结果";
	print_query_header(title);
	printf("查询关键词：%s\n", gj);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p != NULL) {
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0) {

			char* compare_str = NULL;
			// 缴费日期模糊查询仅匹配已缴费记录
			if (query_type == 2) {
				if (strcmp(p->yn, "n") == 0) {
					p = p->next1;
					continue;
				}
				compare_str = p->date2;
			}
			else {
				compare_str = p->date;
			}

			if (strncmp(compare_str, gj, gj_len) == 0) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 3.按金额精确查询
void cx_by_jine_jiandan(const char* je) {
	fee* p = head1->next1;
	int count = 0;
	print_query_header("金额精确查询结果");

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			if (!strcmp(p->sum, je)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 4.按金额模糊查询
void cx_by_jine_mohu(const char* gj) {
	fee* p = head1->next1;
	int count = 0;
	print_query_header("金额模糊查询结果");
	printf("查询关键词：%s\n", gj);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p != NULL) {
		if (strcmp(p->add1, current->add1) == 0 &&
			strcmp(p->add2, current->add2) == 0 &&
			strcmp(p->add3, current->add3) == 0) {

			if (strstr(p->sum, gj) != NULL) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 5. 状态+时间组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian(const char* zhuangtai, const char* rq) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao);
	fee* p = head1->next1;
	int count = 0;
	char status[10];
	strcpy(status, strcmp(zhuangtai, "y") == 0 ? "已缴费" : "未缴费");
	const char* time_type = query_type == 1 ? "账单日期" : "缴费日期";
	char title[50];
	sprintf(title, "状态+%s查询结果", time_type);
	print_query_header(title);
	printf("查询条件：状态=%s | %s=%s\n", status, time_type, rq);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			// 缴费日期查询需过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				zfc_to_rq(p->date, dangqian_rq);
			}
			else {
				zfc_to_rq(p->date2, dangqian_rq);
			}

			if (bj_rq(dangqian_rq, mubiao)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 6. 状态+金额组合查询（精准匹配）
void cx_by_zhuangtai_jine(const char* zhuangtai, const char* je) {
	fee* p = head1->next1;
	int count = 0;
	char status[10];
	strcpy(status, strcmp(zhuangtai, "y") == 0 ? "已缴费" : "未缴费");
	print_query_header("状态+金额查询结果");
	printf("查询条件：状态=%s | 金额=%s元\n", status, je);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			if (!strcmp(p->sum, je)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 7. 时间+金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_shijian_jine(const char* rq, const char* je) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao_rq[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao_rq);
	fee* p = head1->next1;
	int count = 0;
	const char* time_type = query_type == 1 ? "账单日期" : "缴费日期";
	char title[50];
	sprintf(title, "%s+金额查询结果", time_type);
	print_query_header(title);
	printf("查询条件：%s=%s | 金额=%s元\n", time_type, rq, je);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3)) {
			// 缴费日期查询过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				zfc_to_rq(p->date, dangqian_rq);
			}
			else {
				zfc_to_rq(p->date2, dangqian_rq);
			}

			if (bj_rq(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}

// 8. 状态+时间+金额组合查询（精准匹配，支持账单/缴费日期）
void cx_by_zhuangtai_shijian_jine(const char* zhuangtai, const char* rq, const char* je) {
	int query_type;
	printf("请选择查询时间维度：\n1. 账单日期  2. 缴费日期\n");
	scanf("%d", &query_type);
	clean();

	int mubiao_rq[3], dangqian_rq[3];
	zfc_to_rq(rq, mubiao_rq);
	fee* p = head1->next1;
	int count = 0;
	char status[10];
	strcpy(status, strcmp(zhuangtai, "y") == 0 ? "已缴费" : "未缴费");
	const char* time_type = query_type == 1 ? "账单日期" : "缴费日期";
	char title[50];
	sprintf(title, "状态+%s+金额查询结果", time_type);
	print_query_header(title);
	printf("查询条件：状态=%s | %s=%s | 金额=%s元\n", status, time_type, rq, je);
	printf("| %-15s | %-8s | %-6s | %-15s |\n", "账单日期", "金额(元)", "状态", "缴费日期");
	printf("|-----------------|----------|--------|-----------------|\n");

	while (p) {
		if (!strcmp(p->add1, current->add1) && !strcmp(p->add2, current->add2) && !strcmp(p->add3, current->add3) && !strcmp(p->yn, zhuangtai)) {
			// 缴费日期查询过滤未缴费记录
			if (query_type == 2 && strcmp(p->yn, "n") == 0) {
				p = p->next1;
				continue;
			}

			if (query_type == 1) {
				zfc_to_rq(p->date, dangqian_rq);
			}
			else {
				zfc_to_rq(p->date2, dangqian_rq);
			}

			if (bj_rq(dangqian_rq, mubiao_rq) && !strcmp(p->sum, je)) {
				print_fee_row(p);
				count++;
			}
		}
		p = p->next1;
	}

	if (count == 0) {
		print_no_result();
	}
	else {
		printf("==========================================================\n");
	}
}
#include "tou.h"
/// 启动登录和注册
/// ////////////////////////////////////////////////////////////////
void removen(char* str)//去换行符
{
	char* pos = strchr(str, '\n');
	if (pos != NULL)
	{
		*pos = '\0';
	}
}
void freeList() {
	yezhu* current = head;
	yezhu* nextnode = NULL;
	while (current != NULL) {
		nextnode = current->next;
		free(current);
		current = nextnode;
	}
	head = NULL;
	prear = NULL;
}
int loadfee()//加载缴费到链表
{
	FILE* fp = fopen("fee.txt", "r");
	if (fp == NULL)
	{
		printf("错误，无法打开文件");
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
			printf("内存分配失败！\a\n");
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
		char* sum= strtok(NULL, ",");
		char* yn = strtok(NULL, ",");
		char* year2 = strtok(NULL, ",");
		char* month2 = strtok(NULL, ",");
		char* day2 = strtok(NULL, ",");
		if (!add1 || !add2 || !add3 || !year || !month || !day || !sum || !yn || !year2 || !month2 || !day2) {
			printf("跳过错误行：%s（字段缺失/格式错误）\n", line);
			free(newnode);
			continue;
		}
		strcpy(newnode->add1, add1);
		strcpy(newnode->add2, add2);
		strcpy(newnode->add3, add3);
		strcpy(newnode->sum, sum);
		strcpy(newnode->yn,yn);
		sprintf(newnode->date, "%s,%s,%s", year, month, day);
		sprintf(newnode->date2, "%s,%s,%s", year2, month2, day2);
		prear1->next1 = newnode;
		prear1 = newnode;
		prear1->next1 = NULL;
	}
	fclose(fp);
	return 0;
}
int loadusers()//加载用户到链表
{
	FILE* fp = fopen("user.txt", "r");
	if (fp == NULL)
	{
		printf("错误，无法打开文件");
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
			printf("内存分配失败！\a\n");
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
		if (!add1 || !add2 || !add3 || !name || !pwd) {
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
		prear->next = newnode;
		prear = newnode;
		prear->next = NULL;
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
//操作页面
void caozuo()
{
	printf("\n==================== 业主功能菜单 ====================\n");
	printf("1.修改信息");
	printf("2.缴费查询");
	printf("3.缴费");
	printf("4.显示所有缴费记录");
	printf("5.查询服务人员");
	printf("6.信息排序");
	printf("7.缴费统计");
	printf("0.退出登录");
	printf("======================================================\n");
	printf("请选择\n");
}
//注册页面
int zhuce()
{
	char add1[20];//楼号
	char add2[20];//楼层
	char add3[20];//户号
	char tel[30];//电话
	char name[30];
	char pwd[30];
	int tmp = 0;
start1:
	printf("请输入用户楼号（如“123”，1-999内）");
	scanf("%10s", add1);
	tmp = checkadd(add1, 4);
	if (tmp == 1)
	{
		printf("格式错误\a\n");
		goto start1;
	}
start2:
	printf("请输入用户楼层（如“10”，1-99内）");
	scanf("%10s", add2);
	tmp = checkadd(add2, 3);
	if (tmp == 1)
	{
		printf("格式错误\a\n");
		goto start2;
	}
start3:
	printf("请输入用户户号（如“123”，1-999内）");
	scanf("%10s", add3);
	tmp = checkadd(add3, 4);
	if (tmp == 1)
	{
		printf("格式错误\a\n");
		goto start3;
	}
	tmp = checkadd2(add1, add2, add3,NULL);
	if (tmp == 1)
	{
		return 1;
	}
start4:
	printf("请输入用户名（1-20字）");
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
	printf("请输入密码（5-20字）");
	scanf("%30s", pwd);
	tmp = checkpwd2(pwd);
	if (tmp == 1)
	{
		goto start5;
	}
	yezhu* newnode = (yezhu*)malloc(sizeof(yezhu));
	if (newnode == NULL)
	{
		printf("内存分配失败！\a\n");
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
	prear->next = newnode;
	prear = newnode;
	current = newnode;//切换当前业主
	save(prear);
	printf("注册成功\n");
	return 0;
}
// 清空残留字符
void clean()
{
	int c;
	while ((c = getchar()) != '\n' && c != EOF);
}
int checkname(char* name)//查重名（用户名）
{
	yezhu* cur = head->next;
	while (cur != NULL)
	{
		if (strcmp(cur->name, name) == 0)
		{
			printf("错误：该用户名已被占用！\n\a");
			return 1;
		}
		cur = cur->next;
	}
	return 0;//正常返回0
}
int checkname2(char* name) // 检查用户名长度(1-20)
{
	int len = strlen(name);
	if (len < 1 || len > 20)
	{
		printf("错误：用户名长度必须在1-20个字符之间！\n\a");
		clean();
		return 1;
	}
	return 0;
}
yezhu* checkname3(char* name)//登录名字
{
	yezhu* cur = head->next;
	while (cur != NULL)
	{
		if (strcmp(cur->name, name) == 0)
		{
			return cur;
		}
		cur = cur->next;
	}
	return NULL;//正常返回0
}
int checkadd2(char* add1, char* add2, char* add3, yezhu* current)//查重名（地址）
{
	yezhu* cur = head->next;
	while (cur != NULL)
	{
		if ((strcmp(cur->add1, add1) == 0) && (strcmp(cur->add2, add2) == 0) && (strcmp(cur->add3, add3) == 0))
		{
			if (current == NULL)
			{
				printf("错误：该地址已注册！\n\a");
				return 1;
			}
			else if (current!=cur)
			{
				printf("错误：该地址已注册！\n\a");
				return 1;
			}

		}
		cur = cur->next;
	}
	return 0;//正常返回0
}
int checkadd(char* add, int n)//查地址格式
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
//保存业主
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
//登录
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
			current = cur;//切换当前业主
			return 0;
		}
		else
		{
			printf("用户名或密码错误\a\n");
			return 1;
		}
	}
	
}
int checkpwd(char* pwd, yezhu* cur)//核对密码
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
int checkpwd2(char* pwd)
{
	int len = strlen(pwd);
	if (len < 5 || len > 20)
	{
		printf("错误：密码长度必须在5-20个字符之间！\n\a");
		clean();
		return 1; // 长度不合法返回1
	}
	return 0; // 合法返回0
}
void zhaohui()//密码找回
{
	char add1[20];//楼号
	char add2[20];//楼层
	char add3[20];//户号
	char tel[30];//电话
	printf("请输入用户楼号");
	scanf("%10s", add1);
	printf("请输入用户楼层");
	scanf("%10s", add2);
	printf("请输入用户户号");
	scanf("%10s", add3);
	int temp = 1;
	yezhu* cur = head->next;
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
		printf("未找到该地址\a\n");
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
		printf("电话错误\a\n");
	}
}


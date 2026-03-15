#include "Manager.h"
int main() {
    if (imfor == NULL) {
        imfor = Init_imfor(imfor);
    }
    Imfor_Read();
    Get_time();
    while (1) {
        printf("您以获得管理员权限！\n今天是%d年%d月%d日\n1.展示2.添加3.删除4.修改5.查询6.统计7.维护密码0.退出\n",year,month,day);
        int choice;
        scanf("%d", &choice);
        switch (choice) {
        case 1:
            Show_Imfor(head, imfor);
            system("pause");
            system("cls");
            break;
        case 2:
            AddImfor();
            system("pause");
            system("cls");
            break;
        case 3:
            head = Delimfor(head);
            system("pause");
            system("cls");
            break;
        case 4:
            head = ModImfor(head, imfor);
            Save(head);
            system("pause");
            system("cls");
            break;
        case 5:
            FindPerson();
            system("pause");
            system("cls");
            break;
        case 6:
            Sta_Imfor(head);
            system("pause");
            system("cls");
            break;
        case 7:
            maintain_password(head);
            system("pause");
            system("cls");
            break;
        case 0:
            Save(head);
            exit(0);
            break;
        default:
            break;
        }
    }
    return 0;
}
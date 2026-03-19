#include "Manager.h"
int main() {
    Authorize();
    system("cls");
    if (imfor == NULL) {
        imfor = Init_imfor(imfor);
    }
    Imfor_Read();
    Get_time();
    while (1) {
        printf("您已获得管理员权限！\n今天是%d年%d月%d日\n1.展示\n2.添加\n3.删除\n4.修改\n5.查询\n6.统计\n7.维护密码\n8.账单生成\n9.排序\n10.备份\n0.退出\n", year, month, day);
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
        case 8:
            Generate();
            system("pause");
            system("cls");
        case 9:
            head = Sort_Person(head);
            system("pause");
            system("cls");
            break;
        case 10:
            Backup(imfor,head);
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
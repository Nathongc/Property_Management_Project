#include "../include/main.h"

int main() {
    if (imfor == NULL) {
        imfor = Init_imfor(imfor);
    }
    Imfor_Read();
    Get_time();
    while (1) {
        system("clear");
        //先决定进入哪一个系统界面
        printf("今天是%d年%d月%d日\n1.进入管理员界面\n2.进入物业服务人员界面\n3.进入业主界面\n0.退出\n",year,month,day);
        int first_choice;
        scanf("%d", &first_choice);
        switch (first_choice) {
        case 1:
            manager_system();
            break;
        case 2:
            server_system();
            break;
        case 3:
            owner_system();
            break;
        default:
            printf("谢谢使用，再见!\n");
            exit(0);
            break;
        }
    }
    return 0;
}
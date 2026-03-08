#include "Manager.h"
int main(){
    Person person;
    Server server;
    Owner owner;
    AddImfor(&person,&server,&owner);
    ShowServer_Imfor(server_head);
    return 0;
}
#include <stdio.h>
#include "picomms.h"

void drive(){
    int initialleft,initialright,left,right;
    get_motor_encoders(&initialleft,&initialright);
    while(left<=initialleft+1146){
        get_motor_encoders(&left,&right);
        set_motors(20,20);
        log_trail();
    }
    initialleft=left;
    while(left<=initialleft+215){
        get_motor_encoders(&left,&right);
        set_motors(20,-20);
        log_trail();
    }
}

int main(){
    connect_to_robot();
	initialize_robot();
    
    
    drive();
    drive();
    drive();
    drive();
    return 0;
}
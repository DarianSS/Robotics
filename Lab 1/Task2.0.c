#include <stdio.h>
#include <unistd.h>
#include "picomms.h"

void rightangle(){
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

int main(void) {
	connect_to_robot();
	initialize_robot();
	rightangle();
	return 0;
}




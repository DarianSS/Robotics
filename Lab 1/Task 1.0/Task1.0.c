#include <stdio.h>
#include "picomms.h"

int main(){
	connect_to_robot();
	initialize_robot();
	int speed=20;	
    while(1){
        set_motor(LEFT, speed);
        set_motor(RIGHT, -speed);
    }
    return 0;
}
#include <stdio.h>
#include "picomms.h"
#include <unistd.h>
void drive(){

	int k = 0;
	while(k < 50){
		set_motor(LEFT, 50);
		set_motor(RIGHT, 50);
		k = k+1;
	}
	for(int i = 0; i < 75; i++){
	set_motor(LEFT, -25);
	set_motor(RIGHT, 25);
	}
}

int main() {
	connect_to_robot();
	initialize_robot();
	drive();
	drive();
	drive();
	drive();
	return 0;
}


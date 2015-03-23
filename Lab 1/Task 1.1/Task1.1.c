#include <stdio.h>
#include "picomms.h"
int main() {
connect_to_robot();
initialize_robot();
int i;
for (i = 0; i < 50; i++) {
	int speed = 50;
	set_motor(LEFT, speed);
	set_motor(RIGHT, speed);
}
}
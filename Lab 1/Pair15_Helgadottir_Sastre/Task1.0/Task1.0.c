#include <stdio.h>
#include "picomms.h"
int main() {
connect_to_robot();
initialize_robot();
while (1) {
	int speed = -5;
	set_motor(LEFT, speed);
	set_motor(RIGHT, -speed);
}
}
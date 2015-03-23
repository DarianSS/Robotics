#include <stdio.h>
#include "picomms.h"
int main() {
connect_to_robot();
initialize_robot();
    
int i = 0;
while (i < 100)
{
 	int speed = 10;
	set_motor(LEFT, speed);
	set_motor(RIGHT, speed);
	i++;
	if (i % 25 > 0 && i % 25 < 6)
	{
	set_motor(LEFT, 0);
	set_motor(RIGHT, 50);
	}
}
    return 0;
}
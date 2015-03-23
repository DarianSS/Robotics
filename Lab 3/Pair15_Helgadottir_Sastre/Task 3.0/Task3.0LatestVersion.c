#include <stdio.h>
#include "picomms.h"

#define SIDE_DIST 23
#define FRONT_DIST 35
#define FRONT_DIST_US 20

#define GAIN 80
#define MAX_DIFF 35

#define GAIN_MIN 0.3
#define GAIN_MAX 0.98

#define SPEED 80
#define METER 1146.0
#define SLOW_DIST METER*0.1


float speed = SPEED;
int fleftdist,frightdist,US;

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	US = get_us_dist();
}

int speed_correction(int actual_dist, int target_dist)
{
	//float new_gain = 0.0, difference = 0.0;
	float difference = (float)target_dist - (float)actual_dist;
	float speed_correction = GAIN * (difference / MAX_DIFF);
	return speed_correction;
}

void move(){
	while(1){
		sensors_read();
/*		if (US<=20)  //STOP case
		{  
			if (US<=13)
			{
				break;
			}
			speed = speed_correction(US,13);
			set_motors(speed,speed);
		}*/
		//else
		{
			float correction = speed_correction(fleftdist, FRONT_DIST);
			set_motors(speed + correction, speed - correction);
		}
	}
}

int main(){
	connect_to_robot();
	initialize_robot();
	set_ir_angle(RIGHT,-45);
	move();
	set_motors(0,0);
	return 0;
}
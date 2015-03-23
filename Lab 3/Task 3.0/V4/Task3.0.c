#include <stdio.h>
#include "picomms.h"
#include <math.h>


//#define SIDE_DIST 23
#define FRONT_DIST 35
#define FRONT_DIST_US 23

#define GAIN 80/50
#define STOPPING_GAIN SPEED
#define MAX_DIFF 35

#define SPEED 50


float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	get_side_ir_dists(&sleftdist, &srightdist);
	US = get_us_dist();
}

int speed_correction(int actual_dist, int target_dist, int gain, int limit)
{
	float difference = (float)target_dist - (float)actual_dist;
	float speed_correction = gain * difference;
	if (speed_correction < -limit)
	{
		return -limit;
	}
	else
	{
		return speed_correction;
	}
}

void move(){
	while(1){
		sensors_read();
		if ((US<=FRONT_DIST_US) && ((sleftdist>=22) && (sleftdist<=25)))                //STOP case
		{  
			if (US<=FRONT_DIST_US-5)
			{
				printf("Stopped\n");
				break;
			}
			float correction = speed_correction(US, 13, STOPPING_GAIN, SPEED);
			set_motors(fabs(correction), fabs(correction));
			printf("Stopping with speed %f and correction %f\n", speed+correction, correction);

		}
		else
		{
			float correction = speed_correction(fleftdist, FRONT_DIST, GAIN, MAX_DIFF);
			set_motors(speed + correction, speed - correction);
			printf("Correction is: %f\n", correction);
		}
	}
}

int main(){
	connect_to_robot();
	initialize_robot();
	//set_ir_angle(RIGHT,-45);
	move();
	set_motors(0,0);
	return 0;
}
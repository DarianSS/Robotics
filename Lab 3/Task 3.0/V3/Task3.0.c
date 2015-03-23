#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define SIDE_DIST 23
#define FRONT_DIST 35
#define FRONT_DIST_US 20

#define GAIN 0.1
#define GAIN_MIN 0.3
#define GAIN_MAX 0.98

#define SPEED 50

float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	//get_side_ir_dists(&sleftdist, &srightdist);
	US = get_us_dist();
}

int new_speed(int actual_dist, int target_dist)
{
	float new_gain = 0.0;
	float difference = (float)target_dist - (float)actual_dist;
	printf("   Gain\n");
	printf("   difference %f and gain * diff %f \n", difference, difference * GAIN);
	if ((difference * GAIN <= GAIN_MAX) && (difference * GAIN >= GAIN_MIN))
	{
		new_gain = 1 - difference * GAIN;
		printf("   If 1\n");
	}
	else if (difference * GAIN > GAIN_MAX)
	{
		new_gain = 1 - GAIN_MAX;
		printf("   If 2\n");
	}
	else 
	{
		new_gain = 1 - GAIN_MIN;
		printf("   If 3\n");
	}
	printf("   new gain: %f\n   ---\n",new_gain);
	return new_gain * SPEED;
}

int new_new_speed(int actual_dist, int target_dist)
{
	float new_gain = 0.0;
	float difference = (-1)*((float)target_dist - (float)actual_dist);
	printf("   Gain\n");
	printf("   difference %f and gain * diff %f \n", difference, difference * GAIN);
	if ((difference * GAIN <= GAIN_MAX) && (difference * GAIN >= GAIN_MIN))
	{
		new_gain = 1 - difference * GAIN;
		printf("   If 1\n");
	}
	else if (difference * GAIN > GAIN_MAX)
	{
		new_gain = 1 - GAIN_MIN;
		printf("   If 2\n");
	}
	else 
	{
		new_gain = 1 - GAIN_MAX;
		printf("   If 3\n");
	}
	printf("   new gain: %f\n   ---\n",new_gain);
	return new_gain * SPEED;
}

void move(){
	while(1){
		sensors_read();
		if (US<=20)              //STOP case
		{  
			if (US<=13)
			{
				break;
			}
			printf("Stopping\n");
			printf("ultrasound dist is: %d\n",US);
			speed=new_new_speed(US,13);
			printf("speed is: %f\n----\n",speed);
			set_motors(speed,speed);
		}
		else
		{
			printf("else statement\n");
			if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1)
			{
				set_motors(speed,speed);
			}
			else if (fleftdist > FRONT_DIST+1)
			{
				set_motors(new_speed(fleftdist,FRONT_DIST), speed);
			}
			else if (fleftdist < FRONT_DIST-1) 
			{
				set_motors(speed, new_speed(fleftdist,FRONT_DIST));
			}
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

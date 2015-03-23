#include <stdio.h>
#include "picomms.h"

//#define SIDE_DIST 20
#define FRONT_DIST 32
#define SPEED 80
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98


float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

void readings(){
	get_front_ir_dists(&fleftdist, &frightdist);
	//get_side_ir_dists(&sleftdist, &srightdist);
	//US = get_us_dist();
}

int new_speed()
{
	float new_gain;
	float difference = FRONT_DIST - fleftdist;
	if (difference * GAIN <= NEW_GAIN_MAX && difference * GAIN >= NEW_GAIN_MIN)
	{
		new_gain = 1 - difference * GAIN;
	}
	else if (difference * GAIN > NEW_GAIN_MAX)
	{
		new_gain = 1 - NEW_GAIN_MAX;
	}
	else 
	{
		new_gain = 1 - NEW_GAIN_MIN;
	}
	return new_gain * SPEED;
}

void move(){
	while (1){
	readings();
	if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1)
	{
		set_motors(speed,speed);
		printf("forward\n");
	}
	else if (fleftdist > FRONT_DIST)
	{
		set_motors(new_speed(), speed);
		printf("Move to left\n");
	}
	else 
	{
		set_motors(speed, new_speed());
		printf("Move to right - new speed is %i\n", new_speed());

	}
}
}

int main(){
	connect_to_robot();
	initialize_robot();
	move();
	set_motors(0,0);
	return 0;
}

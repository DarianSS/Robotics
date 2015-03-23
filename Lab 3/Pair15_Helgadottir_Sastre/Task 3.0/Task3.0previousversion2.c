//redo stop clause
// //make new slowing down function while following wall

#include <stdio.h>
#include "picomms.h"

#define SIDE_DIST 23
#define FRONT_DIST 35
#define FRONT_DIST_US 20

#define GAIN 0.1
#define GAIN_MIN 0.3
#define GAIN_MAX 0.98

#define SPEED 50
#define METER 1146
#define SLOW_DIST METER*0.1


float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

int new_slowing_speed(int distance, int S){
    double x=(distance*(METER/100))*(double)SPEED/(double)SLOW_DIST;
    if (x>S){ 
        return x;
    }
    else{
        return S;   //makes the new speed not to decrease beyond threshold S
    }
}

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	//get_side_ir_dists(&sleftdist, &srightdist);
	US = get_us_dist();
}

int new_speed()
{
	float new_gain;
	float difference = FRONT_DIST - fleftdist;
	if (difference * GAIN <= GAIN_MAX && difference * GAIN >= GAIN_MIN)
	{
		new_gain = 1 - difference * GAIN;
	}
	else if (difference * GAIN > GAIN_MAX)
	{
		new_gain = 1 - GAIN_MAX;
	}
	else 
	{
		new_gain = 1 - GAIN_MIN;
	}
	return new_gain * SPEED;
}

void move(){
	while(1){
		sensors_read();
		if (get_us_dist()<=20)  //STOP case
		{  
			speed=new_slowing_speed(get_us_dist()-10,0);
			set_motors(speed,speed);
		}
		else
		{
			if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1)
			{
				set_motors(speed,speed);
			}
			if (fleftdist > FRONT_DIST+1)
			{
				set_motors(new_speed(), speed);
			}
			if (fleftdist < FRONT_DIST-1) 
			{
				set_motors(speed, new_speed());
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

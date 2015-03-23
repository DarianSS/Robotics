#include <stdio.h>
#include "picomms.h"
#include <math.h>

#define FRONT_DIST 32
#define SPEED 80
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define CENTIMETER 11.46

float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist;
int ticks_left = 0, ticks_right = 0;
float previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0;
float final_angle = 0, cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;
float d_y = 0;
float d_x = 0;

void readings(){
	get_front_ir_dists(&fleftdist, &frightdist);
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

void angle_change()
{
	get_motor_encoders(&ticks_left, &ticks_right);
	
	previous_cm_right = cm_right;   		  		//the length right wheel had travelled before the current reading
	previous_cm_left = cm_left;
	cm_left = ticks_left / CENTIMETER;				//length that left wheel has travelled from beginning in cm
	cm_right = ticks_right / CENTIMETER;			//length that right wheel has travelled from beginning in cm
	float d_l = cm_left - previous_cm_left;
	float d_r = cm_right - previous_cm_right;
	delta_angle = (d_l - d_r) / ROBOT_WIDTH;
	if (delta_angle == 0)
	{
		d_y = d_r * cos(final_angle);
		d_x = d_r * sin(final_angle);
	}
	else 
	{
		float r_r = d_r / delta_angle;
		float r_l = d_l / delta_angle;
		float r_m = (r_l + r_r) / 2.0;
		d_y = r_m * (sin(delta_angle + final_angle) - sin(final_angle));
		d_x = r_m * (cos(final_angle) - cos(delta_angle + final_angle));
	}
	total_d_y = total_d_y + d_y;
	total_d_x = total_d_x + d_x;
	final_angle = final_angle + delta_angle; // * M_PI / 180 for degrees;		//what the current angle of the robot is including this reading
	set_point(total_d_x, total_d_y);
	printf("Distance in loop is: %f , %f cm.\n", total_d_x, total_d_y);
}

void move(){
	while (1){
	readings();
	angle_change();
	if (frightdist < 23 && frightdist >= 14)
	{
		set_motors(5,5);
	}
	else if (frightdist < 14)
	{
		break;
	}
	else if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1)
	{
		set_motors(speed,speed);
	}
	else if (fleftdist > FRONT_DIST)
	{
		set_motors(new_speed(), speed);
	}
	else 
	{
		set_motors(speed, new_speed());
	}
}
}

int main(){
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);
	move();
	set_motors(0,0);
	printf("The final angle is: %f radians.\n", final_angle);
	printf("The final distance is: %f, %f cm.\n", total_d_x, total_d_y);
	return 0;
}

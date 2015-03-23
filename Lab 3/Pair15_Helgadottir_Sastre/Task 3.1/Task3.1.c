#include <stdio.h>
#include "picomms.h"
#include <math.h>

#define FRONT_DIST 32
#define SPEED 80 //80
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define CENTIMETER 11.46

float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist;
int ticks_left = 0, ticks_right = 0;
float previous_cm_right = 0, previous_cm_left = 0, previous_delta_angle = 0, delta_angle = 0;
float final_angle = 0, previous_angle_from_start = 0, cm_right = 0, cm_left = 0, distance = 0, total_distance = 0, total_d_y = 0, total_d_x = 0;
float d_y = 0;
float d_x = 0;

struct coords {
	int x=0,y=0;
	struct coords *next, *previous;
};

struct coords *start, *current, *new;

struct coords *create(void)
{
	struct stats *baby;

	baby = (struct coords *)malloc(sizeof(struct coords));
	if(baby == NULL)
	{
		puts("Memory error");
		exit(1);
	}
	return(baby);
}

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
	previous_delta_angle = delta_angle; 		//the angle of the previous delta step
	previous_cm_right = cm_right;   		  		//the length right wheel had travelled before the current reading
	previous_cm_left = cm_left;
	cm_left = ticks_left / CENTIMETER;				//length that left wheel has travelled from beginning in cm
	cm_right = ticks_right / CENTIMETER;			//length that right wheel has travelled from beginning in cm
	float d_l = cm_left - previous_cm_left;
	float d_r = cm_right - previous_cm_right;
	delta_angle = (d_l - d_r) / ROBOT_WIDTH;
	final_angle = final_angle + delta_angle; // * M_PI / 180;				//what the current angle of the robot is including this reading
	float r_m = ((d_r + d_l) * 2 ) / delta_angle;
	if (delta_angle == 0)
	{
		d_y = d_r * sin(previous_delta_angle);
		d_x = d_r * cos(previous_delta_angle);
		//distance = sqrt(2*(d_r * d_r));
	}
	else 
	{
		d_y = r_m * (sin(delta_angle + previous_delta_angle) - sin(previous_delta_angle));
		d_x = r_m * (cos(previous_delta_angle) - cos(delta_angle + previous_delta_angle));
		//distance = sqrt(d_x * d_x + d_y * d_y);
	}
	//total_distance = total_distance + distance;
	total_d_x = total_d_x + d_x;
	total_d_y = total_d_y + d_y;

	current->x=total_d_x;
	current->y=total_d_y;

	printf("Distance in loop is: %f , %f cm.\n", d_x, d_y);
}

void move(){
	while (1){
		readings();
		
		new = create();
	    new->previous=current;
	    current->next = new;
	    current = new;

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
	set_ir_angle(1, -45);

	start = create();
	start->previous = NULL;
	current = start;

	move();
	set_motors(0,0);
	printf("The final angle is: %f radians.\n", final_angle);
	printf("The final distance is: %f, %f cm.\n", total_d_x, total_d_y);
	return 0;
}

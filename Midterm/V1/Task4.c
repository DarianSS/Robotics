//I added a parameter to the new speed function, check it out and
//in the turn function I just put the random number 100 in there
//maybe you get my idea


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define FRONT_DIST 32
#define SPEED 80 
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define METER 1146
#define CENTIMETER 11.46
#define SLOW_DIST METER*0.1

float speed = SPEED, d_x = 0, d_y = 0;
int ticks_left = 0, ticks_right = 0;
int fleftdist,frightdist,sleftdist,srightdist;
float previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0;
float final_angle = 0, cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;

typedef struct coords
{
	int x, y;
	struct coords *next, *previous;
} coords;

coords *start, *current, *new;

coords *create(void)
{
	coords *baby;

	baby = (coords *)malloc(sizeof(coords));

	baby->x = 0;
	baby->y = 0;
	if(baby == NULL)
	{
		puts("Memory error");
		exit(1);
	}
	return(baby);
}

void readings()
{
	get_front_ir_dists(&fleftdist, &frightdist);
}

int new_speed(int reading)
{
	float new_gain;
	float difference = FRONT_DIST - reading;
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

	new = create();
	    new->previous = current;
	    current->next = new;
	    current = new;
	    
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
	total_d_x += d_x;
	total_d_y += d_y;
	final_angle += delta_angle; // * M_PI / 180 for degrees;		//what the current angle of the robot is including this reading

	current->x=d_x;
	current->y=d_y;
	set_point(total_d_x, total_d_y);
	//printf("Distance in loop is: %f , %f cm.\n\n", d_x, d_y);
}

void move()
{
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
			set_motors(new_speed(fleftdist), speed);
		}
		else 
		{
			set_motors(speed, new_speed(fleftdist));
		}
	}
}

void turn()
{	
	reset_motor_encoders();
	int initialleft,initialright,left,right;
    get_motor_encoders(&initialleft,&initialright);
    while ((left-right)/2<=(initialleft+initialright)/2+385)
    {
        get_motor_encoders(&left,&right);
        set_motors(50,-50);
    }
    set_motors(0,0);
}

void trace()
{
	while (current)
	{
		printf("Coordinates: X=%d Y=%d\n",current->x,current->y);
		current=current->previous;
	}
}

void go_to_point()
{

}

int main()
{
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);

	start = create();
	start->previous = NULL;
	current = start;

	move();
	turn();
	trace();
	set_motors(0,0);
	printf("The final angle is: %f radians.\n", final_angle);
	printf("The final distance is: %f, %f cm.\n", total_d_x, total_d_y);
	return 0;
}

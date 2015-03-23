#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define FRONT_DIST 32
#define SPEED 50.0
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define METER 1146
#define CENTIMETER 11.46
#define SLOW_DIST METER*0.1

float speed = SPEED, d_x = 0, d_y = 0;
int ticks_left = 0, ticks_right = 0, i;
int fleftdist,frightdist,sleftdist,srightdist;
float previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0;
float final_angle = 0, cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;
float target_point_x = 0, target_point_y = 0;

void setLinkedList(float total_d_x, float total_d_y);

float sqr(float x){
	return x*x;
}
float hypotenuse(float x, float y){
	return sqrt(sqr(x)+sqr(y));
}

typedef struct coords
{
	float x;
	float y;
	struct coords *next, *previous;
} coords;

coords *start, *current, *new;

coords *create(void)
{
	coords *baby;
	baby = (coords *)malloc(sizeof(coords));
	baby->x = 0.0;
	baby->y = 0.0;
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

void setLinkedList(float total_d_x, float total_d_y)
{
	new = create();
	new->previous = current;
	current->next = new;
	current = current->next;
	current->x=total_d_x;
	current->y=total_d_y;
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
	float d_m = (d_r + d_l) / 2.0;
	delta_angle = (d_r - d_l) / ROBOT_WIDTH;
	total_d_x += d_m * cos(final_angle);
	total_d_y += d_m * sin(final_angle);
	final_angle += delta_angle; // * M_PI / 180 for degrees;		//what the current angle of the robot is including this reading
	final_angle = fmod((final_angle + M_PI), 2*M_PI) - M_PI;
	//printf("Distance in loop is: x: %f , y: %f cm.\n", total_d_x, total_d_y);
	//printf("Angle%f\n", final_angle);
}

void move()
{
	while (1){
		readings();
	    angle_change();
	    setLinkedList(total_d_x, total_d_y);
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

void target_point()
{
	//printf("%f\n", hypotenuse(total_d_y - current->y, total_d_x - current->x));
	if (hypotenuse(total_d_y - current->y, total_d_x - current->x) > 25.0) //15 cm CONFIRM THAT IT IS IN CM!
	{
		//printf("Reached\n");
		target_point_x = current ->x;
		target_point_y = current ->y;
	}
	else
	{
		current = current -> previous;
	}
}

void drive_to_target()
{
	double heading_r_to_p = atan2(target_point_y - total_d_y, target_point_x - total_d_x);
	angle_change();
	double error = fmod(final_angle - heading_r_to_p + 2*M_PI, 2*M_PI);
	printf("%f\n",error);
	if (fabs(error)<3)
	{
		set_motors((int)round(SPEED + (SPEED * error)), (int)round(SPEED - (SPEED * error)));
	}
}

void trace()
{
	while(hypotenuse(total_d_y - current ->y, total_d_x - current ->x) < 25.0) {
		//printf("Is smaller than 25.\n");
		current = current->previous;
		set_point(current ->x, current ->y);
	}
	//printf("Target point: %f %f \n", current ->x, current ->y);
	while(current)
	{
		angle_change();
		set_point((int)current->x, (int)current->y);
		//if(hypotenuse(total_d_y - current->y, total_d_x - current->x) < 25.0)
			target_point();
		drive_to_target();
	}
	target_point_x = current ->x;
	target_point_y = current ->y;
	while(hypotenuse((total_d_x - current->x), (total_d_y - current->y)))
	{
		drive_to_target();
	}
}

void turn()
{
    for (int i=0; i<137; i++)
    {
        set_motors(50,-50);
    }
    set_motors(0,0);
}

/*void printLinkedList()
{
	while (current)
	{
		printf("x: %f y: %f\n", current->x, current->y);
		current = current->previous;
	}
}*/

int main()
{
	final_angle = M_PI / 2;
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);

	start = create();
	start->previous = NULL;
	current = start;
	move();
	turn();
	//printLinkedList();
	trace();
	set_motors(0,0);
	//printf("The final angle is: %f radians.\n", final_angle);
	//printf("The final distance is: %f, %f cm.\n", total_d_x, total_d_y);
	return 0;
}

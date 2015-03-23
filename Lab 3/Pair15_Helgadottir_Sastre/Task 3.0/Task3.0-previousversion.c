//Darians code

#include <stdio.h>
#include "picomms.h"

#define SIDE_DIST 20
#define FRONT_DIST_US 20
#define FRONT_DIST 32
#define SPEED 80
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define METER 1146
#define SLOW_DIST METER*0.1


float speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	get_side_ir_dists(&sleftdist, &srightdist);
	US = get_us_dist();
}

int new_slowing_speed(int distance, int S){
    double x = distance * (double)SPEED / (double)SLOW_DIST;
    if (x > S){ 
        return x;
    }
    else{
        return S;   //makes the new speed not to decrease beyond threshold S
    }
}

int new_speed(){
	float new_gain;
	float difference = FRONT_DIST - fleftdist;
	if (difference * GAIN <= NEW_GAIN_MAX && difference * GAIN >= NEW_GAIN_MIN){
		new_gain = 1 - difference * GAIN;
	}
	else if (difference * GAIN > NEW_GAIN_MAX){
		new_gain = 1 - NEW_GAIN_MAX;
	}
	else {
		new_gain = 1 - NEW_GAIN_MIN;
	}
	return new_gain * SPEED;
}

void move() {
	while (1){
	sensors_read();
	if (fleftdist < FRONT_DIST-1 && sleftdist <= SIDE_DIST+1 && sleftdist >= SIDE_DIST-1 && frightdist > FRONT_DIST_US) {
		set_motors(0,0);
	}

	/*if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1 && sleftdist <= SIDE_DIST+1 && sleftdist >= SIDE_DIST-1 && frightdist <= 35){  //STOP case
		set_motors(new_slowing_speed(get_us_dist() - 10,0),new_slowing_speed(get_us_dist() - 10,0));
		printf("Darian Slow Down\n");
	}*/
	/*else if (US <= FRONT_DIST_US || (sleftdist < SIDE_DIST/2 && fleftdist < sleftdist*1.5)){
		set_motors(speed,-speed);
		printf("Darian Turn on the spot\n");
	}*/
	/*if (frightdist < 27) {
		set_motors(0,0);
	}*/
	else if (fleftdist <= FRONT_DIST+1 && fleftdist >= FRONT_DIST-1 && sleftdist <= SIDE_DIST+1 && sleftdist >= SIDE_DIST-1) {
		set_motors(speed,speed);
		printf("forward\n");
	}
	else if (fleftdist > FRONT_DIST) {
		set_motors(new_speed(), speed);
		printf("Move to left\n");
	}
	else {
		set_motors(speed, new_speed());
		printf("Move to right - new speed is %i\n", new_speed());

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


// IF droppes below fleftdist and side is correct and us / rsidedist droppes below a certain value: slow down


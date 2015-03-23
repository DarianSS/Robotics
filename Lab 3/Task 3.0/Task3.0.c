//if you want to tweak the speed you should  
 //also modify the turning from lines 39 and 43

//implement ultrasound for stopping clause


//REDO STOPPING CLAUSE
//CHECK SLOWING DOWN WHILE FOLLOWING (NOT STOPPING AT THE 90 DEGREE)
 	//might try getting motor encoders at the start of the stopping sequence

#include <stdio.h>
#include "picomms.h"

#define SIDE_DIST 23
#define FRONT_DIST 32
#define FRONT_DIST_US 20
#define MIN_FRONT_DIST FRONT_DIST-3
#define MAX_FRONT_DIST FRONT_DIST+4

#define SPEED 40
#define METER 1146
#define SLOW_DIST METER*0.1

int speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;


int new_slowing_speed(int distance, int S){
    double x=(distance*(METER/100))*(double)SPEED/(double)SLOW_DIST;
    //printf("%f\n",x);
    if (x>S){ 
        return x;
    }
    else{
        return S;   //makes the new speed not to decrease beyond threshold S
    }
}

void sensors_read(){
	get_front_ir_dists(&fleftdist, &frightdist);
	get_side_ir_dists(&sleftdist, &srightdist);
	US=get_us_dist();
}

void follow_wall(){
	while(1){
		sensors_read();
		if (fleftdist==FRONT_DIST && sleftdist==SIDE_DIST && frightdist<=35){  //STOP case
			speed=new_slowing_speed(get_us_dist()-10,0);
			set_motors(speed,speed);
		}
		else{
			if (US<=FRONT_DIST_US || (sleftdist<SIDE_DIST/2 && fleftdist<sleftdist*1.5)){
				set_motors(speed,-speed);
				//printf("Case 0 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
			}
			else{
				if (fleftdist <= MAX_FRONT_DIST && fleftdist > MIN_FRONT_DIST){ 
					set_motors(speed,speed); 
					//printf("Case 1 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
				if (fleftdist < MIN_FRONT_DIST){
					set_motors(speed,speed*2/3);
					//printf("Case 2 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
				if (fleftdist > MAX_FRONT_DIST){
					set_motors(speed*2/3,speed);
					//printf("Case 3 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
			}
		}
	}
}

int main(){
	connect_to_robot();
	initialize_robot();
	set_ir_angle(RIGHT,-45);
	follow_wall();
	set_motors(0,0);
	return 0;
}

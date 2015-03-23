#include <stdio.h>
#include "picomms.h"

#define SIDE_DIST 20
#define FRONT_DIST 29
#define SPEED 30

int speed = SPEED;
int fleftdist,frightdist,sleftdist,srightdist,US;

void readings(){
	get_front_ir_dists(&fleftdist, &frightdist);
	get_side_ir_dists(&sleftdist, &srightdist);
	US=get_us_dist();
}

void move(){
	while (1){
	readings();
			if (US<=20 || (sleftdist<10 && fleftdist<sleftdist*1.5)){
				set_motors(speed,-speed);
				//printf("Case 0 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
			}
			else{
				if (fleftdist <= FRONT_DIST+3 && fleftdist > FRONT_DIST-2){ 
					set_motors(speed,speed); 
					//printf("Case 1 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
				if (fleftdist < FRONT_DIST-2){
					set_motors(speed,speed/2);
					//printf("Case 2 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
				if (fleftdist > FRONT_DIST+3){
					set_motors(speed/2,speed);
					//printf("Case 3 | side-left: %d side-right: %d front-left: %d front-right: %d\n",sleftdist,srightdist,fleftdist,frightdist);
				}
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

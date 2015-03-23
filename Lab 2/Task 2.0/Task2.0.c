#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "picomms.h"

#define SPEED 50
#define METER 1146
#define SLOW_DIST METER*0.1

int new_speed(int distance){
    double x=distance*(double)SPEED/(double)SLOW_DIST;
    //printf("%f\n",x);
    if (x>10){ 
        return x;
    }
    else{
        return 10;
    }
}

void straight(int dist){
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    while ((left+right)/2<=(initialleft+initialright)/2+dist){
        get_motor_encoders(&left,&right);
        if ((initialleft+initialright)/2+dist-(left+right)/2<=SLOW_DIST)
            speed=new_speed((initialleft+initialright)/2+dist-(left+right)/2);
        set_motors(speed,speed);
        log_trail();
    }
}

void turn(int degrees){
    reset_motor_encoders();
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    while ((left-right)/2<=(initialleft+initialright)/2+degrees){
        get_motor_encoders(&left,&right);
        if ((initialleft+initialright)/2+degrees-(left-right)/2<=SLOW_DIST)
            speed=new_speed((initialleft+initialright)/2+degrees-(left-right)/2);
        set_motors(speed,-speed);
        log_trail();
    }
    set_motors(0,0);
}

int main() {
	connect_to_robot();
	initialize_robot();
	straight(METER);
    turn(205);
    straight(METER);
    turn(310);
    straight(METER*sqrt(2));
    set_motors(0,0);
	return 0;
}




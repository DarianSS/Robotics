#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "picomms.h"

#define MAX_VOLTAGE 60
#define FULL_CIRCLE 870 // degrees both wheels need to rotate for the robot to make a full circle
#define METRE 900       // degrees a wheel needs to rotate to travel a metre
#define INERTIA 2

#ifndef max
#define max(a, b) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
#define min(a, b) ( ((a) < (b)) ? (a) : (b) )
#endif

int clamp(int value, int lower, int upper) {
    return max(min(value, upper), lower);
}

int sign(int value) {
    return (value < 0) ? -1 : 1;
}

typedef struct {
    int left;
    int right;
} sides;

void get_motor_encodings(sides *a) {
    two_sensor_read("MELR", &((*a).left), &((*a).right));
}	
int voltage_scale(int voltage) {
    return clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    if (voltage != 0 && voltage/2 == 0) return sign(voltage);
    else return (voltage / 2);
}

void move(char action, int deg) {
    
    sides initial,
    current,
    previous,
    diff,      // difference between current and previous values
    travelled, // difference between current and initial values
    remaining,
    voltage,
    target;
    
    get_motor_encodings(&initial);
    
    current.left  = initial.left;
    current.right = initial.right;
    
    if (action == 'r') {
        // Rotate
        target.left  = deg;
        target.right = -deg;
    } else {
        // Straight line
        target.left = target.right = deg;
    }
    
    do {
        
        log_trail();
        
        previous.left  = current.left;
        previous.right = current.right;
        
        get_motor_encodings(&current);
        
        diff.left  = current.left  - previous.left;
        diff.right = current.right - previous.right;
        
        travelled.left  = current.left  - initial.left;
        travelled.right = current.right - initial.right;
        
        remaining.left  = target.left  - travelled.left;
        remaining.right = target.right - travelled.right;
        
        voltage.left  = voltage_scale(remaining.left - (diff.left * INERTIA));
        voltage.right = voltage_scale(remaining.right - (diff.right * INERTIA));
        
        if (action == 's' || action == 'r') {
            if (remaining.left > remaining.right) {
                voltage.right = sign(voltage.right) * (abs(voltage.right) - 1);
            } else if (remaining.left < remaining.right) {
                voltage.left  = sign(voltage.left)  * (abs(voltage.left)  - 1);
            } else {
                voltage.left  = sign(voltage.left)  * abs(max(voltage.left, voltage.right));
                voltage.right = sign(voltage.right) * abs(max(voltage.left, voltage.right));
            }
        }
        
        set_motors(voltage.left, voltage.right);
        
        printf("%c %3d %3d %3d %3d %3d %3d\n", action, travelled.left, travelled.right, voltage.left, voltage.right, diff.left, diff.right);
        
        usleep(10000);
        
    } while (voltage.left != voltage.right || abs(voltage.left) > 1 || abs(diff.left) > 1 || abs(diff.right) > 1);
    
    
    set_motors(0, 0);
    usleep(10000);
}

void rotate(double angle) {
    move('r', (int)((angle * FULL_CIRCLE) / 360) + 0.5);
}

void straight(double distance) {
    move('s', (int)((distance * METRE) + 0.5));
}

int main() {
    int i;
    connect_to_robot();
    initialize_robot();
    // for (i = 0; i < 4; i++) {
    //   straight(0.8);
    //   rotate(90);
    // }
    rotate(30);
    for (i = 0; i < 3; i++) {
        straight(0.7);
        rotate(90);
    }
    return 0;
}
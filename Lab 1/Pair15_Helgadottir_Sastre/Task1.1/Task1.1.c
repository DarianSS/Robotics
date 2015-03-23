
#include <stdio.h>
#include "picomms.h"

int main() {
int speed = 10;
connect_to_robot();
initialize_robot();
for(int i = 0; i < 300; i++){
set_motors(speed, speed);
}
}
void turn(int angle) {
	if (angle==180)
		for (int i = 0; i < 137; i++)
        	set_motors(50, -50);
    if (angle==90)
    	for (int i = 0; i < 49; i++)
        	set_motors(30, -30);
    if (angle==-90)
    	for (int i = 0; i < 68; i++) 
        	set_motors(-50, 50);
    set_motors(0, 0);
}

int main() {
    connect_to_robot();
    initialize_robot();
    turn(90);
    return 0;
}

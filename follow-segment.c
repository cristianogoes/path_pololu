/*
 * follow_segment.c
 *
 * This file just contains the follow_segment() function, which causes
 * 3pi to follow a segment of the maze until it detects an
 * intersection, a dead end, or the finish.
 *
 */

#include <pololu/3pi.h>

void init_ultra()
{
  set_digital_output(IO_D1, LOW);  // make trigger pin low
  delay_ms(100);  // give the sensor time to start up
}

int read_ultra()
{
  set_digital_output(IO_D1, HIGH);  // make a 10 us pulse on TRIG
  delay_us(10);
  set_digital_output(IO_D1, LOW);

  while (!is_digital_input_high(IO_D0));  // wait until pin PD0 goes high (start of echo pulse)
  unsigned long ticks = get_ticks();  // get the current system time in ticks
  while (is_digital_input_high(IO_D0));  // wait until pin PD0 goes low (end of echo pulse)
  ticks = get_ticks() - ticks;  // length of the echo pulse in ticks (units are 0.4 us)

  return ((ticks*4/(10*148))*25.4);
}

int follow_segment()
{
	//set_digital_output(IO_D1, LOW);  // make trigger pin low
  	//delay_ms(100);  // give the sensor time to start up

	int last_proportional = 0;
	long integral=0;

	while(1)
	{
		// Normally, we will be following a line.  The code below is
		// similar to the 3pi-linefollower-pid example, but the maximum
		// speed is turned down to 60 for reliability.

		// Get the position of the line.
		unsigned int sensors[5];
		unsigned int position = read_line(sensors,IR_EMITTERS_ON);
		unsigned long dist_sensor = read_ultra();
		//unsigned long dist_sensor = 100;

		// The "proportional" term should be 0 when we are on the line.
		int proportional = ((int)position) - 2000;

		// Compute the derivative (change) and integral (sum) of the
		// position.
		int derivative = proportional - last_proportional;
		integral += proportional;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the left.  If it is a negative number, the robot will
		// turn to the right, and the magnitude of the number determines
		// the sharpness of the turn.
		int power_difference = proportional/20 + integral/10000 + derivative*3/2;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		const int max = 20; // the maximum speed
		//const int max = 100; // the maximum speed
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;
		
		if(power_difference < 0)
			set_motors(max+power_difference,max);
		else
			set_motors(max,max-power_difference);

		// We use the inner three sensors (1, 2, and 3) for
		// determining whether there is a line straight ahead, and the
		// sensors 0 and 4 for detecting lines going to the left and
		// right.

		if((sensors[0] > 200) && (sensors[1] > 200) && (sensors[2] > 200) && (sensors[3] > 200) && (sensors[4] > 200))
		{
			// Found an intersection.
			set_motors(0, 0);
			return 0;
		}

		if(dist_sensor < 75){
			set_motors(0,0);
			return 1;
		}
		#if 0		
		if(((ticks*4/(10*148))*25.4) < 75){
			set_motors(0,0);
			return 1;
		}
		#endif

	}
}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **

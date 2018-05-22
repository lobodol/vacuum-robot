//-------------------------------------------------------------------------------------------------------------------
#define FULL_SPEED 255
#define HALF_SPEED 123

#define BACKWARD true
#define FORWARD  false

#define MIN_DISTANCE 2
#define LOW_DISTANCE 4
//-------------------------------------------------------------------------------------------------------------------
int pin_lmot_direction = 12;
int pin_lmot_brake     = 9;
int pin_lmot_speed     = 3;

int pin_rmot_direction = 13;
int pin_rmot_brake = 8;
int pin_rmot_speed = 11;

int pin_left_switch = 2;
int pin_right_switch = 3;

float distance;
int instruction;

bool is_forward = true;
unsigned long start;   // When robot started an action (running, rotating, ...)
unsigned int duration; // The maximum duration of the action in ms

//-------------------------------------------------------------------------------------------------------------------
/**
 * Setup routine.
 */
void setup()
{
	pinMode(pin_lmot_direction, OUTPUT); // Left motor direction pin
	pinMode(pin_lmot_brake, OUTPUT);     // Left motor brake pin

	pinMode(pin_rmot_direction, OUTPUT); // Right motor direction pin
	pinMode(pin_rmot_brake, OUTPUT);     // Right motor bake pin

	pinMode(pin_left_switch, INPUT_PULLUP);
	pinMode(pin_right_switch, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(pin_right_switch), 'collisionRight', FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_left_switch), 'collisionLeft', FALLING);

	instruction = RUN;
	start = millis();
	duration = 5000;
}

/**
 * Main loop.
 */
void loop()
{
	// take care of the obstacles only when running forward
	if (is_forward) {
		if (distance = readDistance() <= MIN_DISTANCE) {
			brakeAll();

			backward();
			delay(250);

			turnRight();
			delay(500);

			runSlow();
		} else if (distance <= LOW_DISTANCE) {
			runSlow();
		} else {
			run();
		}
	} else {
		// That means the robot is rotating or moving back
	}
}


/**
 * Read the distance of the next obstacle from the sonar.
 *
 * @return float
 */
float readDistance()
{
	// TODO
	return 10;
}

/**
 * Make both motors run backward at half speed.
 */
void backward()
{
	runLeftMotor(HALF_SPEED, BACKWARD);
	runRightMotor(HALF_SPEED, BACKWARD);
	is_forward = false;
}

/**
 * Make robot rotate on right.
 */
void turnRight()
{
	runLeftMotor(HALF_SPEED, FORWARD);
	runRightMotor(HALF_SPEED, BACKWARD);
	is_forward = false;
}

/**
 * Make robot rotate on left.
 */
void turnLeft()
{
	runLeftMotor(HALF_SPEED, BACKWARD);
	runRightMotor(HALF_SPEED, FORWARD);
	is_forward = false;
}

/**
 * Make both motors run forward at full speed.
 */
void run()
{
	runLeftMotor(FULL_SPEED, FORWARD);
	runRightMotor(FULL_SPEED, FORWARD);
	is_forward = true;
}

/**
 * Make both motors run forward at half speed.
 */
void runSlow()
{
	runLeftMotor(HALF_SPEED, FORWARD);
	runRightMotor(HALF_SPEED, FORWARD);	
	is_forward = true;
}

/**
 * Handle collisions on right side.
 */
void collisionRight()
{
	brakeAll();
	turnLeft();
}

/**
 * Handle collisions on left side.
 */
void collisionLeft()
{
	brakeAll();
	turnRight();
}

/**
 * @param int  speed   : Motor speed within [0,255]
 * @param bool reverse : Make motor run forward if TRUE, backward otherwhise
 */
void runLeftMotor(int speed, bool reverse)
{
	digitalWrite(pin_lmot_direction, !reverse); // Set direction
	digitalWrite(pin_lmot_brake, LOW);          // Disengage brake
	analogWrite(pin_lmot_speed, speed);         // Apply speed
}

/**
 * @param int  speed   : Motor speed within [0,255]
 * @param bool reverse : Make motor run forward if TRUE, backward otherwhise
 */
void runRightMotor(int speed, bool reverse)
{
	digitalWrite(pin_rmot_direction, !reverse); // Set direction
	digitalWrite(pin_rmot_brake, LOW);          // Disengage brake
	analogWrite(pin_rmot_speed, speed);         // Apply speed
}

/**
 * Brake all motors for emergency stop.
 */
void brakeAll()
{
	digitalWrite(pin_lmot_brake, HIGH); // Engage brake for left motor
	digitalWrite(pin_rmot_brake, HIGH); // Engage brake for right motor
}

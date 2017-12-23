//============================================================================
// Name        : WallFollower.cpp
// Author      : $(author)
// Version     :
// Copyright   : $(copyright)
// Description : Hello World in C++
//============================================================================

#include <ev3.h>
#include <ev3_button.h>
#include <ev3_command.h>
#include <ev3_constants.h>
#include <ev3_lcd.h>
#include <ev3_output.h>
#include <ev3sensor.h>
#include <cstdlib>
#include <ctime>
#include <string>

#define MIN_WALL_DIST 50
#define MAX_WALL_DIST 60
#define MEAN_WALL_DIST (MAX_WALL_DIST + MIN_WALL_DIST) / 2

#define SPEED 20 // CHANGE IN SPEED NEEDS UPDATE TO REVERSE LOGIC
#define DEVIATION_FACTOR 2

#define K_V 0.08
#define TURN_ANGLE 183

#define CIRCUMFERENCE 17.5929

#define INTENSITY_THRESHOLD 10

int speed_offset_b = 0, speed_offset_d = 0, deviation, wander_counter = 0,
		choice;
int room_intensity, currDistance, lastDistance, fireDistance;

enum Actions {
	WANDER, FIND_WALL, FOLLOW_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL
};
Actions currAction;

void turnLeft() {
	int currentCountD = MotorRotationCount(OUT_D);
	int currentCountB = MotorRotationCount(OUT_B);
	while (((TURN_ANGLE - (MotorRotationCount(OUT_D) - currentCountD)) > 10)
			&& ((TURN_ANGLE + (MotorRotationCount(OUT_B) - currentCountB)) > 10)) {

		OnFwdReg(OUT_D,
				K_V*(TURN_ANGLE-(MotorRotationCount(OUT_D) - currentCountD)) + 3);
		OnRevReg(OUT_B,
				K_V*(TURN_ANGLE+(MotorRotationCount(OUT_B) - currentCountB)) + 3);
		Wait(100);
	}
	RotateMotor(OUT_D, 5,
			(TURN_ANGLE-(MotorRotationCount(OUT_D) - currentCountD)));
	RotateMotor(OUT_B, 5,
			(TURN_ANGLE+(MotorRotationCount(OUT_B) - currentCountB)) + 3);
}

void turnRight() {
	int currentCountD = MotorRotationCount(OUT_D);
	int currentCountB = MotorRotationCount(OUT_B);
	while (((TURN_ANGLE + (MotorRotationCount(OUT_D) - currentCountD)) > 10)
			&& ((TURN_ANGLE - (MotorRotationCount(OUT_B) - currentCountB)) > 10)) {

		OnFwdReg(OUT_B,
				K_V*(TURN_ANGLE-(MotorRotationCount(OUT_B) - currentCountB)) + 3);
		OnRevReg(OUT_D,
				K_V*(TURN_ANGLE+(MotorRotationCount(OUT_D) - currentCountD)) + 3);
		Wait(100);
	}
	RotateMotor(OUT_B, 5,
			(TURN_ANGLE-(MotorRotationCount(OUT_B) - currentCountB)));
	RotateMotor(OUT_D, 5,
			(TURN_ANGLE+(MotorRotationCount(OUT_D) - currentCountD)) + 3);
}

void moveForward(int distance) {
	distance /= 10;
	int FWD_ANGLE = distance / CIRCUMFERENCE * 360;

	int initialCountD = MotorRotationCount(OUT_D);
	int initialCountB = MotorRotationCount(OUT_B);

	while (((FWD_ANGLE - (MotorRotationCount(OUT_D) - initialCountD)) > 10)
			&& ((FWD_ANGLE - (MotorRotationCount(OUT_B) - initialCountB)) > 10)
			&& ButtonIsUp(BTNCENTER)) {

		OnFwdSync(OUT_BD,
				0.04*(FWD_ANGLE-abs(MotorRotationCount(OUT_D) - initialCountD)) + 3);
		Wait(100);
	}
	RotateMotor(OUT_D, 5,
			(FWD_ANGLE-abs(MotorRotationCount(OUT_D) - initialCountD))-5);
	RotateMotor(OUT_B, 5,
			(FWD_ANGLE-abs(MotorRotationCount(OUT_B) - initialCountB))-5);
}

int getDistance() {
	return readSensor(IN_4) - 40;
}

bool scanEnvironment(int room_intensity) {
	int maxIntensity = 0, currIntensity = 0;
	int lSourceAngle = 0;

	while (ButtonIsUp(BTNCENTER) && MotorRotationCount(OUT_A) > -180) {
		currIntensity = readSensor(IN_1);
		OnRevReg(OUT_A, 5);
		if (currIntensity > maxIntensity) {
			maxIntensity = currIntensity;
			lSourceAngle = MotorRotationCount(OUT_A);
		}
		Wait(100);
	}
	while (ButtonIsUp(BTNCENTER) && MotorRotationCount(OUT_A) < 0) {
		OnFwdReg(OUT_A, 5);
		Wait(100);
	}
	Off(OUT_A);

	LcdPrintf(1, "%d %d ", maxIntensity, room_intensity);

	if (maxIntensity >= room_intensity + 6) {

		int currentCountD = MotorRotationCount(OUT_D);
		int currentCountB = MotorRotationCount(OUT_B);

		lSourceAngle = 2 * abs(lSourceAngle) + 10;

		while (((lSourceAngle - (MotorRotationCount(OUT_D) - currentCountD))
				> 10)
				&& ((lSourceAngle + (MotorRotationCount(OUT_B) - currentCountB))
						> 10) && ButtonIsUp(BTNCENTER)) {

			OnFwdReg(OUT_D,
					K_V*(lSourceAngle-(MotorRotationCount(OUT_D) - currentCountD)) + 3);
			OnRevReg(OUT_B,
					K_V*(lSourceAngle+(MotorRotationCount(OUT_B) - currentCountB)) + 3);
			Wait(100);
		}

		RotateMotor(OUT_D, 5,
				(lSourceAngle-(MotorRotationCount(OUT_D) - currentCountD)));
		RotateMotor(OUT_B, 5,
				(lSourceAngle+(MotorRotationCount(OUT_B) - currentCountB)) + 3);

		turnLeft();
		int distance = getDistance();
		turnRight();

		moveForward(distance);
		return true;

	} else {
		return false;
	}
}

void wander() {
	currAction = WANDER;
	if (wander_counter++ == 50) {
		choice = rand() % 3;
		wander_counter = 0;
	}
	switch (choice) {
	case 0:
		OnFwdSync(OUT_BD, SPEED);
		break;
	case 1:
		turnLeft();
		wander_counter = 50;
		break;
	case 2:
		turnRight();
		wander_counter = 50;
		break;
	}

//	OnFwdSync(OUT_BD, SPEED);
}

void touchSensorActive() {
	OnRevSync(OUT_BD, SPEED-10);
	Wait(900);
	turnLeft();
}

void findWall() {
	currAction = FIND_WALL;
	OnFwdReg(OUT_B, SPEED + 40);
	OnFwdReg(OUT_D, SPEED);
}

void followWallStraight() {
	currAction = FOLLOW_WALL;
	if (currDistance > MEAN_WALL_DIST) {
		speed_offset_b = currDistance - MEAN_WALL_DIST;
	} else {
		speed_offset_d = MEAN_WALL_DIST - currDistance;
	}
	if (lastDistance < currDistance) {
		OnFwdReg(OUT_B, SPEED + (DEVIATION_FACTOR * speed_offset_d));
		OnFwdReg(OUT_D, SPEED + (DEVIATION_FACTOR * speed_offset_b));
	} else {
		OnFwdReg(OUT_B, SPEED + (DEVIATION_FACTOR * speed_offset_b));
		OnFwdReg(OUT_D, SPEED + (DEVIATION_FACTOR * speed_offset_d));
	}
	speed_offset_b = 0;
	speed_offset_d = 0;
}

void moveAwayFromWall() {
	currAction = MOVE_AWAY_FROM_WALL;
	deviation = MIN_WALL_DIST - currDistance;
	OnFwdReg(OUT_B, SPEED);
	OnFwdReg(OUT_D, SPEED + ((int) DEVIATION_FACTOR * deviation));
}

int moveTowardsWall() {
	currAction = MOVE_TO_WALL;
	deviation = currDistance - MAX_WALL_DIST;
	OnFwdReg(OUT_B, SPEED + ((int) DEVIATION_FACTOR * deviation));
	OnFwdReg(OUT_D, SPEED);
	return deviation;
}

void followWall() {
	int while_counter = 0, action_counter = 0, find_wall_action_counter = 0;

	while (ButtonIsUp(BTNCENTER)) {
		while_counter++;
		Off(OUT_BD);
//		LcdClean();
		currDistance = readSensor(IN_4);
		if (readSensor(IN_2) == 1) {
			touchSensorActive();
		} else if (while_counter == 60) {
			while_counter = 0;
			if (scanEnvironment(room_intensity)) {
				break;
			} else {
				continue;
			}
		} else if (currDistance > 400
				&& (currAction != FIND_WALL || action_counter > 50)) {
			find_wall_action_counter = 0;
			action_counter = 0;
			wander();
		} else if (currDistance > 80 && action_counter++ < 50) {
			find_wall_action_counter = 0;
			findWall();
		} else if (currDistance >= MIN_WALL_DIST && currDistance < MAX_WALL_DIST) {
			if (find_wall_action_counter++ > 200) {
				wander();
			} else {
				action_counter = 0;
				followWallStraight();
			}
		} else if (currDistance < MIN_WALL_DIST) {
			if (find_wall_action_counter++ > 200) {
				wander();
			} else {
				action_counter = 0;
				moveAwayFromWall();
			}
		} else if (currDistance >= MAX_WALL_DIST) {
			if (find_wall_action_counter++ > 200) {
				wander();
			} else {
				action_counter = 0;
				moveTowardsWall();
			}
		}
		lastDistance = currDistance;
		Wait(100);
	}
}

void init() {
	InitEV3();
	ResetRotationCount(OUT_D);
	ResetRotationCount(OUT_B);
	ResetRotationCount(OUT_A);
	setAllSensorMode(COL_AMBIENT, TOUCH_PRESS, NO_SEN, US_DIST_MM);
	srand(time(NULL));
	room_intensity = readSensor(IN_1);
}

int main() {
	init();
	int choice = 1;
//	while (ButtonIsUp(BTNCENTER)) {
//		LcdPrintf(1, "%d ", currIntensity);
	switch (choice) {
	case 0:
		scanEnvironment(room_intensity);
		break;
	case 1:
		followWall();
		break;
	}
	ButtonWaitForPressAndRelease(BTNCENTER);
//	}
	FreeEV3();
}

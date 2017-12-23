//============================================================================
// Name        : StairClimber.cpp
// Author      : Mindstorms
// Description : Stair Climbing Robot
//============================================================================

#include <ev3.h>
#include <string>

void climbStair() {
	OnFwdSync(OUT_AD, 11);
	OnRevReg(OUT_C, 17);
}
void decendStair() {

	while (ButtonIsUp(BTNCENTER)) {
		OnRevSync(OUT_AD, 4);
		OnFwdReg(OUT_C, 4);
		Wait(100);
	}
}

int main() {
	InitEV3();

	setAllSensorMode(US_DIST_MM, COL_COLOR, NO_SEN, NO_SEN);
	ResetRotationCount(OUT_B);
	int distance = 0, counter = 0, color;
	int prevDistance = 0;

	while (ButtonIsUp(BTNCENTER)) {
		color = readSensor(IN_2);
		distance = readSensor(IN_1);
		LcdPrintf(1, "%d ", distance);

		if (color == 5) {
			Off(OUT_ALL);
			decendStair();
		} else {
			if (counter++ > 50) {
				prevDistance = distance;
			}
			if (ButtonIsUp(BTNCENTER) && prevDistance > distance - 3 && prevDistance < distance + 3) {
				OnRevReg(OUT_C, 19);
				RotateMotor(OUT_B, 100, 300);
				RotateMotor(OUT_B, -100, -300);
				counter = 0;
				prevDistance = 0;
			}

			if (distance > 100) {
				OnFwdSync(OUT_AD, 20);
				OnRevReg(OUT_C, 15);
			} else if (distance > 60) {
				climbStair();
			}
		}
		Wait(100);
	}

	FreeEV3();
}

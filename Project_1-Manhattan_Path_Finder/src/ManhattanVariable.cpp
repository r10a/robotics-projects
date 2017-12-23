/*
 ============================================================================
 Name        : ManhattanVariable.c
 Author      : Mindsotrms (Rohit - rvm2815, Ashutosh - aju1542, Sarthak - sxm0726)
 Version     : 3
 Copyright   :
 Description : Robot Path Planning using BFS
 ============================================================================
 */

#include <ev3.h>
#include <ev3_button.h>
#include <ev3_command.h>
#include <ev3_constants.h>
#include <ev3_lcd.h>
#include <ev3_output.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <string>

using namespace std;

#define LENGTHX 10
#define LENGTHY 12
#define OBS_SIDE 0.305
#define MAX_OBSTACLES 25	/* maximum number of obstacles */
#define START_DIRECTION 't'
#define K_V 0.08
#define TURN_ANGLE 183
#define FWD_ANGLE 635

typedef struct node {
	int x;
	int y;
	int distance;bool isGoal;bool isStart;
} node;

int error_B = 0;
int error_D = 0;

//	Column wise reversing
void waterImage(node a[LENGTHX][LENGTHY]) {
	node temp;
	int x_c;
	for (int j = 0; j < LENGTHY; j++) {
		for (int i = 0; i < LENGTHX / 2; i++) {
			temp = a[i][j];
			a[i][j] = a[LENGTHX - i - 1][j];
			a[LENGTHX - i - 1][j] = temp;
			x_c = a[i][j].x;
			a[i][j].x = a[LENGTHX - i - 1][j].x;
			a[LENGTHX - i - 1][j].x = x_c;
		}
	}
}

void fillGrid(node grid[LENGTHX][LENGTHY], double obstacle[MAX_OBSTACLES][2],
		double start[2], double goal[2]) {

	for (int i = 0; i < LENGTHX; i++) {
		for (int j = 0; j < LENGTHY; j++) {
			grid[i][j].x = i;
			grid[i][j].y = j;
			grid[i][j].distance = 23;
			grid[i][j].isGoal = false;
			grid[i][j].isStart = false;
		}
	}
	int x, y;
	for (int i = 0; i < MAX_OBSTACLES && obstacle[i][0] != -1; i++) {
		x = round(obstacle[i][1] / OBS_SIDE) - 1;
		y = round(obstacle[i][0] / OBS_SIDE);
		grid[x][y].distance = -1;
	}

	x = round(start[1] / OBS_SIDE) - 1;
	y = round(start[0] / OBS_SIDE);
	grid[x][y].isStart = true;

	x = round(goal[1] / OBS_SIDE) - 1;
	y = round(goal[0] / OBS_SIDE);
	grid[x][y].distance = 0;
	grid[x][y].isGoal = true;

	for (int i = 0; i < LENGTHY; i++) {
		grid[LENGTHX - 1][i].distance = -1;
	}
}

void printGrid(node grid[LENGTHX][LENGTHY]) {

	for (int i = 0; i < LENGTHX; i++) {
		printf("\n");
		for (int j = 0; j < LENGTHY; j++) {
			printf(" %d", grid[i][j].distance);
		}
	}

}

node* findGoal(node grid[LENGTHX][LENGTHY]) {
	for (int i = 0; i < LENGTHX; i++) {
		for (int j = 0; j < LENGTHY; j++) {
			if (grid[i][j].isGoal == true)
				return &grid[i][j];
		}
	}
}

node* findStart(node grid[LENGTHX][LENGTHY]) {
	for (int i = 0; i < LENGTHX; i++) {
		for (int j = 0; j < LENGTHY; j++) {
			if (grid[i][j].isStart == true)
				return &grid[i][j];
		}
	}
}

void calculateManhattan(node grid[LENGTHX][LENGTHY]) {
	bool visited[LENGTHX][LENGTHY];

	for (int i = 0; i < LENGTHX; i++) {
		for (int j = 0; j < LENGTHY; j++) {
			visited[i][j] = false;
		}
	}

	std::queue<node*> q;
	node *goal = findGoal(grid);

	visited[goal->x][goal->y] = true;
	q.push(goal);
	while (!q.empty()) {
		node* current = q.front();
		q.pop();
		visited[current->x][current->y] = true;

		if (current->x + 1 < LENGTHX) {
			node* bottom = &grid[current->x + 1][current->y];
			if (bottom->distance != -1 && !visited[bottom->x][bottom->y]) {
				bottom->distance = current->distance + 1;
				q.push(bottom);
			}
		}
		if (current->x - 1 >= 0) {
			node* top = &grid[current->x - 1][current->y];
			if (top->distance != -1 && !visited[top->x][top->y]) {
				top->distance = current->distance + 1;
				q.push(top);
			}
		}
		if (current->y + 1 < LENGTHY) {
			node* right = &grid[current->x][current->y + 1];
			if (right->distance != -1 && !visited[right->x][right->y]) {
				right->distance = current->distance + 1;
				q.push(right);
			}
		}
		if (current->y - 1 >= 0) {
			node* left = &grid[current->x][current->y - 1];
			if (left->distance != -1 && !visited[left->x][left->y]) {
				left->distance = current->distance + 1;
				q.push(left);
			}
		}

	}
}

node* findSmallestNeighbour(node grid[LENGTHX][LENGTHY], node* current,
		char* nextDirection) {
	node *bottom, *top, *right, *left;

	if (current->x + 1 < LENGTHX)
		bottom = &grid[current->x + 1][current->y];
	if (current->x - 1 >= 0)
		top = &grid[current->x - 1][current->y];
	if (current->y + 1 < LENGTHY)
		right = &grid[current->x][current->y + 1];
	if (current->y - 1 >= 0)
		left = &grid[current->x][current->y - 1];

	char directions[4] = { 'r', 'l', 'b', 't' };
	int i = 0;
	while (true) {
		switch (*nextDirection) {
		case 'r':
			if (current->distance > right->distance && right->distance != -1) {
				*nextDirection = 'r';
				return right;
			}
		case 'l':
			if (current->distance > left->distance && left->distance != -1) {
				*nextDirection = 'l';
				return left;
			}
		case 't':
			if (current->distance > top->distance && top->distance != -1) {
				*nextDirection = 't';
				return top;
			}
		case 'b':
			if (current->distance > bottom->distance
					&& bottom->distance != -1) {
				*nextDirection = 'b';
				return bottom;
			}
		default:
			*nextDirection = directions[rand() % 4];
			i++;
			if (i > 100) {
				Off(OUT_BD);
				LcdPrintf(1, "\n%s : X %d | Y: %d", "Stuck at", current->x,
						current->y);
				ButtonWaitForPressAndRelease(BTNCENTER);
				FreeEV3();
				break;
			}
		}
	}
}

void moveForward() {
//	printf("forward");
	int initialCountD = MotorRotationCount(OUT_D) + error_D;
	int initialCountB = MotorRotationCount(OUT_B) + error_B;

	while (((FWD_ANGLE - (MotorRotationCount(OUT_D) - initialCountD)) > 10)
			&& ((FWD_ANGLE - (MotorRotationCount(OUT_B) - initialCountB)) > 10)) {

		OnFwdSync(OUT_BD,
				0.04*(FWD_ANGLE-abs(MotorRotationCount(OUT_D) - initialCountD)) + 3);
		Wait(100);
	}
	RotateMotor(OUT_D, 5,
			(FWD_ANGLE-abs(MotorRotationCount(OUT_D) - initialCountD))-5);
	RotateMotor(OUT_B, 5,
			(FWD_ANGLE-abs(MotorRotationCount(OUT_B) - initialCountB))-5);

}

void turnLeft() {
	int currentCountD = MotorRotationCount(OUT_D) + error_D;
	int currentCountB = MotorRotationCount(OUT_B) + error_B;
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

//	printf("left");
}

void turnRight() {
	int currentCountD = MotorRotationCount(OUT_D) + error_D;
	int currentCountB = MotorRotationCount(OUT_B) + error_B;
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
//	printf("right");
}

int main(void) {

	InitEV3();

	ResetRotationCount(OUT_D);
	ResetRotationCount(OUT_B);

	int num_obstacles = 13; /* number of obstacles */
	double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
	{ { 0.915, 0.305 }, { 0.915, 0.61 }, { 0.915, 0.915 }, { 0.915, 1.219 }, {
			0.915, 1.524 }, { 2.134, 2.743 }, { 2.134, 2.438 },
			{ 2.134, 2.134 }, { 2.134, 1.829 }, { 2.134, 1.524 },
			{ 2.134, 1.219 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, {
					-1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, {
					-1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 } };

	double start[2] = { 0.305, 0.61 }; /* start location */
	double goal[2] = { 3.05, 2.134 }; /* goal location */

	srand(time(NULL));

	node grid[LENGTHX][LENGTHY];

	fillGrid(grid, obstacle, start, goal);
	waterImage(grid);
	calculateManhattan(grid);
	printGrid(grid);

	/*Path Finding starts*/
	node *current = findStart(grid);
	char currDirection = START_DIRECTION;
	char nextDirection = START_DIRECTION;

	/*Start Location*/
	printf("\nX: %d | Y: %d | Distance: %d | Direction: %c", current->x,
			current->y, current->distance, currDirection);

	/*Driver Loop*/
	if (current->distance == 23) {
		/*Goal unreachable*/
		LcdPrintf(1, "\n\n%s ",
				"Goal path unreachable. \n\nPlease move obstacles or allow me to run over them. :P");
	} else {
		while (current->distance != 0) {
			node *next = findSmallestNeighbour(grid, current, &nextDirection);
			if (currDirection == nextDirection) {
				moveForward();
				current = next;
				printf("\nX: %d | Y: %d | Distance: %d | Direction: %c",
						current->x, current->y, current->distance,
						currDirection);
			} else {
				switch (currDirection) {
				case 'r':
					switch (nextDirection) {
					case 't':
						turnLeft();
						moveForward();
						break;
					case 'b':
						turnRight();
						moveForward();
						break;
					default:
						printf("something broke case R");
					}
					break;
				case 'l':
					switch (nextDirection) {
					case 't':
						turnRight();
						moveForward();
						break;
					case 'b':
						turnLeft();
						moveForward();
						break;
					default:
						printf("something broke case L");
					}
					break;
				case 't':
					switch (nextDirection) {
					case 'l':
						turnLeft();
						moveForward();
						break;
					case 'r':
						turnRight();
						moveForward();
						break;
					default:
						printf("something broke case T");
					}
					break;
				case 'b':
					switch (nextDirection) {
					case 'l':
						turnRight();
						moveForward();
						break;
					case 'r':
						turnLeft();
						moveForward();
						break;
					default:
						printf("something broke case B");
					}
					break;
				default:
					printf("something broke final");
				}
				currDirection = nextDirection;
				current = next;
				printf("\nX: %d | Y: %d | Distance: %d | Direction: %c",
						current->x, current->y, current->distance,
						currDirection);
			}
		}
		if (current->distance == 0)
			LcdPrintf(1, "\n\n%s ", "Reached Goal :)");
	}
	Off(OUT_BD);
	ButtonWaitForPressAndRelease(BTNCENTER);

	FreeEV3();
	return EXIT_SUCCESS;
}

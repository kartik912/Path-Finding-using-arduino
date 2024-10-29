#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

LCD display;
Buzzer buzzer;
LineSensors lineSensors;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
IMU imu;


uint16_t maxSpeed;
int16_t minSpeed;
uint16_t baseSpeed;
uint16_t calibrationSpeed;
uint16_t integralGain;
int16_t integral;
uint16_t proportional; // coefficient of the P term * 256
uint16_t derivative; // coefficient of the D term * 256


int16_t lastError = 0;

unsigned int lineSensorValues[5];

char path[100];
uint8_t pathLength = 0;


#include "TurnSensor.h"
#include "GridMovement.h"

void selectHyper()
{
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  maxSpeed = 75;
  minSpeed = 0;
  baseSpeed = maxSpeed;
  calibrationSpeed = 60;
  proportional = 32;
  derivative = 2000;
  integralGain = 0.0625;
}

void setup()
{
  integral=0;
  buzzer.play(">g32>>c32");

  // Directly select the Hyper configuration
  selectHyper();

  while(!buttonB.getSingleDebouncedPress());
  Serial.println("Hi1");
  gridMovementSetup();
  Serial.println("Hi2");
  mazeSolve();
  Serial.println("Hi3");


  // calibrateSensors();
  // buzzer.play("L16 cdegreg4");
  // while(buzzer.isPlaying());
}
int ct=0;
void simplifyPath()
{
  ct = ct+1;
  // Only simplify the path if it is at least three instructions
  // long and the second-to-last turn was a 'B'.
  if(pathLength < 3 || path[pathLength - 2] != 'B')
  {
    return;
  }

  int16_t totalAngle = 0;

  for(uint8_t i = 1; i <= 3; i++)
  {
    switch(path[pathLength - i])
    {
    case 'L':
      totalAngle += 90;
      break;
    case 'R':
      totalAngle -= 90;
      break;
    case 'B':
      totalAngle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch(totalAngle)
  {
  case 0:
    path[pathLength - 3] = 'S';
    break;
  case 90:
    path[pathLength - 3] = 'L';
    break;
  case 180:
    path[pathLength - 3] = 'B';
    break;
  case 270:
    path[pathLength - 3] = 'R';
    break;
  }

  // The path is now two steps shorter.
  pathLength -= 2;
}

char selectTurn(bool foundLeft, bool foundStraight, bool foundRight)
{
  if(foundLeft) { return 'L'; }
  else if(foundStraight) { return 'S'; }
  else if(foundRight) { return 'R'; }
  else { return 'B'; }
}

void mazeFollowPath()
{
  // Play a starting note.
  buzzer.playFromProgramSpace(PSTR("!>c32"));

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  for(uint16_t i = 0; i < pathLength; i++)
  {
    // Follow a line segment until we get to the center of an
    // intersection.
    followSegment();
    driveToIntersectionCenter();

    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[i]);
  }

  // Follow the last segment.
  followSegment();

  // The end of the path has been reached.  Stop the motors and
  // play a note with the buzzer.
  motors.setSpeeds(0, 0);
  buzzer.playFromProgramSpace(PSTR("!>>a32"));
}

void mazeSolve()
{
  pathLength = 0;

  // Play a starting tune.
  buzzer.playFromProgramSpace(PSTR("!L16 cdegreg4"));

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  while(1)
  {
    // Navigate current line segment until we enter an intersection.
    followSegment();
    Serial.println("Hi5");


    // Drive stright forward to get to the center of the
    // intersection and check for exits to the left, right, and
    // straight ahead.
    bool foundLeft, foundStraight, foundRight;
    driveToIntersectionCenter(&foundLeft, &foundStraight, &foundRight);
    Serial.println("direction");

    if(aboveDarkSpot())
    {
      Serial.println("Dark spot");
      // We found the end of the maze, so we succeeded in solving
      // the maze.
      break;
    }

    // Choose a direction to turn.
    char dir = selectTurn(foundLeft, foundStraight, foundRight);

    // Make sure we don't overflow the pathLength buffer,
    // which could lead to unpredictable behavior of the
    // robot.
    if (pathLength >= sizeof(path))
    {
      display.clear();
      display.print(F("pathfull"));
      while(1)
      {
        ledRed(1);
      }
    }

    // Store the intersection in the path variable.
    Serial.println(dir);
    path[pathLength] = dir;
    pathLength++;

    // Simplify the learned path.
    simplifyPath();

    // Show the path on the display.

    // If the path is equal to "BB", it means we have searched the
    // whole maze and not found the path.  We beep but
    // continue searching, because maybe the sensors missed
    // something earlier.
    if (pathLength == 2 && path[0] == 'B' && path[1] == 'B')
    {
      buzzer.playFromProgramSpace(PSTR("!<b4"));
    }

    // Make the turn.
    turn(dir);
    // motors.setSpeeds(0,0);
    // delay(1000);
    // delay(10000);
  }

  // We have solved the maze and found an optimal path.  Stop the
  // motors and play a note with the buzzer.
  motors.setSpeeds(0, 0);
  buzzer.playFromProgramSpace(PSTR("!>>a32"));
}

void loop()
{
  // path[0] = 'R';
  // path[1] = 'R';
  // path[2] = 'R';
  // path[3] = 'S';
  // path[4] = 'L';
  // path[5] = 'L';
  // path[6] = 'R';
  // path[7] = 'L';
  // path[8] = 'R';
  // path[9] = 'R';
  // path[10] = 'L';
  // path[11] = 'L';
  // path[12] = 'S';
  // path[13] = 'L';
  // path[14] = 'R';
  // path[15] = 'L';
  // path[16] = 'S';
  // path[17] = 'R';
  // path[18] = 'L';
  // path[19] = 'L';
  // path[20] = 'L';
  // path[21] = 'R';
  // path[22] = 'L';
  // path[23] = 'L';
  // path[24] = 'R';
  buttonA.waitForButton();
  for(int i=0;i<pathLength;i++) {
    Serial.print(path[i]);
  }
  Serial.println("");
  Serial.println(pathLength);
  Serial.println(ct);
  // delay(5000);
  mazeFollowPath();
}

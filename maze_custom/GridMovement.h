const uint16_t straightSpeed = 75;
const uint16_t intersectionDelay = 200;
const uint16_t turnSpeed = 50;
const uint16_t sensorThreshold = 300;
const uint16_t sensorThresholdDark = 600;
const uint8_t numSensors = 5;
const int32_t gyroAngle45 = 0x5000000;

void driveToIntersectionCenter()
{
  motors.setSpeeds(30,30);
  delay(50);
}


bool aboveLine(uint8_t sensorIndex)
{
  return lineSensorValues[sensorIndex] > sensorThreshold;
}
bool aboveLineDark(uint8_t sensorIndex)
{
  return lineSensorValues[sensorIndex] > sensorThresholdDark;
}

uint16_t readSensors()
{
  return lineSensors.readLineBlack(lineSensorValues);
}

static void lineSensorSetup()
{
  display.clear();
  delay(1000); // Wait 1 second
  turnSensorReset();

  for(uint16_t i = 0; i < 80; i++)
  {
    if (i > 20 && i <= 60)
    {
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);

  while(!buttonA.getSingleDebouncedPress())
  {
    readSensors();

    display.gotoXY(0, 0);
    for (uint8_t i = 0; i < 5; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
    }
  }
}

void driveToIntersectionCenter(bool * foundLeft, bool * foundStraight, bool * foundRight)
{
  *foundLeft = 0;
  *foundStraight = 0;
  *foundRight = 0;
  motors.setSpeeds(25,20);
  for(uint16_t i = 0; i < intersectionDelay / 2; i++)
  {
    readSensors();
    if(aboveLine(0))
    {
      *foundLeft = 1;
    }
    if(aboveLine(4))
    {
      *foundRight = 1;
    }
  }

  readSensors();
  if(aboveLine(1) || aboveLine(2) || aboveLine(3))
  {
    *foundStraight = 1;
  }
}

bool aboveDarkSpot()
{
  return aboveLineDark(1) && aboveLineDark(2) && aboveLineDark(3);
}

void turn(char dir)
{
  if (dir == 'S')
  {
    return;
  }

  turnSensorReset();
  turnSensorReset();

  uint8_t sensorIndex;

  switch(dir)
  {
  case 'B':
    motors.setSpeeds(-turnSpeed-10, turnSpeed);
    while((int32_t)turnAngle < turnAngle45 * 3)
    {
      turnSensorUpdate();
    }
    sensorIndex = 1;
    break;

  case 'L':
    motors.setSpeeds(-turnSpeed, turnSpeed);
    while((int32_t)turnAngle < turnAngle45)
    {
      turnSensorUpdate();
      if((int32_t)turnAngle < -2 * turnAngle45)
      {
          // Stop the motors immediately if the robot has rotated too far
          motors.setSpeeds(0, 0);
          Serial.println("Overshoot detected! Stopping turn.");
          return;
      }
    }
    sensorIndex = 1;
    break;

  case 'R':
    motors.setSpeeds(turnSpeed, -turnSpeed);
    while((int32_t)turnAngle > -turnAngle45)
    {
      turnSensorUpdate();

      // Backup condition to prevent over-rotation
      if((int32_t)turnAngle < -2 * turnAngle45)
      {
          // Stop the motors immediately if the robot has rotated too far
          motors.setSpeeds(0, 0);
          Serial.println("Overshoot detected! Stopping turn.");
          return;
      }
    }
    sensorIndex = 1;
    break;

  default:
    return;
  }
  while(1)
  {
    readSensors();
    if (aboveLine(sensorIndex))
    {
      break;
    }
  }
}

void followSegment()
{
  while(1)
  {
    int16_t position = lineSensors.readLineBlack(lineSensorValues);
    int16_t error = (int16_t)position - 2000;

    int16_t speedDifference = (error * (int32_t)proportional / 256) +
                            // (integral * (int32_t)integralGain / 256) +
                            ((error - lastError) * (int32_t)derivative / 256);
    lastError = error;

    int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;

    leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    if(!aboveLine(0) && !aboveLine(1) && !aboveLine(2) && !aboveLine(3) && !aboveLine(4))
    {
      Serial.println("no found");
      break;
    }

    if(aboveLine(0) || aboveLine(4))
    {
      Serial.println("left or right found");
      break;
    }
  }
}

void gridMovementSetup()
{
  turnSensorSetup();
  lineSensorSetup();
}
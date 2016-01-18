#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <SoftwareSerial.h>
#include "compilerreturn.h"
SoftwareSerial XBee(2, 3); // RX, TX

//compass
#include <Wire.h>
#include <LSM303.h>

#define LED 13

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  270 // microseconds
  
// these might need to be tuned for different motor types
#define TURN_SPEED        200
#define FORWARD_SPEED     110
#define TURN_DURATION     320 

//#define TURN_SPEED        200
//#define FORWARD_SPEED     110
//#define TURN_DURATION     300 


// compass
#define COMPASS_CALIBRATION_SPEED   175 // spin speed when calibrating compass
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 2

// current direction it is facing (north being the starting face pos)
Direction currentDirection;

// map idea
float northAngle;
float eastAngle;
float southAngle;
float westAngle;
float x;
float y;

// bump timer idea
long oldTime; // time of last bump into wall
long lastBumpTime; // the latest bumps time interval
int bumpCounter;
int roomChangeDirectionCounter;

bool inARoom;

Direction bumpOne;
Direction bumpTwo;
Direction bumpThree;

Turn previousTurn; // instance of the last turn
bool outwardBound; // is it on its way out or not

LSM303 compass;
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
int lastError = 0;
 
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
  }
 
void setup()
{
  Serial.begin(9600);
  XBee.begin(9600);
  
  outwardBound = true;
  inARoom = false;
  pinMode(LED, HIGH);
  waitForButtonAndCountDown();
  setupCompass();
  XBee.write("Starting... \n");
  XBee.write("In a corridor... \n");
}

void loop()
{ 
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }

  // read values
  sensors.read(sensor_values);


  // navigate in a room
  if(inARoom){
//    navigateRoom();
  }
  
  // navigate non room
  if(inCorner() && !inARoom){
    getOutOfCorner();
  }
  else 
  if(!inARoom){
    navigateCorridor();
  }

}


// This isn't being used. All it did was enter the room, it takes the direction it came in from
// (e.g left), and travels forward until sensor 4 and 5 on the right detected anything, then turned left.
// it did the turn at half duration so it traversed the outer edge of the room, thus 'exploring it'.

void navigateRoom(){
  int currentAngle = averageHeading();
  
  if(previousTurn == left){
    if ((sensor_values[5] > QTR_THRESHOLD) || (sensor_values[4] > QTR_THRESHOLD)){
    // if either of right sensors detects line, turn to the left
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION/2);
    }else{
       motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
       updateMap(currentAngle);
    }
  }

    if(previousTurn == right){
    if ((sensor_values[0] > QTR_THRESHOLD) || (sensor_values[1] > QTR_THRESHOLD)){
    // if either of right sensors detects line, turn to the left
    XBee.write("Detected left line");
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION/2);
    }else{
       motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
       updateMap(currentAngle);
    }
  }
}

// this bounced back and forth while setting the previous turn direction to the previousTurn variable
// if black was on the right, it would turn left, vice versa
void navigateCorridor(){
    if (sensor_values[0] > QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, turn to the right
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    previousTurn = right;
  } else if (sensor_values[5] > QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, turn to the left
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    previousTurn = left;
  }
  else
  {
    // move
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    int currentAngle = averageHeading();
       // update map
    updateMap(currentAngle);
  }
}

  

// When travelling in a direction, the direction is noted and
// increased in counter depending on how close it is to
// N E S W. North is what i'm considering the original 
// facing direction. 
void updateMap(int currentAngle){

  // get all info available, e.g what angle facing & off angle degrees
  DirectionInfo directionInfo = getDirections(currentAngle);

  if(directionInfo.leadDirection != currentDirection){
     currentDirection = directionInfo.leadDirection;

     
    /// XBee.write("Changing direction to ");
     switch (currentDirection) {
    case 0:
    //  XBee.write("north. \n");
      break;
    case 1:
     //  XBee.write("east. \n");
      break;
    case 2:
     //  XBee.write("south. \n");
      break;
     case 3:
      // XBee.write("west.\n");
      break;
    };
  }

  

  // facing direction could be north east south or west
  // off direction is the angle that it is towards
  // angleDifference is the size of the angle
  Direction facingDirection = directionInfo.leadDirection;
  Direction offDirection = directionInfo.offDirection;
  float angleDifference = directionInfo.leadAngleDifference;
  //Serial.println(angleDifference);
  // if the angle is perfectly faced, increase just one axis
  if(angleDifference == 0.00){
    if(facingDirection == north){
      y+=90;
    }
     if(facingDirection == south){
      y-=90;
    }
     if(facingDirection == east){
      x+=90;
    }
     if(facingDirection == west){
      x-=90;
    }
  }else{
    // find out how much to increase the facing axis by
    float facingIncreaseValue = 90-angleDifference;
    float offAngleIncreaseValue = angleDifference;
    // update map with new current location on main axis
  if(facingDirection == north){
      y+=facingIncreaseValue;
     // Serial.print("Increasing Y by..");
     // Serial.println(facingIncreaseValue);
    }
  if(facingDirection == south){
      y-=facingIncreaseValue;
  //    Serial.print("Decreasing Y by..");
   //   Serial.println(facingIncreaseValue);
    }
  if(facingDirection == east){
      x+=facingIncreaseValue;
    }
  if(facingDirection == west){
      x-=facingIncreaseValue;
    }

    // change map location on off axis
  if(offDirection == north){
      y+=angleDifference;
    //  Serial.print("OFF ANGLE: Increasing Y by..");
    //  Serial.println(angleDifference);
    }
  if(offDirection == south){
      y-=angleDifference;
     //  Serial.print("OFF ANGLE: Decreasing Y by..");
     // Serial.println(angleDifference);
    }
  if(offDirection == east){
      x+=angleDifference;
    }
  if(offDirection == west){
      x-=angleDifference;
    }
  }
 
}

  // Now that I have the direction info that it is travelling in
  // I can start to map the robots movements in an x, y 
  // grid. e.g, when it's travelling north at a perfect
  // angle, y will increase, but if its at a slight off angle,
  // y will increase fast, but x will increase too, but much slower. 
  // 0 degree difference would mean Y would increase fast
  // -45 degree would mean Y increases and X decreases at the same rate 

DirectionInfo getDirections(int currentAngle){

  DirectionInfo directionInfo;

// set incase there isn't an off angle and it isn't set
  directionInfo.leadAngleDifference = 0.00;

  // find the difference of the angles
  float northDiff = currentAngle - northAngle;
  float eastDiff = currentAngle - eastAngle;
  float southDiff = currentAngle - southAngle;
  float westDiff = currentAngle - westAngle;

  // If the north difference as in range of 45 either side, the lead direction was north, and if it was slightly off center to one direction,
  // the direction that it was off would be assigned to the off direction and the off angle logged too.
  if(northDiff > -45 && northDiff <= 45){
    directionInfo.leadDirection = north;
    directionInfo.leadAngleDifference = abs(northDiff); // take the absoloute as it doesn't matter if it's minus as I know what angle it is going towards
     if(northDiff == 0.00){
      directionInfo.offDirection = north;
    } else if(northDiff < 0){
      directionInfo.offDirection = west;
    }else{
      directionInfo.offDirection = east;
    }
  }

  if(eastDiff > -45 && eastDiff <= 45){
    directionInfo.leadDirection = east;
    directionInfo.leadAngleDifference = abs(eastDiff);
     if(eastDiff == 0.00){
      directionInfo.offDirection = east;
    } else if(eastDiff < 0){
      directionInfo.offDirection = north;
    }else{
      directionInfo.offDirection = south;
    }
  }
  

  if(southDiff > -45 && southDiff <= 45){
    directionInfo.leadDirection = south;
    directionInfo.leadAngleDifference = abs(southDiff);
    if(southDiff == 0.00){
      directionInfo.offDirection = south;
    } else if(southDiff < 0){
      directionInfo.offDirection = east;
    }else{
      directionInfo.offDirection = west;
    }
  }

  if(westDiff > -45 && westDiff <= 45){
    directionInfo.leadDirection = west;
    directionInfo.leadAngleDifference = abs(westDiff);
     if(westDiff == 0.00){
     directionInfo.offDirection = west;
    } else if(westDiff < 0){
      directionInfo.offDirection = south;
    }else{
      directionInfo.offDirection = north;
    }
  }
 
  return directionInfo;
}


// if an angle passes 360, just take it back through 0 upto what is remaining after hitting 360
float fixAngle(float intendedAngle){
  if(intendedAngle > 360){
    intendedAngle -= 360;
  }
  return intendedAngle;
}

  // if either of the left sensors, and either of the right sensors
  // have detected black, then its in a corner
bool inCorner(){
  if((sensor_values[0] > QTR_THRESHOLD) &&
    (sensor_values[5] > QTR_THRESHOLD)) 
   {
    return true; }  
   else 
   { return false; }
}

// leave corner by turning twice times the original turn
void getOutOfCorner(){
   XBee.write("leave corner \n");
   if(previousTurn == left){
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION*2); // increase turn duration by *2
    previousTurn = right;
   }else{
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION*2); // increase turn size *2
    previousTurn = left;
   }
}


void setupCompass(){
   // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;
   // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

 // button.waitForButton();

  XBee.write("Starting calibration... \n");
  
  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(COMPASS_CALIBRATION_SPEED);
  motors.setRightSpeed(-COMPASS_CALIBRATION_SPEED);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  Serial.println("Calibrated compass...");
  delay(100);

  // assign the original facing angle to 'currentNorth'
  northAngle = averageHeading();
  eastAngle = fixAngle(northAngle+90);
  southAngle = fixAngle(northAngle+180);
  westAngle = fixAngle(northAngle+270);

  currentDirection = north;
  
  delay(1000);
}


// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

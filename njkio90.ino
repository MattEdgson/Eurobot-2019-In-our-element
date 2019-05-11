//This code has all movement with corrections. Correction can be used or not by typing Move(distance,speed, 0/1, sensors), with 1 casuing correction to be used
//Sensors are controlled similarley, with Move(distance, speed, correction, 1/0) with 1/0 controlling the sensors
//Has code for 4 sensors, all 4 servos (arm, leg, guards), switch to swap between two sides
//On switch is also added, robot will not start until switch is pulled by a chord
//Servo code includes safety measures to avoid going too far, and to not distrupt the sensors

//This code combines the trigger pin of all 4 sensors, ad=nd the echo pins of sensors on the same side. This does work
//Also includes gold() function to extract goldenium, as yet untested
//Code includes servoTest() to give motors a run


#include <Wire.h>                                             // Calls for I2C bus library

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion 
#define SPEED2              0x01                              // Byte to send speed for turn speed 
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

#include <Servo.h>                                            //Calls for servos library
Servo arm;                                                    //Includes all 4 srrvos
Servo leg;
Servo left;
Servo right;

int armAngle;                                                 //Defines global variable for the arm angle

int Mode = 2;                                                 //MODE in which the MD25 will operate selector value
int ledPin_R = 6;                                             //RED LED for warnings/debugging
int ledPin_G = 7;                                             //GREEN LED for warnings/debggging
const long pi = 3.14159265359;                                //Used to calculate turns
int width = 31;                                               //Distance between wheels in cm for turns

int startTime = 0;                                            //Used to count seconds since activation
int trigPin = 13;                                             //Trig Pin for all 4 sensors
int echoPinBack = 12;                                         //Echo Pin for 2 back sensors
int echoPinFront = 11;                                        //Echo Pin for 2 Front sensors

const long backStop = 15.00;                                  //Distance for robot to detect and stop for a collision in cm on the back                                                       
const long frontStop = 20.00;                                 //Distance for robot to detect and stop for a collision in cm on the back
int DefaultSpeed = 170;                                       //Defualt speed for Turning, Must be between 129 and 255, 255 being fastest                                           

int side = 0;                                                 //Side that robot is on, 0 defualts to yellow side
int switchPin = 2;                                            //Pin for rocker switch to detect which side bot is on
int switchState = 0;                                          //Used to store state of switchPin

int powerPin = 3;                                             //Pin for Paddle switch to detect when to start
int powerState = 0;                                           //Used to store state of powerPin

int resetPin = 8;                                             //Pin for detecting arduino restarts
int x = 0;                                                    //Dummy variable for false starts  

void setup() {
  pinMode(ledPin_R, OUTPUT);                                  //Sets LED for RED
  pinMode(ledPin_G, OUTPUT);                                  //Sets LED for GREEN

  pinMode(trigPin, OUTPUT);                                   //Sets sensor send
  pinMode(echoPinBack, INPUT);                                //Sets back sensor recieve
  pinMode(echoPinFront, INPUT);                               //Sets front sensor recieve

  pinMode(switchPin, INPUT);                                  //Sets switchPin as input
  pinMode(powerPin, INPUT);                                   //Sets powerPin as input
  
  pinMode(resetPin, OUTPUT);                                  //Sets resetPin as Output
  
  arm.attach(5);                                              //Attaches all 4 servos to dorresponding pins
  leg.attach(4);
  left.attach(10);
  right.attach(9);

  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial for debugging
  delay(100);                                                 // Wait for everything to power up


  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(Mode);
  Wire.endTransmission();

  encodeReset();                                              // Calls a function that resets the encoder values to 0
  Serial.print("Front: ");
  Serial.println();

}

void loop() {                                                                                  

  //Choose between flash(), arm(), leg(), Move(), Turn(), guards(), gold() to control robot

  //flash(1) makes a green light, flash(2) makes a red light

  //arm() sets arm angle between 160 and 20 degrees. 160 is upwards, 0 is down (unless servo is reversed in manufacture)

  //leg() sets the ramp to tip upwrds and then back down again.

  //guards(angle) sets the little servos on the bakc to a set angle, might need some tinkering as the servos are reversed

  //Move(Distance, Speed, Correction, Sensors) makes the robot go forwards or backwards input number in cm, positive for forwards, negative for backwards
  //Speed should be betwwen 128 and 255, 128 being stationary and 255 being max speed
  //It also takes in a 1 or a 0, 1 meaning you want to use self correcting movement, 0 if you don't. Same for Sensors
  //For instance Move(-20,255, 0) will Move the robot 20 cm backwards at max speed with no correction
  //To avoid the arm getting in the way of the sensor, if the arm is below it's max height, it will be set to it's max height

  //turn() turns the robot. Input in degrees, positive for right turn, negative for left turn (On purple side)

  //The switch for swapping between the two sides will multiply this value by -1 if on the yellow side

  //start() should always be called at the beginning but don't use it again
  
  //guards(angle, side) sets the angle of the little servos in the back.
  //On purple side, guards(180, 1) will set the right gaurd to fully extended, guards(180, 2) the left
  //Vice versa on the Yellow side

    start();
    
    Move(100,200 , 0, 1); //1
    Move(10, 150, 0, 0);
    armFunc(180);
    armFunc(0);
    Turn(92);  //3
    delay(50);
    //Turn(-5);
    Move(20, 200, 0, 0);  //4
    Turn(5);
    Move(70,200, 0, 0);
//Top of ramp, deposited green on the floor
    Move(-10,180, 0, 0);
    
    if (side == 1){
      legFunc(25);
      Move(20,180, 0, 0);
    }
    else{
      Turn(2);
      legFunc(30);
      Move(20,190, 0, 0);
    }
 
    delay(250);
    Move(-100,180, 0, 0);
//  Bottom of ramp
  
    Turn(-90);  //6
    Turn(-40);
    Move(15, 170, 0, 0);
    if (side == 1){
      Move(5, 150, 0, 0);
    }
    armFunc(0);
    legFunc(90);

//Bottom of ramp facing at wall

    Move(-40,180, 0, 1);  //7
    Turn(-80);  //8
    guards(180, 2);
    Move(-32, 170, 0, 1); //9
    Turn(60);  //10
    Move(35,170 , 0, 0);
    Turn(7);
    Move(5, 150, 0, 0);

//Facing 2 red and 1 green at base of ramp

    armFunc(180);
    armFunc(0);
    guards(180, 1);
    
//Just got wall elements 2 red and 1 green//

//MOVE STRAIGHT, sweep up elements///    
    Move(-75,200, 0, 1); //11
    Turn(95);   //12
    Move(-25,190, 0, 1); //13
    Move(20, 190, 0, 1);
    guards(0, 1);
    Turn(-180);
    legFunc(0);
    delay(300);
    legFunc(90);
    guards(0, 2);
    //In red area, just dumped 6 elements

////PLAN TO FOR WALL ELEMENTS THEN RAMP///
//  Move(-5, 170, 0, 1); //14
//  Turn(100); //15
//  Move(70, 200, 0, 0); //16
//  Move(20, 150, 0, 0);
//  armFunc(180);
//  armFunc(0);
//  Move(-5, 170, 0, 1); //17
//  Turn(-90); //18
//  Move(-10, 170, 0, 0); //19
//  Turn(90); //20
//  //Looking at elements, chaos area behind
//  Move(9, 170, 1, 0);
//  armFunc(180);
//  armFunc(0);
//  //red and blue elements on tray
//
//  Move(-15, 0, 0, 1);
//  Turn(-90);
//  Move(70, 170, 0, 1);
//  Turn(90);
//  Move(50, 170, 0, 0);
//  //At base of ramp
//
//  Turn(105);  //3
//  Turn(-8);
//  Move(20, 200, 0, 0);  //4
//  Turn(5);
//  Move(70,200, 0, 0);
//  Move(-10,180, 0, 0);
//  legFunc(40);
//  Move(20,200, 0, 0);
//  legFunc(90);
//  delay(500);
//  armFunc(0);
//  Move(-100,150, 0, 0);
////END//

//  ////GOLD FIRST, STRAIGHT LINE///
//  
//    Move(-125, 170, 0, 1);
//    Turn(-90);
//    Move(36, 150, 0, 0);
//    Move(-6, 150, 0, 0);
//    //Facing blue, about to take off
//  
//    Turn(30);
//    armFunc(55);
//    Turn(-30);
//    armFunc(0);
//    //Knocked off blue
//    Move(10, 150, 0, 0);
//    Move(-15, 170, 0, 1);
//    Turn(-90);
//    Move(45, 170, 0, 1);
//    Turn(90);
//    //Facing Gold
//    Move(20, 150, 0, 0);
//    Move(-1, 150, 0, 1);
//    armFunc(70);
//    Move(-10,200, 0, 1);
//    armFunc(0);
//    //Gold on the floor

////Turn to avoid opposing robots, then get gold////
  Turn(-20);
  Move(-127, 170, 0, 1);
  Turn(-70);
  Move(45, 170, 0, 1);
  if (side == 1){
    Move(20, 170, 0, 0);
  }
  Move(35, 150, 0, 0);
  Move(-4, 150, 0, 0);
  
//Facing blue at particle acc, about to take off//
  Turn(20);
  armFunc(55);
  Turn(-20);
  armFunc(0);
//Knocked off blue
  Move(10, 140, 0, 0);
  Move(-10, 170, 0, 1);
  Turn(-90);
  Move(40, 170, 0, 1);
  Turn(90);
//Facing Gold
  Move(15, 150, 0, 0);
  Move(-3, 150, 0, 1);
  armFunc(70);
  Move(-10,190, 0, 1);
  armFunc(0);
//Gold on the floor
  
  


////GET WALL ELEMENTS THEN RAMP FROM GOLD//
//  Turn(-45);
//  Move(-130, 170, 0, 1);
//  Turn(45);
//  Move(-25, 150, 0, 0);
////Back against Ramp, squared off//
//
//  Move(10, 170, 0, 1);
//  Turn(35);
//  Move(70, 170, 0, 1);
//  Turn(155);
//  Move(60, 170, 0, 1);
//  Move(20, 150, 0, 0);

////Facing wall elements

//  armFunc(180);
//  armFunc(0);


////GO chaos elements then wall from gold//
    Turn(-45);
    Move(-100, 170, 0, 1);
    delay(50);
    guards(180, 0);
    delay(50);
    Turn(-43);
    guards(180, 1);
    Move(-100, 170, 0, 1);
    
    //Chaos elements in green/blue
    
    Move(32, 190, 0, 1);
    guards(0, 1);
    Turn(-90);
    guards(0, 2);
    Move(40, 150, 0, 0);
    
//Facing Blue and Red wall elements
    
    Move(-1, 140, 0, 0);
    armFunc(180);
    delay(200);
    armFunc(0);
    Move(2, 150, 0, 0);
    //Picked up 2 elements from wall
    Move(-5, 170, 0, 1);
    Turn(-90);
    Move(50, 190, 0, 1);
    Turn(90);
    Move(60, 150, 0, 0);
////Bottom of ramp//
    Turn(100);  //3
    Turn(-5);
    Move(20, 200, 0, 0);  //4
    Turn(5);
    legFunc(30);
    Move(80,190, 0, 0);
    delay(300);
    Move(-100,170, 0, 0);
////  Bottom of ramp, fin

  
  while (1);


}

/////////////////////////////////////////////////////// FUNCTIONS /////////////////////////////////////////////////////


void Move(float distance, int velocity, boolean correction, boolean sensors){ //Parent function to handle movement
  int Speed = velocity;                                                       //Sets speed to inputted value
  int CorrectionSpeed = 125;                                                  //Sets correction speed to a very slow value
  
//  if (velocity < 0) {                                                       //Section is used to check the speed input
//    Speed = 128; //If the user entered a negative velocity, sets to no movement
//  }                                                                         //redundant when path has been checked 
//  else if (velocity > 255) {                                                //Commented out for now
//    Speed = 255; //If velocity is higher than 255, sets to 255
//  }
//  else if (velocity < 128) {
//    Speed = (255 - velocity); //If velocity is set between 0-128, corrects to forward motion
//  }
  if (distance < 0) {
    Speed = (255 - velocity); //If distance is -ve, sets speed to go back
  }
  else {                                                                      //If distance is +ve, set arm to move away from sensor FoV
    if (armAngle < 35) {
      armFunc(35);
    }
  }
  if (sensors == 1) {                                                         //If sensors are needed
    forward(distance, Speed);                                                 //Calls fucntion to control motors w/ sensors
  }
  else {                                                                      //If sensors not needed
    forwardNoSensor(distance, Speed);                                         //Calls function to control w/out sensors
  }
  stopMotor();                                                                //Calls fucntion to stop motor
  if (correction == 1) {                                                      //If correction is enabled, goes down this path
    delay(200);
    encoder1();                                                               //Checks how far one wheel has moved
    encoder2();                                                               //Checks how far other wheel has moved
    float dif1 = abs(abs(encoder1()) - abs(distance));                        //Calculates difference between how far robot wanted to move
    float dif2 = abs(abs(encoder2()) - abs(distance));                        //and how far it actually moved
    encodeReset();                                                            //Resets encoders
    if (distance < 0) {                                                       //Sorts out sppeds to work out which direction it needs to go
      CorrectionSpeed = 135;
      dif1 = 0 - dif1;
      dif2 = 0 - dif2;
    }
    Correction_forward(dif1, dif2, CorrectionSpeed);                          //Calls another movement funtion which has no sensors
    stopMotor();
  }                                                                           //END of IF pathway
  encodeReset();                                                              //resets encoders
  delay(50);
}

void forward(float Distance, int DualSpeedValue) {                                //Function to control motors
  timeCheck();                                                                    //Checks time hasn't run out

  encoder1();                                                                     //Calls a function that reads value of encoder 1
  encoder2();                                                                     //Calls a function that reads value of encoder 2
  if (Distance >= 0) {                                                            //If statement to check direction og robot, this path if +ve
    if (abs(encoder1()) <= abs(Distance) && abs(encoder2()) <= abs(Distance)) {   //If statement to check the status of the traveled distance
      DistanceFront();                                                            //Checks if anything is front of front sensors

      Wire.beginTransmission(MD25ADDRESS);                                        //Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();

      Wire.beginTransmission(MD25ADDRESS);                                        // Sets a combined motor speed value
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();

      forward(Distance, DualSpeedValue);                                          //Enters IF loop that breaks when the robot has travelled far enough
    }
  }
  else {                                                                          //If Robot is going backwards, goes down this path
    if (abs(encoder1()) <= abs(Distance) && abs(encoder2()) <= abs(Distance)) {   // If statement to check the status of the traveled distance
      DistanceBack();                                                             //Checks if anyting is in front of back sensors

      Wire.beginTransmission(MD25ADDRESS);                                        // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();

      Wire.beginTransmission(MD25ADDRESS);                                        // Sets a combined motor speed value
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();

      forward(Distance, DualSpeedValue);                                          //Enters IF loop that breaks when the robot has travelled far enough
    }
  }
}

void forwardNoSensor(int Distance, int Speed) {
  timeCheck();

  encoder1();
  encoder2();
  if (abs(encoder1()) <= abs(Distance) && abs(encoder2()) <= abs(Distance)) {
    Wire.beginTransmission(MD25ADDRESS);                                          // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                                          // Sets a combined motor speed value
    Wire.write(SPEED1);
    Wire.write(Speed);
    Wire.endTransmission();

    forwardNoSensor(Distance, Speed);
  }
}

void Correction_forward(float dif1, float dif2, int CorrectionSpeed) {                     //Fucntion to correct how far the robot has moved
  timeCheck();                                                                             //Checks to see if time is up
  encoder1();                                                                              //Checks encoder 1
  encoder2();                                                                              //Checks encoder 2

  if (abs(encoder1()) <= abs(dif1) && abs(encoder2()) <= abs(dif2)) {                      //IF fucntion to see if distance travelled or not
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(CorrectionSpeed);
    Wire.endTransmission();
    Correction_forward(dif1, dif2, CorrectionSpeed);                                      //Calls IF loop until robot has travelled far enough
  }
}

void Turn(float angle) {                                             //Function to control the robot turning
  timeCheck();                                                       //Checks if time is up or not
  int TurnSpeed = DefaultSpeed;                                      //Sets speed to DefualtSpeed set at begnning
  int CorrectionSpeed = 135;                                         //Sets correction speed to very slow
  angle = angle * 0.95;                                              //A correction of around 5% was found to improve turning during testing
  if (side == 1) {
    angle = angle * -1.0; //Reverses the angle depending on which side the robot is on
  }
  if (angle < 0) {                                                   //If angle is negative, sets speed to between 0 and 128 to turn other way
    TurnSpeed = 255 - DefaultSpeed;
  }
  turn_command(angle, TurnSpeed);                                    //Calls function to control motors
  stopMotor();                                                       //Stops motor
  
                                                                     //Section is used for corrections turn
                                                                     //This is commented out as we are no longer using it                                                                    
  //  float Wheel_1_dist = (angle / 360.0) * pi * width;             //Calculates distance wheels should have moved
  //  float Wheel_2_dist = - (Wheel_1_dist);                         //Used for correction but no longer used
  //  delay(200);
  //  encoder1();
  //  encoder2();
  //
  //  float dif1 = (encoder1()) - Wheel_1_dist;           
  //  float dif2 = (encoder2()) - Wheel_2_dist;                
  //  encodeReset();
  //  if(dif1 > 0){CorrectionSpeed = 120;}
  //  CorrectionTurn((dif1), (dif2), CorrectionSpeed);
  //  stopMotor();
  encodeReset();                                                     //Resets encoders
}

void turn_command(float angle, int DualSpeedValue) {                              //Function to control motors during a turn
  timeCheck();
  float Wheel_1 = (angle / 360) * pi * width;                                     //Calculates how far motors whould move in cm
  float Wheel_2 = -Wheel_1;
  encoder1();
  encoder2();

  if (abs(encoder1()) <= abs(Wheel_1) && abs(encoder2()) <= abs(Wheel_2)) {       //Checks if wheels have moved far enough
    Wire.beginTransmission(MD25ADDRESS);                                          // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                                          // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();

    turn_command(angle, DualSpeedValue);                                          //IF loop that breaks when correct distance has been travelled
  }
}

void CorrectionTurn(float dif1, float dif2, int CorrectionSpeed) {                //Correction turn function, no longer used
  timeCheck();
  encoder1();
  encoder2();

  if (abs(encoder1()) <= abs(dif1) && abs(encoder2()) <= abs(dif2)) {
    Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(CorrectionSpeed);
    Wire.endTransmission();

    CorrectionTurn(dif1, dif2, CorrectionSpeed);
  }
}

void encodeReset() {                                               // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(50);
}

float encoder1() {                                                // Function to read and display value of encoder as distance moved in cm
  Wire.beginTransmission(MD25ADDRESS);                            // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                               // Request 4 bytes from MD25
  while (Wire.available() < 4);                                   // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                       // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                           // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                           // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  += Wire.read();                                          // Fourth byte for encoder 1, LL value
  delay(5);                                                       // Wait for everything to make sure everything is sent
  return (poss1 * 0.0873);                                        // Convert encoder value to cm
}

float encoder2() {                                                //Function idetical to above but with encoder 2
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                               // Request 4 bytes from MD25
  while (Wire.available() < 4);                                   // Wait for 4 bytes to become available
  long poss2 = Wire.read();                                       // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();                                           // Second byte for encoder 2, HL
  poss2 <<= 8;
  poss2 += Wire.read();                                           // Third byte for encoder 2, LH
  poss2 <<= 8;
  poss2  += Wire.read();                                          // Fourth byte for encoder 2, LLalue
  delay(5);                                                       // Wait to make sure everything is sent
  return (poss2 * 0.0873);                                        // Convert encoder value to cm
}

void stopMotor() {                                                // Function to stop motors

  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);  // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);  // Stops motors motor 2 when operated in MODE 0 or 1 and
  Wire.write(SPEED2);                   //Stops both motors while in turning sequence if operated in MODE 2 or 3
  Wire.write(128);
  Wire.endTransmission();
  delay(50);

  encoder1();                           // Calls a function that reads value of encoder 1
  encoder2();                           // Calls a function that reads value of encoder 2

}

void start() { //Stores beginning time in milliseconds as startTime, works out what side the robot is on and waits until switch is hit to begin
  guards(0, 2);                                                          //Sets first guard to defualt position
  armFunc(180);                                                          //Sets arm servo to as high as it can go
  leg.write(90);                                                         //Sets leg servo to be flat
  guards(0, 1);                                                          //Sets second gaurd to defualt position
  switchState = digitalRead(switchPin);                                  //Reads state of switch that decides which side robot is on
  if (switchState == HIGH) {                                             //If switch is set one way, robot knows it is on yellow side
    flash(2);                                                            //Flash Green light twice to show yellow side is selected
    flash(2);
    side = 1;                                                            //Sets side value to yellow
  }
  else {                                                                 //ELSE, the robot must be on the Purple side
    flash(1);                                                            //Flashes red light twice
    flash(1);
    side = 0;                                                            //Sets side to purple
  }
  powerState = digitalRead(powerPin);                                    //Reads input from Paddle switch                                     
  
  if (powerState == HIGH ) {                                             //If signal detected from paddle switch (Chord has not been pulled)
    delay(50);
    start();                                                             //Then start() runs again in an IF loop
  }                                                                      //Breaks IF loop when chord is pulled/positive signal fows into Pin

  startTime = millis();                                                  //Once chord is pulled, robot begins counting to 100 seconds
  armFunc(0);                                                            //Arm deploys
  digitalWrite(resetPin, HIGH);                                          //Turn on resetPin to detect resets, also acts as confirmation of start
  delay(50);
}

void timeCheck() {                                                           //Checks to see if it's been over 100 seconds
  Serial.print((millis() - startTime)/1000);
  Serial.println();
  if ((millis() - startTime) >= 980000) {                                    //IF difference between current time and start time is 100 seconds
    stopMotor();                                                             //Stops Motor
    digitalWrite(ledPin_G, HIGH);                                            //Turns on both lights to show to user what has happened
    digitalWrite(ledPin_R, HIGH);
    while(1);                                                                //Stops forever
  }
}

void DistanceFront() {                                                      //Checks nothing is in front of front sensors
  long duration, distance;                                                  //Defines two varibales for calculations
  digitalWrite(trigPin, LOW);                                               //No Output from Trigh pin in case it was left on from before
  delayMicroseconds(2);                                                     //Waits
  digitalWrite(trigPin, HIGH);                                              //Sends out Ultrasonic signal

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);                                               //Stops Signal after 0.01 seconds, essentially a pulse
  duration = pulseIn(echoPinFront, HIGH);                                   //Time taken for sensor the rebounded signal recorded

  distance = (duration / 58);                                               //Time converted into Distance in cm
  //Serial.print("Front: ");                                                  //Outputs distance in Serial Moniter for debugging
  //Serial.print(distance);
  //Serial.print("cm");
  //Serial.println();
  if (distance <= frontStop && distance >= 3.00) {                          //IF distance is less than stopping distance specified in beginning
                                                                            //To avoid disturbances, any less than 2 cm is ignored
    digitalWrite(ledPin_R, HIGH);                                           //Turns on Red LED to show soemthing is wrong
    stopMotor();                                                            //Stops motors from moving
    DistanceFront();                                                        //Enters IF loop until object is removed
  }
  else {
    digitalWrite(ledPin_R, LOW); //Breaks IF loop and turns off LED
  }
}

void DistanceBack() {                                                       //Checks nothing is in front of back sensors, identical to above
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPinBack, HIGH);

  distance = (duration / 58);
  Serial.print("Back: ");
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  if (distance <= backStop && distance >= 3.00) {
    digitalWrite(ledPin_G, HIGH);                                          //Uses Green LED to differentiate between front and back
    stopMotor();
    DistanceBack();
  }
  else {
    digitalWrite(ledPin_G, LOW);
  }
}

void flash(boolean colour) {                                 //Function to flash an LED cause I got tired of writing the same thing
  if (colour == 1) {                                         //IF input == 1, flash red LED
    digitalWrite(ledPin_R, HIGH);
    delay(150);
    digitalWrite(ledPin_R, LOW);
    delay(15);
  }
  else {                                                     //IF input =/= 1. flash Green LED
    digitalWrite(ledPin_G, HIGH);
    delay(250);
    digitalWrite(ledPin_G, LOW);
    delay(50);
  }
}

void armFunc(int angle) {                                 //Sets angle of arm, 0 is vertical up, 180 is vertical down
  if (angle >= 83) {                                      //IF angle is greater than 83, the arm would hit the robot
    angle = 83;                                           //Angle is changed to avoid this
  }
  if (angle <= 35) {                                      //IF angle is less than 35, arm would go above height limit
    angle = 35;
  }
  arm.write(angle);                                       //Sets servo to angle
  armAngle = angle;                                       //Writes angle as a variable used in Move() fucntion
  delay(500);
}


void legFunc(int angle) {                                 //Sets angle of leg, 90 is flat, 0 is tipped up
  if (angle > 90) {
    angle = 90;                                           //Checks angle so it's not too high
  }
  else if (angle < 0) {
    angle = 0; //Same but other way
  }
  leg.write(angle);                                       //Writes angle to servo
  delay(500);
}

void guards(int angle, int arm) {                        //Sets angle of 1  of 2 guards at back of robot SIDE == 1 is yellow
  if (side == 1){                                        //If side is yellow
      if (arm == 1){                                     //guards(angle, 1) controls left guard
        left.write(angle);
    }
      else{                                              //guards(angle, 2) controls right guard
        right.write(180 - angle);
    }
  }
  else{                                                  //If side is purple
      if (arm == 1){                                     //Guards(angle,1) controls right guard
        right.write(180-angle);
  }
      else{                                              //Guards(angle,2) controls left guards
        left.write(angle);
    }
  }
  delay(50);
}

void ServoTest(boolean val){                                               //Extends servos fully or retracts for testing
  
  if (val == 1){                                                           //If val == 1, extends fully
    guards(180, 1);
    delay(750);
    armFunc(0);
    delay(750);
    legFunc(0);
    delay(750);
    guards(180, 2);
    delay(750);
  }
  
  else {                                                                  //If val == 0, retracts fully
    guards(0, 1);
    delay(750);
    armFunc(180);
    delay(750);
    legFunc(90);
    delay(750);
    guards(0, 2);
    delay(750);
  }
}                                                                        //This is not used for start() as it takes too long


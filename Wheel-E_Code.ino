#include <PWMServo.h>
//Raggio di Curvatura (72, 76)cm a PWM = 160 con batterie cariche
//Tempo per fare un cerchio completo: 8,5 s a PWM = 160 con batterie cariche

//Max Speed: The car has done a sprint of 300cm in 3.4s
//If we remove the human error is fair to say that the max speed of our car is around 100cm in a second
#define MIN_ERROR 0
#define MAX_ERROR 150

//PID Values
double Kp = 2, Ki = 0, Kd = 0.15, N = 100;
double P, I, D;
int PWM;
//Time in milli seconds (10^(-3))
int ts = 10, t0 = 0;
//Distance
double currDistance, prevDistance, reffDistance;
//Error
double currError, prevError;


//Dc Motors
//Movement Motor
//Pin 8 and 12 to set the back motor movement.
const int backwardPin = 12;
const int forwardPin = 8;
//Pin 11 to set a Speed Control
const int movementSpeed = 11;

//Steering Motor
//Pin 2 and 4 to set the forward motor movement.
const int rightPin = 2;
const int leftPin = 4;
//Pin 3 to set a Speed Control
const int steeringSpeed = 3;

//Servo Motor
PWMServo sensorServo;

//Ultrasonic Sensor
const int echo = 6;
const int trigger = 7;

//Stop Time: Used to be sure that the car is in the reffDistance spot for more than 1 second.
//Block Time: Used to check if the distance hasn't changed in 3 seconds. If it is than the car is blocked and it must go back.
unsigned long stopTime = 0, blockTime = 0;

//Sonar SX, DX distances
volatile double distances[3];

volatile boolean forward;
int trueAngle;

double prevDistanceCheck;


void setup() 
{ 
  //Movement Motor
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  pinMode(movementSpeed, OUTPUT);

  //Steering Motor
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(steeringSpeed, OUTPUT);

  //Servo Motor
  sensorServo.attach(9);
  sensorServo.write(90);

  //Ultrasonic Sensor
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);

  //Set PID
  reffDistance = 75;
  //Trigger the Ultrasonic Sensor
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  //Read from the Ultrasonic Sensor
  prevDistance = pulseIn(echo, HIGH) * 0.034 / 2;
  prevError = reffDistance - prevDistance;

  
  Serial.begin(9600);
}

void loop()
{
  if((millis() - t0) >= ts){
    t0 = millis();

    //Trigger the Ultrasonic Sensor
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    //Read from the Ultrasonic Sensor
    currDistance = pulseIn(echo, HIGH) * 0.034 / 2;
    delay(30);

    //PID: Get the PWM based on the Distance from the Object
    carPID();

    //Saturation: convert the PWM to avoid the Saturation
    saturation();

    //DeadZone: convert the PWM to avoid the DeadZone
    deadZone();
    
    //Movement: Allow the Robot to move
    movement();
  }
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void carPID(){
  //Calculate the Error
  currError = abs(reffDistance - currDistance);

  //Calculate the PID variables
  //The proportional controller give a proportional value to the error.
  P = Kp * currError;
  //The integral controller attenuates the Steady State errors. It is meant to correct small errors that in the long time can affect the stability of the system.
  //I = Ki * error * ts;
  //The derivative controller eliminates the overshoot caused by the proportional controller. Why? Because with the proportional controller the response applied by the system is
  //multiplied by a costant, so the response of the system can be too much and it can cause overrshoots.
  //The derivative controller is not caused by the error in the position, is caused by the velocity acquired. So in both cases, if the car is going forward or backward, the control
  //will be applied on the velocity of the car, not its position.
  I = 0;
  
  D = Kd * N * (currError - prevError)/ts;


  //Sum the Control Action
  float U = P + I + D;

  //Update the prev Distance and Error
  prevDistance = currDistance;
  prevError = currError;
  

  //Get the Equivalent PWM
  //PWM = map(constrain(U, MIN_ERROR, MAX_ERROR), MIN_ERROR, MAX_ERROR, 0, 255);
  
  //1670 è il valore max che è funzione dei parametri del pid
  PWM = map(U, 0, 1670, 0, 255);
  //PWM = map(U, 0, 1500, 0, 255);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void saturation(){
  //230 PWM is the Max for the forward movement and 20 PWM is the Min for the backward movement
  if(PWM >= 200) PWM = 200;
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void deadZone(){
  //If the PWM is inside that range: [100, 150], the motor will not have enough power to move the car. This is the DeadZone
  if(forward && PWM < 60) PWM = 60;
  if(!forward && PWM < 100) PWM = 100;
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void movement(){
  //Go Forward
  if(currDistance > reffDistance + 10){
    analogWrite(movementSpeed, PWM);
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);

    //Check if the car is blocked for more than 3 seconds
    if(blockTime == 0){
      blockTime = millis();
      prevDistanceCheck = currDistance; 
    }
    else if(millis() - blockTime > 7000){
      //Range of 4cm
      if(prevDistanceCheck >= currDistance - 1 && prevDistanceCheck <= currDistance + 1) goBack();
      else blockTime = 0;

      prevDistanceCheck = currDistance;
    }

    //The car is going forward
    forward = true;
    //Set the stopTime to 0 cause the car is moving
    stopTime = 0;
  }
  //Go Backward
  else if(currDistance < reffDistance){
    analogWrite(movementSpeed, PWM);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);

    //Check if the car is blocked for more than 3 seconds
    if(blockTime == 0){
      blockTime = millis();
      prevDistanceCheck = currDistance; 
    }
    else if(millis() - blockTime > 7000){
      //Range of 4cm
      if(prevDistanceCheck >= currDistance - 1 && prevDistanceCheck <= currDistance + 1) goForward();
      else blockTime = 0;

      prevDistanceCheck = currDistance;
    }

    //The car is going forward
    forward = false;
    //Set the stopTime to 0 cause the car is moving
    stopTime = 0;
  }
  else{
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);

    //If the car stops for more than 2s than activate the Sonar
    if(stopTime == 0) stopTime = millis();
    else if(millis() - stopTime > 1000) sonar();
  }
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//This function will move the sonar and will find the minimum distance from the obstacle
void findRightAngle(){
  //Start from 90° degree
  sensorServo.write(90);
  double minDistance = getRealDistance();
  double minAngle = 90;

  //Turn Right
  for(int i = 0; i <= 45; i = i + 5){
    sensorServo.write(90 + i);
    double tryDistance = getRealDistance();
    if(tryDistance < minDistance){
      minDistance = tryDistance;
      minAngle = 90 + i;
    }

    Serial.print("DX ");
    Serial.print(90 + i);
    Serial.print(" = ");
    Serial.println(tryDistance);
  }

  //delay(10000);

  //Go Back
  sensorServo.write(90);

  //Turn Left
  for(int i = 0; i <= 45; i = i + 5){
    sensorServo.write(90 - i);
    double tryDistance = getRealDistance();
    if(tryDistance < minDistance){
      minDistance = tryDistance;
      minAngle = 90 - i;
    }

    Serial.print("SX ");
    Serial.print(90 - i);
    Serial.print(" = ");
    Serial.println(tryDistance);
  }

  //Update true angle
  trueAngle = minAngle;

  Serial.println();
  Serial.print("Min Angle = ");
  Serial.println(minAngle);
  Serial.println();

  //delay(10000);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//This function will returns two distances. The first one is the right Distance, the second one is the left Distance
void sonar(){
  //Correct the Angle of the Sensor
  findRightAngle();
  sensorServo.write(trueAngle);
  delay(100);
  
  //Get the front distance to calculate the approx hypotenuse
  distances[0] = getRealDistance();
  delay(100);
  
  //Servo Motor movement to the right
  //Get the average distance
  sensorServo.write(trueAngle + 45);
  delay(100);
  distances[1] = getRealDistance();
  //delay(1000);

  //Go back to the Center
  sensorServo.write(trueAngle);
  
  //Servo Motor movement to the left
  //Get the average distance
  sensorServo.write(trueAngle - 45);
  delay(100);
  distances[2] = getRealDistance();
  //delay(1000);

  //Go back to the Center
  sensorServo.write(90);
  //delay(1000);

  //Evade Object
  evade();

  //Delete the Stop Timer
  stopTime = 0;
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//This function will get 20 different values of distance and than will return the average of them to get a more accurate distance value of where the Ultrasonic Sensor is pointed.
double getRealDistance(){
  double sum = 0.0;

  //Loop to get the values
  for(int i = 0; i < 3; i++){
    //Trigger the Ultrasonic Sensor
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    
    sum += pulseIn(echo, HIGH) * 0.034 / 2; 

    delay(50);
  }

  //Return the average
  return sum/10;
  /*delay(100);

  //Trigger the Ultrasonic Sensor
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  return pulseIn(echo, HIGH) * 0.034 / 2;*/
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Evade the Obstacle
void evade(){
  //Angolo di curvatura: +-74cm. Distanza base dall'ostacolo da 75cm a 90cm
  //Vogliamo far si che la macchinina arrivi parallela all'ostacolo dopo aver fatto un quarto di circonferenza. Tenendo conto del tempo di pervorrenza del cerchio, dovremmo curvare per
  //2125 ms
  
  //Distance 0 = Front
  //Distance 1 = Right
  //Distance 2 = Left

  //Check Wall
  double hypotenuse = distances[0]/0.7;

  Serial.print("Front: ");
  Serial.println(distances[0]);
  Serial.print("Hypotenuse: ");
  Serial.println(hypotenuse);
  Serial.print("DX: ");
  Serial.println(distances[1]);
  Serial.print("SX: ");
  Serial.println(distances[2]);
  Serial.println();
  //delay(20000);

  //Check Right
  boolean dxWall, sxWall;
  if(distances[1] <= hypotenuse){
    Serial.println("Right Wall");
    Serial.println();
    dxWall = true;
  }
  else
    dxWall = false;
  //Check Left
  if(distances[2] <= hypotenuse){
    Serial.println("Left Wall");
    Serial.println();
    sxWall = true; 
  }
  else
    sxWall = false;

  //If there's a wall go back
  if(dxWall && sxWall){
    //If the angle < 90° it means that the car is approaching to the wall from the left. If it's coming from the left it can go back in the same direction.
    if(trueAngle < 90){
      //Left Steering
      analogWrite(steeringSpeed, 255);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);

      delay(500);
      
      analogWrite(movementSpeed, 150);
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, HIGH);

      delay(1700);

      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);

      delay(1800);
    }
    else{
      //Right Steering
      analogWrite(steeringSpeed, 255);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);

      delay(500);
      
      analogWrite(movementSpeed, 150);
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, HIGH);

      delay(1700);

      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);

      delay(1800);
    }
  }
  else{
    //Go Right
    if(distances[1] >= distances[2]){
      analogWrite(steeringSpeed, 255);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);

      delay(500);

      analogWrite(movementSpeed, 150);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
  
      delay(1700);

      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);

      delay(1800);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);
    }
    //Go Left
    else{
      analogWrite(steeringSpeed, 255);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);

      delay(500);

      analogWrite(movementSpeed, 150);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
  
      delay(1700);

      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);

      delay(1800);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
    }
  }
  //Stop Movement
  delay(750);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void goBack(){
  //Go back for 2 seconds and then turn right
  analogWrite(movementSpeed, 255);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, HIGH);
  delay(1500);

  analogWrite(steeringSpeed, 255);
  digitalWrite(rightPin, HIGH);
  digitalWrite(leftPin, LOW);
  delay(500);

  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  digitalWrite(rightPin, LOW);
  digitalWrite(leftPin, LOW);
  delay(500);

  blockTime = 0;
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void goForward(){
  //Go back for 2 seconds and then turn right
  analogWrite(movementSpeed, 255);
  digitalWrite(forwardPin, HIGH);
  digitalWrite(backwardPin, LOW);
  delay(1500);

  analogWrite(steeringSpeed, 255);
  digitalWrite(rightPin, HIGH);
  digitalWrite(leftPin, LOW);
  delay(500);

  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  digitalWrite(rightPin, LOW);
  digitalWrite(leftPin, LOW);
  delay(500);

  blockTime = 0;
}

//To Do:

//Use encoder counts to make robot travel backwards

#include <NewPing.h>
#include <RotaryEncoder.h>

// Modify pin numbers to work with our wiring
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int leftForward = 10; // These pins are not named intuitively
int leftBackward = 11; // These pins are not named intuitively
int rightForward = 2; // These pins are not named intuitively
int rightBackward = 3; // These pins are not named intuitively
int sensor1 = A12;
int sensor2 = A2;
int sensor3 = A3;
int sensor4 = A4;
int sensor5 = A5;
int sensor6 = A6;
int sensor7 = A7;
int sensor8 = A8;
int sensor9 = A9;
int sensor10 = A10;
int sensor11 = A11;

int val1; //Store sensor readings
int val2;
int val3;
int val4;
int val5;
int val6;
int val7;
int val8;
int val9;
int val10;
int val11;

int maxLeft;
int maxRight;

int velocity = 100;
int threshold = 800; // IR required to start moving forward
int distance; // Distance between ultrasonic sensor and box

// Create encoder object
//int R = 0; // Number of revolutions
// Modify pin numbers to work with our wiring
int encoderPower = 4;
int encoderGround = 5;
int encoder1Output1 = A13;
int encoder1Output2 = A14;
int encoder2Output1 = A15;
int encoder2Output2 = A3; // FIX THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
RotaryEncoder encoder1(encoder1Output1, encoder1Output2); // This is an encoder object
RotaryEncoder encoder2(encoder2Output1, encoder2Output2);
int Efinal1;  // store final encoder counts before turning around
int Efinal2;

int retrieve = 0; //if retrieve = 0, run the code to find the container and move to it
                  //if retrieve = 1, run code to go backwards

void setup(){
  // Send outputs to screen for debugging
  Serial.begin(9600);
  
  // Assign motor pins
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  
  // Assign sensor pins
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  pinMode(sensor7, INPUT);
  pinMode(sensor8, INPUT);
  pinMode(sensor9, INPUT);
  pinMode(sensor10, INPUT);
  pinMode(sensor11, INPUT);  
  
  // Assign encoder pins
  pinMode(encoderPower, OUTPUT);
  pinMode(encoderGround, OUTPUT);
}


void loop(){
  
if (retrieve = 0){  
  // Read sensor values 
  val1 = analogRead(sensor1);
  val2 = analogRead(sensor2);
  val3 = analogRead(sensor3);
  val4 = analogRead(sensor4);
  val5 = analogRead(sensor5);
  val6 = analogRead(sensor6);
  val7 = analogRead(sensor7);
  val8 = analogRead(sensor8);
  val9 = analogRead(sensor9);
  val10 = analogRead(sensor10);
  val11 = analogRead(sensor11);
 
// Print sensor values (for debugging only)
//  Serial.print("1: ");
//  Serial.println(val1);
//  Serial.print("2: ");
//  Serial.println(val2);
//  Serial.print("3: ");
//  Serial.println(val3);
//  Serial.print("4: ");
//  Serial.println(val4);
//  Serial.print("5: ");
//  Serial.println(val5);
//  Serial.print("6: ");
//  Serial.println(val6);
//  Serial.print("7: ");
//  Serial.println(val7);
//  Serial.print("8: ");
//  Serial.println(val8);
//  Serial.print("9: ");
//  Serial.println(val9);
//  Serial.print("10: ");
//  Serial.println(val10);
//  Serial.print("11: ");
//  Serial.println(val11);
//  Serial.println();

  maxLeft = max(val1, val2); //COMPARE SENSOR VALUES OF LEFT SIDE
  maxLeft = max(maxLeft, val3);
  maxLeft = max(maxLeft, val4);
  maxLeft = max(maxLeft, val5);
  
  maxRight = max(val7, val8); //COMPARE SENSOR VALUES OF RIGHT SIDE
  maxRight = max(maxRight, val9);
  maxRight = max(maxRight, val10);
  maxRight = max(maxRight, val11);
  
  // What is both are bigger than val6? We should code this in
  
  Serial.print("Left: ");  // Print sensor readings
  Serial.println(maxLeft);
  Serial.print("Middle: ");
  Serial.println(val6);
  Serial.print("Right: ");
  Serial.println(maxRight);
  
  if (maxLeft > threshold && maxLeft > val6){      // Turn left
     digitalWrite(leftForward, LOW);
     analogWrite(leftBackward, velocity);
     digitalWrite(rightForward, LOW);
     analogWrite(rightBackward, velocity);
  }   
  else if (maxRight > threshold && maxRight > val6){    // Turn right
    analogWrite(leftForward, velocity);
    digitalWrite(leftBackward, LOW);
    analogWrite(rightForward, velocity);
    digitalWrite(rightBackward, LOW);
    }
  else if (val6 > threshold && val6 > maxLeft && val6 > maxRight){ // Go forward when middle sensor detects most IR
    distance = sonar.ping_cm();
    if (distance < 20 && distance > 10){   // Throttle velocity based on ultrasonic distance reading
      velocity = 0.5*velocity;
    }
    else if (distance < 10){    //ADJUST VALUE OF DISTANCE BASED ON LEGNTH OF MAGNET ARM
      velocity = 0; 
      retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase
    }
    
   // Drive straight towards box (velocity is determined by distance from box)
    analogWrite(leftForward, velocity);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    analogWrite(rightBackward, velocity);
    
    // Read encoder 1 values
    static int pos1 = 0;
    int newPos1 = encoder1.getPosition();
    if (pos1 != newPos1) {
    //Serial.print(newPos1);
    //Serial.println();
    pos1 = newPos1;
    }
  
    // Read encoder 2 values
    static int pos2 = 0;
    int newPos2 = encoder2.getPosition();
    if (pos2 != newPos2) {
    //Serial.print(newPos2);
    //Serial.println();
    pos2 = newPos2;
    }
  }
  
  else {
    // If no IR detected, stop moving
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);
  }
  
// Use encoder readings to calculate number of revolutions
//  R = newPosition/4480;
//  Serial.println(R); // Print number of revolutions
//  Serial.println(); // Line break
}

Efinal1 = pos1;
Efinal2 = pos2;

//start retrieval process 
else if (retrieve = 1){ //if on retrieval phase, turn around


     digitalWrite(leftForward, LOW);
     analogWrite(leftBackward, velocity);
     digitalWrite(rightForward, LOW);
     analogWrite(rightBackward, velocity);

delay(2869)
    
 pos1 = Efinal1;
 pos2 = Efinal2; 
 velocity = 100;
 
while (pos1 > 0 && pos 2 > 0){

  analogWrite(leftForward, LOW);
  digitalWrite(leftBackward, velocity);
  digitalWrite(rightForward, velocity);
  analogWrite(rightBackward, LOW);
    
  int newPos1 = encoder1.getPosition(); //Encoder 1 reading
  if (pos1 != newPos1) {
    pos1 = newPos1;
  }
    
  int newPos2 = encoder2.getPosition(); //Encoder 2 reading
  if (pos2 != newPos2) {
    pos2 = newPos2;
  }
 }
 
  analogWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  analogWrite(rightBackward, LOW);

}

  
  delay(500);
}

//To Do:

//Use encoder counts to make robot travel backwards

#include <NewPing.h>
#include <RotaryEncoder.h>

// Modify pin numbers to work with our wiring
#define TRIGGER_PIN  22  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     24  // Arduino pin tied to echo pin on the ultrasonic sensor.
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
// int R = 0; // Number of revolutions
// Modify pin numbers to work with our wiring
int leftEncoder1 = A15;
int leftEncoder2 = A0;
int rightEncoder1 = A14;
int rightEncoder2 = A13;
RotaryEncoder leftEncoder(leftEncoder1, leftEncoder2); // Creates left encoder object
RotaryEncoder rightEncoder(rightEncoder1, rightEncoder2); // Creates right encoder object
//static int leftPos;
//static int rightPos;
//int newLeftPos;
//int newRightPos;
//int leftEncoderFinal; // Stores final encoder count before turning around
//int rightEncoderFinal;
//int leftBacktrack = 0;
//int newLeftBacktrack;
//int rightBacktrack = 0;
//int newRightBacktrack = 0;

int retrieve = 0; //if retrieve = 0, run the code to find the container and move to it
//if retrieve = 1, run code to go backwards

void setup() {
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

  // Send outputs to screen for debugging
  Serial.begin(9600);
}


void loop() {


//        static int leftPos = 0;
//      leftEncoder.tick();
//      int leftNewPos = leftEncoder.getPosition();
//      if (leftPos != leftNewPos) {
//        Serial.print("Left Encoder: ");
//        Serial.println(leftNewPos);
//        leftPos = leftNewPos;
//      }
//      
  if (retrieve == 0) {
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
    //      Serial.print("1: ");
    //      Serial.println(val1);
    //      Serial.print("2: ");
    //      Serial.println(val2);
    //      Serial.print("3: ");
    //      Serial.println(val3);
    //      Serial.print("4: ");
    //      Serial.println(val4);
    //      Serial.print("5: ");
    //      Serial.println(val5);
    //      Serial.print("6: ");
    //      Serial.println(val6);
    //      Serial.print("7: ");
    //      Serial.println(val7);
    //      Serial.print("8: ");
    //      Serial.println(val8);
    //      Serial.print("9: ");
    //      Serial.println(val9);
    //      Serial.print("10: ");
    //      Serial.println(val10);
    //      Serial.print("11: ");
    //      Serial.println(val11);
    //      Serial.println();

    maxLeft = max(val1, val2); // COMPARE SENSOR VALUES OF LEFT SIDE
    maxLeft = max(maxLeft, val3);
    maxLeft = max(maxLeft, val4);
    maxLeft = max(maxLeft, val5);

    maxRight = max(val7, val8); // COMPARE SENSOR VALUES OF RIGHT SIDE
    maxRight = max(maxRight, val9);
    maxRight = max(maxRight, val10);
    maxRight = max(maxRight, val11);

    unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
    int distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

    // What if both are bigger than val6? We should code this in

//        Serial.print("Left: ");  // Print sensor readings
//        Serial.println(maxLeft);
//        Serial.print("Middle: ");
//        Serial.println(val6);
//        Serial.print("Right: ");
//        Serial.println(maxRight);
                                                      Serial.print("Distance (cm): ");
                                                      Serial.println(distance);

    if (maxLeft > threshold && maxLeft > val6) {     // Turn left
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, velocity);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, velocity);
    }
    else if (maxRight > threshold && maxRight > val6) {   // Turn right
      analogWrite(leftForward, velocity);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, velocity);
      digitalWrite(rightBackward, LOW);
    }
    else if (val6 > threshold && val6 > maxLeft && val6 > maxRight) { // Go forward when middle sensor detects most IR

      if (distance < 30 && distance >= 15) {  // Throttle velocity based on ultrasonic distance reading (distances in cm)
        velocity = 0.5 * 100;
      }
      else if (distance < 15 && distance > 1 && retrieve == 0) {   //ADJUST VALUE OF DISTANCE BASED ON LEGNTH OF MAGNET ARM
        velocity = 0;
        retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase
        distance = 20; // arbitrarily reset the distance so that it isn't 0/doesn't stop the code
      }

      //      // Read encoder 1 values
      //      static int leftPos = 0;
      //      leftEncoder.tick();
      //      newLeftPos = leftEncoder.getPosition();
      //      Serial.print("Left Encoder Count: ");
      //      Serial.println(newLeftPos);
      //      if (leftPos != newLeftPos) {
      //        //    Serial.print(newLeftPos);
      //        //    Serial.println();
      //        leftPos = newLeftPos;
      //      }
      //
      //      // Read encoder 2 values
      //      static int rightPos = 0;
      //      rightEncoder.tick();
      //      newRightPos = rightEncoder.getPosition();
      //      Serial.print("Right Encoder Count: ");
      //      Serial.println(newRightPos);
      //      if (rightPos != newRightPos) {
      //        //    Serial.print(newRightPos);
      //        //    Serial.println();
      //        rightPos = newRightPos;
      //      }


//Serial.print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
      // Count using left encoder
      static int leftPos = 0;
      leftEncoder.tick();
      int leftNewPos = leftEncoder.getPosition();
      if (leftPos != leftNewPos) {
        Serial.print("Left Encoder: ");
        Serial.println(leftNewPos);
        leftPos = leftNewPos;
      }

      // Count using right encoder
      static int rightPos = 0;
      rightEncoder.tick();
      int rightNewPos = rightEncoder.getPosition();
      if (rightPos != rightNewPos) {
        Serial.print("Right Encoder: ");
        Serial.println(rightNewPos);
        rightPos = rightNewPos;
      }
      
      // Drive straight towards box (velocity is determined by distance from box)
      analogWrite(leftForward, velocity);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, velocity);
      //
      //      leftEncoderFinal = leftPos;
      //      rightEncoderFinal = rightPos;
    }

    else {
      // If no IR detected, stop moving
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
  }

  //start retrieval process
  else if (retrieve == 1) { //if on retrieval phase, turn around
    Serial.print("Retrieve? ");
    Serial.println(retrieve);
//    Serial.print("Left Encoder Count: ");
//    Serial.println(leftEncoderFinal);
//    Serial.print("Right Encoder Count: ");
//    Serial.println(rightEncoderFinal);
//    Serial.println();
//
//    digitalWrite(leftForward, LOW);
//    analogWrite(leftBackward, velocity);
//    digitalWrite(rightForward, LOW);
//    analogWrite(rightBackward, velocity);
//
//    delay(2869);
//
//    //  leftPos = leftEncoderFinal;
//    //  rightPos = rightEncoderFinal;
//    //  velocity = 100;
//
//    // create new variables to store encoder values during return travel
//
//    digitalWrite(leftForward, LOW);
//    analogWrite(leftBackward, velocity);
//    digitalWrite(rightForward, LOW);
//    analogWrite(rightBackward, velocity);
//
//    while (leftBacktrack < leftEncoderFinal && rightBacktrack < rightEncoderFinal) {
//
//
//      // Count left backtrack
//      static int leftBacktrack = 0;
//      leftEncoder.tick();
//
//      newLeftBacktrack = leftEncoder.getPosition();
//      if (leftBacktrack != newLeftBacktrack) {
//        leftBacktrack = newLeftBacktrack;
//      }
//
//      // Count right backtrack
//      static int rightBacktrack = 0;
//      rightEncoder.tick();
//      newRightBacktrack = rightEncoder.getPosition();
//      if (rightBacktrack != newRightBacktrack) {
//        rightBacktrack = newRightBacktrack;
//      }

    }




    //  newLeftBacktrack = leftEncoder.getPosition(); //Encoder 1 reading
    //  if (leftEncoderFinal != newLeftPos) {
    //    leftEncoderFinal = newLeftPos;
    //  }

    //  int newRightPos = rightEncoder.getPosition(); //Encoder 2 reading
    //  if (rightPos != newRightPos) {
    //    rightPos = newRightPos;
    //  }
  

  //  digitalWrite(leftForward, LOW);
  //  digitalWrite(leftBackward, LOW);
  //  digitalWrite(rightForward, LOW);
  //  digitalWrite(rightBackward, LOW);

  //Serial.println();
}

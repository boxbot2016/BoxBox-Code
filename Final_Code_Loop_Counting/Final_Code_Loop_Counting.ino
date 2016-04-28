// This code makes robot travel to start location, and can maneuver around objects, but not both together

int printTime = 0;

#include <NewPing.h>
#include <RotaryEncoder.h>

#define TRIGGER_PIN  22  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     24  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int leftForward = 10; // These pins are not named intuitively
int leftBackward = 11;
int rightForward = 2;
int rightBackward = 3;
int sensor1 = A2;
int sensor2 = A3;
int sensor3 = A4;
int sensor4 = A5;
int sensor5 = A6;
int sensor6 = A7;
int sensor7 = A8;
int sensor8 = A9;
int sensor9 = A10;
int sensor10 = A11;
int sensor11 = A12;

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

int vForward =  100; //velocity when moving forwards
int vTurning = 100;  //velocity when turning
int velocity = vForward;  //velocity variable that gets changed sometimes

int turnedAround = 0;
int leftLoopCount = 0;
int rightLoopCount = 0;
int forwardLoopCount = 0;
int netLeftLoop;


int threshold = 200; //   ***************************************************************THRESHOLD IR TO MAKE THE ROBOT MOVE FORWARD
int superThreshold = 800; // For going forward
int distance; // Distance between ultrasonic sensor and box
unsigned int uS; // Send ping, get ping time in milliseconds (uS)

int retrieve = 0; //if retrieve = 0, run the code to find the container and move to it
//if retrieve = 1, run code to go backwards
int stopTurning = 1;
int backwards = 0;

int rightAvoid = 1;
int forwardAvoid1 = 1;
int leftAvoid = 1;
int forwardAvoid2 = 1;
int obstacleFound = 0;

int fullTurnTime = 2900;

void setup() {

  pinMode(leftForward, OUTPUT); // Assign motor pins
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(sensor1, INPUT); // Assign sensor pins
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

  Serial.begin(9600);       // Send outputs to screen for debugging
}

//45s for 10 turns
void loop() {
  while (retrieve == 0) {
    val1 = analogRead(sensor1); // Read sensor values
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

    maxLeft = max(val1, val2); // COMPARE SENSOR VALUES OF LEFT SIDE
    maxLeft = max(maxLeft, val3);
    maxLeft = max(maxLeft, val4);
    //    maxLeft = max(maxLeft, val5);

    maxRight = max(val8, val9);// COMPARE SENSOR VALUES OF RIGHT SIDE (sensor 7 is deactivated)
    maxRight = max(maxRight, val10);
    maxRight = max(maxRight, val11);

    uS = sonar.ping(); // Send ping, get ping time in milliseconds (uS)
    distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

    // Print sensor values (for debugging only)
    //    Serial.print("1: ");
    //    Serial.println(val1);
    //    Serial.print("2: ");
    //    Serial.println(val2);
    //    Serial.print("3: ");
    //    Serial.println(val3);
    //    Serial.print("4: ");
    //    Serial.println(val4);
    //    Serial.print("5: ");
    //    Serial.println(val5);
    //    Serial.print("6: ");
    //    Serial.println(val6);
    //    Serial.print("7: ");
    //    Serial.println(val7);
    //    Serial.print("8: ");
    //    Serial.println(val8);
    //    Serial.print("9: ");
    //    Serial.println(val9);
    //    Serial.print("10: ");
    //    Serial.println(val10);
    //    Serial.print("11: ");
    //    Serial.println(val11);

    // What if both are bigger than val6? We should code this in

    //  Serial.print("Left: ");  // Print sensor readings
    //  Serial.println(maxLeft);
    //  Serial.print("Middle: ");
    //  Serial.println(val6);
    //  Serial.print("Right: ");
    //  Serial.println(maxRight);

    if (retrieve == 0 && distance < 15 && distance > 8 && maxRight < threshold && maxLeft < threshold && val5 < threshold && val6 < threshold && val7 < threshold){
      Serial.println("Maneuvering around obstacle");
      obstacleFound = 1;
      analogWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
      delay(500); //stop for 0.5s so robot does not run into object

      if (rightAvoid == 1){ // Turn 90 degrees right, calculate # loops for 45 degrees
        int rightTurnStart = millis();
        //        int rightTurnCurrent = millis();
        while (millis() - rightTurnStart < (fullTurnTime/4)){
          //          int rightS = millis();
          analogWrite(leftForward, vTurning);
          digitalWrite(leftBackward, LOW);
          analogWrite(rightForward, vTurning);
          digitalWrite(rightBackward, LOW);
          //          rightLoopCount = rightLoopCount + 0.5;
          //          int rightE = millis();
          //          int dt = rightE - rightE;
          //          if (printTime == 0){
          //            Serial.print("Right Avoid Turn Time: ");
          //            Serial.println(dt); 
          //            printTime = 1;
          //          }
          //          rightTurnCurrent = millis();
        }
        rightAvoid = 0;
      }

      if (forwardAvoid1 == 1){ // Go forward for 1.5 seconds, calculate # loops * sqrt(2)
        int forwardStart = millis();
        //        int forwardAvoidLoop = 0;
        while (millis() - forwardStart < fullTurnTime/3){
          analogWrite(leftForward, velocity);
          digitalWrite(leftBackward, LOW);
          digitalWrite(rightForward, LOW);
          analogWrite(rightBackward, velocity);
          //          forwardAvoidLoop = forwardAvoidLoop + 1;
        }
        //        forwardAvoidLoop = forwardAvoidLoop * sqrt(2);
        //        forwardLoopCount = forwardLoopCount + forwardAvoidLoop;
        forwardAvoid1 = 0;
      }

      if (leftAvoid == 1){ // Turn 90 degrees left
        digitalWrite(leftForward, LOW);
        analogWrite(leftBackward, vTurning);
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, vTurning);
        delay(fullTurnTime/4); // 90 degrees
        leftAvoid = 0;
      }

      if (forwardAvoid2 == 1){ // Go forward again (2nd leg of avoid maneuver)
        analogWrite(leftForward, velocity);
        digitalWrite(leftBackward, LOW);
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, velocity);
        delay(fullTurnTime/3); // proportional forward distance
        forwardAvoid2 = 0;
      }
    }

    while (retrieve == 0 && maxRight < 0.75*threshold && maxLeft < 0.75*threshold && val5 < 0.75*threshold && val6 < 0.75*threshold && val7 < 0.75*threshold) {
      // This while loop tells the robot to turn right until it picks up IR signal.
      // Since this loop runs continuously outside of the parent while loop,
      // analogRead(sensor) has to be reimplemented inside the loop.
      val1 = analogRead(sensor1); // Read sensor values
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

      maxLeft = max(val1, val2); // COMPARE SENSOR VALUES OF LEFT SIDE
      maxLeft = max(maxLeft, val3);
      maxLeft = max(maxLeft, val4);
      //    maxLeft = max(maxLeft, val5);

      maxRight = max(0, val8); // COMPARE SENSOR VALUES OF RIGHT SIDE (sensor 7 is deactivated)
      maxRight = max(maxRight, val9);
      maxRight = max(maxRight, val10);
      maxRight = max(maxRight, val11);

      Serial.println("********Sweeping*********");
      //      Serial.println();
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

      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      //      delay(100);
    }

    //    if (maxLeft > threshold && maxLeft > val6 && distance >= 8) {     // Turn left
    //      digitalWrite(leftForward, LOW);
    //      analogWrite(leftBackward, vTurning);
    //      digitalWrite(rightForward, LOW);
    //      analogWrite(rightBackward, vTurning);
    //      leftLoopCount = leftLoopCount + 1;
    //    }
    if (maxRight > threshold && maxRight > val6 && distance >= 8) {   // Turn right
      //      int rightS = millis();
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      rightLoopCount = rightLoopCount + 1;
      //      int rightE = millis();
      //      int dt = rightE - rightS;
      //      if (printTime == 1){
      //        Serial.print("Right Turn Loop Time: ");
      //        Serial.println(dt);
      //        printTime = 2; 
      //      }
    }
    else if (maxLeft > threshold && maxLeft > val6 && distance >= 8) {     // Turn left
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning);
      leftLoopCount = leftLoopCount + 1;
    }
    else if (val6 > threshold && val6 > maxLeft && val6 > maxRight) { // Go forward when middle sensor detects most IR
      //    else if (val6 > threshold && val6 > maxLeft && val6 > maxRight || (((val5 > superThreshold) &&  (val6 > superThreshold) ) || ((val6 > superThreshold) && (val7 > superThreshold)))) { // Go forward

      if (distance >= 8) {  // Throttle velocity based on ultrasonic distance reading (distances in cm)
        velocity = vForward; // this makes our new backtracking mechanism easier to implement
      }
      else if (distance < 8 && distance > 1 && retrieve == 0 && val6 > threshold) {
        delay(100);
        uS = sonar.ping(); // Send ping, get ping time in milliseconds (uS)
        distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

        if (distance < 8 && distance > 1 && val6 > threshold) {
          velocity = 0;
          retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase
          Serial.println("RETRIEVE STATEMENT #1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
      }
      // Drive straight towards box (velocity is determined by distance from box)
      analogWrite(leftForward, velocity);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, velocity);
      delay(100); // Have the robot move forward for a decent amount of time instead of being so jittery side to side
      forwardLoopCount = forwardLoopCount + 1;
    }

    else if (val5 > superThreshold && val6 > superThreshold && val7 > superThreshold && distance < 8 && distance > 1) {
      delay(100);
      uS = sonar.ping(); // Send ping, get ping time in milliseconds (uS)
      distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

      if (distance < 8 && distance > 1 && val6 > threshold) {
        velocity = 0;
        retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase

        Serial.println("RETRIEVE STATEMENT #2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      }
    }


    else { // If no IR detected, stop moving
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
    Serial.print("Distance (cm): ");
    Serial.println(distance);
    //    Serial.print("Retrieve? ");
    //    Serial.println(retrieve);

    //    leftLoopCount = ceil(leftLoopCount); // round to nearest larger integer
    //    rightLoopCount = ceil(rightLoopCount);
    //    forwardLoopCount = ceil(forwardLoopCount);
  }

  //********************************************************START RETRIEVAL PROCESS**************************************
  if (retrieve == 1 && stopTurning == 1) { //if on retrieval phase, turn around
    //velocity = vForward;
    Serial.println("***********************RETRIEVE***********************");
    Serial.print("Left Loop Count: ");
    Serial.println(leftLoopCount);
    Serial.print("Right Loop Count: ");
    Serial.println(rightLoopCount);
    Serial.print("Forward Loop Count: ");
    Serial.println(forwardLoopCount);

    if (turnedAround == 0){
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning);
      delay(fullTurnTime/2); // 180 degrees
      turnedAround = 1;
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
      netLeftLoop = leftLoopCount - rightLoopCount;
    }

    if (obstacleFound == 1){
      digitalWrite(leftForward, LOW); // Turn left
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning); 
      delay(fullTurnTime/8); // 45 degrees
    }
    Serial.print("Net Left Loop: ");
    Serial.println(netLeftLoop);

    if (netLeftLoop > 0){
      while (netLeftLoop > 0){ // Turn right
        analogWrite(leftForward, vTurning);
        digitalWrite(leftBackward, LOW);
        analogWrite(rightForward, vTurning);
        digitalWrite(rightBackward, LOW);
        netLeftLoop = netLeftLoop - 1;
        Serial.print("Net Left Loop: ");
        Serial.println(netLeftLoop);
      }
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
    else if (netLeftLoop < 0){
      while (abs(netLeftLoop) > 0){ // Turn left
        digitalWrite(leftForward, LOW);
        analogWrite(leftBackward, vTurning);
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, vTurning); 
        netLeftLoop = netLeftLoop + 1;
        Serial.print("Net Left Loop: ");
        Serial.println(netLeftLoop);
      } 
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
    else { // netLeftLoop == 0
      velocity = vForward;
      if (obstacleFound == 1){
        analogWrite(leftForward, velocity);
        digitalWrite(leftBackward, LOW);
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, velocity);
        delay(ceil((fullTurnTime/3)*sqrt(2))); // hypotenuse of proportional distance
      }
      while (forwardLoopCount > 0){ // Go forward toward starting location
        analogWrite(leftForward, velocity);
        digitalWrite(leftBackward, LOW);
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, velocity);
        delay(100); // Have the robot move forward for a decent amount of time instead of being so jittery side to side
        forwardLoopCount = forwardLoopCount - 1;
      }
      stopTurning = 0;
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
  }
}












































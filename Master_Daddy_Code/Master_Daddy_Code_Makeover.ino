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

int vForward =  100; //velocity when moving forwards
int vBackwards = 100;  //velocity when moving backwards
int vTurning = 80;  //velocity when turning
int velocity = vForward;  //velocity variable that gets changed sometimes

int leftTurnTime = 0;
int rightTurnTime = 0;
int forwardTime = 0;
int leftTurnStart;
int leftTurnEnd;
int leftTurnDelta;
int rightTurnStart;
int rightTurnEnd;
int rightTurnDelta;
int forwardStart;
int forwardEnd;
int forwardDelta;

int threshold = 200; //   ***************************************************************THRESHOLD IR TO MAKE THE ROBOT MOVE FORWARD
int superThreshold = 800; // For going forward
int distance; // Distance between ultrasonic sensor and box
unsigned int uS; // Send ping, get ping time in microseconds (uS)

int retrieve = 0; //if retrieve = 0, run the code to find the container and move to it
//if retrieve = 1, run code to go backwards
int stopTurning = 1;
int backwards = 0;
int i = 0;
int j = 0;

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
    // delay(1000);
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

    uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
    distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

    // Print sensor values (for debugging only)
    Serial.print("1: ");
    Serial.println(val1);
    Serial.print("2: ");
    Serial.println(val2);
    Serial.print("3: ");
    Serial.println(val3);
    Serial.print("4: ");
    Serial.println(val4);
    Serial.print("5: ");
    Serial.println(val5);
    Serial.print("6: ");
    Serial.println(val6);
    Serial.print("7: ");
    Serial.println(val7);
    Serial.print("8: ");
    Serial.println(val8);
    Serial.print("9: ");
    Serial.println(val9);
    Serial.print("10: ");
    Serial.println(val10);
    Serial.print("11: ");
    Serial.println(val11);
    Serial.println();

    // What if both are bigger than val6? We should code this in

    //  Serial.print("Left: ");  // Print sensor readings
    //  Serial.println(maxLeft);
    //  Serial.print("Middle: ");
    //  Serial.println(val6);
    //  Serial.print("Right: ");
    //  Serial.println(maxRight);

    while (retrieve == 0 && maxRight < threshold && maxLeft < threshold && val5 < threshold && val6 < threshold && val7 < threshold) {
      // This while loop tells the robot to turn right until it picks up IR signal. 
      // Since this loop runs continuously outside of the parent while loop, 
      // analogRead(sensor) has to be reimplemented inside the loop.
      delay(2000);
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
      
      // rightTurnStart = micros(); // wendy
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      // rightTurnEnd = micros(); // wendy
      // rightTurnDelta = rightTurnEnd - rightTurnStart;
      // rightTurnTime = rightTurnTime + rightTurnDelta;
      // delay(500);
    }

    if (maxRight > threshold && maxRight > val6 && distance >= 8) {   // Turn right
      rightTurnStart = micros(); // wendy
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      rightTurnEnd = micros(); // wendy
      rightTurnDelta = rightTurnEnd - rightTurnStart;
      rightTurnTime = rightTurnTime + rightTurnDelta;
      // delay(500);
    }

    else if (maxLeft > threshold && maxLeft > val6 && distance >= 8) {     // Turn left
      leftTurnStart = micros(); // wendy
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning);
      leftTurnEnd = micros(); // wendy
      leftTurnDelta = leftTurnEnd - leftTurnStart;
      leftTurnTime = leftTurnTime + leftTurnDelta;
      // delay(500);
    }

    else if (val6 > threshold && val6 > maxLeft && val6 > maxRight) { // Go forward when middle sensor detects most IR

      //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      //If val6 > threshold but not greater than maxLeft or maxRight even if it has picked up the container, retrieve does not get toggled because code does not enter this else if statement


      if (distance >= 20) {
        velocity = vForward;
      }
      else if (distance < 20 && distance >= 8) {  // Throttle velocity based on ultrasonic distance reading (distances in cm)
        // velocity = 0.75 * vForward;
        velocity = vForward; // this makes our new backtracking mechanism easier to implement
      }
      else if (distance < 8 && distance > 1 && retrieve == 0 && val6 > threshold) {
        velocity = 0;
        retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase
        //distance = 20; // arbitrarily reset the distance so that it isn't 0/doesn't stop the code
      }

      // Drive straight towards box (velocity is determined by distance from box)
      forwardStart = millis();
      analogWrite(leftForward, velocity);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, velocity);
      delay(100); //Have the robot move forward for a decent amount of time instead of being so jittery side to side
      forwardEnd = millis(); // wendy
      forwardDelta = forwardEnd - forwardStart;
      Serial.print("Forward Delta");
      Serial.println(forwardDelta);
      forwardTime = forwardTime + forwardDelta;

      i = i + 1;
    }

    else if (val5 > superThreshold && val6 > superThreshold && val7 > superThreshold && distance < 8 && distance > 1) {
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
      retrieve = 1;
    }

    else { // If no IR detected, stop moving
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }

    Serial.print("Distance (cm): ");
    Serial.println(distance);
    Serial.print("Retrieve? ");
    Serial.println(retrieve);
  }

  //********************************************************START RETRIEVAL PROCESS**************************************
  if (retrieve == 1 && stopTurning == 1) { //if on retrieval phase, turn around
    //velocity = vForward;
    Serial.print("***********************RETRIEVE = ");
    Serial.print("Left Turn Time: ");
    Serial.println(leftTurnTime);
    Serial.print("Right Turn Time: ");
    Serial.println(rightTurnTime);
    Serial.print("Forward Time: ");
    Serial.println(forwardTime);

    //ASSUMING 2800 ms IS THE TIME IT TAKES FOR IT TO TURN 180
    if (leftTurnTime > rightTurnTime) {
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning);
      delay(2250 - (leftTurnTime - rightTurnTime));
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);

    }
    else if (rightTurnTime > leftTurnTime) {
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      delay(2250 - (leftTurnTime - rightTurnTime));

      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }
    else if (leftTurnTime == rightTurnTime) {
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
      delay(2250);

      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
    }

    Serial.print("Velocity: ");
    Serial.println(velocity);
    velocity = vForward;
    Serial.print("New Velocity: ");
    Serial.println(velocity);
    analogWrite(leftForward, velocity);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    analogWrite(rightBackward, velocity);
    delay(forwardTime);

    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);

    stopTurning = 0;

    //    Serial.println(retrieve);
    //    val1 = analogRead(sensor1); // Read sensor values
    //    val2 = analogRead(sensor2);
    //    val3 = analogRead(sensor3);
    //    val4 = analogRead(sensor4);
    //    val5 = analogRead(sensor5);
    //    val6 = analogRead(sensor6);
    //    val7 = analogRead(sensor7);
    //    val8 = analogRead(sensor8);
    //    val9 = analogRead(sensor9);
    //    val10 = analogRead(sensor10);
    //    val11 = analogRead(sensor11);
    //
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
    //    Serial.println();
    //
    //    digitalWrite(leftForward, LOW); // Turn left to turn around
    //    analogWrite(leftBackward, vTurning);
    //    digitalWrite(rightForward, LOW);
    //    analogWrite(rightBackward, vTurning);
    //
    //    delay(3300); //change depending on velocity so that it turns exactly around
    //    stopTurning = 0;
    //
    //    digitalWrite(leftForward, LOW);
    //    digitalWrite(leftBackward, LOW);
    //    digitalWrite(rightForward, LOW);
    //    digitalWrite(rightBackward, LOW);
    //  }
    //
    //  //*********************************** BACKTRACKING ************************************
    //  if (backwards == 0) { //If statement not needed, was used to ensure loop ran only once during testing
    //    i = 1.4 * i;
    //    while (j < i) {
    //      Serial.println("************WHILE LOOP TO GO BACKWAAAAAAAAAAAAAAAAAAAAAAAAAAAAARDS ");
    //      analogWrite(leftForward, vBackwards);
    //      digitalWrite(leftBackward, LOW);
    //      digitalWrite(rightForward, LOW);
    //      analogWrite(rightBackward, vBackwards);
    //      j = j + 1;
    //    }
    //    backwards = 1;

  }
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}


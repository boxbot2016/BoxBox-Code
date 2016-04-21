//To Do:

//Use encoder counts to make robot travel backwards

#include <NewPing.h>
#include <RotaryEncoder.h>

// Modify pin numbers to work with our wiring
#define TRIGGER_PIN  22  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     24  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
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

int vForward = 125; //velocity when moving forwards
int vBackwards = 125;  //velocity when moving backwards
int vTurning = 100;  //velocity when turning
int velocity = vForward;  //velocity variable that gets changed sometimes

int threshold = 500; //   ************THRESHOLD IR TO MAKE THE ROBOT MOVE FORWARD
int distance; // Distance between ultrasonic sensor and box
unsigned int uS; // Send ping, get ping time in microseconds (uS)

unsigned long int forwardTime; //amount of time the robot spends moving forward
unsigned long int totalTime;  //Amount of time since the robot was turned on/serial monitor reopened
unsigned long int forwardTimeBegin; //store that time
unsigned long int forwardTimeEnd; //the time at which the robot has picked up the container
int ftick = 0;  //help remember the time at which the robot stops turning, and is pointed at the IR

int retrieve = 0; //if retrieve = 0, run the code to find the container and move to it
//if retrieve = 1, run code to go backwards
int stopTurning = 1;
int backwards = 0;

void setup() {
  pinMode(leftForward, OUTPUT); // Assign motor pins
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(sensor1, INPUT);    // Assign sensor pins
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
    maxLeft = max(maxLeft, val5);

    maxRight = max(val7, val8); // COMPARE SENSOR VALUES OF RIGHT SIDE
    maxRight = max(maxRight, val9);
    maxRight = max(maxRight, val10);
    maxRight = max(maxRight, val11);

    uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
    distance = (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm (0 = outside set distance range)

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

    // What if both are bigger than val6? We should code this in

    //  Serial.print("Left: ");  // Print sensor readings
    //  Serial.println(maxLeft);
    //  Serial.print("Middle: ");
    //  Serial.println(val6);
    //  Serial.print("Right: ");
    //  Serial.println(maxRight);
    //  Serial.print("Distance (cm): ");
    //  Serial.println(distance);

    if (maxLeft > threshold && maxLeft > val6) {     // Turn left
      digitalWrite(leftForward, LOW);
      analogWrite(leftBackward, vTurning);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vTurning);
    }

    else if (maxRight > threshold && maxRight > val6) {   // Turn right
      analogWrite(leftForward, vTurning);
      digitalWrite(leftBackward, LOW);
      analogWrite(rightForward, vTurning);
      digitalWrite(rightBackward, LOW);
    }

    else if (val6 > threshold && val6 > maxLeft && val6 > maxRight) { // Go forward when middle sensor detects most IR

      if (ftick == 0) {    //remember the time ONLY ONCE and ONLY when robot is pointed at container
        forwardTimeBegin = millis();
        Serial.print("**********************************forwardTimeBegin = ");
        Serial.println(forwardTimeBegin);
        ftick = 1;
      }

      if (distance < 20 && distance >= 7) {  // Throttle velocity based on ultrasonic distance reading (distances in cm)
        velocity = 0.75 * vForward;
      }

      else if (distance < 7 && distance > 1 && retrieve == 0) {   //ADJUST VALUE OF DISTANCE BASED ON LEGNTH OF MAGNET ARM
        velocity = 0;
        retrieve = 1;  //value to signify that the robot has reached the object and should now begin retrieval phase
        //distance = 20; // arbitrarily reset the distance so that it isn't 0/doesn't stop the code
      }

      // Drive straight towards box (velocity is determined by distance from box)
      analogWrite(leftForward, velocity);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, velocity);
    }
    
    else {
      // If no IR detected, stop moving
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

  //****************************START RETRIEVAL PROCESS**************************************
  if (retrieve == 1 && stopTurning == 1) { //if on retrieval phase, turn around
    forwardTimeEnd = millis();
    Serial.print("*********************FORWARDTIMEEND = ");
    Serial.println(forwardTimeEnd);
    Serial.print("***********************RETRIEVE = ");
    Serial.println(retrieve);

    digitalWrite(leftForward, LOW);
    analogWrite(leftBackward, vTurning);
    digitalWrite(rightForward, LOW);
    analogWrite(rightBackward, vTurning);

    delay(2400); //change depending on velocity so that it turns exactly around
    stopTurning = 0;

    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);

    forwardTime = forwardTimeEnd - forwardTimeBegin; //Time it took for robot to get to container after it first located it
    Serial.print("************forwardTime = ");
    Serial.println(forwardTime);
    totalTime = millis();
  }

  //*********************************** BACKTRACKING ************************************

  if (backwards == 0) { //If statement not needed, was used to ensure loop ran only once during testing
   
    while ( (millis() - totalTime) < forwardTime) { //run until current time - time it stopped
      Serial.println("************WHILE LOOP TO GO BACKWAAAAAAAAAAAAAAAAAAAAAAAAAAAAARDS ");
      analogWrite(leftForward, vBackwards);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(rightBackward, vBackwards);
    }
    
    backwards = 1;
  }
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}


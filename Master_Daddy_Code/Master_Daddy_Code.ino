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

void setup(){
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
  Serial.begin(9600); // Send outputs to screen for debugging
}

int val1;
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

void loop(){
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

  maxLeft = max(val1, val2);
  maxLeft = max(maxLeft, val3);
  maxLeft = max(maxLeft, val4);
  maxLeft = max(maxLeft, val5);
  
  maxRight = max(val7, val8);
  maxRight = max(maxRight, val9);
  maxRight = max(maxRight, val10);
  maxRight = max(maxRight, val11);
  
  Serial.print("Left: ");
  Serial.println(maxLeft);
  Serial.print("Right: ");
  Serial.println(maxRight);
  Serial.print("Middle: ");
  Serial.println(val6);
  Serial.println();
  
  if (maxLeft > threshold && maxLeft > val6){
     // Turn left
     digitalWrite(leftForward, LOW);
     analogWrite(leftBackward, velocity);
     digitalWrite(rightForward, LOW);
     analogWrite(rightBackward, velocity);
     
//    analogWrite(leftForward, velocity);
//    digitalWrite(leftBackward, LOW);
//    digitalWrite(rightForward, LOW);
//    analogWrite(rightBackward, velocity);
  }   
   
  else if (maxRight > threshold && maxRight > val6){
    // Turn right
    analogWrite(leftForward, velocity);
    digitalWrite(leftBackward, LOW);
    analogWrite(rightForward, velocity);
    digitalWrite(rightBackward, LOW);
    
//    digitalWrite(leftForward, LOW);
//    analogWrite(leftBackward, velocity);
//    analogWrite(rightForward, velocity);
//    digitalWrite(rightBackward, LOW);
    }
    
  else if (val6 > threshold){
    // Go forward when middle sensor detects most IR
    analogWrite(leftForward, velocity);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    analogWrite(rightBackward, velocity);
    
//    digitalWrite(leftForward, LOW);
//    analogWrite(leftBackward, velocity);
//    digitalWrite(rightForward, LOW);
//    analogWrite(rightBackward, velocity);
  }
  
  else {
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);
  }
  
  delay(500);
}

int left1 = 10;
int left2 = 11;
int right1 = 2;
int right2 = 3;
int sensor1 = A2;
int sensor2 = A3;
int sensor3 = A4;

void setup() {
  // put your setup code here, to run once:
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  Serial.begin(9600);
}

int out1 = 0;
int out2 = 0;
int out3 = 0;

int val1;
int val2;
int val3;

void loop() {
  // put your main code here, to run repeatedly:
  val1 = analogRead(sensor1);
  val2 = analogRead(sensor2);
  val3 = analogRead(sensor3);
  
  out1 = map(val1, 0, 1023, 0, 255);
  Serial.print("out1 = ");
  Serial.println(out1);
  out2 = map(val2, 0, 1023, 0, 255);
  Serial.print("out2 = ");
  Serial.println(out2);
  out3 = map(val3, 0, 1023, 0, 255);
  Serial.print("out3 = ");
  Serial.println(out3);
  
  if (out1 > 200 && out1 > out2 && out1 > out3){
    analogWrite(left1, 100);
    digitalWrite(left2, LOW);
    analogWrite(right2, 100);
    digitalWrite(right1, LOW);
  }
  else if (out2 > 200 && out2 > out1 && out2 > out3){
    analogWrite(left1, 100);
    digitalWrite(left2, LOW);
    analogWrite(right1, 100);
    digitalWrite(right2, LOW);
  }
  else if (out3 > 200 && out3 > out1 && out3 > out2){
    analogWrite(left2, 100);
    digitalWrite(left1, LOW);
    analogWrite(right2, 100);
    digitalWrite(right1, LOW);  
  } 
  else{
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    digitalWrite(right2, LOW);
    digitalWrite(right1, LOW);
  }
  delay(100);
}

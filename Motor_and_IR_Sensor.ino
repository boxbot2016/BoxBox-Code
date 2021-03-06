int mot1 = 10;
int mot2 = 11;
int sensorPin = A5;

void setup() {
  // put your setup code here, to run once:
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

int out1 = 0;
int out2 = 0;

void loop() {
  // put your main code here, to run repeatedly:
  int val = analogRead(sensorPin);
  out1 = map(val, 0, 1023, 0, 255);
  Serial.print("out1 = ");
  Serial.println(out1);
  if (out1 > 200){
    analogWrite(mot1, out1);
    digitalWrite(mot2, LOW);
  }
  else{
    digitalWrite(mot1, LOW);
  }
  delay(100);
}

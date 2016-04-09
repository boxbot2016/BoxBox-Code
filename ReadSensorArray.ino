//int sensor1 = A0;
//int sensor2 = A2;
//int sensor3 = A3;
//int sensor4 = A4;
//int sensor5 = A5;
//int sensor6 = A6;
//int sensor7 = A7;
//int sensor8 = A8;
//int sensor9 = A9;
//int sensor10 = A10;
//int sensor11 = A11;
int values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);  
  pinMode(13, OUTPUT);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(12,HIGH);
  digitalWrite(13,HIGH);      
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for (int i = 0; i <=13; i++){
//    values[i] = analogRead(i);
//  }\
  for (int i = 0; i <= 10; i++){
    if (i == 0){
      values[i] = analogRead(A0);
    }
    else if (i == 1){
      values[i] = analogRead(A2);
    }
    else if (i == 2){
      values[i] = analogRead(A3);
    }
    else if (i == 3){
      values[i] = analogRead(A4);
    }
    else if (i == 4){
      values[i] = analogRead(A5);
    }
    else if (i == 5){
      values[i] = analogRead(A6);
    }
    else if (i == 6){
      values[i] = analogRead(A7);
    }
    else if (i == 7){
      values[i] = analogRead(A8);
    }
    else if (i == 8){
      values[i] = analogRead(A9);
    }
    else if (i == 9){
      values[i] = analogRead(A10);
    }
    else
      values[i] = analogRead(A11);
    }
  for (int i = 0; i <= 10; i++){
  Serial.print(values[i]);
  Serial.print(' ');
  }
  Serial.println(' ');
   delay(1000); 
}


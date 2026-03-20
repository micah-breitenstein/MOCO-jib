

int driverDIR = 2;
int driverPUL = 4;

int downbutton = 6;
int upbutton  = 7;
int downbuttonread;
int upbuttonread;

int speedup = 8;
int speeddown = 9;
int speedupbuttonread;
int speeddownbuttonread;
int stage = 0;
int lastreadup = 0;
int lastreaddown = 0;
int lastreadup2 = 0;
int lastreaddown2 = 0;

int potentiometer = A0;
int potread;
int potadjust;

int pd;
int count = 5000;


//to test
//int stage0count = 5000;
//int stage1count = 2500;
//int stage2count = 1000;
//int stage3count = 500;
//int stage4count = 100;


void setup() {
  //Serial.begin(57600);

  pinMode (driverDIR, OUTPUT);
  pinMode (driverPUL, OUTPUT);
  pinMode(speedup, INPUT);
  pinMode(speeddown, INPUT);
  pinMode(upbutton, INPUT);
  pinMode(downbutton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  //Serial.println(stage);
  // Serial.println(count);
  digitalWrite(LED_BUILTIN, LOW);

  upbuttonread = digitalRead(upbutton);
  downbuttonread = digitalRead(downbutton);
  speedupbuttonread = digitalRead(speedup);
  speeddownbuttonread = digitalRead(speeddown);

  switch (stage) {
    case 0:
      count = 5000;
      break;
    case 1:
      count = 2500;
      break;
    case 2:
      count = 1000;
      break;
    case 3:
      count = 500;
      break;
    case 4:
      count = 250;
      break;
  }

  //speedup

  //speedup
  if (lastreaddown == 0 && stage == 1 && speeddownbuttonread == 1) {
    stage = 0;
  } if (lastreaddown == 0 && stage == 2 && speeddownbuttonread == 1) {
    stage = 1;
  }
  if (lastreaddown == 0 && stage == 3 && speeddownbuttonread == 1) {
    stage = 2; 
  }
  if (lastreaddown == 0 && stage == 4 && speeddownbuttonread == 1) {
    stage = 3;
  }
  if (lastreadup == 0 && stage == 3 && speedupbuttonread == 1) {
    stage = 4;
  }
  if (lastreadup == 0 && stage == 2 && speedupbuttonread == 1) {
    stage = 3;
  }
  if (lastreadup == 0 && stage == 1 && speedupbuttonread == 1) {
    stage = 2;
  }
  if (stage == 0 && speedupbuttonread == 1 ) {
    stage = 1;
  }
////////
  lastreadup = speedupbuttonread;
  lastreaddown = speeddownbuttonread;
  


pd = count;

  if (upbuttonread == HIGH) {
    digitalWrite(driverDIR, HIGH);
    digitalWrite(driverPUL, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
  }

  if (downbuttonread == HIGH) {
    digitalWrite(driverDIR, LOW);
    digitalWrite(driverPUL, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
  }

}

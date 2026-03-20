
/////ARDUINO INPUTS
int driverDIR = 2;
int driverPUL = 4;

int downbutton = 6;
int upbutton  = 7;
int downbuttonread;
int upbuttonread;

int speedupbutton = 8;
int speeddownbutton = 9;
int speedupbuttonread;
int speeddownbuttonread;

int speedupadj = 11;
int speeddownadj = 12;
int speedupadjread;
int speeddownadjread;

/////INTERNAL VARIABLES
int pd;
int count = 9000;
int stage = 0;
int lastreadup = 0;
int lastreaddown = 0;

int stage0count = 9000;
int stage1count = 4200;
int stage2count = 1900;
int stage3count = 1075;
int stage4count = 400;

void setup() {
  //Serial.begin(57600);


  pinMode (driverDIR, OUTPUT);
  pinMode (driverPUL, OUTPUT);
  pinMode(speeddownbutton, INPUT);
  pinMode(speedupbutton, INPUT);
  pinMode(upbutton, INPUT);
  pinMode(downbutton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (speedupadj, INPUT);
  pinMode (speeddownadj, INPUT);
}



///////////////////////////////////////////////////////////////////////////////////////////
void loop() {

 Serial.println(count);
  digitalWrite(LED_BUILTIN, LOW);
 
  upbuttonread = digitalRead(upbutton);
  downbuttonread = digitalRead(downbutton);
  speedupbuttonread = digitalRead(speedupbutton);
  speeddownbuttonread = digitalRead(speeddownbutton);
  speedupadjread = digitalRead(speedupadj);
  speeddownadjread = digitalRead(speeddownadj);


  //////////SPEED STAGE CHANGES
  if (lastreaddown == 0 && stage == 1 && speeddownbuttonread == 1) {
    Serial.println("STAGE0");
    stage = 0;
    count = stage0count;
  }
  if (lastreaddown == 0 && stage == 2 && speeddownbuttonread == 1) {
    Serial.println("STAGE1");
    stage = 1;
    count = stage1count;
  }
  if (lastreaddown == 0 && stage == 3 && speeddownbuttonread == 1) {
    Serial.println("STAGE2");
    stage = 2;
    count = stage2count;
  }
  if (lastreaddown == 0 && stage == 4 && speeddownbuttonread == 1) {
    Serial.println("STAGE3");
    stage = 3;
    count = stage3count;
  }
  if (lastreadup == 0 && stage == 3 && speedupbuttonread == 1) {
    Serial.println("STAGE4");
    stage = 4;
    count = stage4count;
  }
  if (lastreadup == 0 && stage == 2 && speedupbuttonread == 1) {
    Serial.println("STAGE3");
    stage = 3;
    count = stage3count;
  }
  if (lastreadup == 0 && stage == 1 && speedupbuttonread == 1) {
    Serial.println("STAGE2");
    stage = 2;
    count = stage2count;
  }
  if (stage == 0 && speedupbuttonread == 1 ) {
    Serial.println("STAGE1");
    stage = 1;
    count = stage1count;
  }

  lastreadup = speedupbuttonread;
  lastreaddown = speeddownbuttonread;


  //////////SPEED ADJUSTMENTS
  if (speedupadjread == 1 && speeddownadjread == 0) {
    Serial.println("SPEED UP ADJUST");
    count--;
    if (count < 100) {
      count = 100;
    }
  }

  if (speeddownadjread == 1 && speedupadjread == 0) {
    Serial.println("SPEED DOWN ADJUST");
    count++;
  }


  //////////HIGH SPEED MODE (used for solo axis adjustment)
  if (speedupadjread == 1 && speeddownadjread == 1) {
    Serial.println("HIGH SPEED MODE");
    pd = 100;
  }

  else {
    pd = count;
  }


  //////////MOTOR MOVEMENTS
  if (upbuttonread == HIGH) {
    Serial.println("TILT DOWN");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(driverDIR, HIGH);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
  }

  if (downbuttonread == HIGH) {
    Serial.println("TILT UP");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(driverDIR, LOW);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
  }

}

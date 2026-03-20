
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

/////INTERNAL VARIABLES
int pd;
int count = 2500;
int stage = 1;
int lastreadup = 0;
int lastreaddown = 0;

int stage1count = 2500;
int stage2count = 800;
int stage3count = 100;


void setup() {
  //Serial.begin(57600);

  pinMode (driverDIR, OUTPUT);
  pinMode (driverPUL, OUTPUT);
  pinMode(speeddownbutton, INPUT);
  pinMode(speedupbutton, INPUT);
  pinMode(upbutton, INPUT);
  pinMode(downbutton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}


//////////////////////////////////////////////////////////////////////////////////
void loop() {

Serial.println(count);

  digitalWrite(LED_BUILTIN, LOW);

  upbuttonread = digitalRead(upbutton);
  downbuttonread = digitalRead(downbutton);
  speedupbuttonread = digitalRead(speedupbutton);
  speeddownbuttonread = digitalRead(speeddownbutton);


  //////////SPEED STAGE CHANGES
  if (lastreaddown == 0 && stage == 2 && speeddownbuttonread == 1) {
    stage = 1;
    Serial.println("stage1");
    count = stage1count;
  }

  if (lastreaddown == 0 && stage == 3 && speeddownbuttonread == 1) {
    stage = 2;
    Serial.println("stage2");
    count = stage2count;
  }

  if (lastreadup == 0 && stage == 2 && speedupbuttonread == 1) {
    stage = 3;
    Serial.println("stage3");
    count = stage3count;
  }

  if (lastreadup == 0 && stage == 1 && speedupbuttonread == 1) {
    stage = 2;
    Serial.println("stage2");
    count = stage2count;
  }

  lastreadup = speedupbuttonread;
  lastreaddown = speeddownbuttonread;

  pd = count;


  //////////MOTOR MOVEMENTS
  if (upbuttonread == HIGH) {
    digitalWrite(driverDIR, HIGH);
    digitalWrite(driverPUL, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
    Serial.println("Hi");
  }

  if (downbuttonread == HIGH) {
    digitalWrite(driverDIR, LOW);
    digitalWrite(driverPUL, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
    Serial.println("Lo");
  }

}

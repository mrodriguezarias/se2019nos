int wheelDirPin1[] = {8, 10};
int wheelDirPin2[] = {9, 11};
int pingGndPin = 35;
int pingEchoPin = 37;
int pingTriggerPin = 39;
int pingVccPin = 41;
int distanceThreshold = 10;
int turnDelay = 250;

int speed0 = 0;
int speed1 = 130;
int speed2 = 160;
int speed3 = 200;
int speed4 = 255;

int leftWheel = 0;
int rightWheel = 1;

int forwardDirection = 1;
int backwardDirection = 0;

void _drive(int whichWheel, int wheelDirection, int wheelSpeed) {
  if (wheelDirection == forwardDirection) {
    Serial.println("going forward");
    analogWrite(wheelDirPin1[whichWheel], wheelSpeed);
    analogWrite(wheelDirPin2[whichWheel], 0);
  } else if (wheelDirection == backwardDirection) {
    Serial.println("going backward");
    analogWrite(wheelDirPin1[whichWheel], 0);
    analogWrite(wheelDirPin2[whichWheel], wheelSpeed);
  }
}

void drive(int wheelSpeed) {
  _drive(rightWheel, forwardDirection, wheelSpeed);
  _drive(leftWheel, forwardDirection, wheelSpeed);
}

void stop() {
  _drive(rightWheel, forwardDirection, 0);
  _drive(leftWheel, forwardDirection, 0);
}

void turn(int wheelSpeed) {
  _drive(leftWheel, backwardDirection, wheelSpeed);
  _drive(rightWheel, forwardDirection, wheelSpeed);
  delay(turnDelay);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

int isWithinThreshold() {
  digitalWrite(pingTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTriggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTriggerPin, LOW);
  long duration = pulseIn(pingEchoPin, HIGH);
  long cm = microsecondsToCentimeters(duration);
  return cm < distanceThreshold;
}

void setup() {
  Serial.begin(9600);
  pinMode(pingVccPin, OUTPUT);
  pinMode(pingGndPin, OUTPUT);
  pinMode(pingTriggerPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  digitalWrite(pingVccPin, HIGH);
  digitalWrite(pingGndPin, LOW);
}

void loop() {
  while (isWithinThreshold()) {
    turn(speed1);
  }
  drive(speed2);
}

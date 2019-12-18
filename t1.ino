#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include "WiFiEsp.h"
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
SoftwareSerial BT1(13, 12); // RX | TX

int wheelDirPin1[] = {8, 10};
int wheelDirPin2[] = {9, 11};
int pingGndPin = 35;
int pingEchoPin = 37;
int pingTriggerPin = 39;
int pingVccPin = 41;
int lcdVccPin = 22;

int distanceThresholdMax = 20;
int distanceThresholdMin = 3;
int distanceThresholdDegree = 1;
int distanceThreshold = 10;

int turnDelayMax = 1000;
int turnDelayMin = 100;
int turnDelayDegree = 50;
int turnDelay = 250;

int normalMode = 0;
int testMode = 1;
int operationMode = normalMode;

int speed0 = 0;
int speed1 = 130;
int speed2 = 160;
int speed3 = 200;
int speed4 = 255;
int speedMax = speed4;
int speedMin = speed0;
int speedDegree = 30;
int currentSpeed = 255;
int previousSpeed = 0;

int turnRight = 0;
int turnLeft = 1;
int turnDirection = turnRight;

int leftWheel = 0;
int rightWheel = 1;

int forwardDirection = 1;
int backwardDirection = 0;

char ssid[] = "Tinchox";
char pass[] = "22334455";
int status = WL_IDLE_STATUS;
int receivedConfig = 0;
bool wifiModuleStarted = false;
WiFiEspClient client;

int thresholdMode = 0;
int turnMode = 1;
int thresholdTurnMode = thresholdMode;

int lcdScreenOn = 0;
int lcdScreenOff = 1;
int lcdScreenStatus = lcdScreenOn;

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
  if (turnDirection == turnLeft) {
    _drive(leftWheel, forwardDirection, wheelSpeed);
    _drive(rightWheel, backwardDirection, wheelSpeed);
  } else {
    _drive(leftWheel, backwardDirection, wheelSpeed);
    _drive(rightWheel, forwardDirection, wheelSpeed);
  }
  delay(turnDelay);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
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

void doTests() {
  // TODO GET REQUEST CHECK 200
}

String getSpeedPercent() {
  double ratio = double(currentSpeed) / 255;
  return String(int(ratio * 100));
}

void writeToLcd(String str) {
  lcd.clear();
  if (operationMode == testMode) {
    lcd.print(" TEST MODE ON");
  } else {
    lcd.print(" T1 - SE2019NOS");
  }
  lcd.setCursor(0, 1);
  lcd.print(str);
}

void updateLcdScreen() {
  Serial.println("updateLcdScreen called");
  writeToLcd("Speed: " + getSpeedPercent() + "%");
}

void fetchConfig() {
  if (!wifiModuleStarted) {
    return;
  }
  String response = "";
  Serial.println("fetchConfig called");
  while (receivedConfig == 0) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
      response += String(c);
      receivedConfig = 1;
    }
  }
  Serial.println("***");
  int index = response.indexOf("\r\n\r\n");
  response = response.substring(index); // skip header
  response.trim();
  Serial.println(response);
  operationMode = String(response[0]).toInt();
  currentSpeed = String(response[1]).toInt() * 100 + String(response[2]).toInt() * 10 + String(response[3]).toInt();
  turnDelay =  String(response[4]).toInt() * 100 + String(response[5]).toInt() * 10 + String(response[6]).toInt();
  distanceThreshold = String(response[7]).toInt() * 100 + String(response[8]).toInt() * 10 + String(response[9]).toInt();
  turnDirection = String(response[10]).toInt();
  Serial.println(operationMode);
  Serial.println(currentSpeed);
  Serial.println(turnDelay);
  Serial.println(distanceThreshold);
  Serial.println(turnDirection);
}

void initWifiModule() {
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  
  Serial1.begin(115200);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    return;
  }

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect("se2019nos.nixi.icu", 80)) {
    Serial.println("Connected to server");
    // Make a HTTP request
    client.println("GET / HTTP/1.1");
    client.println("Host: se2019nos.nixi.icu");
    client.println("Connection: close");
    client.println();
    wifiModuleStarted = true;
  }
}

void initPing() {
  pinMode(pingVccPin, OUTPUT);
  pinMode(pingGndPin, OUTPUT);
  pinMode(pingTriggerPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  digitalWrite(pingVccPin, HIGH);
  digitalWrite(pingGndPin, LOW);
}

void initLcdScreen() {
  pinMode(lcdVccPin, OUTPUT);
  digitalWrite(lcdVccPin, HIGH);
  lcd.begin(16, 2);
  writeToLcd("Initializing...");
}

void initBluetoothModule() {
  Serial.println("Initializing Bluetooth module..."); 
  BT1.begin(9600);
}

void fetchCommands() {
  if (BT1.available()) {
    char cmd = BT1.read();
    Serial.write(cmd);
    Serial.write('\n');
    switch (cmd) {
     case 'a':
        if (thresholdTurnMode == thresholdMode) {
          distanceThreshold = max(distanceThresholdMin, distanceThreshold - distanceThresholdDegree);
        } else {
          turnDelay = max(turnDelayMin, turnDelay - turnDelayDegree);
        }
        break;
     case 'b':
        currentSpeed = min(speedMax, currentSpeed + speedDegree);
        updateLcdScreen();
        break;
     case 'c':
        if (thresholdTurnMode == thresholdMode) {
          distanceThreshold = min(distanceThresholdMax, distanceThreshold + distanceThresholdDegree);
        } else {
          turnDelay = min(turnDelayMax, turnDelay + turnDelayDegree);
        }
        break;
     case 'd':
        currentSpeed = max(speedMin, currentSpeed - speedDegree);
        updateLcdScreen();
        break;
     case 'e':
        operationMode = operationMode == normalMode ? testMode : normalMode;
        break;
     case 'f':
        if (currentSpeed != speed0) {
          previousSpeed = currentSpeed;
          currentSpeed = speed0;
        } else {
          currentSpeed = previousSpeed;
        }
        break;
     case 'g':
        turnDirection = turnDirection == turnLeft ? turnRight : turnLeft;
        break;
     case 'h':
        thresholdTurnMode = thresholdTurnMode == thresholdMode ? turnMode : thresholdMode;
        break;
     case 'i':
        if (lcdScreenStatus == lcdScreenOn) {
          digitalWrite(lcdVccPin, LOW);
          lcdScreenStatus = lcdScreenOff;
        } else {
          digitalWrite(lcdVccPin, HIGH);
          lcdScreenStatus = lcdScreenOn;
        }
        break;
     case 'j':
        fetchConfig();
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  initLcdScreen();
  initWifiModule();
  fetchConfig();
  initBluetoothModule();
  updateLcdScreen();
  initPing();
}

void doTestMode() {
  drive(speed1);
  delay(500);
  drive(speed2);
  delay(500);
  drive(speed3);
  delay(500);
  drive(speed4);
  delay(500);
  turn(speed2);
  delay(1000);
  drive(speed4);
  delay(500);
  drive(speed3);
  delay(500);
  drive(speed2);
  delay(500);
  drive(speed1);
  delay(500);
  drive(speed0);
  delay(500);
}

void doNormalMode() {
  while (isWithinThreshold()) {
    turn(currentSpeed);
  }
  drive(currentSpeed);
}

void loop() {
  fetchCommands();
  if (operationMode == testMode) {
    doTestMode();
  } else {
    doNormalMode();
  }
}

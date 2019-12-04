#include <LiquidCrystal.h>
#include "WiFiEsp.h"
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

int wheelDirPin1[] = {8, 10};
int wheelDirPin2[] = {9, 11};
int pingGndPin = 35;
int pingEchoPin = 37;
int pingTriggerPin = 39;
int pingVccPin = 41;
int distanceThreshold = 10;
int turnDelay = 250;
int testMode = 0;

int speed0 = 0;
int speed1 = 130;
int speed2 = 160;
int speed3 = 200;
int speed4 = 255;
int currentSpeed = speed4;

int leftWheel = 0;
int rightWheel = 1;

int forwardDirection = 1;
int backwardDirection = 0;

char ssid[] = "Tinchox";
char pass[] = "22334455";
int status = WL_IDLE_STATUS;
int receivedConfig = 0;
WiFiEspClient client;

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

void fetchConfig() {
  int params[] = {0, 
                2, 0, 0,
                0, 8, 0, 
                0, 1, 5 
        };
  /*while (receivedConfig == 0) {
    for (int i = 0; client.available(); i++) {
      params[i] = client.read() - 48;
      receivedConfig = 1;
    }
  }*/
  testMode = params[0];
  currentSpeed = params[1] * 100 + params[2] * 10 + params[3];
  turnDelay =  params[4] * 100 + params[5] * 10 + params[6];
  distanceThreshold = params[7] * 100 + params[8] * 10 + params[9];
  
  Serial.println(testMode);
  Serial.println(currentSpeed);
  Serial.println(turnDelay);
  Serial.println(distanceThreshold);
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
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");
  
  printWifiStatus();

  Serial.println();
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect("arduino.cc", 80)) {
    Serial.println("Connected to server");
    // Make a HTTP request
    client.println("GET /asciilogo.txt HTTP/1.1");
    client.println("Host: arduino.cc");
    client.println("Connection: close");
    client.println();
  }
}

void initLcdScreen() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  pinMode(pingVccPin, OUTPUT);
  pinMode(pingGndPin, OUTPUT);
  pinMode(pingTriggerPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  digitalWrite(pingVccPin, HIGH);
  digitalWrite(pingGndPin, LOW);
  if (testMode) {
    doTests();
  } else {
    lcd.print(" T1 - SE2019NOS");
  }
}

void setup() {
  initWifiModule();
  fetchConfig();
  initLcdScreen();
}

void loop() {
  if (testMode) {
    return;
  }
  while (isWithinThreshold()) {
    turn(currentSpeed);
  }
  drive(currentSpeed);
}

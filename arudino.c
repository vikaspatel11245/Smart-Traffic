// ================= PIN DEFINITIONS =================

// Signal 1
int R1 = 2;
int Y1 = 3;
int G1 = 4;

// Signal 2
int R2 = 5;
int Y2 = 6;
int G2 = 7;

// Signal 3
int R3 = 8;
int Y3 = 9;
int G3 = 10;

// Default green time
int greenTime = 5000;   // 5 seconds
int yellowTime = 2000;  // 2 seconds

void setup() {
  pinMode(R1, OUTPUT);
  pinMode(Y1, OUTPUT);
  pinMode(G1, OUTPUT);

  pinMode(R2, OUTPUT);
  pinMode(Y2, OUTPUT);
  pinMode(G2, OUTPUT);

  pinMode(R3, OUTPUT);
  pinMode(Y3, OUTPUT);
  pinMode(G3, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  signalGreen(1);
  delay(greenTime);
  signalYellow(1);

  signalGreen(2);
  delay(greenTime);
  signalYellow(2);

  signalGreen(3);
  delay(greenTime);
  signalYellow(3);
}

void signalGreen(int signal) {
  allRed();

  if (signal == 1) digitalWrite(G1, HIGH);
  if (signal == 2) digitalWrite(G2, HIGH);
  if (signal == 3) digitalWrite(G3, HIGH);
}

void signalYellow(int signal) {
  allRed();

  if (signal == 1) digitalWrite(Y1, HIGH);
  if (signal == 2) digitalWrite(Y2, HIGH);
  if (signal == 3) digitalWrite(Y3, HIGH);

  delay(yellowTime);
}

void allRed() {
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH);

  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  digitalWrite(Y3, LOW);

  digitalWrite(G1, LOW);
  digitalWrite(G2, LOW);
  digitalWrite(G3, LOW);
}

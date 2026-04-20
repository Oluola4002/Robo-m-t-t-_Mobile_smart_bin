/**
 ******************************************************************************
 * @file    smart_bin_final_2cmd.ino
 * @brief   Autonomous Mobile Smart Bin — 2 Voice Commands FINAL
 * @version 3.2 Final
 ******************************************************************************
 *
 * OPERATION:
 * ─────────────────────────────────────────────────────────────────
 *  Power ON  → Bin automatically begins IR line-follow patrol
 *  "OPEN"    → Patrol stops + lid opens to 90°
 *  "CLOSE"   → Lid closes to 0° + patrol resumes automatically
 *  Power OFF → System stops
 *
 *  AUTOMATIC: Obstacle < 20cm → motors stop
 *             Obstacle gone   → patrol resumes (no voice needed)
 *
 * PIN CONNECTIONS:
 * ─────────────────────────────────────────────────────────────────
 *  L298N Motor Driver:
 *    Pin 4  → IN4  (Left  motor backward)
 *    Pin 5  → IN3  (Left  motor forward)
 *    Pin 6  → IN2  (Right motor backward)
 *    Pin 7  → IN1  (Right motor forward)
 *    Pin 8  → ENB  (Left  motor speed PWM) ← remove jumper
 *    Pin 10 → ENA  (Right motor speed PWM) ← remove jumper
 *
 *  SG90 Servo:
 *    Pin 9  → Signal (orange wire)
 *    5V     → VCC   (red wire)
 *    GND    → GND   (brown wire)
 *
 *  HC-SR04 Ultrasonic:
 *    Pin 11 → TRIG
 *    Pin 12 → ECHO
 *    5V     → VCC
 *    GND    → GND
 *
 *  TCRT5000 IR Sensors x2:
 *    A0     → Left  sensor OUT
 *    A1     → Right sensor OUT
 *    5V     → VCC (both)
 *    GND    → GND (both)
 *
 *  HC-05 Bluetooth:
 *    Pin 2  → HC-05 TX
 *    Pin 3  → HC-05 RX (direct connection — 3.3V or divider)
 *    5V     → VCC
 *    GND    → GND
 *
 * PHONE APP:
 * ─────────────────────────────────────────────────────────────────
 *  App  : Arduino Bluetooth Controller
 *  Mode : Voice Terminal
 *  Pair : HC-05  (PIN: 1234 or 0000)
 *  Say  : "open"  or  "close"
 ******************************************************************************
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================

const int IN1       = 7;
const int IN2       = 6;
const int IN3       = 5;
const int IN4       = 4;
const int ENA       = 10;   // Right motor speed (PWM)
const int ENB       = 8;    // Left  motor speed (PWM)
const int SERVO_PIN = 9;
const int TRIG_PIN  = 11;
const int ECHO_PIN  = 12;
const int IR_LEFT   = A0;
const int IR_RIGHT  = A1;
const int BT_RX     = 2;    // Arduino RX ← HC-05 TX
const int BT_TX     = 3;    // Arduino TX → HC-05 RX

// ============================================================
//  OBJECTS
// ============================================================

Servo          binServo;
SoftwareSerial bluetooth(BT_RX, BT_TX);

// ============================================================
//  SPEED SETTINGS  (tune for your motors)
// ============================================================

const int FULL_SPEED = 200;   // Forward patrol  (0–255)
const int TURN_SPEED = 140;   // Line correction (0–255)

// ============================================================
//  SYSTEM PARAMETERS
// ============================================================

const int           OBSTACLE_DIST = 20;    // cm — auto stop
const int           CLEAR_DIST    = 25;    // cm — auto resume
const unsigned long US_INTERVAL   = 150;   // ms between polls

#define LINE    LOW     // TCRT5000 LOW = over black tape
#define NO_LINE HIGH    // TCRT5000 HIGH = off tape

// ============================================================
//  STATE MACHINE  (3 states — no IDLE needed)
// ============================================================

enum SystemState : uint8_t {
  PATROLLING,       // Following IR line — motors running
  BIN_OPEN,         // Lid open — motors stopped — waiting for CLOSE
  OBSTACLE_STOP     // Obstacle detected — motors stopped — auto resumes
};

SystemState state = PATROLLING;   // Boot straight into patrol

// ============================================================
//  FLAGS AND BUFFERS
// ============================================================

bool          servoOpen   = false;
unsigned long lastUSCheck = 0;

// char array replaces String — no heap fragmentation
#define BT_BUF 64
char    btBuffer[BT_BUF];
uint8_t btLen = 0;

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  analogWrite(ENA, FULL_SPEED);
  analogWrite(ENB, FULL_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Right sensor off line → steer right
void turnRight() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, FULL_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Left sensor off line → steer left
void turnLeft() {
  analogWrite(ENA, FULL_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ============================================================
//  SERVO
// ============================================================

void openBin() {
  if (!servoOpen) {
    binServo.write(90);
    servoOpen = true;
    Serial.println(F(">>> LID OPEN (90)"));
    bluetooth.println(F("Bin open - say CLOSE"));
  }
}

void closeBin() {
  if (servoOpen) {
    binServo.write(0);
    servoOpen = false;
    Serial.println(F(">>> LID CLOSED (0)"));
  }
}

// ============================================================
//  ULTRASONIC
// ============================================================

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur  = pulseIn(ECHO_PIN, HIGH, 30000);
  long dist = dur * 0.034 / 2;
  return (dist <= 0 || dist > 400) ? 0 : dist;
}

// ============================================================
//  IR LINE FOLLOWING  (2 sensors)
// ============================================================
/*
 *  LEFT     RIGHT    Action
 *  LINE     LINE   → moveForward  (centred on line)
 *  LINE     NO_LINE→ turnRight    (right sensor off — correct right)
 *  NO_LINE  LINE   → turnLeft     (left  sensor off — correct left)
 *  NO_LINE  NO_LINE→ moveForward  (both off — junction or reacquire)
 */
void followLine() {
  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);
  if      (L == LINE    && R == LINE)    moveForward();
  else if (L == LINE    && R == NO_LINE) turnRight();
  else if (L == NO_LINE && R == LINE)    turnLeft();
  else                                   moveForward();
}

// ============================================================
//  OBSTACLE CHECK  (automatic — no voice needed)
// ============================================================

void checkObstacle() {
  unsigned long now = millis();
  if (now - lastUSCheck < US_INTERVAL) return;
  lastUSCheck = now;

  long dist = getDistance();

  // Auto stop when obstacle detected while patrolling
  if (state == PATROLLING && dist > 0 && dist <= OBSTACLE_DIST) {
    stopMotors();
    state = OBSTACLE_STOP;
    Serial.println(F("!!! OBSTACLE < 20cm"));
    bluetooth.println(F("Obstacle! Waiting..."));
  }

  // Auto resume when obstacle clears
  if (state == OBSTACLE_STOP && (dist == 0 || dist > CLEAR_DIST)) {
    state = PATROLLING;
    Serial.println(F("Obstacle cleared - resuming"));
    bluetooth.println(F("Resuming patrol"));
  }
}

// ============================================================
//  BLUETOOTH VOICE PARSER  (OPEN and CLOSE only)
// ============================================================
/*
 * Receives text string from Arduino Bluetooth Controller
 * Voice Terminal mode.
 *
 * Characters lowercased as they arrive → no extra pass.
 * strstr() substring match → handles natural speech phrases.
 * e.g. "please open the bin" → detected as "open" ✅
 *
 * Only two commands:
 *   "open"  → stop patrol + open lid
 *   "close" → close lid  + resume patrol
 */

void processVoiceCommands() {
  while (bluetooth.available()) {
    char c = (char)bluetooth.read();

    if (c == '\n' || c == '\r') {
      if (btLen > 0) {
        btBuffer[btLen] = '\0';

        Serial.print(F("[BT] ")); Serial.println(btBuffer);

        // ── OPEN → stop patrol + open lid ─────────────────────────────────
        if (strstr(btBuffer, "open")) {
          if (state == PATROLLING || state == OBSTACLE_STOP) {
            stopMotors();
            openBin();
            state = BIN_OPEN;
            Serial.println(F(">>> BIN OPEN - patrol paused"));
          } else if (state == BIN_OPEN) {
            bluetooth.println(F("Bin already open"));
          }
        }

        // ── CLOSE → close lid + resume patrol ─────────────────────────────
        else if (strstr(btBuffer, "close")) {
          if (state == BIN_OPEN) {
            closeBin();
            delay(300);
            state = PATROLLING;
            Serial.println(F(">>> LID CLOSED - patrol resumed"));
            bluetooth.println(F("Lid closed - patrol resumed"));
          } else {
            bluetooth.println(F("Bin already closed"));
          }
        }

        // ── Unknown ───────────────────────────────────────────────────────
        else {
          bluetooth.println(F("Say OPEN or CLOSE"));
        }

        btLen = 0;
      }

    } else {
      // Lowercase as it arrives + overflow guard
      if (btLen < BT_BUF - 1) {
        btBuffer[btLen++] = tolower((unsigned char)c);
      } else {
        btLen = 0;  // overflow — reset
      }
    }
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  // Servo — start closed
  binServo.attach(SERVO_PIN);
  binServo.write(0);
  servoOpen = false;
  delay(500);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // IR sensors
  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Sensor diagnostics
  delay(300);
  long d = getDistance();

  Serial.println(F("============================"));
  Serial.println(F("  RoboMimo Smart Bin v3.2"));
  Serial.println(F("  OPEN / CLOSE voice control"));
  Serial.println(F("============================"));
  Serial.print(F("[OK] Ultrasonic: ")); Serial.print(d); Serial.println(F("cm"));
  if (d == 0) Serial.println(F("[!!] Check ultrasonic wiring"));
  Serial.print(F("[OK] IR Left : "));
  Serial.println(digitalRead(IR_LEFT)  == LINE ? F("ON LINE") : F("off line"));
  Serial.print(F("[OK] IR Right: "));
  Serial.println(digitalRead(IR_RIGHT) == LINE ? F("ON LINE") : F("off line"));
  Serial.println(F("[OK] Lid: closed (0 deg)"));
  Serial.println(F("Say OPEN  -> lid opens"));
  Serial.println(F("Say CLOSE -> lid closes + patrol"));
  Serial.println(F("============================"));

  bluetooth.println(F("RoboMimo Ready!"));
  bluetooth.println(F("Say OPEN to open bin"));

  // Boot straight into patrol
  state = PATROLLING;
  Serial.println(F(">>> PATROL STARTED"));
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {

  // 1. Always listen for voice commands
  processVoiceCommands();

  // 2. Obstacle check during patrol only
  if (state == PATROLLING || state == OBSTACLE_STOP) {
    checkObstacle();
  }

  // 3. Execute current behaviour
  switch (state) {
    case PATROLLING:    followLine();  break;
    case BIN_OPEN:      stopMotors();  break;
    case OBSTACLE_STOP: stopMotors();  break;
  }

  delay(10);
}

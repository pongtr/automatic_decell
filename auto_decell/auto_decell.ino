//== PUMP DEFINITIONS ============================
#define ENone 34
#define Aone  35
#define ENtwo 36
#define Atwo  37

//== PUMP FUNCTIONS =========================
int EN, A;
void pumps_init() {
  pinMode(ENone, OUTPUT);
  pinMode(Aone,  OUTPUT);
  pinMode(ENtwo, OUTPUT);
  pinMode(Atwo,  OUTPUT);
  Serial.println("Pumps Initialized");
  digitalWrite(ENone, LOW);
  digitalWrite(Aone,  LOW);
  digitalWrite(ENtwo, LOW);
  digitalWrite(Atwo,  LOW);
}

void pump_on(int pump) {
  pump_select(pump);
  digitalWrite(EN, HIGH);
  digitalWrite(A,  HIGH);
}

void pumps_on() {
  pump_on(1); pump_on(2);
}

void pump_off(int pump) {
  pump_select(pump);
  digitalWrite(EN, LOW);
  digitalWrite(A,  LOW);
}

void pumps_off() {
  pump_off(1); pump_off(2);
}

void pump_select(int pump) {
  switch (pump) {
    case 1:
      EN = ENone;
      A  = Aone;
      break;
    case 2:
      EN = ENtwo;
      A  = Atwo;
      break;
  }
}

//== SERVO DEFINITIONS =====================================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  100
#define SERVOMAX  520

//== SERVO FUNCTIONS =======================================
double pulseLengthOpen, pulseLengthClose;
bool servo_isOpen[9];

// to put in void setup()
void servo_init() {
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();
  for (int i = 0; i < 9; i++) {
    valve_setAngles(i);
    pwm.setPWM(i, 0, pulseLengthClose);
    servo_isOpen[i] = false;
  }
  Serial.println("Servos Initialized");
}

void valve_Close(uint8_t servonum) {
  if (!servo_isOpen[servonum]) return;
  valve_setAngles(servonum);
  valve_sweep(pulseLengthOpen, pulseLengthClose, servonum);
  servo_isOpen[servonum] = false;
}

void valve_Open(uint8_t servonum) {
  if (servo_isOpen[servonum]) return;
  valve_setAngles(servonum);
  valve_sweep(pulseLengthClose, pulseLengthOpen, servonum);
  servo_isOpen[servonum] = true;
}

void valve_AllClose() {
  for (int i = 0; i < 9; i++)
    valve_Close(i);
}

void valve_AllOpen() {
  for (int i = 0; i < 9; i++)
    valve_Open(i);
}

void valve_setAngles(int servonum) {
  int degreesOpen, degreesClose;
  switch (servonum) {
    case 1:
      degreesOpen  = 20;
      degreesClose = 83;
      break;
    case 2:
      degreesOpen  = 20;
      degreesClose = 84;
      break;
    case 3:
      degreesOpen  = 20;
      degreesClose = 85 ;
      break;
    case 4:
      degreesOpen  = 20;
      degreesClose = 81;
      break;
    case 0:
      degreesOpen  = 180;
      degreesClose = 122;
      break;
    case 7:
      degreesOpen  = 180;
      degreesClose = 125;
      break;
    case 5:
      degreesOpen  = 180;
      degreesClose = 134;
      break;
    case 6:
      degreesOpen  = 180;
      degreesClose = 130;
      break;
    case 8 :
      degreesOpen  = 60;
      degreesClose = 155;
      break;
  }
  pulseLengthOpen  = map(degreesOpen,  0, 180, SERVOMIN, SERVOMAX);
  pulseLengthClose = map(degreesClose, 0, 180, SERVOMIN, SERVOMAX);
}

void valve_sweep(int from, int to, int servonum) {
  if (from < to) {
    for (int pulselen = from; pulselen < to; pulselen++) {
      pwm.setPWM(servonum, 0, pulselen);
      delay(3);
    }
  } else {
    for (int pulselen = from; pulselen > to; pulselen--) {
      pwm.setPWM(servonum, 0, pulselen);
      delay(3);
    }
  }
}

//== SD & CHRONODOT DEFINITIONS ====================
#include <SD.h>
#include <Wire.h>

//== SD FUNCTIONS =================================
void sd_init() {
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("Done!");
}

void sd_write(String print_string, String filename) {
  File myFile = SD.open(filename, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(print_string);
    Serial.print("Wrote \"");
    Serial.print(print_string);
    Serial.print("\" to \"");
    Serial.print(filename);
    Serial.println("\"");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  myFile.close();
}

//== CHRONODOT FUNCTIONS ==========================
void chronodot_init() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.write(0b00011100);
  Wire.endTransmission();
  Serial.println("ChronoDot Initialized");
}

#define FORMAT(V) ((V >= 10) ? String(V) : String(0) + String(V))

String chronodot_read() {
  int seconds, minutes, hours;
  // send request to receive data starting at register 0
  Wire.beginTransmission(0x68); // 0x68 is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(0x68, 3); // request three bytes (seconds, minutes, hours)

  while (Wire.available())
  {
    seconds = Wire.read(); // get seconds
    minutes = Wire.read(); // get minutes
    hours = Wire.read();   // get hours

    seconds = (((seconds & 0b11110000) >> 4) * 10 + (seconds & 0b00001111)); // convert BCD to decimal
    minutes = (((minutes & 0b11110000) >> 4) * 10 + (minutes & 0b00001111)); // convert BCD to decimal
    hours = (((hours & 0b00100000) >> 5) * 20 + ((hours & 0b00010000) >> 4) * 10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)
  }
  return FORMAT(hours) + ":" + FORMAT(minutes) + ":" + FORMAT(seconds);
}

//== LCD DEFINITIONS ====================
// Libraries to include
#include <LiquidCrystal.h>

// Initialize the LCD library
LiquidCrystal lcd(43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53);
// LiquidCrystal lcd(43, 45, 50, 51, 52, 53);

//== LCD FUNCTIONS ======================
void lcd_init() {
  // initialize LCD
  lcd.begin(16, 2);
  // Print the initial message to the LCD
  lcd.clear();
  lcd.print("Set up config A!");
  // delay(5000);
  Serial.println("LCD initialized");
}

//== BUTTON BUZZER DEFINITIONS ==========
// initialize pins for and buttons
#define BUZZ 38
#define ButR 39
#define ButG 40
#define ButB 41
#define ButW 42

#define PAUSEWHILE(B) while(!digitalRead(B))

//== BUTTON BUZZER FUNCTIONS ============
void butbuz_init() {
  // initialize pins for buttons, and buzzer
  pinMode(BUZZ, OUTPUT);
  pinMode(ButR, INPUT);
  pinMode(ButG, INPUT);
  pinMode(ButB, INPUT);
  pinMode(ButW, INPUT);

  // make sure buzzer is off
  digitalWrite(BUZZ, LOW);

  Serial.println("Buttons and buzzer initialized");
}

void buzz() {
  digitalWrite(BUZZ, HIGH);
  delay(100);
  digitalWrite(BUZZ, LOW);
}

//== SERIAL SETUP =======================
void serial_init() {
  Serial.begin(115200);
}

//== ETAPE DEFINITIONS ===================
#define REJECT_VAL 500  // arbitrary reject value
#define NREADINGS 10    // number of readings
#define ETAPE_RESISTOR     560    // Value of the series resistor in ohms.    
#define ETAPE_PIN          A0
int zeroVol;
float etape_gradient;
float etape_intercept;

//== ETAPE FUNCTIONS ======================

void etape_init() {
  pinMode(ETAPE_PIN, INPUT);
  analogReadResolution(12);
  //etape_calibrate(ETAPE_PIN, ETAPE_RESISTOR);
  etape_setCal(-3.05, 2210.68);
  vol_reset();
}

void etape_calibrate(int pin, int seriesResistance) {
  float resistance[6];

  // grab different resistances
  for (int vol = 0, i = 0; vol <= 500; vol += 100, i++) {
    // prompt user
    Serial.print("Fill to ");
    Serial.print(vol);
    Serial.println(" ml. Press green button to continue");

    // wait for input
    PAUSEWHILE(ButG); // wait until green button press
    buzz();

    resistance[i] = res_read(pin, seriesResistance); // read resistance on eTape
    Serial.print("Resistance: ");
    Serial.println(resistance[i]);
    delay(100);
  }

  // process resistance
  // linear interpolation using end points
  etape_gradient  = (resistance[5] - resistance[1]) / 400.;
  etape_intercept = resistance[1] - etape_gradient * 100;
  // get abs difference of other points
  Serial.println("Absolute Resistance Difference");
  float max_diff = 0; // store max difference
  for (int i = 2; i < 5; i++) {
    Serial.print(i * 100); Serial.print("ml: ");
    float diff = resistance[i] - (etape_gradient * (i * 100) + etape_intercept);
    diff = abs(diff);
    Serial.println(diff);
    if (diff > max_diff) max_diff = diff;
  }

  // approve or reject
  if (max_diff > REJECT_VAL) {
    Serial.println("REJECT!");
  } else {
    Serial.println("PASS");
    empty_waste();
  }
  Serial.print("GRADIENT: ");  Serial.println(etape_gradient);
  Serial.print("INTERCEPT: "); Serial.println(etape_intercept);
  delay(5000);
}

void etape_setCal(float grad, float intercept) {
  etape_gradient  = grad;
  etape_intercept = intercept;
}

int smooth(int pin) {
  int sum = 0;
  for (int i = 0; i < NREADINGS; i++)
    sum += analogRead(pin);
  return sum / NREADINGS;
}

void vol_reset() {
  zeroVol = read_abs_vol(ETAPE_PIN, ETAPE_RESISTOR);
}

float vol_read() {
  return read_abs_vol(ETAPE_PIN, ETAPE_RESISTOR) - zeroVol;
}

float vol_read_abs() {
  return read_abs_vol(ETAPE_PIN, ETAPE_RESISTOR);
}

float res_read(int pin, int seriesResistance) {
  float resistance = (float) smooth(pin);
  resistance = (4095.0 / resistance) - 1.0;
  resistance = seriesResistance / resistance;
  return resistance;
}

float read_abs_vol(int pin, int seriesResistance) {
  float resistance = res_read(pin, seriesResistance);

  if (resistance >= etape_intercept) { // if the resistance is higher than the sensor can read
    return 0.0; // assume zero mL
  }
  float currVol = (resistance - etape_intercept) / etape_gradient;
  return currVol;
}

//== DEBUG MODE ==========================================

#define debug_init() attachInterrupt(digitalPinToInterrupt(ButW), state_debug, RISING)

volatile bool FIRST = true; // first time in state
volatile int STATE = 1;
volatile int oldState = 1;
volatile bool debugged = false;

void state_debug() {
  // hold button for 500 micro seconds
  int start_time = micros();
  while (digitalRead(ButW) && micros() < start_time + 500);
  if (micros() < start_time + 500)
    return;

  // passes 2 seconds
  if (STATE != -1)
    oldState = STATE ;
  STATE = -1;
  debugged = true;
  FIRST = true;
}

void debug_mode() {
  Serial.println("+------------------------------------------------------+");
  Serial.println("| DEBUG: [s]tate / [p]ump / [v]alve / [e]tape / [q]uit |");
  Serial.println("+------------------------------------------------------+");
  while (!Serial.available());
  int mode = Serial.read();
  int input, num;
  float ft1, ft2;
  bool invalid = false;
  switch (mode) {
    case 's':
      Serial.print("Current STATE: "); Serial.println(oldState);
      Serial.println("Jump to STATE: <input number, 0 = exit>");
      while (!Serial.available());
      input = Serial.parseInt();
      if (input != 0) {
        STATE = input;
        Serial.print("Jump to "); Serial.println(STATE);
        debugged = false;
      } else {
        Serial.println("Exit");
      }
      break;
    case 'p':
      Serial.println("PUMP [1]Top / [2] Bottom / [-1]All");
      while (!Serial.available());
      num = Serial.parseInt();
      if (num != 1 && num != 2 && num != -1) {
        invalid = true;
        break;
      }
      if (num == -1) {
        Serial.println("PUMP ALL o[n] / o[f]f");
        while (!Serial.available());
        input = Serial.read();
        if (input == 'n') {
          pumps_on();
          Serial.println("PUMP ALL on");
        } else if (input == 'f') {
          pumps_off();
          Serial.println("PUMP ALL off");
        } else {
          invalid = true;
        }
        break;
      }
      Serial.print("PUMP "); Serial.print(num); Serial.println(" o[n] / o[f]f");
      while (!Serial.available());
      input = Serial.read();
      if (input == 'n') {
        pump_on(num);
        Serial.print("PUMP "); Serial.print(num); Serial.println(" on");
      } else if (input == 'f') {
        pump_off(num);
        Serial.print("PUMP "); Serial.print(num); Serial.println(" off");
      } else {
        invalid = true;
      }
      break;
    case 'v':
      Serial.println("VALVE [0]Lung / [1 - 7]Jars / [8]Drain / [-1]All");
      while (!Serial.available());
      num = Serial.parseInt();
      if (num < -1 || num > 8) {
        invalid = true;
        break;
      }
      if (num == -1) {
        Serial.println("VALVE ALL [o]pen / [c]lose");
        while (!Serial.available());
        input = Serial.read();
        if (input == 'o') {
          valve_AllOpen();
          Serial.println("VALVE ALL open");
        } else if (input == 'c') {
          valve_AllClose();
          Serial.println("VALVE ALL close");
        } else {
          invalid = true;
        }
        break;
      }
      Serial.print("VALVE "); Serial.print(num); Serial.println(" [o]pen / [c]lose");
      while (!Serial.available());
      input = Serial.read();
      if (input == 'o') {
        valve_Open(num);
        Serial.print("VALVE "); Serial.print(num); Serial.println(" open");
      } else if (input == 'c') {
        valve_Close(num);
        Serial.print("VALVE "); Serial.print(num); Serial.println(" close");
      } else {
        invalid = true;
      }
      break;
    case 'e':
      Serial.println("ETAPE: [c]alibrate / [s]et calibration / read [v]olume / read [r]esistance");
      while (!Serial.available());
      input = Serial.read();
      if (input == 'c') {
        etape_calibrate(ETAPE_PIN, ETAPE_RESISTOR);
      } else if (input == 'v') {
        Serial.print(read_abs_vol(ETAPE_PIN, ETAPE_RESISTOR)); Serial.println("ml");
      } else if (input == 'r') {
        Serial.print(res_read(ETAPE_PIN, ETAPE_RESISTOR)); Serial.println("ohms");
      } else if (input == 's') {
        Serial.print("Current Values: Gradient = "); Serial.print(etape_gradient);
        Serial.print(" Intercept = "); Serial.println(etape_intercept);
        Serial.print("Gradient: ");
        while (!Serial.available());
        ft1 = Serial.parseFloat();
        Serial.println(ft1);
        Serial.print("Intercept: ");
        while (!Serial.available());
        ft2 = Serial.parseFloat();
        Serial.println(ft2);

        etape_setCal(ft1, ft2);
      } else {
        invalid = true;
      }
      break;
    case 'q':
      STATE = oldState;
      debugged = false;
      Serial.println("Quit debug mode");
      break;
    default:
      invalid = true;
      break;
  }
  if (invalid) {
    Serial.println("Invalid Input");
  }
}

#define delayDebug(T) {\
    int start = millis();\
    int until = start + T;\
    while (millis() < until) {\
      if (debugged) {\
        return;\
      }\
    }\
  }

#define waitButton(BUT) {\
    while (!digitalRead(BUT)) {\
      if (debugged) {\
        return;\
      }\
    }\
  }


//===========Integrated Functions=====================

void empty_wastebyVol() {
  Serial.println("Emptying");
  //valve_allClose();
  pumps_off();
  valve_Open(8);
  float timeAVol = vol_read_abs();
  delayDebug(500);
  float timeBVol = vol_read_abs();
  while (timeAVol > timeBVol - 20) {
    Serial.print(timeAVol); Serial.print("\t"); Serial.println(timeBVol);
    timeAVol = vol_read_abs();
    delayDebug(500);
    timeBVol = vol_read_abs();
    //Serial.print(".");
  }
  //Serial.println();
  valve_Close(8);
}

void empty_waste() {
  Serial.print("Emptying");
  pumps_off();
  valve_Open(8);
  float vol = vol_read_abs();
  Serial.println(vol);
  while (vol > 250.) {
    Serial.println(vol);
    vol = vol_read_abs();
    delayDebug(10);
  }
  valve_Close(8);
  delayDebug(5000);
  while (vol > 100) {
    valve_Open(8);
    Serial.println(vol);
    vol = vol_read_abs();
    delayDebug(10);
  }
  valve_Close(8);
  delayDebug(5000);
  while (vol > 20) {
    valve_Open(8);
    Serial.println(vol);
    vol = vol_read_abs();
    delayDebug(10);
  }
  valve_Close(8);
}

void state_betweenState() {
  //STATE 12, 14, 16
  int endstep, startstep;
  switch (STATE) {
    case 12: endstep = 1; startstep = 2; break;
    case 14: endstep = 2; startstep = 3; break;
    case 16: endstep = 3; startstep = 4; break;
    case 44: endstep = 9; startstep = 10; break;
    case 72: endstep = 15; startstep = 16; break;
  }
  pumps_off();
  valve_AllClose();
  String printString = "Step " + String(endstep) + ": end" + String(chronodot_read()) + "\n";
  Serial.print(printString);
  sd_write(printString, "time_recordings.txt");
  printString = "Step " + String(startstep) + ": start" + String(chronodot_read()) + "\n";
  Serial.print(printString);
  sd_write(printString, "time_recordings.txt");
  STATE++;
}


void state_toTrachea() {
  //STATE 32, 33
  switch (STATE) {
    case 32:
      String printStringE = "Step 6: Complete" + String(chronodot_read()) + "\n";
      String printStringS = "Step 7: Benz AW Inject";
      break;
    case 33:
      String printStringE = "Step 7: Complete" + String(chronodot_read()) + "\n";
      String printStringS = "Step 8: B+B AW Inject + Cap";
      break;
  }
  pumps_off();
  valve_AllClose();
  Serial.print(printStringE);
  sd_write(printStringE, "time_recordings.txt");
  Serial.print(printStringS);
  sd_write(printStringS, "time_recordings.txt");
  PAUSEWHILE(ButG); // wait until green button press
  STATE++;
}

void state_uncapTrachea() {
  //STATE 42
  pumps_off();
  valve_AllClose();
  Serial.print("Uncap Trachea now ");
  PAUSEWHILE(ButG); // wait until green button press
  String printStringE = "Step 9: Start" + String(chronodot_read()) + "\n";
  Serial.print(printStringE);
  sd_write(printStringE, "time_recordings.txt");
  STATE++;
}

void state_complete() {
  //STATE 125
  pumps_off();
  valve_AllClose();
  Serial.print("Decell Complete");
}

//====================================================

void setup() {
  // INITIALIZE EVERYTHING
  serial_init();
  pumps_init();
  servo_init();
  sd_init();
  chronodot_init();
  lcd_init();
  butbuz_init();
  debug_init();
  etape_init();
  //lcd.print("HELLO");
  delay(10000);
}

unsigned long TIMER_TARGET, TIMER_REMAINING;

void loop() {
  Serial.print("STATE: "); Serial.println(STATE);

  // timer countdown
  if (STATE > 34 && STATE < 42 ||
      STATE > 74 && STATE < 85) {
    TIMER_REMAINING = TIMER_TARGET - millis();
    String remaining_str = FORMAT(TIMER_REMAINING / 60000) + ":" + FORMAT((TIMER_REMAINING / 1000) % 60);
    lcd.clear();
    lcd.setCursor(0, 1);
    Serial.println(remaining_str);
  }
  switch (STATE) {
    case -1:
      debug_mode(); break;
    case 1: // set up bottles
      state1(); break;
    case 2: case 35: case 76: // prime lines 5 -> j
    case 3: case 36: case 77:// prime lines 4 -> j
    case 4: case 37: case 78: // prime lines 3 -> j
    case 5: case 38: case 65: case 79: // prime lines 2 -> j
    case 6: case 39: case 80: // prime lines 1 -> j
    case 25: case 64: case 75: // prime lines 6 -> j
      state_primeJar(); break;

    case 7: case 98: case 105: case 112: // fill jar 2 -> j
    case 28: case 68: // fill jar 6 -> j
    case 119: //fill jar 5 -> j
      state_fillJar(); break;

    case 8: // prime lines 3 -> l
    case 26: // prime lines 6 -> l
    case 40: case 66: case 81:
      state_primeLung(); break;

    case 9: // prep for part A 
      state9(); break;

    case 10: // prepare for step 1
    case 18: // end step 4
    case 20: // prepare for step 5
    case 22: // end step 5
    case 30: case 46: case 48: case 50: case 56: case 60:
    case 70: case 85: case 89: case 93: case 100: case 107:
    case 114: case 54: case 58: case 62: case 87: case 91:
    case 95: case 102: case 109: case 116: case 123:
      state_changeStep(); break;

    case 11: // step 1: PBS/Hep/SNP 50mL 3 to Lung
    case 13: // step 2: PBS+Ions    50mL 5 to Lung
    case 15: // step 3: Abx         50mL 4 to Lung
    case 17: // step 4: PBS+Ions   100mL 5 to Lung
    case 21: // step 5: Triton     425mL 1 to Lung
    case 57: case 101: //  5 -> L
    case 53: case 94: case 115: //  4 -> L
    case 49: case 86: case 108: //  3 -> L
    case 45: case 61: case 73: //  2 -> L
    case 43: case 90: // 1 -> L
    case 31: case 71: case 112://  6 -> L
      state_toLung(); break;

    case 12: // step 1 to step 2
    case 14: // step 2 to step 3
    case 16: // step 3 to step 4
    case 44: // step 9 to step 10
    case 72: // step 15 to step 16
      state_betweenState(); break;

    // EMPTY WASTE
    case 19: case 23: case 29: case 47: case 51: case 55:
    case 59: case 69: case 82: case 88: case 92: case 96:
    case 99: case 103: case 106: case 110: case 113:
    case 117: case 120: case 124:
      empty_waste(); break;

    case 24: // Signal installation of bottle 6
      state24(); break;

    // EMPTY JAR
    case 27:  case 67: case 97: case 104:
    case 111: case 118:
      state_emptyJar(); break;

    // inject Benz AW and B+B AW to trachea and cap
    case 32: case 33:
      state_toTrachea(); break;

    // setup & set timer
    case 34: case 74:
      state_setupTimer(); break;

    case 42:
      state_uncapTrachea(); break;
      
    case 125:
      state_complete(); break;
  }
}

void state1() {
  pumps_off();
  valve_AllClose();
  lcd.clear();
  Serial.print("Set up config A");
  lcd.setCursor(0, 1);
  Serial.println(" Green to go");
  waitButton(ButG);
  buzz();
  STATE++;
}

void state_primeJar() {
  if (!FIRST) {
    if (digitalRead(ButG)) {
      buzz();
      STATE++;
      FIRST = true;
      return;
    } else {
      return;
    }
  }
  int to_open;
  switch (STATE) {
case 2: case 35: cse 76:
      to_open = 5; break;
    case 3: case 36: case 77:
      to_open = 4; break;
    case 4: case 37: case 78:
      to_open = 3; break;
    case 5: case 38: case 65: case 79:
      to_open = 2; break;
    case 6: case 39: case 80:
      to_open = 1; break;
    case 25: case 64: case 75
        to_open = 6; break;
  }
  pumps_off();
  valve_AllClose();
  valve_Open(to_open);
  valve_Open(7);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Prime bottle " + String(to_open));
  FIRST = false;
  //lcd.setCursor(0, 1);
  //lcd.println(" to the jar");
  /*
    waitButton(ButG);
    buzz();
    STATE++;
  */
}

void state_fillJar() {
  int source;
  switch (STATE) {
    case 28:
    case 68:
      source = 6; break;
    case 7:
    case 98:
    case 105:
    case 112:
      source = 2; break;
    case 119:
      source = 5; break;
  }
  valve_AllClose();
  valve_Open(source);
  valve_Open(7);
  pump_on(1);
  int zeroval = vol_read_abs();
  while ( vol_read_abs() <= 10 + zeroval) {
    Serial.println(vol_read_abs());
    delay(500);
  }
  vol_reset();
  pump_off(1);
  valve_Open(2);
  valve_Open(7);
  valve_AllClose();
  pump_on(1);
  delay(10000);
  pump_off(1);
  STATE++;
}

void state_toJar(int to_open) {
  pumps_off();
  valve_AllClose();
  valve_Open(to_open);
  valve_Open(7);
  lcd.clear();
  Serial.print("Prime bottle " + String(to_open));
  lcd.setCursor(0, 1);
  Serial.println(" to the jar");
  waitButton(ButG);
  buzz();
  STATE++;
}

void until_target(int target) {
  int remaining = target;
  int zeroval = vol_read_abs();
  while (remaining > 0) {
    lcd.clear();
    lcd.setCursor(0, 1);
    Serial.println(String(remaining) + " mL left");
    delayDebug(500);
    remaining = target - (vol_read_abs() - zeroval) ;
    if (debugged)
      return;
  }
}

void state_toLung() {
  // STATE 11, 13, 15, 17, 21
  int pump;
  int valve;
  String step_and_solution;
  int amount;
  switch (STATE) {
    case 11:
      valve = 3; pump = 1;
      step_and_solution = "Step 1: P/H/S";
      amount = 50; break;
    case 13:
      valve = 5; pump = 1;
      step_and_solution = "Step 2: PBS+Ions";
      amount = 50; break;
    case 15:
      valve = 4; pump = 1;
      step_and_solution = "Step 3: Abx ";
      amount = 50; break;
    case 17:
      valve = 5; pump = 1;
      step_and_solution = "Step 4: PBS+Ions ";
      amount = 100; break;
    case 21:
      valve = 1; pump = 1;
      step_and_solution = "Step 5: Triton ";
      amount = 425; break;
    case 43:
      valve = 1; pump = 1;
      step_and_solution = "Step 9: NaCl in PBS ";
      amount = 150; break;
    case 90:
      valve = 1; pump = 1;
      step_and_solution = "Step 18: 0.5% Triton ";
      amount = 100; break;
    case 45:
      valve = 2; pump = 1;
      step_and_solution = "Step 10: PBS ";
      amount = 250; break;
    case 61:
      valve = 2; pump = 1;
      step_and_solution = "Step 14: PBS ";
      amount = 250; break;
    case 73:
      valve = 2; pump = 1;
      step_and_solution = "Step 16: Benz+Benz ";
      amount = 250; break;
    case 49:
      valve = 3; pump = 1;
      step_and_solution = "Step 11: SDC 0.01%";
      amount = 425; break;
    case 86:
      valve = 3; pump = 1;
      step_and_solution = "Step 17: PBS";
      amount = 425; break;
    case 108:
      valve = 3; pump = 1;
      step_and_solution = "Step 21: PBS";
      amount = 500; break;
    case 53:
      valve = 4; pump = 1;
      step_and_solution = "Step 12: SDC 0.05% ";
      amount = 425; break;
    case 94:
      valve = 4; pump = 1;
      step_and_solution = "Step 19: PBS ";
      amount = 500; break;
    case 115:
      valve = 4; pump = 1;
      step_and_solution = "Step 22: PBS ";
      amount = 500; break;
    case 57:
      valve = 5; pump = 1;
      step_and_solution = "Step 13: SDC 0.1% ";
      amount = 425; break;
    case 101:
      valve = 5; pump = 1;
      step_and_solution = "Step 20: PBS ";
      amount = 500; break;
    case 31:
      valve = 6; pump = 1;
      step_and_solution = "Step 6: Benzonase ";
      amount = 250; break;
    case 71:
      valve = 6; pump = 1;
      step_and_solution = "Step 17: Benzonase ";
      amount = 250; break;
    case 122:
      valve = 6; pump = 1;
      step_and_solution = "Step 23: Abx ";
      amount = 500; break;
  }
  valve_AllClose();
  valve_Open(valve);
  valve_Open(0);
  lcd.clear();
  Serial.println(step_and_solution);
  pump_on(pump);
  until_target(amount);
  pump_off(pump);
  STATE++;
}



void state_primeLungOLD() {
  pumps_off();
  valve_AllClose();
  lcd.clear();
  int valve;
  switch (STATE) {
    case 8:
      valve = 3;
      lcd.print("Prime Bottle 3 "); break;
    case 26:
      valve = 6;
      lcd.print("Prime Bottle 6 "); break;
  }
  valve_Open(valve);
  valve_Open(0);
  lcd.setCursor(0, 1);
  lcd.print("to the lung, green to continue");
  waitButton(ButG);
  buzz();
  STATE++;
}

void state_primeLung() {
  // not first time in state =>
  if (!FIRST) {
    if (digitalRead(ButG)) {
      buzz();
      STATE++;
      FIRST = true;
      return;
    } else {
      return;
    }
  }
  int valve;
  switch (STATE) {
    case 8: valve = 3; break;
    case 26: valve = 6; break;
    case 40: valve = 1; break;
    case 66: valve = 6; break;
    case 81: valve = 2; break;
  }
  pumps_off();
  valve_AllClose();
  valve_Open(valve);
  valve_Open(0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Prime bottle " + String(valve));
  //lcd.setCursor(0, 1);
  //Serial.println(" to the jar");
  /*
    waitButton(ButG);
    buzz();
    STATE++;
  */
  FIRST = false;
}

void state9() {
  pumps_off();
  valve_AllClose();
  lcd.clear();
  Serial.print("Press Green Button");
  lcd.setCursor(0, 1);
  Serial.println(" to begin Part A");
  waitButton(ButG);
  buzz();
  STATE++;
}

void state_changeStep() {
  // STATE 10, 18, 20, 22
  pumps_off();
  valve_AllClose();
  String str;
  switch (STATE) {
    case 10:
      str = "Step 1: Start " + chronodot_read(); break;
    case 18:
      str = "Step 4: End " + chronodot_read(); break;
    case 20:
      str = "Step 5: Start " + chronodot_read(); break;
    case 22:
      str = "Step 5: End " + chronodot_read(); break;
    case 30:
      str = "Step 6: Start " + chronodot_read(); break;
    case 46:
      str = "Step 10: End " + chronodot_read(); break;
    case 48:
      str = "Step 11: Start " + chronodot_read(); break;
    case 50:
      str = "Step 11: End " + chronodot_read(); break;
    case 52:
      str = "Step 12: Start " + chronodot_read(); break;
    case 54:
      str = "Step 12: End " + chronodot_read(); break;
    case 56:
      str = "Step 13: Start " + chronodot_read(); break;
    case 58:
      str = "Step 13: End " + chronodot_read(); break;
    case 60:
      str = "Step 14: Start " + chronodot_read(); break;
    case 62:
      str = "Step 14: End " + chronodot_read(); break;
    case 70:
      str = "Step 15: Start " + chronodot_read(); break;
    case 85:
      str = "Step 17: Start " + chronodot_read(); break;
    case 87:
      str = "Step 17: End " + chronodot_read(); break;
    case 89:
      str = "Step 18: Start " + chronodot_read(); break;
    case 91:
      str = "Step 18: End " + chronodot_read(); break;
    case 93:
      str = "Step 19: Start " + chronodot_read(); break;
    case 95:
      str = "Step 19: End " + chronodot_read(); break;
    case 100:
      str = "Step 20: Start " + chronodot_read(); break;
    case 102:
      str = "Step 20: End " + chronodot_read(); break;
    case 107:
      str = "Step 21: Start " + chronodot_read(); break;
    case 109:
      str = "Step 21: End " + chronodot_read(); break;
    case 114:
      str = "Step 22: Start " + chronodot_read(); break;
    case 116:
      str = "Step 22: End " + chronodot_read(); break;
    case 123:
      str = "Step 23: End " + chronodot_read(); break;
  }
  sd_write(str, "Time_Recordings");
  STATE++;
}

void state24() {
  pumps_off();
  valve_AllClose();
  lcd.clear();
  lcd.print("Set up bottle 6");
  lcd.setCursor(0, 1);
  lcd.print("Benzonase");
  waitButton(ButG);
  buzz();
  STATE++;
}

void state_emptyJar() {
  valve_AllClose();
  pump_on(2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Emptying Jar");
  int current_vol, old_vol;
  old_vol = vol_read_abs();
  do {
    delayDebug(2000);
    old_vol = current_vol;
    current_vol = vol_read_abs();
  } while (current_vol < 250 && abs(current_vol - old_vol) < 10);
  // make sure volumen is greater than 250 ml + same (with 10mL error)
  pump_off(2);
  STATE++;
}

// for 34 & 74
void state_setupTimer() {
  pumps_off();
  valve_AllClose();
  int mins;
  switch (STATE) {
    case 34: mins = 30; break;
    case 74: mins = 60; break;
  }
  sd_write("Step 8: Complete " + chronodot_read(), "Time_Recordings");
  TIMER_TARGET = millis() + (1000 * 60 * mins); // 30 mins
  STATE++;
}




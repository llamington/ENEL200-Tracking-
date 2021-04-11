#include <LiquidCrystal_I2C.h>

#define SERIAL_BAUD_RATE 115200
#define DISP_SHOW_BTN_PIN 2
#define COUNTER_RESET_PIN 3
#define CAMERA_TRIGGER_PIN LED_BUILTIN // SET THIS

#define ADC_MIN 0
#define ADC_MAX 1023 // 2^10 - 1

#define IR_RANGE_MAX 80.0
#define IR_RANGE_MIN 5.70760233918128
#define IR_PIN A0
#define IR_OUT_OF_RANGE -1
#define IR_CURVE_COEFFICIENT 13.5
#define IR_CURVE_EXP -0.791

#define DETECTION_MS 100
#define DETECTION_DEV_CM 0.5 // deviation in width for detection event

#define CAMERA_TRIGGER_PULSE_MS 30

double tunnelWidth;
float curRange;
uint16_t nPredators = 0;
uint8_t msDetected = 0;
bool animalInTunnel = false; // if animal has exited tunnel, as to not be tracked twice
bool dispOn = false;
uint16_t dispOnTime = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

// runs once
void setup(void)
{
  pinMode(DISP_SHOW_BTN_PIN, INPUT_PULLUP);
  pinMode(COUNTER_RESET_PIN, INPUT_PULLUP);
  pinMode(CAMERA_TRIGGER_PIN, OUTPUT);

  digitalWrite(CAMERA_TRIGGER_PIN, LOW);

  Serial.begin(SERIAL_BAUD_RATE);
  lcd.init();
  lcd.print(F("INITIALISED"));
  Serial.println(F("TRACKING TUNNEL INITIALISED"));
  attachInterrupt(digitalPinToInterrupt(DISP_SHOW_BTN_PIN), handleDispShow, FALLING);
  attachInterrupt(digitalPinToInterrupt(COUNTER_RESET_PIN), handleAnimalCountReset, FALLING);
  delay(1000);
  tunnelWidth = readRangeCm();
  lcd.setCursor(0, 0);
  lcd.print("TUNNEL WIDTH = ");
  lcd.setCursor(0, 1);
  lcd.print(tunnelWidth);
  Serial.print("TUNNEL WIDTH = "); //if out of range display error
  Serial.println(tunnelWidth);
  delay(1000);
  displayAnimalCount();
}

// runs indefinitely
void loop(void)
{
  curRange = readRangeCm();
  if(dispOn && (dispOnTime <= 10000)) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
    dispOn = false;
    dispOnTime = 0;
  }
  if((curRange <= (tunnelWidth - 0.5)) || (curRange >= (tunnelWidth + 0.5))) {
    if(((msDetected += 50) >= DETECTION_MS) && !animalInTunnel) {
      animalInTunnel = true;
      nPredators++;
      msDetected = 0;
      pulseCameraTrigger();
      Serial.println("Animal Detected");
      displayAnimalCount();
      delay(100);
    } 
  } else animalInTunnel = false;
  dispOnTime += 50;
  delay(50);
}

// converts raw ADC output to voltage
float rawToVoltage(uint16_t raw)
{
  Serial.print(raw);
  Serial.print(" ");
  return raw * (5.0 / 1023.0);
}

// converts voltage to range in cm
double voltageToRange(float voltage)
{
  float range = IR_CURVE_COEFFICIENT * pow(voltage, IR_CURVE_EXP);
  Serial.print(voltage);
  Serial.print(" ");
  if((range <= IR_RANGE_MAX) && (range >= IR_RANGE_MIN)) {
    Serial.println(range);
    return range;
  }
  return IR_OUT_OF_RANGE;
}

// handler for display show button press
void handleDispShow(void)
{
  // may need debouncing
  dispOn = true;
}

// handler for 
void handleAnimalCountReset(void)
{
  // may need debouncing
  nPredators = 0;
}

// reads IR sensor, returns range in cm
float readRangeCm(void)
{
  uint16_t rangeRaw = analogRead(IR_PIN);
  float rangeVolt = rawToVoltage(rangeRaw);
  return voltageToRange(rangeVolt);
}

void pulseCameraTrigger(void)
{
    digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
    delay(CAMERA_TRIGGER_PULSE_MS);
    digitalWrite(CAMERA_TRIGGER_PIN, LOW);
}

void displayAnimalCount() {
  lcd.clear();
  lcd.print("Animal Count =");
  lcd.setCursor(0, 1);
  lcd.print(nPredators);
}

#include <LiquidCrystal_I2C.h>

#define SERIAL_BAUD_RATE 115200

#define DISP_I2C_ADDR 0x27
#define DISP_RES_H 20
#define DISP_RES_V 4

#define DISP_SHOW_BTN_PIN 2
#define IR_PIN A0
#define COUNTER_RESET_PIN 3
#define CAMERA_TRIGGER_PIN 12

#define ADC_MAX_V 5.0
#define ADC_MAX 1023.0 // 2^10 - 1 as 10 bit ADC

#define IR_RANGE_MAX 80.0
#define IR_RANGE_MIN 5.7076
#define IR_OUT_OF_RANGE -1
#define IR_CURVE_COEFFICIENT 26.85
#define IR_CURVE_EXP -1.2642

#define DETECTION_DEV_CM 1.5 // deviation in width for detection event

#define CAMERA_TRIGGER_PULSE_MS 30 // pulse that camera requires
#define INITIALISATION_DELAY_MS 5000
#define DISPLAY_TIMEOUT_MS 10000
#define POLLING_DELAY_MS 50
#define DETECTION_MS 100 // ms required for a detection event

double tunnelWidth;
float curRange;
uint16_t nPredators = 0;
uint8_t msDetected = 0;
bool animalInTunnel = false; // if animal has exited tunnel, as to not be tracked twice
bool dispOn = false;
uint16_t dispOnTime = 0;
bool shouldUpdateDisp = false;

LiquidCrystal_I2C lcd(DISP_I2C_ADDR, DISP_RES_H, DISP_RES_V);

// runs once
void setup(void)
{

  pinMode(DISP_SHOW_BTN_PIN, INPUT_PULLUP);
  pinMode(COUNTER_RESET_PIN, INPUT_PULLUP);
  pinMode(CAMERA_TRIGGER_PIN, OUTPUT);

  digitalWrite(CAMERA_TRIGGER_PIN, LOW);

  Serial.begin(SERIAL_BAUD_RATE);

//  delay(INITIALISATION_DELAY_MS);
  lcd.init();
  lcd.backlight();
  
  lcd.print(F("INITIALISED"));
  Serial.println(F("TRACKING TUNNEL INITIALISED"));
  
  attachInterrupt(digitalPinToInterrupt(DISP_SHOW_BTN_PIN), handleDispShow, FALLING);
  attachInterrupt(digitalPinToInterrupt(COUNTER_RESET_PIN), handleAnimalCountReset, FALLING);
  
  tunnelWidth = readRangeCm();
  if(tunnelWidth == IR_OUT_OF_RANGE) {
    Serial.println(F("OUT OF RANGE. PLEASE RESET TUNNEL"));
    lcd.setCursor(0,0);
    lcd.print(F("OUT OF RANGE."));
    lcd.setCursor(0, 1);
    lcd.print(F("RESET TUNNEL"));
    while(true);
  }
  lcd.setCursor(0, 0);
  lcd.print(F("TUNNEL WIDTH = "));
  lcd.setCursor(0, 1);
  lcd.print(tunnelWidth);
  Serial.print(F("TUNNEL WIDTH = "));
  Serial.println(tunnelWidth);
  delay(INITIALISATION_DELAY_MS);
  lcd.noBacklight();
  displayAnimalCount();
}

// runs indefinitely
void loop(void)
{
  curRange = readRangeCm();

  // turn lcd off if display timeout elapsed
  if(dispOn && (dispOnTime <= DISPLAY_TIMEOUT_MS)) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
    dispOn = false;
    dispOnTime = 0;
  }
  
  //update display if display signal received
  if(shouldUpdateDisp) {
    displayAnimalCount();
    shouldUpdateDisp = false;
  }
  
  // if current range deviates from tunnel width
  if((curRange <= (tunnelWidth - DETECTION_DEV_CM)) || (curRange >= (tunnelWidth + DETECTION_DEV_CM))) { 
    // only handle new animal if detected for the set time, and not already an animal in the tunnel
    if(((msDetected += POLLING_DELAY_MS) >= DETECTION_MS) && !animalInTunnel) { 
      animalInTunnel = true;
      nPredators++;
      msDetected = 0;
      pulseCameraTrigger();
      shouldUpdateDisp = true;
      Serial.println("ANIMAL DETECTED");
    } 
  } else animalInTunnel = false;
  dispOnTime += POLLING_DELAY_MS;
  delay(POLLING_DELAY_MS);
}

// converts raw ADC output to voltage
float rawToVoltage(uint16_t raw)
{
  return raw * (ADC_MAX_V / ADC_MAX);
}

// converts voltage to range in cm
double voltageToRange(float voltage)
{
  float range = IR_CURVE_COEFFICIENT * pow(voltage, IR_CURVE_EXP);
  Serial.println(range);
  if((range <= IR_RANGE_MAX) && (range >= IR_RANGE_MIN)) {
    return range;
  }
  return IR_OUT_OF_RANGE;
}

// handler for display show button press
void handleDispShow(void)
{
  dispOn = true;
}

// handler for animal count reset
void handleAnimalCountReset(void)
{
  // may need debouncing
  nPredators = 0;
  shouldUpdateDisp = true;
}

// reads IR sensor, returns range in cm
float readRangeCm(void)
{
  uint16_t rangeRaw = analogRead(IR_PIN);
  float rangeVolt = rawToVoltage(rangeRaw);
  return voltageToRange(rangeVolt);
}

// pulses camera trigger output
void pulseCameraTrigger(void)
{
    digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
    delay(CAMERA_TRIGGER_PULSE_MS);
    digitalWrite(CAMERA_TRIGGER_PIN, LOW);
}

// updates LCD with current animal count
void displayAnimalCount() {
  lcd.clear();
  lcd.print(F("ANIMAL COUNT ="));
  lcd.setCursor(0, 1);
  lcd.print(nPredators);
}

#define DISP_SHOW_BTN_PIN 2
#define CALIB_BTN_PIN 3

#define ADC_MIN 0
#define ADC_MAX 1023 // 2^10 - 1

#define IR_VOLTAGE_MIN 40
#define IR_VOLTAGE_MAX 230
#define IR_DISTANCE_MIN 10
#define IR_DISTANCE_MAX 80
#define IR_IN A0
#define IR_OUT_OF_RANGE -1

// runs once
void setup()
{
  pinMode(DISP_SHOW_BTN_PIN, INPUT_PULLUP);
  pinMode(CALIB_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DISP_SHOW_BTN_PIN), handleDispShow, FALLING);
  attachInterrupt(digitalPinToInterrupt(CALIB_BTN_PIN), handleCalibrate, FALLING);
}

// runs indefinitely
void loop()
{
  ;
}

// converts raw ADC output to voltage
float rawToVoltage(uint16_t raw)
{
  return (map(raw, ADC_MIN, ADC_MAX, 0, 500)) / 100.0;
}

// converts voltage to range in cm
int8_t voltageToRange(float voltageTimes100)
{
  if((voltageTimes100 <= IR_VOLTAGE_MAX) && (voltageTimes100 >= IR_VOLTAGE_MIN)) {
    return map(voltageTimes100, IR_VOLTAGE_MAX, IR_VOLTAGE_MIN, IR_DISTANCE_MIN, IR_DISTANCE_MAX);
  }
  return IR_OUT_OF_RANGE;
}

// handler for display show button press
void handleDispShow(void)
{
  ;
}

// handler for calibration button press
void handleCalibrate(void)
{
  ;
}

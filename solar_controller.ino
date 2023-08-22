#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

static const uint8_t batChargeControlPin = 3;

float solar_volt = 0;  // variable for solar panel voltage
float bat_volt = 0;    // variable for battery voltage
float pwm = 0;         // pwm out put to mosfet
static const uint8_t loadControlPin = 4;
const int delayTime = 200;  // Time between shifting LCD messages (in milliseconds)
const float min_bat_volt = 5.0;

void setup() {
  Serial.begin(9600);
  pinMode(loadControlPin, OUTPUT);
  digitalWrite(batChargeControlPin, LOW);
  digitalWrite(loadControlPin, LOW);
  lcd.begin(16, 2);
  lcd.print("LUMUMBA 217016375");
}

void loop() {
  senseVoltage();
  //for (float i = 0; i < 15; i += 1.5)
  chargeControl();

  chargeIndicators();
  loadControl();
}

void loadControl() {
  if (solar_volt > 10 && bat_volt > 7) {
    digitalWrite(loadControlPin, HIGH);
  } else if (bat_volt > min_bat_volt + 2) {
    digitalWrite(loadControlPin, HIGH);
  } else
    digitalWrite(loadControlPin, LOW);
}

unsigned long previousMillis = 0;
bool shiftedLeft = false;
void chargeIndicators() {
  // Non-blocking delay using millis()
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= delayTime) {
    previousMillis = currentMillis;
    if (shiftedLeft)
      lcd.scrollDisplayRight();
    else
      lcd.scrollDisplayLeft();
    shiftedLeft = !shiftedLeft;
    lcd.setCursor(0, 1);
    float charged_percent = bat_volt * 100 / 14;
    if (charged_percent >= 90) charged_percent = 100;
    lcd.print(String(" ") + bat_volt + String(" V ") + charged_percent);
  }
}

void chargeControl() {
  if (solar_volt > bat_volt) {
    if (bat_volt < 5) {
      pwm = 60;
    } else if ((bat_volt > 5) && (bat_volt <= 7)) {
      pwm = 150;
    } else if ((bat_volt > 7) && (bat_volt <= 12)) {
      pwm = 200;
    } else if ((bat_volt > 12) && (bat_volt <= 14)) {
      pwm = 150;
    } else if (bat_volt >= 14) {
      pwm = 95;
    } else if (bat_volt >= 14.2) {
      pwm = 30;
    }
  }

  if ((bat_volt >= 14.4) or (solar_volt < bat_volt)) {
    pwm = 0;
  }

  analogWrite(batChargeControlPin, pwm);
}

unsigned long previousMillis_VoltageSerial = 0;

float solarAnalogVolt = 1023;
float solarMeasuredMaxVolt = 22;
float solarMeasuredInput = 4.97;

float batteryAnalogVolt = 1023;
float batteryMeasuredMaxVolt = 15.6;
float batteryMeasuredInput_At_14_4_V = 4.97;
const uint8_t samples = 100;

void senseVoltage() {
  float solarSample = 0;
  float batsample = 0;
  for (int i = 0; i < samples; i++) {
    batsample += analogRead(A0);
    solarSample += analogRead(A1);
  }

  batsample /= samples;
  solarSample /= samples;

  solar_volt = mapFloat(solarSample, 0, solarAnalogVolt, 0, solarMeasuredMaxVolt);
  bat_volt = mapFloat(batsample, 0, batteryAnalogVolt, 0, batteryMeasuredMaxVolt);
  // delay(100);
  // Serial.println(solarSample);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_VoltageSerial >= delayTime) {
    previousMillis_VoltageSerial = currentMillis;
    Serial.println();
    Serial.print("solar input voltage :");
    Serial.println(solar_volt);
    Serial.print("battery voltage :");
    Serial.println(bat_volt);
    Serial.print("pwm duty cycle is : ");
    Serial.println(pwm);
  }
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  // Make sure not to divide by zero
  if (fromHigh == fromLow) {
    return toLow;
  }

  // Perform the mapping
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/*
  https://playground.arduino.cc/Main/TimerPWMCheatsheet/
  // TCCR0B = TCCR0B & 0b11111000 | 0x05;    // prescaling 1024. This sets the frequency of the pwm to 61.03515625 Hz. Pins 5 and 6: controlled by Timer 0 in fast PWM mode (cycle length = 256)
  // TCCR1B = (TCCR1B & 0b11111000) | 0x05;  // prescaling 1024. This sets the frequency of the pwm to 30.64 Hz. Pins 9 and 10: controlled by timer 1 in phase-correct PWM mode (cycle length = 510)
  TCCR2B = (TCCR2B & 0b11111000) | 0x07;  //  prescaling 1024. This sets the frequency of the pwm to 30.64 Hz. Pins 11 and 3: controlled by timer 2 in phase-correct PWM mode (cycle length = 510)
*/
void set_1024_PWM_Frequency_Prescaler(uint8_t pwmPin) {
  pinMode(pwmPin, OUTPUT);
  switch (pwmPin) {
    case 3:
    case 11:
      //TCCR2B = (TCCR2B & 0b11111000) | 0x05;
      break;
    case 9:
    case 10:
      // Set Timer 1 prescaler to 1024 (for pins 9 and 10)
      TCCR1B = (TCCR1B & 0b11111000) | 0x05;  // Prescaler = 1024
      break;
    case 5:
    case 6:
      // Set Timer 0 prescaler to 1024 (for pins 5 and 6)
      TCCR0B = (TCCR0B & 0b11111000) | 0x05;  // Prescaler = 1024
      break;
  }
}

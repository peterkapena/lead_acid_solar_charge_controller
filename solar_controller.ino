#include <LiquidCrystal.h>
#define BAT_CHARGE_CONTROL_PIN 3
#define LOAD_CONTROL_PIN 4
#define MAX_SOLAR_VOLT_DIVIDER 22
#define MAX_BAT_VOLT_DIVIDER 15.6
#define VOLT_READING_SAMPLES 100
#define LCD_OUTPUT_DELAY 1000
#define SERIAL_OUTPUT_DELAY 700
#define MIN_BAT_VOLT 5

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

float solar_volt = 0;  //  solar panel voltage
float bat_volt = 0;    //  battery voltage
float pwm_value = 0;   // pwm out put to npn base

unsigned long previous_millis_lcd = 0;
bool lcd_shifted_left = false;
unsigned long previous_millis_voltage_serial = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  digitalWrite(BAT_CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  lcd.begin(16, 2);
  lcd.print("LUMUMBA 217016375");
}

void loop() {
  senseVoltage();
  chargeControl();
  chargeIndicators();
  loadControl();
}

void loadControl() {
  if (solar_volt > 10 && bat_volt > 7) {
    digitalWrite(LOAD_CONTROL_PIN, HIGH);
  } else if (bat_volt > MIN_BAT_VOLT + 2) {
    digitalWrite(LOAD_CONTROL_PIN, HIGH);
  } else
    digitalWrite(LOAD_CONTROL_PIN, LOW);
}

void chargeIndicators() {
  unsigned long current_millis = millis();
  if (current_millis - previous_millis_lcd >= LCD_OUTPUT_DELAY) {
    previous_millis_lcd = current_millis;
    if (lcd_shifted_left)
      lcd.scrollDisplayRight();
    else
      lcd.scrollDisplayLeft();
    lcd_shifted_left = !lcd_shifted_left;
    lcd.setCursor(0, 1);
    float charged_percent = bat_volt * 100 / 14;
    if (charged_percent >= 90)
      charged_percent = 100;
    lcd.print(String(" ") + bat_volt + String(" V ") + charged_percent);
  }
}

void chargeControl() {
  if (solar_volt < 14) {
    pwm_value = 250;
  } else if (solar_volt > bat_volt) {
    if (bat_volt < 5) {
      pwm_value = 60;
    } else if ((bat_volt > 5) && (bat_volt <= 7)) {
      pwm_value = 200;
    } else if ((bat_volt > 7) && (bat_volt <= 12)) {
      pwm_value = 200;
    } else if ((bat_volt > 12) && (bat_volt <= 14)) {
      pwm_value = 150;
    } else if (bat_volt >= 14) {
      pwm_value = 95;
    } else if (bat_volt >= 14.2) {
      pwm_value = 30;
    }
  }

  if ((bat_volt >= 14.4) or (solar_volt < bat_volt)) {
    pwm_value = 0;
  }

  analogWrite(BAT_CHARGE_CONTROL_PIN, pwm_value);
}

void senseVoltage() {
  float solarSample = 0;
  float batsample = 0;
  for (int i = 0; i < VOLT_READING_SAMPLES; i++) {
    batsample += analogRead(A0);
    solarSample += analogRead(A1);
  }

  batsample /= VOLT_READING_SAMPLES;
  solarSample /= VOLT_READING_SAMPLES;

  solar_volt = mapFloat(solarSample, 0, 1023, 0, MAX_SOLAR_VOLT_DIVIDER);
  bat_volt = mapFloat(batsample, 0, 1023, 0, MAX_BAT_VOLT_DIVIDER);
  unsigned long currentMillis = millis();

  if (currentMillis - previous_millis_voltage_serial >= SERIAL_OUTPUT_DELAY) {
    previous_millis_voltage_serial = currentMillis;
    Serial.println();
    Serial.print("solar input voltage :");
    Serial.println(solar_volt);
    Serial.print("battery voltage :");
    Serial.println(bat_volt);
    Serial.print("pwm duty cycle is : ");
    Serial.println(pwm_value);
  }
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  if (fromHigh == fromLow) return toLow;
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
      // TCCR2B = (TCCR2B & 0b11111000) | 0x05;
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

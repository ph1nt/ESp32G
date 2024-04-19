// ESP32 C3
// 400kB RAM

#include <Arduino.h>

#define readPeriod 7.8125  // 128 samples per second
#define adcPin A0

uint16_t minAdc, maxAdc, curAdc;
uint32_t invalidTime, beat_old;
uint16_t adcData[60 * 1000 / readPeriod];  // 60 sec * 1000 ms / 10 ms = 60000
uint8_t leftHand, rightHand;
uint16_t beatIndex;
float beats[500];

void calculateBPM() {
  uint8_t BPM;
  int beat_new = millis();          // get the current millisecond
  int diff = beat_new - beat_old;   // find the time between the last two beats
  float currentBPM = 60000 / diff;  // convert to beats per minute
  beats[beatIndex] = currentBPM;    // store to array to convert the average
  float total = 0.0;
  for (int i = 0; i < 500; i++) {
    total += beats[i];
  }
  BPM = int(total / 500);
  beat_old = beat_new;
  beatIndex = (beatIndex + 1) %
              500;  // cycle through the array instead of using FIFO queue
}

boolean dataValid() {
  leftHand = digitalRead(10);
  rightHand = digitalRead(11);
  return !((leftHand == 1) || (rightHand == 1));
}

void adcTask(void *pvParameters) {
  while (1) {
    if (dataValid()) {
      curAdc = analogRead(adcPin);
      if (curAdc < minAdc) {
        minAdc = curAdc;
      }
      if (curAdc > maxAdc) {
        maxAdc = curAdc;
      }
    } else {
      invalidTime++;
    }
    vTaskDelay(readPeriod / portTICK_PERIOD_MS);  // 128 samples per second
  }
}

void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
  pinMode(10, INPUT);                     // Setup for leads off detection LO +
  pinMode(11, INPUT);                     // Setup for leads off detection LO -
  analogSetPinAttenuation(A0, ADC_11db);  // 11db attenuation
  xTaskCreate(&adcTask, "periodic Adc read", 2048, NULL, 5, NULL);
}

void loop() {
  if ((digitalRead(10) == 1) || (digitalRead(11) == 1)) {
    Serial.println('!');
  } else {
    // send the value of analog input 0:
    Serial.println(analogRead(A0));
  }
  // Wait for a bit to keep serial data from saturating
  delay(1);
}

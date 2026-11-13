// Slow color-wheel rotation for the Freenove 8 RGB LED Module: all 8 pixels
// share one color, hue rotating slowly through the full wheel at constant
// brightness, so the module's output can be judged for an actual
// application rather than just for diagnosing the stuck pixel.
#include <Adafruit_NeoPixel.h>

const uint8_t LED_OUT_PIN = 9;
const uint16_t NUM_PIXELS = 8;

const unsigned long HUE_PERIOD_MS = 20000; // one full trip around the color wheel
const uint8_t BRIGHTNESS = 255;            // constant, no breathing

Adafruit_NeoPixel rgb(NUM_PIXELS, LED_OUT_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  rgb.begin();
  rgb.show();
}

void loop() {
  unsigned long now = millis();

  uint16_t hue = (uint32_t)(now % HUE_PERIOD_MS) * 65536UL / HUE_PERIOD_MS;

  uint32_t color = rgb.gamma32(rgb.ColorHSV(hue, 255, BRIGHTNESS));
  for (uint16_t i = 0; i < NUM_PIXELS; i++) {
    rgb.setPixelColor(i, color);
  }
  rgb.show();

  if (now - lastPrint >= 250) {
    lastPrint = now;
    Serial.print(F("hue="));
    Serial.println((uint32_t)hue * 360UL / 65536UL);
  }
}

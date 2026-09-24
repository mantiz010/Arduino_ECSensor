#include <Arduino.h>
#include <ECSensorV2.h>

// Example pins only. Change these to suit your board.
// Avoid boot/strapping pins where possible.
static constexpr uint8_t PIN_CAP_POS = 4;
static constexpr uint8_t PIN_CAP_NEG = 5;
static constexpr uint8_t PIN_EC      = 6;

ECSensorV2 ec;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!ec.begin(PIN_CAP_POS, PIN_CAP_NEG, PIN_EC)) {
    Serial.println("ECSensorV2: unsupported platform");
    return;
  }

  ec.setOversampling(5); // 32 measurements per reading.

  // For normal use, calibrate once with a known EC standard,
  // then store ec.getCalibrationConstant() in Preferences/NVS.
  //
  // Example:
  // Place probe in a 1.413 mS/cm calibration solution and run:
  // if (ec.calibrate(1.413f)) {
  //   Serial.printf("Calibration constant = %.3f\n", ec.getCalibrationConstant());
  // }
  //
  // On later boots:
  // ec.setCalibrationConstant(YOUR_SAVED_CONSTANT);
}

void loop() {
  const uint32_t raw = ec.readRaw();

  Serial.printf("raw=%lu status=%s",
                static_cast<unsigned long>(raw),
                ec.getStatusText());

  if (ec.getCalibrationConstant() > 0.0f) {
    const float value = ec.readEC();
    if (!isnan(value)) {
      Serial.printf(" EC=%.3f mS/cm", value);
    }
  }

  Serial.println();
  delay(2000);
}

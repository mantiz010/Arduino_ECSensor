#pragma once

#include <Arduino.h>

class ECSensorV2 {
public:
  enum class Status : uint8_t {
    Ok = 0,
    NotBegun,
    Timeout,
    InvalidReading,
    UnsupportedPlatform
  };

  ECSensorV2();

  // ESP32 family: choose any suitable GPIOs. capPos must support GPIO interrupts.
  bool begin(uint8_t capPosPin, uint8_t capNegPin, uint8_t ecPin);

  // Average 2^rate samples. Default rate=5 => 32 samples.
  void setOversampling(uint8_t rate);
  uint8_t getOversampling() const;

  // Raw capacitor discharge duration in CPU cycles.
  uint32_t readRaw();

  // One-point calibration model:
  // EC (mS/cm) = calibrationConstant / rawCycles.
  // Use calibrate(referenceEC) while the probe is in a known standard.
  bool calibrate(float referenceECmScm);
  void setCalibrationConstant(float value);
  float getCalibrationConstant() const;

  float readEC();
  float readEC(float temperatureC);
  float readTDS(float tdsFactor = 500.0f);
  float readTDS(float temperatureC, float tdsFactor);

  void setTemperatureCoefficient(float alphaPerC);
  float getTemperatureCoefficient() const;

  Status getStatus() const;
  const char* getStatusText() const;
  bool probeConnected() const;

private:
  uint8_t capPosPin_;
  uint8_t capNegPin_;
  uint8_t ecPin_;
  uint8_t oversamplingRate_;
  bool begun_;
  volatile uint32_t edgeCycle_;
  float calibrationConstant_;
  float alpha_;
  Status status_;

  uint32_t singleMeasurement();
  void disconnectPins();
  uint32_t cyclesPerMicrosecond() const;

#if defined(ARDUINO_ARCH_ESP32)
  static void ARDUINO_ISR_ATTR onFallingEdge(void* arg);
#endif
};

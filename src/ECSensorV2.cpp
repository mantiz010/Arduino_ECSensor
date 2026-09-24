#include "ECSensorV2.h"

#if defined(ARDUINO_ARCH_ESP32)
  #include "esp_cpu.h"
#endif

namespace {
constexpr uint32_t CHARGE_DELAY_US = 40;
constexpr uint32_t DISCHARGE_DELAY_US = 15;
constexpr uint32_t EC_TIMEOUT_US = 2000;
constexpr uint8_t MAX_OVERSAMPLING = 8;
}

ECSensorV2::ECSensorV2()
  : capPosPin_(0),
    capNegPin_(0),
    ecPin_(0),
    oversamplingRate_(5),
    begun_(false),
    edgeCycle_(0),
    calibrationConstant_(0.0f),
    alpha_(0.02f),
    status_(Status::NotBegun) {}

bool ECSensorV2::begin(uint8_t capPosPin, uint8_t capNegPin, uint8_t ecPin) {
#if defined(ARDUINO_ARCH_ESP32)
  capPosPin_ = capPosPin;
  capNegPin_ = capNegPin;
  ecPin_ = ecPin;

  disconnectPins();
  begun_ = true;
  status_ = Status::Ok;
  return true;
#else
  (void)capPosPin;
  (void)capNegPin;
  (void)ecPin;
  begun_ = false;
  status_ = Status::UnsupportedPlatform;
  return false;
#endif
}

void ECSensorV2::setOversampling(uint8_t rate) {
  oversamplingRate_ = (rate > MAX_OVERSAMPLING) ? MAX_OVERSAMPLING : rate;
}

uint8_t ECSensorV2::getOversampling() const {
  return oversamplingRate_;
}

uint32_t ECSensorV2::cyclesPerMicrosecond() const {
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t mhz = getCpuFrequencyMhz();
  return (mhz == 0) ? 1 : mhz;
#else
  return 1;
#endif
}

#if defined(ARDUINO_ARCH_ESP32)
void ARDUINO_ISR_ATTR ECSensorV2::onFallingEdge(void* arg) {
  auto* sensor = static_cast<ECSensorV2*>(arg);
  if (sensor->edgeCycle_ == 0) {
    sensor->edgeCycle_ = static_cast<uint32_t>(esp_cpu_get_cycle_count());
  }
}
#endif

uint32_t ECSensorV2::singleMeasurement() {
#if !defined(ARDUINO_ARCH_ESP32)
  status_ = Status::UnsupportedPlatform;
  return 0;
#else
  // Stage 1: charge capacitor in the positive direction.
  digitalWrite(capPosPin_, HIGH);
  digitalWrite(capNegPin_, LOW);
  pinMode(ecPin_, INPUT);
  pinMode(capPosPin_, OUTPUT);
  pinMode(capNegPin_, OUTPUT);
  delayMicroseconds(CHARGE_DELAY_US);

  // Stage 2: discharge through the EC probe and capture the falling edge.
  edgeCycle_ = 0;
  pinMode(capPosPin_, INPUT);
  attachInterruptArg(capPosPin_, &ECSensorV2::onFallingEdge, this, FALLING);

  const uint32_t startCycle = static_cast<uint32_t>(esp_cpu_get_cycle_count());
  const uint32_t startUs = micros();

  digitalWrite(ecPin_, LOW);
  pinMode(ecPin_, OUTPUT);

  while (edgeCycle_ == 0) {
    if (static_cast<uint32_t>(micros() - startUs) >= EC_TIMEOUT_US) {
      detachInterrupt(capPosPin_);
      disconnectPins();
      status_ = Status::Timeout;
      return 0;
    }
  }

  detachInterrupt(capPosPin_);
  const uint32_t dischargeCycles = edgeCycle_ - startCycle;

  if (dischargeCycles == 0) {
    disconnectPins();
    status_ = Status::InvalidReading;
    return 0;
  }

  // Stage 3: fully discharge before changing polarity.
  digitalWrite(capPosPin_, LOW);
  digitalWrite(capNegPin_, LOW);
  pinMode(ecPin_, INPUT);
  pinMode(capPosPin_, OUTPUT);
  pinMode(capNegPin_, OUTPUT);
  delayMicroseconds(DISCHARGE_DELAY_US);

  // Stage 4: charge in the negative direction.
  digitalWrite(capNegPin_, HIGH);
  delayMicroseconds(CHARGE_DELAY_US);

  // Stage 5: apply an equal-duration negative compensation pulse.
  digitalWrite(ecPin_, HIGH);
  pinMode(capPosPin_, INPUT);
  pinMode(ecPin_, OUTPUT);

  uint32_t compensateUs = dischargeCycles / cyclesPerMicrosecond();
  if (compensateUs == 0) {
    compensateUs = 1;
  }
  delayMicroseconds(compensateUs);

  // Stage 6: return the capacitor to a neutral state.
  digitalWrite(capPosPin_, HIGH);
  pinMode(capPosPin_, OUTPUT);
  pinMode(ecPin_, INPUT);
  delayMicroseconds(DISCHARGE_DELAY_US);

  disconnectPins();
  status_ = Status::Ok;
  return dischargeCycles;
#endif
}

uint32_t ECSensorV2::readRaw() {
  if (!begun_) {
    status_ = Status::NotBegun;
    return 0;
  }

  const uint16_t samples = static_cast<uint16_t>(1U << oversamplingRate_);
  uint64_t total = 0;

  for (uint16_t i = 0; i < samples; ++i) {
    const uint32_t value = singleMeasurement();
    if (value == 0) {
      return 0;
    }
    total += value;
  }

  const uint32_t average = static_cast<uint32_t>(total / samples);
  if (average == 0) {
    status_ = Status::InvalidReading;
  }
  return average;
}

bool ECSensorV2::calibrate(float referenceECmScm) {
  if (referenceECmScm <= 0.0f) {
    status_ = Status::InvalidReading;
    return false;
  }

  const uint32_t raw = readRaw();
  if (raw == 0) {
    return false;
  }

  calibrationConstant_ = referenceECmScm * static_cast<float>(raw);
  status_ = Status::Ok;
  return true;
}

void ECSensorV2::setCalibrationConstant(float value) {
  calibrationConstant_ = (value < 0.0f) ? 0.0f : value;
}

float ECSensorV2::getCalibrationConstant() const {
  return calibrationConstant_;
}

float ECSensorV2::readEC() {
  const uint32_t raw = readRaw();
  if (raw == 0 || calibrationConstant_ <= 0.0f) {
    if (raw != 0) {
      status_ = Status::InvalidReading;
    }
    return NAN;
  }
  return calibrationConstant_ / static_cast<float>(raw);
}

float ECSensorV2::readEC(float temperatureC) {
  const float ec = readEC();
  if (isnan(ec)) {
    return NAN;
  }

  const float denominator = 1.0f + alpha_ * (temperatureC - 25.0f);
  if (denominator <= 0.0f) {
    status_ = Status::InvalidReading;
    return NAN;
  }
  return ec / denominator;
}

float ECSensorV2::readTDS(float tdsFactor) {
  const float ec = readEC();
  if (isnan(ec)) {
    return NAN;
  }
  // ec is in mS/cm. Typical TDS factors are 500, 640 or 700 ppm per mS/cm.
  return ec * tdsFactor;
}

float ECSensorV2::readTDS(float temperatureC, float tdsFactor) {
  const float ec25 = readEC(temperatureC);
  if (isnan(ec25)) {
    return NAN;
  }
  return ec25 * tdsFactor;
}

void ECSensorV2::setTemperatureCoefficient(float alphaPerC) {
  if (alphaPerC >= 0.0f) {
    alpha_ = alphaPerC;
  }
}

float ECSensorV2::getTemperatureCoefficient() const {
  return alpha_;
}

ECSensorV2::Status ECSensorV2::getStatus() const {
  return status_;
}

const char* ECSensorV2::getStatusText() const {
  switch (status_) {
    case Status::Ok: return "OK";
    case Status::NotBegun: return "NOT_BEGUN";
    case Status::Timeout: return "TIMEOUT";
    case Status::InvalidReading: return "INVALID_READING";
    case Status::UnsupportedPlatform: return "UNSUPPORTED_PLATFORM";
    default: return "UNKNOWN";
  }
}

bool ECSensorV2::probeConnected() const {
  return status_ != Status::Timeout;
}

void ECSensorV2::disconnectPins() {
#if defined(ARDUINO_ARCH_ESP32)
  pinMode(capPosPin_, INPUT);
  pinMode(capNegPin_, INPUT);
  pinMode(ecPin_, INPUT);
#endif
}

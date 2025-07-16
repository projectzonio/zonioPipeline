// ====================================================================
// ZonioPipeline Library v1.0.0 - Implementation
// Adaptivní Pipeline systém pro ESP32 IoT zařízení
// 
// Author: Lukáš Machka for project Z.O.N.I.O.
// License: BSD
// ====================================================================

#include "ZonioPipeline.h"

// ===== MQTT HELPER KONSTANTY =====
const char* MqttHelper::ShortKeys::STATUS = "st";
const char* MqttHelper::ShortKeys::IP = "ip";
const char* MqttHelper::ShortKeys::RSSI = "rs";
const char* MqttHelper::ShortKeys::UPTIME = "up";
const char* MqttHelper::ShortKeys::DEVICE = "dev";
const char* MqttHelper::ShortKeys::FIRMWARE = "fw";
const char* MqttHelper::ShortKeys::TEMP = "t";
const char* MqttHelper::ShortKeys::HUMIDITY = "h";
const char* MqttHelper::ShortKeys::PRESSURE = "p";
const char* MqttHelper::ShortKeys::LUX = "l";
const char* MqttHelper::ShortKeys::CPU_FREQ = "cf";
const char* MqttHelper::ShortKeys::PIPE_STATE = "ps";
const char* MqttHelper::ShortKeys::STABLE = "stb";
const char* MqttHelper::ShortKeys::FREE_HEAP = "fH";
const char* MqttHelper::ShortKeys::INTERVAL = "int";

// ===== SENSOR HISTORY IMPLEMENTACE =====
SensorHistory::SensorHistory(int historySize, const char* name) 
  : size(historySize), index(0), filled(false), sensorName(name) {
  values = new float[size];
  timestamps = new unsigned long[size];
  init();
}

SensorHistory::~SensorHistory() {
  delete[] values;
  delete[] timestamps;
}

void SensorHistory::init() {
  index = 0;
  filled = false;
  for (int i = 0; i < size; i++) {
    values[i] = 0;
    timestamps[i] = 0;
  }
}

void SensorHistory::add(float value, unsigned long timestamp) {
  if (timestamp == 0) timestamp = millis();
  
  values[index] = value;
  timestamps[index] = timestamp;
  index = (index + 1) % size;
  if (index == 0) filled = true;
}

float SensorHistory::getChangeRate(int sampleCount) {
  if (index == 0 && !filled) return 0;
  
  int count = filled ? size : index;
  count = min(count, sampleCount);
  
  if (count < 2) return 0;
  
  int startIdx = (index - count + size) % size;
  int endIdx = (index - 1 + size) % size;
  
  float valueChange = fabs(values[endIdx] - values[startIdx]);
  unsigned long timeChange = timestamps[endIdx] - timestamps[startIdx];
  
  if (timeChange == 0) return 0;
  
  return valueChange * 60000 / timeChange; // změna za minutu
}

bool SensorHistory::isStable(float maxChangePerMinute, int sampleCount) {
  float rate = getChangeRate(sampleCount);
  return rate <= maxChangePerMinute;
}

float SensorHistory::getLastValue() {
  if (index == 0 && !filled) return 0;
  int lastIdx = (index - 1 + size) % size;
  return values[lastIdx];
}

float SensorHistory::getAverageValue(int sampleCount) {
  if (index == 0 && !filled) return 0;
  
  int count = filled ? size : index;
  if (sampleCount > 0) count = min(count, sampleCount);
  
  float sum = 0;
  for (int i = 0; i < count; i++) {
    int idx = (index - 1 - i + size) % size;
    sum += values[idx];
  }
  return sum / count;
}

float SensorHistory::getMinValue(int sampleCount) {
  if (index == 0 && !filled) return 0;
  
  int count = filled ? size : index;
  if (sampleCount > 0) count = min(count, sampleCount);
  
  float minVal = values[(index - 1 + size) % size];
  for (int i = 1; i < count; i++) {
    int idx = (index - 1 - i + size) % size;
    if (values[idx] < minVal) minVal = values[idx];
  }
  return minVal;
}

float SensorHistory::getMaxValue(int sampleCount) {
  if (index == 0 && !filled) return 0;
  
  int count = filled ? size : index;
  if (sampleCount > 0) count = min(count, sampleCount);
  
  float maxVal = values[(index - 1 + size) % size];
  for (int i = 1; i < count; i++) {
    int idx = (index - 1 - i + size) % size;
    if (values[idx] > maxVal) maxVal = values[idx];
  }
  return maxVal;
}

bool SensorHistory::hasEnoughData(int minSamples) {
  int count = filled ? size : index;
  return count >= minSamples;
}

void SensorHistory::printHistory() const {
  if (Serial) {
    Serial.printf("Sensor %s history:\n", sensorName);
    int count = filled ? size : index;
    for (int i = 0; i < count; i++) {
      int idx = (index - count + i + size) % size;
      Serial.printf("  [%d] %.2f at %lu\n", i, values[idx], timestamps[idx]);
    }
  }
}

// ===== ZONIO PIPELINE IMPLEMENTACE =====
ZonioPipeline::ZonioPipeline(int numSensors) 
  : numSensors(numSensors), currentState(STATE_REACTIVE), lastState(STATE_REACTIVE),
    stabilityDetected(false), initialized(false), currentCpuFreq(160), lastCpuFreq(160) {
  
  sensorHistories.reserve(numSensors);
  stabilityThresholds.reserve(numSensors);
  sensorNames.reserve(numSensors);
  
  for (int i = 0; i < numSensors; i++) {
    sensorHistories.push_back(new SensorHistory(config.historySize, ("Sensor" + String(i)).c_str()));
    stabilityThresholds.push_back(1.0); // default threshold
    sensorNames.push_back(("Sensor" + String(i)).c_str());
  }
}

ZonioPipeline::~ZonioPipeline() {
  for (auto* history : sensorHistories) {
    delete history;
  }
}

void ZonioPipeline::begin() {
  stateStartTime = millis();
  stabilityStartTime = millis();
  currentInterval = config.intervalReactive;
  
  // Inicializace CPU na max frekvenci
  setCpuFrequency(config.maxFreq);
  
  initialized = true;
  debugPrint("ZonioPipeline initialized");
}

void ZonioPipeline::setConfig(const PipelineConfig& newConfig) {
  config = newConfig;
  
  // Update historie sizes pokud se změnila
  if (initialized) {
    for (auto* history : sensorHistories) {
      delete history;
    }
    sensorHistories.clear();
    
    for (int i = 0; i < numSensors; i++) {
      sensorHistories.push_back(new SensorHistory(config.historySize, sensorNames[i]));
    }
  }
}

PipelineConfig ZonioPipeline::getConfig() const {
  return config;
}

void ZonioPipeline::setSensorName(int index, const char* name) {
  if (index >= 0 && index < numSensors) {
    sensorNames[index] = name;
    if (index < sensorHistories.size()) {
      // Update historie name - would need to add setter in SensorHistory
      // For now, just store the name
    }
  }
}

void ZonioPipeline::setStabilityThreshold(int index, float threshold) {
  if (index >= 0 && index < numSensors) {
    if (index >= stabilityThresholds.size()) {
      stabilityThresholds.resize(index + 1, 1.0);
    }
    stabilityThresholds[index] = threshold;
  }
}

// ===== CALLBACK REGISTRACE =====
void ZonioPipeline::onStateChanged(StateChangeCallback callback) {
  onStateChange = callback;
}

void ZonioPipeline::onCpuChanged(CpuChangeCallback callback) {
  onCpuChange = callback;
}

void ZonioPipeline::onIntervalChanged(IntervalChangeCallback callback) {
  onIntervalChange = callback;
}

void ZonioPipeline::onStabilityChanged(StabilityCallback callback) {
  onStabilityChange = callback;
}

void ZonioPipeline::onSensorUpdated(SensorUpdateCallback callback) {
  onSensorUpdate = callback;
}

// ===== CPU MANAGEMENT =====
void ZonioPipeline::setCpuFrequency(uint8_t freq) {
  if (!config.enableSpeedStep) {
    freq = config.maxFreq;
  }
  
  if (currentCpuFreq != freq) {
    lastCpuFreq = currentCpuFreq;
    debugPrintf("CPU: %d -> %d MHz", currentCpuFreq, freq);
    
    setCpuFrequencyMhz(freq);
    currentCpuFreq = freq;
    lastCpuChange = millis();
    
    // Callback
    if (onCpuChange) {
      onCpuChange(freq, lastCpuFreq);
    }
    
    // Stabilizační delay
    delay(getStabilizationDelay(freq));
  }
}

uint8_t ZonioPipeline::getStabilizationDelay(uint8_t targetFreq) {
  if (targetFreq >= 160) return 50;   // Rychlé CPU - krátká stabilizace
  if (targetFreq >= 140) return 80;   // Střední CPU
  if (targetFreq >= 100) return 120;  // Pomalé CPU - delší stabilizace
  return 100;                         // Default
}

// ===== PIPELINE TRANSITIONS =====
void ZonioPipeline::transitionState(PipelineState newState, const char* reason) {
  if (currentState != newState) {
    lastState = currentState;
    currentState = newState;
    stateStartTime = millis();
    
    debugPrintf("Pipeline: %s -> %s (%s)", getStateFullName(lastState), getStateFullName(newState), reason);
    
    switch (newState) {
      case STATE_REACTIVE:
        setCpuFrequency(config.maxFreq);
        currentInterval = config.intervalReactive;
        break;
      case STATE_SHORT:
        setCpuFrequency(config.midFreq);
        currentInterval = config.intervalShort;
        break;
      case STATE_MEDIUM:
        setCpuFrequency(config.midFreq);
        currentInterval = config.intervalMedium;
        break;
      case STATE_LONG:
        setCpuFrequency(config.minFreq);
        currentInterval = config.intervalLong;
        break;
    }
    
    // Reset stability při změně stavu
    stabilityDetected = false;
    
    // Callbacks
    if (onStateChange) {
      onStateChange(newState, lastState, reason);
    }
    if (onIntervalChange) {
      onIntervalChange(newState, currentInterval);
    }
  }
}

// ===== STABILITY DETECTION =====
bool ZonioPipeline::checkStability() {
  if (sensorHistories.empty()) return false;
  
  bool allStable = true;
  
  for (int i = 0; i < sensorHistories.size(); i++) {
    float threshold = (i < stabilityThresholds.size()) ? stabilityThresholds[i] : 1.0;
    if (!sensorHistories[i]->isStable(threshold, config.stabilitySamples)) {
      allStable = false;
      break;
    }
  }
  
  unsigned long now = millis();
  
  // Detekce začátku/konce stability
  if (allStable && !stabilityDetected) {
    stabilityDetected = true;
    stabilityStartTime = now;
    debugPrint("Stability detected");
    
    if (onStabilityChange) {
      onStabilityChange(true, 0);
    }
  } else if (!allStable && stabilityDetected) {
    stabilityDetected = false;
    debugPrint("Stability lost -> REACTIVE");
    
    transitionState(STATE_REACTIVE, "Stability lost");
    
    if (onStabilityChange) {
      onStabilityChange(false, now - stabilityStartTime);
    }
    
    return false;
  }
  
  return stabilityDetected;
}

// ===== SENSOR MANAGEMENT =====
void ZonioPipeline::updateSensor(int sensorIndex, float value, unsigned long timestamp) {
  if (sensorIndex >= 0 && sensorIndex < sensorHistories.size()) {
    sensorHistories[sensorIndex]->add(value, timestamp);
    
    if (onSensorUpdate) {
      float changeRate = sensorHistories[sensorIndex]->getChangeRate(config.stabilitySamples);
      onSensorUpdate(sensorIndex, value, changeRate);
    }
  }
}

float ZonioPipeline::getSensorValue(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < sensorHistories.size()) {
    return sensorHistories[sensorIndex]->getLastValue();
  }
  return 0.0;
}

float ZonioPipeline::getSensorChangeRate(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < sensorHistories.size()) {
    return sensorHistories[sensorIndex]->getChangeRate(config.stabilitySamples);
  }
  return 0.0;
}

bool ZonioPipeline::isSensorStable(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < sensorHistories.size()) {
    float threshold = (sensorIndex < stabilityThresholds.size()) ? stabilityThresholds[sensorIndex] : 1.0;
    return sensorHistories[sensorIndex]->isStable(threshold, config.stabilitySamples);
  }
  return false;
}

// ===== PIPELINE CONTROL =====
void ZonioPipeline::update() {
  if (!initialized) return;
  
  unsigned long now = millis();
  
  // Kontrola stability
  bool isStable = checkStability();
  
  if (isStable) {
    unsigned long stabilityDuration = now - stabilityStartTime;
    
    switch (currentState) {
      case STATE_REACTIVE:
        if (stabilityDuration >= config.stabilityShort) {
          transitionState(STATE_SHORT, "1 min stability");
        }
        break;
      case STATE_SHORT:
        if (stabilityDuration >= config.stabilityMedium) {
          transitionState(STATE_MEDIUM, "5 min stability");
        }
        break;
      case STATE_MEDIUM:
        if (stabilityDuration >= config.stabilityLong) {
          transitionState(STATE_LONG, "10 min stability");
        }
        break;
      case STATE_LONG:
        // Zůstáváme v LONG
        break;
    }
  } else {
    // Bez stability -> REACTIVE
    if (currentState != STATE_REACTIVE) {
      transitionState(STATE_REACTIVE, "No stability");
    }
  }
}

void ZonioPipeline::forceState(PipelineState state, const char* reason) {
  transitionState(state, reason);
}

void ZonioPipeline::resetStability() {
  stabilityDetected = false;
  stabilityStartTime = millis();
}

void ZonioPipeline::reset() {
  currentState = STATE_REACTIVE;
  lastState = STATE_REACTIVE;
  stabilityDetected = false;
  stateStartTime = millis();
  stabilityStartTime = millis();
  currentInterval = config.intervalReactive;
  setCpuFrequency(config.maxFreq);
}

// ===== GETTERS =====
unsigned long ZonioPipeline::getStabilityDuration() const {
  if (stabilityDetected) {
    return millis() - stabilityStartTime;
  }
  return 0;
}

unsigned long ZonioPipeline::getTimeInState() const {
  return millis() - stateStartTime;
}

// ===== UTILITY METHODS =====
const char* ZonioPipeline::getStateShortName(PipelineState state) {
  switch (state) {
    case STATE_REACTIVE: return "R";
    case STATE_SHORT:    return "S";
    case STATE_MEDIUM:   return "M";
    case STATE_LONG:     return "L";
    default: return "?";
  }
}

const char* ZonioPipeline::getStateFullName(PipelineState state) {
  switch (state) {
    case STATE_REACTIVE: return "REACTIVE";
    case STATE_SHORT:    return "SHORT";
    case STATE_MEDIUM:   return "MEDIUM";
    case STATE_LONG:     return "LONG";
    default: return "UNKNOWN";
  }
}

void ZonioPipeline::printDebugInfo() const {
  if (!config.enableSerialOutput || !Serial) return;
  
  Serial.println("=== ZonioPipeline Debug Info ===");
  Serial.printf("State: %s (%s)\n", getStateFullName(currentState), getStateShortName(currentState));
  Serial.printf("CPU: %d MHz | Interval: %lu ms\n", currentCpuFreq, currentInterval);
  Serial.printf("Stability: %s", stabilityDetected ? "DETECTED" : "NO");
  
  if (stabilityDetected) {
    Serial.printf(" (%lu sec)", getStabilityDuration() / 1000);
  }
  Serial.println();
  
  Serial.printf("Time in state: %lu sec\n", getTimeInState() / 1000);
  Serial.printf("Sensors: %d\n", numSensors);
  
  for (int i = 0; i < sensorHistories.size(); i++) {
    float value = sensorHistories[i]->getLastValue();
    float rate = sensorHistories[i]->getChangeRate(config.stabilitySamples);
    bool stable = isSensorStable(i);
    Serial.printf("  Sensor %d: %.2f (%.2f/min) %s\n", 
                  i, value, rate, stable ? "[STABLE]" : "[CHANGING]");
  }
  Serial.println("===============================");
}

void ZonioPipeline::printSensorStatus() const {
  if (!config.enableSerialOutput || !Serial) return;
  
  Serial.println("=== Sensor Status ===");
  for (int i = 0; i < sensorHistories.size(); i++) {
    Serial.printf("Sensor %d (%s):\n", i, sensorNames[i]);
    Serial.printf("  Value: %.2f\n", sensorHistories[i]->getLastValue());
    Serial.printf("  Change rate: %.2f/min\n", sensorHistories[i]->getChangeRate(config.stabilitySamples));
    Serial.printf("  Stable: %s\n", isSensorStable(i) ? "YES" : "NO");
    Serial.printf("  Avg: %.2f | Min: %.2f | Max: %.2f\n", 
                  sensorHistories[i]->getAverageValue(),
                  sensorHistories[i]->getMinValue(),
                  sensorHistories[i]->getMaxValue());
  }
  Serial.println("====================");
}

String ZonioPipeline::getStatusJson() const {
  String json = "{";
  json += "\"state\":\"" + String(getStateShortName(currentState)) + "\",";
  json += "\"cpuFreq\":" + String(currentCpuFreq) + ",";
  json += "\"interval\":" + String(currentInterval) + ",";
  json += "\"stable\":" + String(stabilityDetected ? "true" : "false") + ",";
  json += "\"stabilityDuration\":" + String(getStabilityDuration()) + ",";
  json += "\"timeInState\":" + String(getTimeInState()) + ",";
  json += "\"sensors\":[";
  
  for (int i = 0; i < sensorHistories.size(); i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"value\":" + String(sensorHistories[i]->getLastValue(), 2) + ",";
    json += "\"changeRate\":" + String(sensorHistories[i]->getChangeRate(config.stabilitySamples), 2) + ",";
    json += "\"stable\":" + String(isSensorStable(i) ? "true" : "false");
    json += "}";
  }
  
  json += "]}";
  return json;
}

// ===== DEBUG HELPERS =====
void ZonioPipeline::debugPrint(const char* message) {
  if (config.enableDebug && config.enableSerialOutput && Serial) {
    Serial.print("[ZonioPipeline] ");
    Serial.println(message);
  }
}

void ZonioPipeline::debugPrintf(const char* format, ...) {
  if (config.enableDebug && config.enableSerialOutput && Serial) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.print("[ZonioPipeline] ");
    Serial.println(buffer);
  }
}

// ===== I2C ADAPTER IMPLEMENTACE =====
I2CAdapter::I2CAdapter(bool enableAdaptiveDelays) 
  : adaptiveDelays(enableAdaptiveDelays), currentCpuFreq(160), initialized(false) {
}

void I2CAdapter::begin() {
  initialized = true;
}

void I2CAdapter::setCpuFreq(uint8_t freq) {
  currentCpuFreq = freq;
}

bool I2CAdapter::switchBus(uint8_t sda, uint8_t scl) {
  Wire.end();
  delay(getSwitchDelay());
  Wire.begin(sda, scl);
  delay(getInitDelay());
  return true;
}

void I2CAdapter::endBus() {
  Wire.end();
}

uint8_t I2CAdapter::getSwitchDelay() const {
  if (!adaptiveDelays) return 5;
  
  switch (currentCpuFreq) {
    case 160: return 5;
    case 140: return 8;
    case 100: return 15;
    default: return 10;
  }
}

uint8_t I2CAdapter::getInitDelay() const {
  if (!adaptiveDelays) return 10;
  
  switch (currentCpuFreq) {
    case 160: return 10;
    case 140: return 15;
    case 100: return 25;
    default: return 15;
  }
}

uint8_t I2CAdapter::getSensorDelay() const {
  if (!adaptiveDelays) return 50;
  
  switch (currentCpuFreq) {
    case 160: return 50;
    case 140: return 70;
    case 100: return 100;
    default: return 70;
  }
}

uint8_t I2CAdapter::getVemlDelay() const {
  if (!adaptiveDelays) return 120;
  
  switch (currentCpuFreq) {
    case 160: return 120;
    case 140: return 160;
    case 100: return 220;
    default: return 160;
  }
}

// ===== MQTT HELPER IMPLEMENTACE =====
String MqttHelper::buildWeatherPayload(float temp, float humidity, float pressure, float lux,
                                       uint8_t cpuFreq, PipelineState state, bool stable) {
  String payload = "{";
  payload += "\"" + String(ShortKeys::TEMP) + "\":" + String(temp, 1) + ",";
  payload += "\"" + String(ShortKeys::HUMIDITY) + "\":" + String(humidity, 0) + ",";
  payload += "\"" + String(ShortKeys::PRESSURE) + "\":" + String(pressure, 0) + ",";
  payload += "\"" + String(ShortKeys::LUX) + "\":" + String(lux, 0) + ",";
  payload += "\"" + String(ShortKeys::CPU_FREQ) + "\":" + String(cpuFreq) + ",";
  payload += "\"" + String(ShortKeys::PIPE_STATE) + "\":\"" + String(ZonioPipeline::getStateShortName(state)) + "\",";
  payload += "\"" + String(ShortKeys::STABLE) + "\":" + String(stable ? "true" : "false");
  payload += "}";
  return payload;
}

String MqttHelper::buildStatusPayload(const char* status, const char* ip, int rssi,
                                     unsigned long uptime, const char* device, const char* firmware) {
  String payload = "{";
  payload += "\"" + String(ShortKeys::STATUS) + "\":\"" + String(status) + "\",";
  payload += "\"" + String(ShortKeys::IP) + "\":\"" + String(ip) + "\",";
  payload += "\"" + String(ShortKeys::RSSI) + "\":" + String(rssi) + ",";
  payload += "\"" + String(ShortKeys::UPTIME) + "\":" + String(uptime) + ",";
  payload += "\"" + String(ShortKeys::DEVICE) + "\":\"" + String(device) + "\",";
  payload += "\"" + String(ShortKeys::FIRMWARE) + "\":\"" + String(firmware) + "\"";
  payload += "}";
  return payload;
}

String MqttHelper::buildSystemPayload(const char* ip, const char* firmware, unsigned long uptime,
                                     int rssi, int reconnects, uint32_t freeHeap,
                                     uint8_t cpuFreq, PipelineState state, unsigned long interval,
                                     bool stability, const std::vector<bool>& sensorStatus) {
  String payload = "{";
  payload += "\"" + String(ShortKeys::IP) + "\":\"" + String(ip) + "\",";
  payload += "\"" + String(ShortKeys::FIRMWARE) + "\":\"" + String(firmware) + "\",";
  payload += "\"" + String(ShortKeys::UPTIME) + "\":" + String(uptime) + ",";
  payload += "\"" + String(ShortKeys::RSSI) + "\":" + String(rssi) + ",";
  payload += "\"reconnects\":" + String(reconnects) + ",";
  payload += "\"" + String(ShortKeys::FREE_HEAP) + "\":" + String(freeHeap) + ",";
  payload += "\"" + String(ShortKeys::CPU_FREQ) + "\":" + String(cpuFreq) + ",";
  payload += "\"" + String(ShortKeys::PIPE_STATE) + "\":\"" + String(ZonioPipeline::getStateShortName(state)) + "\",";
  payload += "\"" + String(ShortKeys::INTERVAL) + "\":" + String(interval) + ",";
  payload += "\"" + String(ShortKeys::STABLE) + "\":" + String(stability ? "true" : "false") + ",";
  payload += "\"sensors\":[";
  
  for (int i = 0; i < sensorStatus.size(); i++) {
    if (i > 0) payload += ",";
    payload += String(sensorStatus[i] ? "true" : "false");
  }
  
  payload += "]}";
  return payload;
}

String MqttHelper::buildSensorPayload(const ZonioPipeline& pipeline) {
  return pipeline.getStatusJson();
}

String MqttHelper::escapeJson(const String& input) {
  String output = input;
  output.replace("\\", "\\\\");
  output.replace("\"", "\\\"");
  output.replace("\n", "\\n");
  output.replace("\r", "\\r");
  output.replace("\t", "\\t");
  return output;
}

// ===== TEMPLATE IMPLEMENTACE =====
// Note: Template implementations are typically in header files
// but for demonstration, key methods are shown here

// Templates would be implemented inline in the header file or in a separate .tpp file
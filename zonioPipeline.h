// ====================================================================
// ZonioPipeline Library v1.0.0
// Adaptivní Pipeline systém pro ESP32 IoT zařízení
// Opensource knihovna pro inteligentní řízení výkonu založené na stabilitě dat
// 
// Author: Lukáš Machka for project Z.O.N.I.O.
// License: BSD
// ====================================================================

#ifndef ZONIO_PIPELINE_H
#define ZONIO_PIPELINE_H

#include <Arduino.h>
#include <functional>
#include <vector>
#include <Wire.h>

// ===== VERSION INFO =====
#define ZONIO_PIPELINE_VERSION "1.0.0"
#define ZONIO_PIPELINE_VERSION_MAJOR 1
#define ZONIO_PIPELINE_VERSION_MINOR 0
#define ZONIO_PIPELINE_VERSION_PATCH 0

// ===== FORWARD DECLARATIONS =====
class ZonioPipeline;
class SensorHistory;
class I2CAdapter;
class MqttHelper;

// ===== PIPELINE STATES =====
enum PipelineState {
  STATE_REACTIVE = 0,   // CPU MAX + nejkratší interval
  STATE_SHORT = 1,      // CPU MID + krátký interval
  STATE_MEDIUM = 2,     // CPU MID + střední interval  
  STATE_LONG = 3        // CPU MIN + dlouhý interval
};

// ===== KONFIGURACE STRUKTURY =====
struct PipelineConfig {
  // CPU frekvence (MHz)
  uint8_t maxFreq = 160;        
  uint8_t midFreq = 140;        
  uint8_t minFreq = 100;        
  bool enableSpeedStep = true;  
  
  // Vzorkovací intervaly (ms)
  unsigned long intervalReactive = 2000;   
  unsigned long intervalShort = 5000;      
  unsigned long intervalMedium = 10000;    
  unsigned long intervalLong = 30000;      
  
  // Stability timeouts (ms)
  unsigned long stabilityShort = 60000;    
  unsigned long stabilityMedium = 300000;  
  unsigned long stabilityLong = 600000;    
  
  // Stability thresholds (change per minute)
  std::vector<float> stabilityThresholds = {0.5, 2.0, 100.0};
  
  // Analýza parametry
  int stabilitySamples = 8;     
  int historySize = 10;         
  
  // Debug
  bool enableDebug = false;
  bool enableSerialOutput = true;
};

// ===== SENSOR HISTORY CLASS =====
class SensorHistory {
private:
  float* values;
  unsigned long* timestamps;
  int size;
  int index;
  bool filled;
  const char* sensorName;

public:
  SensorHistory(int historySize, const char* name = "Unknown");
  ~SensorHistory();
  
  void init();
  void add(float value, unsigned long timestamp = 0);
  float getChangeRate(int sampleCount);
  bool isStable(float maxChangePerMinute, int sampleCount);
  float getLastValue();
  float getAverageValue(int sampleCount = 0);
  float getMinValue(int sampleCount = 0);
  float getMaxValue(int sampleCount = 0);
  bool hasEnoughData(int minSamples);
  void printHistory() const;
  const char* getName() const { return sensorName; }
};

// ===== CALLBACK TYPY =====
typedef std::function<void(PipelineState newState, PipelineState oldState, const char* reason)> StateChangeCallback;
typedef std::function<void(uint8_t newFreq, uint8_t oldFreq)> CpuChangeCallback;
typedef std::function<void(PipelineState state, unsigned long interval)> IntervalChangeCallback;
typedef std::function<void(bool detected, unsigned long duration)> StabilityCallback;
typedef std::function<void(int sensorIndex, float value, float changeRate)> SensorUpdateCallback;

// ===== HLAVNÍ PIPELINE TŘÍDA =====
class ZonioPipeline {
private:
  PipelineConfig config;
  PipelineState currentState;
  PipelineState lastState;
  
  unsigned long stateStartTime;
  unsigned long lastCpuChange;
  unsigned long stabilityStartTime;
  unsigned long currentInterval;
  uint8_t currentCpuFreq;
  uint8_t lastCpuFreq;
  
  bool stabilityDetected;
  bool initialized;
  
  // Historie pro každý sensor
  std::vector<SensorHistory*> sensorHistories;
  std::vector<float> stabilityThresholds;
  std::vector<const char*> sensorNames;
  int numSensors;
  
  // Callbacks
  StateChangeCallback onStateChange;
  CpuChangeCallback onCpuChange;
  IntervalChangeCallback onIntervalChange;
  StabilityCallback onStabilityChange;
  SensorUpdateCallback onSensorUpdate;
  
  // Privátní metody
  void setCpuFrequency(uint8_t freq);
  bool checkStability();
  void transitionState(PipelineState newState, const char* reason);
  uint8_t getStabilizationDelay(uint8_t targetFreq);
  void debugPrint(const char* message);
  void debugPrintf(const char* format, ...);

public:
  ZonioPipeline(int numSensors = 3);
  ~ZonioPipeline();
  
  // Konfigurace
  void setConfig(const PipelineConfig& newConfig);
  PipelineConfig getConfig() const;
  void setSensorName(int index, const char* name);
  void setStabilityThreshold(int index, float threshold);
  
  // Callback registrace
  void onStateChanged(StateChangeCallback callback);
  void onCpuChanged(CpuChangeCallback callback);
  void onIntervalChanged(IntervalChangeCallback callback);
  void onStabilityChanged(StabilityCallback callback);
  void onSensorUpdated(SensorUpdateCallback callback);
  
  // Sensor management
  void updateSensor(int sensorIndex, float value, unsigned long timestamp = 0);
  float getSensorValue(int sensorIndex);
  float getSensorChangeRate(int sensorIndex);
  bool isSensorStable(int sensorIndex) const;
  
  // Pipeline control
  void begin();
  void update();
  void forceState(PipelineState state, const char* reason = "Manual");
  void resetStability();
  void reset();
  
  // Getters
  PipelineState getCurrentState() const { return currentState; }
  PipelineState getLastState() const { return lastState; }
  unsigned long getCurrentInterval() const { return currentInterval; }
  uint8_t getCurrentCpuFreq() const { return currentCpuFreq; }
  uint8_t getLastCpuFreq() const { return lastCpuFreq; }
  bool isStabilityDetected() const { return stabilityDetected; }
  unsigned long getStabilityDuration() const;
  unsigned long getTimeInState() const;
  int getNumSensors() const { return numSensors; }
  
  // Utility
  static const char* getStateShortName(PipelineState state);
  static const char* getStateFullName(PipelineState state);
  void printDebugInfo() const;
  void printSensorStatus() const;
  String getStatusJson() const;
};

// ===== I2C ADAPTER PRO ADAPTIVE DELAYS =====
class I2CAdapter {
private:
  uint8_t currentCpuFreq;
  bool adaptiveDelays;
  bool initialized;

public:
  I2CAdapter(bool enableAdaptiveDelays = true);
  
  void begin();
  void setCpuFreq(uint8_t freq);
  bool switchBus(uint8_t sda, uint8_t scl);
  void endBus();
  
  // Adaptivní delays podle CPU frekvence
  uint8_t getSwitchDelay() const;
  uint8_t getInitDelay() const;
  uint8_t getSensorDelay() const;
  uint8_t getVemlDelay() const;
  
  // Utility
  uint8_t getCurrentCpuFreq() const { return currentCpuFreq; }
  bool isAdaptiveEnabled() const { return adaptiveDelays; }
};

// ===== MQTT HELPER PRO OPTIMALIZOVANÉ PAYLOADY =====
class MqttHelper {
public:
  // Zkrácené klíče pro MQTT payloady
  struct ShortKeys {
    static const char* STATUS;      // "st"
    static const char* IP;          // "ip"
    static const char* RSSI;        // "rs"
    static const char* UPTIME;      // "up"
    static const char* DEVICE;      // "dev"
    static const char* FIRMWARE;    // "fw"
    static const char* TEMP;        // "t"
    static const char* HUMIDITY;    // "h"
    static const char* PRESSURE;    // "p"
    static const char* LUX;         // "l"
    static const char* CPU_FREQ;    // "cf"
    static const char* PIPE_STATE;  // "ps"
    static const char* STABLE;      // "stb"
    static const char* FREE_HEAP;   // "fH"
    static const char* INTERVAL;    // "int"
  };
  
  // Payload builders
  static String buildWeatherPayload(float temp, float humidity, float pressure, float lux,
                                   uint8_t cpuFreq, PipelineState state, bool stable);
  
  static String buildStatusPayload(const char* status, const char* ip, int rssi,
                                  unsigned long uptime, const char* device, const char* firmware);
  
  static String buildSystemPayload(const char* ip, const char* firmware, unsigned long uptime,
                                  int rssi, int reconnects, uint32_t freeHeap,
                                  uint8_t cpuFreq, PipelineState state, unsigned long interval,
                                  bool stability, const std::vector<bool>& sensorStatus);
  
  static String buildSensorPayload(const ZonioPipeline& pipeline);
  
private:
  static String escapeJson(const String& input);
};

// ===== TEMPLATE PRO WEATHER STATION =====
template<typename SensorType1, typename SensorType2 = void*, typename SensorType3 = void*>
class SimpleWeatherStation {
private:
  ZonioPipeline pipeline;
  I2CAdapter i2c;
  
  SensorType1* sensor1;
  SensorType2* sensor2;
  SensorType3* sensor3;
  
  uint8_t sensor1SDA, sensor1SCL;
  uint8_t sensor2SDA, sensor2SCL;
  uint8_t sensor3SDA, sensor3SCL;
  uint8_t displaySDA, displaySCL;
  
  std::function<void()> readSensorsCallback;
  std::function<void()> updateDisplayCallback;
  std::function<void(String)> mqttPublishCallback;
  
  bool sensorsInitialized;
  bool displayInitialized;

public:
  SimpleWeatherStation() : pipeline(3), i2c(true), sensorsInitialized(false), displayInitialized(false) {
    sensor1 = nullptr;
    sensor2 = nullptr;
    sensor3 = nullptr;
  }
  
  // Sensor konfigurace
  void setSensor1(SensorType1* s, uint8_t sda, uint8_t scl, const char* name = "Sensor1");
  void setSensor2(SensorType2* s, uint8_t sda, uint8_t scl, const char* name = "Sensor2");
  void setSensor3(SensorType3* s, uint8_t sda, uint8_t scl, const char* name = "Sensor3");
  void setDisplay(uint8_t sda, uint8_t scl);
  
  // Callback konfigurace
  void onReadSensors(std::function<void()> callback);
  void onUpdateDisplay(std::function<void()> callback);
  void onMqttPublish(std::function<void(String)> callback);
  
  // Lifecycle
  void begin(const PipelineConfig& config = PipelineConfig());
  void loop();
  void end();
  
  // Getters
  ZonioPipeline& getPipeline() { return pipeline; }
  I2CAdapter& getI2C() { return i2c; }
  
  // Utility
  bool areSensorsInitialized() const { return sensorsInitialized; }
  bool isDisplayInitialized() const { return displayInitialized; }
};

#endif // ZONIO_PIPELINE_H

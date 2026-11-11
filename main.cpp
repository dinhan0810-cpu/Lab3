
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <stdarg.h>

// ==================================================
// 1. KHAI BAO CHAN
// ==================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define LED_PIN 18
#define BUZZER_PIN 19

// ==================================================
// 2. WIFI
// ==================================================

const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ==================================================
// 3. MQTT
// ==================================================

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

const char* topicData =
  "falldetection/esp32/an/data";

const char* topicAlert =
  "falldetection/esp32/an/alert";

const char* topicLED =
  "falldetection/esp32/an/control/led";

const char* topicBuzzer =
  "falldetection/esp32/an/control/buzzer";

// ==================================================
// 4. DOI TUONG
// ==================================================

Adafruit_MPU6050 mpu;

WiFiClient espClient;
PubSubClient client(espClient);

// ==================================================
// 5. CAU TRUC DU LIEU
// ==================================================

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;

  float pitch;
  float roll;
  float totalG;

  bool fall;
};

struct AlertData {
  float totalG;
  float pitch;
  float roll;
};

// ==================================================
// 6. QUEUE VA SEMAPHORE
// ==================================================

QueueHandle_t dataQueue;
QueueHandle_t dataAlertQueue;

SemaphoreHandle_t serialMutex;

// Bao ve cac bien dung chung
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

// Trang thai dieu khien tu Web
volatile bool ledManual = false;
volatile bool buzzerManual = false;

// Co bao hieu su kien te nga cho Task 2
volatile bool mqttFallPending = false;

// ==================================================
// 7. TASK HANDLE
// ==================================================

TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;

// ==================================================
// 8. HAM IN SERIAL CO MUTEX
// ==================================================

void logPrint(const char* format, ...) {

  char buffer[300];

  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (serialMutex != NULL) {

    if (xSemaphoreTake(
          serialMutex,
          pdMS_TO_TICKS(1000)
        ) == pdTRUE) {

      Serial.println(buffer);

      xSemaphoreGive(serialMutex);
    }
  }
}

// ==================================================
// 9. MQTT CALLBACK
// TASK 2 TIEP NHAN LENH WEB
// ==================================================

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
) {

  String message = "";

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  message.trim();

  if (String(topic) == topicLED) {

    bool value =
      (message == "1" || message == "ON");

    portENTER_CRITICAL(&controlMux);
    ledManual = value;
    portEXIT_CRITICAL(&controlMux);

    logPrint("[TASK 2] Web dieu khien LED: %s",
             value ? "ON" : "OFF");
  }

  else if (String(topic) == topicBuzzer) {

    bool value =
      (message == "1" || message == "ON");

    portENTER_CRITICAL(&controlMux);
    buzzerManual = value;
    portEXIT_CRITICAL(&controlMux);

    logPrint("[TASK 2] Web dieu khien BUZZER: %s",
             value ? "ON" : "OFF");
  }
}

// ==================================================
// 10. TASK 1 - DOC DU LIEU MPU6050
// CORE 1
// ==================================================

void Task1_ReadData(void *pvParameters) {

  logPrint("========== TASK 1 BAT DAU ==========");
  logPrint("[TASK 1] CORE 1 - DOC MPU6050");

  SensorData data;

  bool impactDetected = false;

  unsigned long impactTime = 0;
  unsigned long lastFallTime = 0;
  unsigned long lastPrint = 0;

  for (;;) {

    // DOC CAM BIEN
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);

    // GIA TOC 3 TRUC
    data.ax = accel.acceleration.x;
    data.ay = accel.acceleration.y;
    data.az = accel.acceleration.z;

    // VAN TOC GOC
    data.gx = gyro.gyro.x;
    data.gy = gyro.gyro.y;
    data.gz = gyro.gyro.z;

    // TONG GIA TOC (G)
    data.totalG = sqrt(
      data.ax * data.ax +
      data.ay * data.ay +
      data.az * data.az
    ) / 9.81;

    // TINH GOC PITCH
    data.pitch = atan2(
      -data.ax,
      sqrt(data.ay * data.ay +
           data.az * data.az)
    ) * 180.0 / PI;

    // TINH GOC ROLL
    data.roll = atan2(
      data.ay,
      data.az
    ) * 180.0 / PI;

    unsigned long now = millis();

    data.fall = false;

    // PHAT HIEN VA CHAM
    if (data.totalG > 2.5 && !impactDetected) {

      impactDetected = true;
      impactTime = now;

      logPrint("[TASK 1] Phat hien va cham!");
    }

    // KIEM TRA GOC NGHIENG
    bool angleAbnormal =
      (fabs(data.pitch) > 55 ||
       fabs(data.roll) > 55);

    // XAC DINH TE NGA
    if (impactDetected &&
        angleAbnormal &&
        now - impactTime <= 2000 &&
        now - lastFallTime >= 10000) {

      data.fall = true;

      lastFallTime = now;
      impactDetected = false;

      logPrint("[TASK 1] PHAT HIEN TE NGA!");

      // GUI DU LIEU VAO DATA ALERT QUEUE
      AlertData alert;

      alert.totalG = data.totalG;
      alert.pitch = data.pitch;
      alert.roll = data.roll;

      if (xQueueSend(
            dataAlertQueue,
            &alert,
            0
          ) == pdPASS) {

        logPrint("[TASK 1] Da gui Data Alert Queue!");
      }

      // Bao hieu cho Task 2 gui MQTT alert
      portENTER_CRITICAL(&controlMux);
      mqttFallPending = true;
      portEXIT_CRITICAL(&controlMux);
    }

    // HUY VA CHAM NEU QUA 2 GIAY
    if (impactDetected &&
        now - impactTime > 2000) {

      impactDetected = false;
    }

    // GUI DATA QUEUE
    // Queue co kich thuoc 1, luu mau moi nhat
    xQueueOverwrite(dataQueue, &data);

    // IN SERIAL MOI 2 GIAY
    if (now - lastPrint >= 2000) {

      lastPrint = now;

      logPrint(
        "[TASK 1] AX: %.2f | AY: %.2f | AZ: %.2f | G: %.2f",
        data.ax,
        data.ay,
        data.az,
        data.totalG
      );

      logPrint(
        "[TASK 1] Pitch: %.2f | Roll: %.2f",
        data.pitch,
        data.roll
      );
    }

    // DOC CAM BIEN MOI 50MS
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==================================================
// 11. TASK 2 - TRUYEN DATA MQTT
// CORE 0
// ==================================================

void Task2_SendData(void *pvParameters) {

  logPrint("========== TASK 2 BAT DAU ==========");
  logPrint("[TASK 2] CORE 0 - WIFI + MQTT");

  // KET NOI WIFI
  WiFi.begin(ssid, password);

  logPrint("[TASK 2] Dang ket noi WiFi...");

  while (WiFi.status() != WL_CONNECTED) {

    vTaskDelay(pdMS_TO_TICKS(500));
  }

  logPrint("[TASK 2] WiFi ket noi thanh cong!");
  logPrint("[TASK 2] IP: %s",
           WiFi.localIP().toString().c_str());

  // CAU HINH MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // THOI GIAN CHO MQTT
  client.setKeepAlive(30);
  client.setSocketTimeout(3);

  SensorData latestData;

  bool hasData = false;

  unsigned long lastPublish = 0;

  for (;;) {

    // KIEM TRA WIFI
    if (WiFi.status() != WL_CONNECTED) {

      logPrint("[TASK 2] Mat WiFi, dang ket noi lai!");

      WiFi.reconnect();

      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // KET NOI MQTT
    if (!client.connected()) {

      logPrint("[TASK 2] Dang ket noi MQTT...");

      String clientID =
        "ESP32_Fall_" +
        String((uint32_t)ESP.getEfuseMac(), HEX);

      if (client.connect(clientID.c_str())) {

        logPrint("[TASK 2] MQTT ket noi thanh cong!");

        client.subscribe(topicLED);
        client.subscribe(topicBuzzer);

        logPrint("[TASK 2] Da dang ky topic dieu khien!");

      } else {

        logPrint("[TASK 2] MQTT loi: %d",
                 client.state());

        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    // XU LY MQTT CALLBACK
    client.loop();

    // NHAN DU LIEU TU DATA QUEUE
    if (xQueueReceive(
          dataQueue,
          &latestData,
          pdMS_TO_TICKS(20)
        ) == pdPASS) {

      hasData = true;
    }

    // =========================================
    // GUI DU LIEU CAM BIEN MOI 1 GIAY
    // =========================================

    if (hasData &&
        millis() - lastPublish >= 1000) {

      lastPublish = millis();

      char json[300];

      snprintf(
        json,
        sizeof(json),

        "{\"ax\":%.2f,"
        "\"ay\":%.2f,"
        "\"az\":%.2f,"
        "\"gx\":%.3f,"
        "\"gy\":%.3f,"
        "\"gz\":%.3f,"
        "\"pitch\":%.2f,"
        "\"roll\":%.2f,"
        "\"totalG\":%.2f,"
        "\"fall\":%d}",

        latestData.ax,
        latestData.ay,
        latestData.az,
        latestData.gx,
        latestData.gy,
        latestData.gz,
        latestData.pitch,
        latestData.roll,
        latestData.totalG,
        latestData.fall
      );

      if (client.publish(topicData, json)) {

        logPrint("[TASK 2] GUI DATA MQTT:");
        logPrint("%s", json);

      } else {

        logPrint("[TASK 2] Gui MQTT that bai!");
      }
    }

    // =========================================
    // GUI CANH BAO MQTT KHI TE NGA
    // =========================================

    bool sendFallAlert = false;

    portENTER_CRITICAL(&controlMux);

    if (mqttFallPending) {

      sendFallAlert = true;
      mqttFallPending = false;
    }

    portEXIT_CRITICAL(&controlMux);

    if (sendFallAlert) {

      if (client.publish(topicAlert, "FALL_DETECTED")) {

        logPrint("[TASK 2] Da gui canh bao te nga len MQTT!");

      } else {

        // Khoi phuc co bao hieu neu gui that bai
        portENTER_CRITICAL(&controlMux);
        mqttFallPending = true;
        portEXIT_CRITICAL(&controlMux);
      }
    }

    // NHI YIELD CHO TASK KHAC
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================================================
// 12. TASK 3 - CANH BAO TE NGA
// CORE 1
// ==================================================

void Task3_Alert(void *pvParameters) {

  logPrint("========== TASK 3 BAT DAU ==========");
  logPrint("[TASK 3] CORE 1 - CANH BAO");
  logPrint("[TASK 3] Dang cho Data Alert Queue...");

  AlertData alert;

  bool fallAlarm = false;

  unsigned long alarmStart = 0;

  for (;;) {

    // CHO NHAN DU LIEU CANH BAO
    if (xQueueReceive(
          dataAlertQueue,
          &alert,
          pdMS_TO_TICKS(100)
        ) == pdPASS) {

      fallAlarm = true;
      alarmStart = millis();

      logPrint("================================");
      logPrint("[TASK 3] PHAT HIEN TE NGA!");
      logPrint("[TASK 3] Tong gia toc: %.2f G",
               alert.totalG);

      logPrint("[TASK 3] Pitch: %.2f do",
               alert.pitch);

      logPrint("[TASK 3] Roll: %.2f do",
               alert.roll);

      logPrint("[TASK 3] BAT LED VA BUZZER");
      logPrint("================================");
    }

    // CANH BAO TU DONG 5 GIAY
    if (fallAlarm &&
        millis() - alarmStart >= 5000) {

      fallAlarm = false;

      logPrint("[TASK 3] Ket thuc canh bao tu dong!");
    }

    // DOC TRANG THAI MANUAL TU WEB
    bool manualLED;
    bool manualBuzzer;

    portENTER_CRITICAL(&controlMux);

    manualLED = ledManual;
    manualBuzzer = buzzerManual;

    portEXIT_CRITICAL(&controlMux);

    // TASK 3 LA TASK DUY NHAT DIEU KHIEN GPIO
    bool finalLED = fallAlarm || manualLED;
    bool finalBuzzer = fallAlarm || manualBuzzer;

    digitalWrite(
      LED_PIN,
      finalLED ? HIGH : LOW
    );

    digitalWrite(
      BUZZER_PIN,
      finalBuzzer ? HIGH : LOW
    );

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================================================
// 13. SETUP
// ==================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("       FALL DETECTION SYSTEM");
  Serial.println(" ESP32 + MPU6050 + FREERTOS + MQTT");
  Serial.println("=====================================");

  // KHOI TAO GPIO
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // TAO MUTEX SEMAPHORE
  serialMutex = xSemaphoreCreateMutex();

  if (serialMutex == NULL) {

    Serial.println("[SETUP] LOI TAO MUTEX!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("[SETUP] Tao Mutex Semaphore thanh cong!");

  // KHOI TAO I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // KHOI TAO MPU6050
  if (!mpu.begin()) {

    Serial.println("[SETUP] KHONG TIM THAY MPU6050!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("[SETUP] MPU6050 khoi tao thanh cong!");

  // CAU HINH CAM BIEN
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // TAO DATA QUEUE - 1 PHAN TU
  dataQueue = xQueueCreate(
    1,
    sizeof(SensorData)
  );

  // TAO DATA ALERT QUEUE
  dataAlertQueue = xQueueCreate(
    5,
    sizeof(AlertData)
  );

  if (dataQueue == NULL ||
      dataAlertQueue == NULL) {

    Serial.println("[SETUP] LOI TAO QUEUE!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("[SETUP] Data Queue thanh cong!");
  Serial.println("[SETUP] Data Alert Queue thanh cong!");

  // =========================================
  // TAO TASK 1 - CORE 1
  // =========================================

  BaseType_t result1 = xTaskCreatePinnedToCore(
    Task1_ReadData,
    "Task 1 Doc data",
    4096,
    NULL,
    2,
    &Task1Handle,
    1
  );

  // =========================================
  // TAO TASK 2 - CORE 0
  // =========================================

  BaseType_t result2 = xTaskCreatePinnedToCore(
    Task2_SendData,
    "Task 2 Truyen data",
    8192,
    NULL,
    1,
    &Task2Handle,
    0
  );

  // =========================================
  // TAO TASK 3 - CORE 1
  // =========================================

  BaseType_t result3 = xTaskCreatePinnedToCore(
    Task3_Alert,
    "Task 3 Canh bao",
    4096,
    NULL,
    3,
    &Task3Handle,
    1
  );

  // KIEM TRA KHOI TAO TASK
  if (result1 == pdPASS &&
      result2 == pdPASS &&
      result3 == pdPASS) {

    Serial.println("[SETUP] TASK 1: CORE 1 - OK");
    Serial.println("[SETUP] TASK 2: CORE 0 - OK");
    Serial.println("[SETUP] TASK 3: CORE 1 - OK");

    Serial.println("[SETUP] DA KHOI TAO DU 3 TASK!");
    Serial.println("[SETUP] HE THONG SAN SANG!");

  } else {

    Serial.println("[SETUP] LOI KHOI TAO TASK!");
  }
}

// ==================================================
// 14. LOOP
// ==================================================

void loop() {

  vTaskDelay(pdMS_TO_TICKS(1000));
}

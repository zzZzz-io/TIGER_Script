#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_now.h>

// Constants
#define MESSAGE_PAYLOAD_SIZE 100
#define SERIAL_BAUD_RATE 115200
#define SEA_LEVEL_PRESSURE 1017.7f
#define PRESSURE_CORRECTION_OFFSET 2.5f
#define BAROMETRIC_FORMULA_EXPONENT 0.1903f
#define BAROMETRIC_FORMULA_CONSTANT 44330.0f

// Hardware pins
#define LED_PIN 5
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13
#define SD_MOSI_PIN 11
#define SD_CS_PIN 10
#define I2C_SDA_PIN 36
#define I2C_SCL_PIN 35
#define EJECTION_SERVO_PIN 4
#define EJECTION_LEDC_CHANNEL 0
#define SERVO_PWM_FREQUENCY 50
#define SERVO_PWM_RESOLUTION 14
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

// Data logging
#define I2C_Clock_Speed 400000 // 400 kHz I2C speed
#define DATA_LOG_INTERVAL_MS 20
#define CSV_FILENAME "/flight_data.csv"

// MAC Address
uint8_t BASE_STATION_MAC_ADDRESS[6] = {0xDC, 0xDA, 0x0C, 0x64, 0x16, 0x60};

// Structs
typedef struct {
    char payload[MESSAGE_PAYLOAD_SIZE];
} message_t;

struct SensorData {
    float timestamp;
    float pressureRaw;
    float altitudeRaw;
    float pressureCorrected;
    float altitudeCorrected;
    float altitudeChange;
    float temperatureESP32;
    float temperatureC;
    float temperatureF;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};

// Global Variables
BMI270 imu;
float initialAltitude = 0.0f;
volatile bool messageSent = false;
volatile esp_now_send_status_t sendStatus;
message_t incomingMessage;
message_t outgoingMessage;

// Utlity
void printTimestamp() {
    unsigned long ms = millis();
    int hours = (ms / 3600000) % 24;
    int minutes = (ms / 60000) % 60;
    int seconds = (ms / 1000) % 60;
    int milliseconds = ms % 1000;
    
    Serial.printf("[%02d:%02d:%02d.%03d] ", hours, minutes, seconds, milliseconds);
}

void printTimestampedMessage(const char* message) {
    printTimestamp();
    Serial.println(message);
}

uint16_t angleToDutyCycle(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    return (pulseWidth * 16383L) / 20000;
}

void ServoWrite(int angle) {
    ledcWrite(EJECTION_LEDC_CHANNEL, angleToDutyCycle(angle));
}




// Parachute Control

void deployParachute() {
    printTimestampedMessage("DEPLOYING PARACHUTE!");
    
    // 0° -> 180° -> 0° -> 180°
    ServoWrite(180);
    delay(1000);
    ServoWrite(0);
    delay(1000);
    ServoWrite(180);
    
    printTimestampedMessage("Parachute deployment sequence completed");
}


void resetServo() {
    ServoWrite(180);
    printTimestampedMessage("Servo reset to 180° position");
}





// Command Processing

void executeCommand(const char* command) {
    printTimestamp();
    Serial.print("Executing command: ");
    Serial.println(command);
    
    if (strcmp(command, "deploy") == 0) {
        deployParachute();
    } else if (strcmp(command, "reset") == 0) {
        resetServo();
    } else {
        printTimestampedMessage("ERROR: Unknown command");
    }
}

// ====== ESP-NOW CALLBACK FUNCTIONS ======

/**
 * Callback for received ESP-NOW messages
 */
void onMessageReceived(const uint8_t *mac, const uint8_t *data, int len) {
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    int messageLength = min(ESP_NOW_MAX_DATA_LEN, len);
    strncpy(buffer, (const char *)data, messageLength);
    buffer[messageLength] = '\0';
    
    executeCommand(buffer);
}

/**
 * Callback for ESP-NOW send status
 */
void onMessageSent(const uint8_t *mac, esp_now_send_status_t status) {
    sendStatus = status;
    messageSent = true;
}

// Sensor Functions

void readSensorData(SensorData& data) {

    // Log timestamp
    data.timestamp = millis() / 1000.0f; // Convert to seconds

    // Read barometric/altitude data
    data.pressureCorrected = data.pressureRaw - PRESSURE_CORRECTION_OFFSET;
    data.altitudeCorrected = BAROMETRIC_FORMULA_CONSTANT * 
                           (1.0f - pow(data.pressureCorrected / SEA_LEVEL_PRESSURE, BAROMETRIC_FORMULA_EXPONENT));
    data.altitudeChange = data.altitudeRaw - initialAltitude;
    
    // Read IMU data
    imu.getSensorData();
    data.accelX = imu.data.accelX;
    data.accelY = imu.data.accelY;
    data.accelZ = imu.data.accelZ;
    data.gyroX = imu.data.gyroX;
    data.gyroY = imu.data.gyroY;
    data.gyroZ = imu.data.gyroZ;
}

/**
 * Print sensor data to serial console
 */
void printSensorData(const SensorData& data) {
    Serial.println("----- SENSOR DATA -----");
    Serial.printf("Pressure (Raw/Corrected): %.2f / %.2f hPa\n", data.pressureRaw, data.pressureCorrected);
    Serial.printf("Altitude (Raw/Corrected): %.2f / %.2f m\n", data.altitudeRaw, data.altitudeCorrected);
    Serial.printf("Temperature: %.2f°C (%.2f°F)\n", data.temperatureC, data.temperatureF);
    Serial.printf("ESP32 Temperature: %.2f°C (%.2f°F)\n", data.temperatureESP32, (data.temperatureESP32 * 9.0f / 5.0f) + 32.0f);
    Serial.printf("Acceleration (mg): X=%.2f, Y=%.2f, Z=%.2f\n", data.accelX, data.accelY, data.accelZ);
    Serial.printf("Gyroscope (mdps): X=%.2f, Y=%.2f, Z=%.2f\n", data.gyroX, data.gyroY, data.gyroZ);
    Serial.println("----------------------");
}

/**
 * Log sensor data to SD card
 */
bool logDataToSD(const SensorData& data) {
    File dataFile = SD.open(CSV_FILENAME, FILE_APPEND);
    if (!dataFile) {
        printTimestampedMessage("ERROR: Failed to open data file for writing");
        return false;
    }
    
    // Write CSV row
    dataFile.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                   data.timestamp, data.pressureRaw, data.altitudeRaw, data.pressureCorrected, data.altitudeCorrected, data.temperatureC, data.temperatureF, data.temperatureESP32,
                   data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);
    
    dataFile.close();
    return true;
}

// ESPNOW Communication

//Send to Base-Station
void transmitFlightData(const SensorData& data) {
    // Create telemetry string (simplified for now)
    snprintf(outgoingMessage.payload, sizeof(outgoingMessage.payload),
             "ALT:%.1f,TEMP:%.1f,ACCEL:%.1f", 
             data.altitudeChange, data.temperatureC, data.accelZ);
    
    esp_now_send(BASE_STATION_MAC_ADDRESS, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
}

// Initialization

//Initialize I2C Sensors
bool initializeSensors() {
    printTimestampedMessage("Initializing I2C bus...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_Clock_Speed);
    delay(500); // Allow sensors to power up

    // Initialize IMU
    printTimestampedMessage("Initializing BMI270 IMU...");
    if (imu.beginI2C() != BMI2_OK) {
        printTimestampedMessage("ERROR: BMI270 initialization failed!");
        return false;
    }
    
    printTimestampedMessage("All sensors initialized successfully");
    return true;
}

//Initialize SD Card
bool initializeSDCard() {
    printTimestampedMessage("Initializing SPI bus for SD card...");
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    printTimestampedMessage("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN)) {
        printTimestampedMessage("ERROR: SD card initialization failed!");
        return false;
    }
    
    // Create CSV header
    File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
    if (!dataFile) {
        printTimestampedMessage("ERROR: Failed to create data file!");
        return false;
    }
    
    dataFile.println("Time,Pressure_Raw,Altitude_Raw,Pressure_Corrected,Altitude_Corrected,Altitude_Change,Temperature_C,Temperature_F,Temperature_ESP32,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
    dataFile.close();
    
    printTimestampedMessage("SD card initialized and data file created");
    return true;
}

//Initilaize ESP-NOW
bool initializeESPNow() {
    printTimestampedMessage("Initializing ESP-NOW...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    if (esp_now_init() != ESP_OK) {
        printTimestampedMessage("ERROR: ESP-NOW initialization failed!");
        return false;
    }
    
    // Register callbacks
    esp_now_register_recv_cb(onMessageReceived);
    esp_now_register_send_cb(onMessageSent);
    
    // Add base station
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BASE_STATION_MAC_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    printTimestampedMessage("ESP-NOW initialized successfully");
    return true;
}

//Initialize Servo
void initializeServo() {
    printTimestampedMessage("Initializing parachute deployment servo...");
    ledcSetup(EJECTION_LEDC_CHANNEL, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
    ledcAttachPin(EJECTION_SERVO_PIN, EJECTION_LEDC_CHANNEL);
    
    // Set to initial position
    ledcWrite(EJECTION_LEDC_CHANNEL, angleToDutyCycle(180));
    printTimestampedMessage("Servo initialized at 180° position");
}





void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        delay(50);
    }
    
    printTimestampedMessage("=== ROCKET FLIGHT COMPUTER STARTING ===");
    
    if (!initializeSensors()) {
        printTimestampedMessage("Sensor initialization failed!");
        while (1) delay(1000);
    }
    
    if (!initializeSDCard()) {
        printTimestampedMessage("SD card initialization failed!");
        while (1) delay(1000);
    }
    
    if (!initializeESPNow()) {
        printTimestampedMessage("ESP-NOW initialization failed!");
        while (1) delay(1000);
    }
    
    initializeServo();
    
    printTimestampedMessage("=== FLIGHT COMPUTER READY FOR LAUNCH ===");
}

void loop() {

 
    // Blink built-in LED every 500ms
    digitalWrite(LED_PIN, (millis() / 500) % 2);

    SensorData currentData;
    
    // Read all sensor data
    readSensorData(currentData);
    
    // Print to serial console
    printSensorData(currentData);
    
    // Log to SD card
    if (!logDataToSD(currentData)) {
        printTimestampedMessage("WARNING: Data logging failed!");
    }
    
    // Transmit to base station
    transmitFlightData(currentData);
    
    // Check transmission status
    if (messageSent) {
        const char* statusMsg = (sendStatus == ESP_NOW_SEND_SUCCESS) ? 
                               "Telemetry sent successfully" : "Telemetry transmission failed";
        printTimestampedMessage(statusMsg);
        messageSent = false;
    }
    

    delay(DATA_LOG_INTERVAL_MS);
}
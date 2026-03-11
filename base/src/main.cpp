#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Communication
#define MESSAGE_PAYLOAD_SIZE 100
#define SERIAL_BAUD_RATE 115200

//MAC Addresses for ESP-NOW 
uint8_t ROCKET_MAC_ADDRESS[6] = {0x34, 0xcd, 0xb0, 0x06, 0x79, 0x60};
uint8_t BASE_STATION_MAC_ADDRESS[6] = {0xDC, 0xDA, 0x0C, 0x64, 0x16, 0x60};

//Data Structures
typedef struct {
    char payload[MESSAGE_PAYLOAD_SIZE];
} message_t;

//Variables
volatile bool g_messageReceived = false;
message_t g_incomingMessage;
message_t g_outgoingMessage;



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

// ====== ESP-NOW CALLBACK FUNCTIONS ======

/**
 * Callback function executed when data is received via ESP-NOW
 * @param mac MAC address of sender
 * @param data Received data buffer
 * @param len Length of received data
 */
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // Ensure we don't exceed buffer limits
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    int messageLength = min(ESP_NOW_MAX_DATA_LEN, len);
    strncpy(buffer, (const char *)data, messageLength);
    
    // Null terminate the string
    buffer[messageLength] = '\0';
    
    // Copy to global message structure
    memcpy(&g_incomingMessage, buffer, sizeof(g_incomingMessage));
    
    // Set flag to process message in main loop
    g_messageReceived = true;
}

// Initialize ESP-NOW
bool initializeESPNow() {
    printTimestampedMessage("Initializing ESP-NOW...");
    
    // Set WiFi mode and disconnect from any networks
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        printTimestampedMessage("ERROR: ESP-NOW initialization failed!");
        return false;
    }
    
    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);
    
    // Add rocket as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ROCKET_MAC_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        printTimestampedMessage("ERROR: Failed to add rocket as peer!");
        return false;
    }
    
    printTimestampedMessage("ESP-NOW initialized successfully");
    return true;
}

void sendCommandToRocket(const String& command) {
    // Copy command to outgoing message structure
    command.toCharArray(g_outgoingMessage.payload, sizeof(g_outgoingMessage.payload));
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(ROCKET_MAC_ADDRESS, (uint8_t*)&g_outgoingMessage, sizeof(g_outgoingMessage));
    
    printTimestamp();
    Serial.print("Command sent: ");
    Serial.print(g_outgoingMessage.payload);
    Serial.println(result == ESP_OK ? " [SUCCESS]" : " [FAILED]");
}

void processReceivedMessage() {
    printTimestamp();
    Serial.print("Received from rocket: ");
    Serial.println(g_incomingMessage.payload);
    
    // Reset the flag
    g_messageReceived = false;
}


void handleSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove whitespace and newlines
        
        if (input.length() > 0) {
            sendCommandToRocket(input);
        }
    }
}


void setup() {
    // Initialize serial 
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        delay(10); // Wait for serial port to connect
    }
    
    printTimestampedMessage("=== ROCKET BASE STATION INITIALIZED ===");
    
    // Initialize ESP-NOW 
    if (!initializeESPNow()) {
        printTimestampedMessage("FATAL ERROR: ESP-NOW initialization failed!");
        while (1) {
            delay(1000);
        }
    }
    
    // Print usage instructions
    Serial.println();
    printTimestampedMessage("Base station ready!");
    printTimestampedMessage("Available commands:");
    printTimestampedMessage("  - 'deploy': Deploy parachute");
    printTimestampedMessage("  - 'reset': Reset rocket systems");
    printTimestampedMessage("Type command and press Enter to send to rocket.");
    Serial.println();
}

void loop() {
    handleSerialInput();
    
    if (g_messageReceived) {
        processReceivedMessage();
    }
}
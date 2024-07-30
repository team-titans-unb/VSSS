#include "communication.h"

Communication::Communication(const char* ssid, const char* password, uint16_t port)
    : ssid_(ssid), password_(password), port_(port), server_(port) {}

void Communication::begin() {
    WiFi.begin(ssid_, password_);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    server_.begin();
}

int Communication::receiveInt() {
    if (!client_ || !client_.connected()) {
        client_ = server_.available();
        if (!client_) {
            return -1; // No client connected
        }
    }
    if (client_.available()) {
        int value = 0;
        client_.read(reinterpret_cast<uint8_t*>(&value), sizeof(value));
        return value;
    }
    return -1;
}

void Communication::sendInt(int value) {
    if (client_ && client_.connected()) {
        client_.write(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    }
}

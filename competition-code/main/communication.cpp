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

uint16_t Communication::receiveData() {
    if (!client_ || !client_.connected()) {
        client_ = server_.available();
        if (!client_) {
            return 0xFFFF; // Valor inválido, sem cliente conectado
        }
    }
    if (client_.available()) {
        uint16_t value = 0;
        client_.read(reinterpret_cast<uint8_t*>(&value), sizeof(value));
        return value;
    }
    return 0xFFFF; // Valor inválido, sem dados disponíveis
}

void Communication::sendData(uint16_t value) {
    if (client_ && client_.connected()) {
        client_.write(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    }
}

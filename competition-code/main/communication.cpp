
/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        communication.cpp                            Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "communication.h"

Communication::Communication(const char* ssid, const char* password, uint32_t port)
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

uint32_t Communication::receiveData() {
    if (!client_ || !client_.connected()) {
        client_ = server_.available();
        if (!client_) {
            return 0xFFFFFFFF; // Valor inválido, sem cliente conectado
        }
    }
    if (client_.available()) {
        uint32_t value = 0;
        client_.read(reinterpret_cast<uint8_t*>(&value), sizeof(value));
        return value;
    }
    return 0xFFFFFFFF; // Valor inválido, sem dados disponíveis
}

void Communication::sendData(uint32_t value) {
    if (client_ && client_.connected()) {
        client_.write(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    }
}
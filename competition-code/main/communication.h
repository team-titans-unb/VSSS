#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <WiFi.h>

class Communication {
public:
    Communication(const char* ssid, const char* password, uint16_t port);
    void begin();
    uint16_t receiveData();  // Atualizado para receber dados combinados de 16 bits
    void sendData(uint16_t value);  // Atualizado para enviar dados combinados de 16 bits

private:
    const char* ssid_;
    const char* password_;
    uint16_t port_;
    WiFiServer server_;
    WiFiClient client_;
};

#endif

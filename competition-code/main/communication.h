#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <WiFi.h>

class Communication {
public:
    Communication(const char* ssid, const char* password, uint16_t port);
    void begin();
    int receiveInt();
    void sendInt(int value);

private:
    const char* ssid_;
    const char* password_;
    uint16_t port_;
    WiFiServer server_;
    WiFiClient client_;
};

#endif

/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        communication.h                              Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <WiFi.h>

class Communication {
public:
    Communication(const char* ssid, const char* password, uint32_t port);
    void begin();
    uint32_t receiveData();  // Atualizado para receber dados combinados de 16 bits
    void sendData(uint32_t value);  // Atualizado para enviar dados combinados de 16 bits

private:
    const char* ssid_;
    const char* password_;
    uint16_t port_;
    WiFiServer server_;
    WiFiClient client_;
};

#endif

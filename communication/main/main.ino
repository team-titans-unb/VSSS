#include <WiFi.h>

const char* ssid = "Wifi";
const char* password = "luiz$123";
bool processing = false;
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client && !processing) {
    processing = true;
    String data = client.readStringUntil('\n');
    Serial.println("Received data: " + data);

    float x , y, z;
    sscanf(data.c_str(), "%f,%f,%f", &x, &y, &z);
    x = 0;
    y = 0;

    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(z);
    String response = String(x, 6) + "," + String(y, 6) + "," + String(z, 6) + "\n";
    client.print(response);
    client.stop();
    processing = false;
  }
}

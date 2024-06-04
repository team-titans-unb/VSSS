# Controle de Robôs Utilizando WiFi e Sockets

Este documento descreve um exemplo de código em C++ que utiliza a biblioteca WiFi para estabelecer comunicação entre um microcontrolador ESP32 e uma rede WiFi. O objetivo é receber dados de posição de robôs de uma aplicação como o CoppeliaSim e processá-los. Cada robô terá um endereço IP fixo, configurado através de um modem usando DHCP.

## Como Rodar o Projeto?

1. **Compilação e Upload do Código para o ESP32:**
   - Entre na pasta **main**.
   - Abra o arquivo `main.ino` usando o Arduino IDE.
   - Compile o código e faça o upload para o ESP32.

2. **Configuração no CoppeliaSim:**
   - Abra o CoppeliaSim e carregue o arquivo de simulação `campoVss.ttt`.
   - Inicie a simulação.

3. **Configuração da Comunicação com Python:**
   - Navegue até a pasta **api**.
   - Abra o arquivo `oneRobot.py`.
   - Certifique-se de configurar a rede no código `oneRobot.py` para a mesma do servidor ESP32, garantindo que a comunicação seja realizada corretamente.
   - No arquivo `oneRobot.py`, troque o endereço IP e a porta para os mesmos utilizados pelo servidor ESP32 (no exemplo, estamos utilizando a porta 80).

4. **Envio de Dados:**
   - Utilize a função `send_and_receive(x, y, z, ip, porta)` no `oneRobot.py` para enviar e receber dados.
   - Substitua `ip` pelo endereço IP do servidor ESP32 e `porta` pela porta configurada (80).

Seguindo esses passos, você estará pronto para rodar o projeto e estabelecer a comunicação entre o CoppeliaSim e o ESP32 através do Python.

## Explicação do Código do Cliente(Python e Coppelia)

### Função `send_and_receive_esp32`

#### Descrição da Função:

```python
def send_and_receive_esp32(x, y, z, esp32_ip, esp32_port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((esp32_ip, esp32_port))
        data = f"{x},{y},{z}\n"
        s.sendall(data.encode())
        
        # Receive modified data from ESP32
        data = s.recv(1024).decode()
        x_mod, y_mod, z = map(float, data.strip().split(','))
        return x_mod, y_mod, z
```

1. **Criação do Socket:** A função cria um socket TCP/IP para comunicação.
2. **Conexão ao ESP32:** O socket se conecta ao ESP32 utilizando o endereço IP e a porta especificados.
3. **Envio de Dados:** Os dados de posição (x, y, z) são enviados ao ESP32. Estes dados são codificados como uma string no formato "x,y,z\n".
4. **Recebimento de Dados:** A função espera uma resposta do ESP32, que contém os dados modificados. Estes dados são recebidos como uma string e, em seguida, convertidos de volta para números de ponto flutuante.
5. **Retorno dos Dados Modificados:** A função retorna os valores modificados de x e y, além do valor de z.

## Estrutura Geral do Código do Servidor main.ino (ROBÔS FÍSICOS)

### Bibliotecas Incluídas

```cpp
#include <WiFi.h>
```

Esta biblioteca permite a conexão do ESP32 a redes WiFi e a criação de um servidor para comunicação via sockets.

### Declaração de Variáveis e Objetos

```cpp
const char* ssid = "Wifi"; // Nome da sua rede
const char* password = "luiz$123"; // Mude a senha para a senha da sua rede 
bool processing = false;
WiFiServer server(80);
```

- **ssid**: Nome da rede WiFi.
- **password**: Senha da rede WiFi.
- **processing**: Flag para indicar se um cliente está sendo processado.
- **server**: Objeto para criar um servidor na porta 80.

### Função `setup()`

```cpp
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
```

- **Serial.begin(115200)**: Inicializa a comunicação serial.
- **WiFi.begin(ssid, password)**: Conecta o ESP32 à rede WiFi.
- **while (WiFi.status() != WL_CONNECTED)**: Loop que espera a conexão WiFi ser estabelecida.
- **server.begin()**: Inicia o servidor.

### Função `loop()`

```cpp
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
```

- **WiFiClient client = server.available()**: Verifica se há um cliente disponível para se comunicar.
- **if (client && !processing)**: Processa o cliente se houver um disponível e não estiver sendo processado.
- **String data = client.readStringUntil('\n')**: Lê os dados recebidos do cliente.
- **sscanf(data.c_str(), "%f,%f,%f", &x, &y, &z)**: Converte a string recebida para três floats.
- **client.print(response)**: Envia a resposta de volta ao cliente.

## Comunicação com Sockets

### Estrutura da Comunicação

1. **Estabelecimento da Conexão**: O ESP32 se conecta à rede WiFi utilizando o SSID e a senha fornecidos.
2. **Aguarda Cliente**: O servidor WiFi aguarda por um cliente que tente se conectar na porta 80.
3. **Recepção de Dados**: Quando um cliente se conecta, os dados são recebidos e processados.
4. **Envio de Resposta**: Uma resposta é enviada de volta ao cliente.

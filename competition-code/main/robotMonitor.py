import sys
import socket
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QTextEdit
from PyQt5.QtCore import pyqtSignal, QObject

class LogEmitter(QObject):
    log_signal = pyqtSignal(str)

class MyApp(QWidget):
    def __init__(self, esp_configs):
        super().__init__()
        self.esp_configs = esp_configs
        self.initUI()

        self.log_emitter = LogEmitter()
        self.log_emitter.log_signal.connect(self.log_message)

        # Flags para monitorar as conexões
        self.connection_status = {name: False for name in esp_configs}

    def initUI(self):
        vbox = QVBoxLayout()

        self.testButton = QPushButton('Test Connection', self)
        self.testButton.clicked.connect(self.test_connections)
        vbox.addWidget(self.testButton)

        self.labels = {}
        for name in self.esp_configs:
            label = QLabel(f'{name}: Not tested', self)
            vbox.addWidget(label)
            self.labels[name] = label

        self.startButton = QPushButton('Start', self)
        self.startButton.clicked.connect(self.send_start)
        vbox.addWidget(self.startButton)

        self.stopButton = QPushButton('Stop', self)
        self.stopButton.clicked.connect(self.send_stop)
        vbox.addWidget(self.stopButton)

        self.message_area = QTextEdit(self)
        self.message_area.setReadOnly(True)
        vbox.addWidget(self.message_area)

        self.setLayout(vbox)
        self.setWindowTitle('TITANS VSS MONITOR')
        self.setGeometry(300, 300, 300, 300)
        self.show()

    def test_connections(self):
        threads = []
        for name, config in self.esp_configs.items():
            thread = threading.Thread(target=self.check_connection, args=(name, config['ip'], config['port']))
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

    def check_connection(self, name, host, port):
        if self.test_connection(host, port):
            self.labels[name].setText(f'{host}: Connected')
            self.connection_status[name] = True
        else:
            self.labels[name].setText(f'{host}: Not Connected')
            self.connection_status[name] = False

    def test_connection(self, host, port):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1)
                s.connect((host, port))
                return True
        except Exception as e:
            self.log_emitter.log_signal.emit(f"Erro ao testar conexão com {host}:{port} - {e}")
            return False

    def send_start(self):
        for name, config in self.esp_configs.items():
            if self.connection_status[name]:
                threading.Thread(target=self.send_to_esp32, args=(config['ip'], config['port'], config['start_msg'])).start()
            else:
                self.log_emitter.log_signal.emit(f"{name} não conectada. Não foi possível enviar 'Start'.")

    def send_stop(self):
        for name, config in self.esp_configs.items():
            if self.connection_status[name]:
                threading.Thread(target=self.send_to_esp32, args=(config['ip'], config['port'], config['stop_msg'])).start()
            else:
                self.log_emitter.log_signal.emit(f"{name} não conectada. Não foi possível enviar 'Stop'.")

    def send_to_esp32(self, ip, port, msg):
        try:
            self.log_emitter.log_signal.emit(f"Tentando enviar {msg} para {ip}:{port}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((ip, port))
                s.sendall(bytes([msg]))
                self.log_emitter.log_signal.emit(f"Enviado: {msg} para {ip}:{port}")
        except Exception as e:
            self.log_emitter.log_signal.emit(f"Erro ao enviar dado para {ip}:{port}: {e}")

    def log_message(self, message):
        self.message_area.append(message)
        print(message)

if __name__ == '__main__':
    esp_configs = {
        'esp1': {'ip': '192.168.33.253', 'port': 80, 'start_msg': 1, 'stop_msg': 0},
        'esp2': {'ip': '192.168.1.102', 'port': 12345, 'start_msg': 1, 'stop_msg': 0},
        'esp3': {'ip': '192.168.1.103', 'port': 12345, 'start_msg': 1, 'stop_msg': 0}
    }

    app = QApplication(sys.argv)
    ex = MyApp(esp_configs)
    sys.exit(app.exec_())

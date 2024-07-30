from robotMonitor import *

esp_configs = {
        'esp1': {'ip': '192.168.33.253', 'port': 80, 'start_msg': 1, 'stop_msg': 0}
    }

app = QApplication(sys.argv)
ex = MyApp(esp_configs)
sys.exit(app.exec_())
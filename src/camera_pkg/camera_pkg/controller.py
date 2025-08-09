import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QHBoxLayout
)
from PyQt5.QtCore import Qt
import serial

# ====== CONFIG ======
SERIAL_PORT = '/dev/ttyUSB0'   # เปลี่ยนเป็นพอร์ตของคุณ เช่น /dev/ttyUSB0 สำหรับ Linux
BAUD_RATE = 9600
POSITION_FILE = "positions.txt"
# ====================

class ServoControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("5 DOF Servo Control")
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        self.sliders = []
        self.labels = []

        layout = QVBoxLayout()

        for i in range(5):
            h_layout = QHBoxLayout()
            label = QLabel(f"Servo {i+1}: 90")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.valueChanged.connect(self.make_slider_handler(i, label))
            self.sliders.append(slider)
            self.labels.append(label)

            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            layout.addLayout(h_layout)

        self.save_btn = QPushButton("Save Position")
        self.save_btn.clicked.connect(self.save_position)
        self.send_btn = QPushButton("Send to Arduino")
        self.send_btn.clicked.connect(self.send_to_arduino)

        layout.addWidget(self.save_btn)
        layout.addWidget(self.send_btn)

        self.setLayout(layout)

    def make_slider_handler(self, index, label):
        def handler(value):
            label.setText(f"Servo {index+1}: {value}")
        return handler

    def get_current_position(self):
        return [slider.value() for slider in self.sliders]

    def save_position(self):
        pos = self.get_current_position()
        with open(POSITION_FILE, "w") as f:
            f.write(",".join(map(str, pos)))
        print(f"Position saved to {POSITION_FILE}: {pos}")

    def send_to_arduino(self):
        pos = self.get_current_position()
        message = f"POS:{','.join(map(str, pos))}\n"
        self.ser.write(message.encode())
        print(f"Sent to Arduino: {message.strip()}")

    def closeEvent(self, event):
        if self.ser.is_open:
            self.ser.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ServoControlUI()
    window.show()
    sys.exit(app.exec_())

def main(args=None):
    app = QApplication(sys.argv)
    window = ServoControlUI()
    window.show()
    sys.exit(app.exec_())
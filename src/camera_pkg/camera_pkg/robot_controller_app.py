# robot_controller_app.py - Code has been modified to read from a file instead of a ROS topic

from PyQt5 import QtCore, QtGui, QtWidgets
import serial
import serial.tools.list_ports
import sys
import os
import re
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QSize
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QVBoxLayout

# Import ROS libraries
try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    import cv2
except ImportError:
    print("ROS libraries not found. Camera features will be disabled.")
    rospy = None
    cv2 = None
    CvBridge = None

class ImageWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super(ImageWidget, self).__init__(parent)
        self.image = None

    def setImage(self, image):
        self.image = image
        self.update()

    def paintEvent(self, event):
        if self.image:
            painter = QtGui.QPainter(self)
            rect = self.rect()
            scaled_image = self.image.scaled(rect.width(), rect.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            x = (rect.width() - scaled_image.width()) // 2
            y = (rect.height() - scaled_image.height()) // 2
            painter.drawImage(x, y, scaled_image)

class ROSTopicSubscriber(QThread):
    image_received = pyqtSignal(QImage)
    string_received = pyqtSignal(str) # This signal will not be used for object data anymore

    def __init__(self, topic_name, msg_type):
        super(ROSTopicSubscriber, self).__init__()
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.bridge = CvBridge()
        self.running = True
        self.last_data = None
        self.subscriber = None

    def run(self):
        if rospy and self.running:
            if self.msg_type == Image:
                self.subscriber = rospy.Subscriber(self.topic_name, Image, self.image_callback)
            # We don't need a string subscriber anymore for object data
            # elif self.msg_type == String:
            #     self.subscriber = rospy.Subscriber(self.topic_name, String, self.string_callback)
            
            print(f"Subscribing to {self.topic_name}...")
            rospy.spin()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            q_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.image_received.emit(q_image)
        except Exception as e:
            print(f"Error processing image: {e}")

    def stop(self):
        self.running = False
        if self.subscriber:
            self.subscriber.unregister()
        self.quit()
        self.wait()

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1106, 510)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        self.horizontalSlider_pos1 = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider_pos1.setGeometry(QtCore.QRect(20, 50, 291, 41))
        self.horizontalSlider_pos1.setMinimum(0)
        self.horizontalSlider_pos1.setMaximum(180)
        self.horizontalSlider_pos1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_pos1.setObjectName("horizontalSlider_pos1")

        self.horizontalSlider_pos2 = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider_pos2.setGeometry(QtCore.QRect(20, 100, 291, 41))
        self.horizontalSlider_pos2.setMinimum(0)
        self.horizontalSlider_pos2.setMaximum(180)
        self.horizontalSlider_pos2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_pos2.setObjectName("horizontalSlider_pos2")
        
        self.horizontalSlider_pos3 = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider_pos3.setGeometry(QtCore.QRect(20, 150, 291, 41))
        self.horizontalSlider_pos3.setMinimum(0)
        self.horizontalSlider_pos3.setMaximum(180)
        self.horizontalSlider_pos3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_pos3.setObjectName("horizontalSlider_pos3")
        
        self.horizontalSlider_pos4 = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider_pos4.setGeometry(QtCore.QRect(20, 200, 291, 41))
        self.horizontalSlider_pos4.setMinimum(0)
        self.horizontalSlider_pos4.setMaximum(180)
        self.horizontalSlider_pos4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_pos4.setObjectName("horizontalSlider_pos4")
        
        self.horizontalSlider_pos5 = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider_pos5.setGeometry(QtCore.QRect(20, 250, 291, 41))
        self.horizontalSlider_pos5.setMinimum(0)
        self.horizontalSlider_pos5.setMaximum(180)
        self.horizontalSlider_pos5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_pos5.setObjectName("horizontalSlider_pos5")

        self.openGLWidget_RawCamera = ImageWidget(self.centralwidget)
        self.openGLWidget_RawCamera.setGeometry(QtCore.QRect(350, 220, 341, 221))
        self.openGLWidget_RawCamera.setObjectName("openGLWidget_RawCamera")

        self.openGLWidget_DetectedObject = ImageWidget(self.centralwidget)
        self.openGLWidget_DetectedObject.setGeometry(QtCore.QRect(710, 220, 341, 221))
        self.openGLWidget_DetectedObject.setObjectName("openGLWidget_DetectedObject")

        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(470, 190, 101, 17))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(820, 190, 131, 17))
        self.label_2.setObjectName("label_2")

        self.btnReset = QtWidgets.QPushButton(self.centralwidget)
        self.btnReset.setGeometry(QtCore.QRect(330, 60, 731, 25))
        self.btnReset.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnReset.setObjectName("btnReset")
        
        self.btnLeft_gotograb = QtWidgets.QPushButton(self.centralwidget)
        self.btnLeft_gotograb.setGeometry(QtCore.QRect(330, 90, 171, 25))
        self.btnLeft_gotograb.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnLeft_gotograb.setObjectName("btnLeft_gotograb")
        
        self.btnLeft_upit = QtWidgets.QPushButton(self.centralwidget)
        self.btnLeft_upit.setGeometry(QtCore.QRect(330, 120, 171, 25))
        self.btnLeft_upit.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnLeft_upit.setObjectName("btnLeft_upit")
        
        self.btnToPlaceRight = QtWidgets.QPushButton(self.centralwidget)
        self.btnToPlaceRight.setGeometry(QtCore.QRect(330, 150, 531, 25))
        self.btnToPlaceRight.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnToPlaceRight.setObjectName("btnToPlaceRight")
        
        self.btnLeft2_gotograb = QtWidgets.QPushButton(self.centralwidget)
        self.btnLeft2_gotograb.setGeometry(QtCore.QRect(510, 90, 171, 25))
        self.btnLeft2_gotograb.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnLeft2_gotograb.setObjectName("btnLeft2_gotograb")
        
        self.btnLeft2_upit = QtWidgets.QPushButton(self.centralwidget)
        self.btnLeft2_upit.setGeometry(QtCore.QRect(510, 120, 171, 25))
        self.btnLeft2_upit.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnLeft2_upit.setObjectName("btnLeft2_upit")
        
        self.btnRight_gotograb = QtWidgets.QPushButton(self.centralwidget)
        self.btnRight_gotograb.setGeometry(QtCore.QRect(690, 90, 171, 25))
        self.btnRight_gotograb.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnRight_gotograb.setObjectName("btnRight_gotograb")
        
        self.btnRight_upit = QtWidgets.QPushButton(self.centralwidget)
        self.btnRight_upit.setGeometry(QtCore.QRect(690, 120, 171, 25))
        self.btnRight_upit.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnRight_upit.setObjectName("btnRight_upit")
        
        self.btnfianalRight_gotograb = QtWidgets.QPushButton(self.centralwidget)
        self.btnfianalRight_gotograb.setGeometry(QtCore.QRect(870, 90, 191, 25))
        self.btnfianalRight_gotograb.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnfianalRight_gotograb.setObjectName("btnfianalRight_gotograb")
        
        self.btnFinalRight_upit = QtWidgets.QPushButton(self.centralwidget)
        self.btnFinalRight_upit.setGeometry(QtCore.QRect(870, 120, 191, 25))
        self.btnFinalRight_upit.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnFinalRight_upit.setObjectName("btnFinalRight_upit")
        
        self.btnToplaceatcenter = QtWidgets.QPushButton(self.centralwidget)
        self.btnToplaceatcenter.setGeometry(QtCore.QRect(870, 150, 191, 25))
        self.btnToplaceatcenter.setStyleSheet("background-color:rgb(246, 97, 81)")
        self.btnToplaceatcenter.setObjectName("btnToplaceatcenter")
        
        self.ArduinoPortList = QtWidgets.QListWidget(self.centralwidget)
        self.ArduinoPortList.setGeometry(QtCore.QRect(20, 10, 281, 31))
        self.ArduinoPortList.setObjectName("ArduinoPortList")
        
        self.btnArduinoStart = QtWidgets.QPushButton(self.centralwidget)
        self.btnArduinoStart.setGeometry(QtCore.QRect(310, 10, 151, 31))
        self.btnArduinoStart.setStyleSheet("background-color:rgb(165, 29, 45)")
        self.btnArduinoStart.setObjectName("btnArduinoStart")
        
        self.btnStart = QtWidgets.QPushButton(self.centralwidget)
        self.btnStart.setGeometry(QtCore.QRect(20, 300, 291, 81))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.btnStart.setFont(font)
        self.btnStart.setObjectName("btnStart")
        
        self.btnStop = QtWidgets.QPushButton(self.centralwidget)
        self.btnStop.setGeometry(QtCore.QRect(20, 390, 291, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.btnStop.setFont(font)
        self.btnStop.setStyleSheet("background-color:rgb(165, 29, 45)")
        self.btnStop.setObjectName("btnStop")
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1106, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        self.init_custom_components()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robot Controller"))
        self.label.setText(_translate("MainWindow", "Raw Camera"))
        self.label_2.setText(_translate("MainWindow", "Detected Camera"))
        self.btnReset.setText(_translate("MainWindow", "Reset Position"))
        self.btnLeft_gotograb.setText(_translate("MainWindow", "Pos_left_gotograb"))
        self.btnLeft_upit.setText(_translate("MainWindow", "Pos_left_upit"))
        self.btnToPlaceRight.setText(_translate("MainWindow", "toplaceatright"))
        self.btnLeft2_gotograb.setText(_translate("MainWindow", "Pos_left2_gotograb"))
        self.btnLeft2_upit.setText(_translate("MainWindow", "Pos_left2_upit"))
        self.btnRight_gotograb.setText(_translate("MainWindow", "Pos_right_gotograb"))
        self.btnRight_upit.setText(_translate("MainWindow", "Pos_right_upit"))
        self.btnfianalRight_gotograb.setText(_translate("MainWindow", "Pos_finalright_gotograb"))
        self.btnFinalRight_upit.setText(_translate("MainWindow", "Pos_finalright_upit"))
        self.btnToplaceatcenter.setText(_translate("MainWindow", "toplaceatcenter"))
        self.btnArduinoStart.setText(_translate("MainWindow", "Connect Arduino"))
        self.btnStart.setText(_translate("MainWindow", "Start"))
        self.btnStop.setText(_translate("MainWindow", "Terminate"))

    def init_custom_components(self):
        self.ser = None
        self.find_arduino_ports()
        self.btnArduinoStart.clicked.connect(self.connect_arduino)
        
        self.sliders = [
            self.horizontalSlider_pos1,
            self.horizontalSlider_pos2,
            self.horizontalSlider_pos3,
            self.horizontalSlider_pos4,
            self.horizontalSlider_pos5
        ]
        
        for slider in self.sliders:
            slider.setValue(90)
            self.horizontalSlider_pos2.setValue(6)
            self.horizontalSlider_pos3.setValue(17)
            slider.valueChanged.connect(self.send_servo_data)
            
        self.btnLeft_gotograb.clicked.connect(lambda: self.save_position("btnLeft_gotograb"))
        self.btnLeft_upit.clicked.connect(lambda: self.save_position("btnLeft_upit"))
        self.btnLeft2_gotograb.clicked.connect(lambda: self.save_position("btnLeft2_gotograb"))
        self.btnLeft2_upit.clicked.connect(lambda: self.save_position("btnLeft2_upit"))
        self.btnRight_gotograb.clicked.connect(lambda: self.save_position("btnRight_gotograb"))
        self.btnRight_upit.clicked.connect(lambda: self.save_position("btnRight_upit"))
        self.btnfianalRight_gotograb.clicked.connect(lambda: self.save_position("btnfianalRight_gotograb"))
        self.btnFinalRight_upit.clicked.connect(lambda: self.save_position("btnFinalRight_upit"))
        self.btnToPlaceRight.clicked.connect(lambda: self.save_position("btnToPlaceRight"))
        self.btnToplaceatcenter.clicked.connect(lambda: self.save_position("btnToplaceatcenter"))

        self.raw_camera_thread = None
        self.detected_camera_thread = None
        
        if rospy:
            rospy.init_node('pyqt_robot_controller', anonymous=True, disable_signals=True)
            self.raw_camera_thread = ROSTopicSubscriber('/camera/image_raw', Image)
            self.raw_camera_thread.image_received.connect(self.openGLWidget_RawCamera.setImage)
            self.raw_camera_thread.start()

            self.detected_camera_thread = ROSTopicSubscriber('/camera_detected', Image)
            self.detected_camera_thread.image_received.connect(self.openGLWidget_DetectedObject.setImage)
            self.detected_camera_thread.start()
        else:
            print("ROS is not available. Camera widgets will be inactive.")
            
        self.btnStart.clicked.connect(self.btnStart_clicked)
        self.btnStop.clicked.connect(self.btnStop_clicked)

    def find_arduino_ports(self):
        self.ArduinoPortList.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or "ttyACM" in port.device or "ttyUSB" in port.device:
                self.ArduinoPortList.addItem(port.device)
    
    def connect_arduino(self):
        selected_items = self.ArduinoPortList.selectedItems()
        if not selected_items:
            print("Please select a port.")
            return

        port = selected_items[0].text()
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.btnArduinoStart.setStyleSheet("background-color:rgb(165, 29, 45)")
            self.btnArduinoStart.setText("Connect Arduino")
            print("Disconnected from Arduino.")
        else:
            try:
                self.ser = serial.Serial(port, 9600, timeout=1)
                self.btnArduinoStart.setStyleSheet("background-color:rgb(76, 175, 80)")
                self.btnArduinoStart.setText("Disconnect Arduino")
                print(f"Successfully connected to Arduino on {port}")
            except serial.SerialException as e:
                print(f"Could not connect to {port}: {e}")
                
    def send_servo_data(self):
        if self.ser and self.ser.is_open:
            pos_values = [slider.value() for slider in self.sliders]
            message = f"POS:{pos_values[0]},{pos_values[1]},{pos_values[2]},{pos_values[3]},{pos_values[4]}\n"
            try:
                self.ser.write(message.encode('utf-8'))
                print(f"Sent: {message.strip()}")
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                self.connect_arduino()
                
    def save_position(self, button_name):
        pos_values = [slider.value() for slider in self.sliders]
        position_string = f"POS:{pos_values[0]},{pos_values[1]},{pos_values[2]},{pos_values[3]},{pos_values[4]}"
        
        filename = f"{button_name}.txt"
        try:
            with open(filename, 'w') as f:
                f.write(position_string)
            print(f"Saved position to {filename}: {position_string}")
            
            button = self.findChild(QtWidgets.QPushButton, button_name)
            if button:
                original_style = button.styleSheet()
                button.setStyleSheet("background-color:rgb(76, 175, 80)")
                QtCore.QTimer.singleShot(1000, lambda: button.setStyleSheet(original_style))
        except Exception as e:
            print(f"Error saving file {filename}: {e}")
            
    def load_and_send_position(self, filename):
        if self.ser and self.ser.is_open:
            try:
                with open(filename, 'r') as f:
                    position_string = f.read().strip()
                
                if position_string.startswith("POS:"):
                    self.ser.write((position_string + "\n").encode('utf-8'))
                    print(f"Sent position from {filename}: {position_string}")
                else:
                    print(f"File {filename} has an invalid format.")

            except FileNotFoundError:
                print(f"File not found: {filename}")
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                self.connect_arduino()
    
    def read_detection_file(self):
        """Reads object detection data from a text file."""
        try:
            with open("object_detected.txt", 'r') as f:
                data = f.read()
            return data
        except FileNotFoundError:
            print("Error: object_detected.txt file not found. Please run the object detection program first.")
            return None
        except Exception as e:
            print(f"Error reading object_detected.txt: {e}")
            return None

    def btnStart_clicked(self):

        self.load_and_send_position("left.txt")
        print("Start button clicked. Checking object detection data from file...")
        
        data = self.read_detection_file()

        if not data:
            print("No object data found in file. Cannot run.")
            return

        objects = re.findall(r'Object \d+ \[Position: (.*?), Label: (.*?), Color: (.*?)\]', data)
        print(data)
        
        left_red_found = False
        center_red_found = False
        right_red_found = False
        left_blue_found = False
        center_blue_found = False
        right_blue_found = False
        left_green_found = False
        center_green_found = False
        right_green_found = False

        for obj_data in objects:
            position = obj_data[0].strip()
            color = obj_data[2].strip()
            print(f"Found: {position} - {color}")

            if position == 'Left' and color == 'Red':
                left_red_found = True
            
            elif position == 'Center' and color == 'Red':
                center_red_found = True

            elif position == 'Right' and color == 'Red':
                right_red_found = True
                
            elif position == 'Left' and color == 'Blue':
                left_blue_found = True

            elif position == 'Center' and color == 'Blue':
                center_blue_found = True

            elif position == 'Right' and color == 'Blue':
                right_blue_found = True

            elif position == 'Left' and color == 'Green':
                left_green_found = True

            elif position == 'Center' and color == 'Green':
                center_green_found = True

            elif position == 'Right' and color == 'Green':
                right_green_found = True
            




















        # if left_red_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_blue_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_green_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")

        
        # if left_red_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_green_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_blue_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")        


        # if left_blue_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_red_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_green_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")

        # if left_blue_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_green_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_red_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")


        # if left_green_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_blue_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_red_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
            
        # if left_green_found :
        #     self.load_and_send_position("btnLeft_gotograb.txt")
        #     self.load_and_send_position("btnLeft_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif center_blue_found:
        #     self.load_and_send_position("btnLeft2_gotograb.txt")
        #     self.load_and_send_position("btnLeft2_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")
        # elif right_red_found:
        #     self.load_and_send_position("btnRight_gotograb.txt")
        #     self.load_and_send_position("btnRight_upit.txt")
        #     self.load_and_send_position("btnToPlaceRight.txt")

        if left_red_found and center_blue_found and right_green_found:
            print("All conditions met! Sending command from btnfianalRight_gotograb.txt")
            self.load_and_send_position("btnfianalRight_gotograb.txt")
        else:
            print("Conditions not met. No action taken.")


    def btnStop_clicked(self):
        print("Terminate button clicked.")
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.btnArduinoStart.setStyleSheet("background-color:rgb(165, 29, 45)")
            self.btnArduinoStart.setText("Connect Arduino")

        if rospy:
            if self.raw_camera_thread:
                self.raw_camera_thread.stop()
            if self.detected_camera_thread:
                self.detected_camera_thread.stop()
            
        sys.exit()

def main():
    if rospy:
        try:
            rospy.get_master().getUri()
        except:
            print("ROS Master is not running. Please start the ROS core first (e.g., roscore).")
            
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
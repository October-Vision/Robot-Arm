import json
from PyQt5.QtWidgets import QMainWindow, QMessageBox
from GUI.Argument_UI import Ui_MainWindow
import GUI as QtFunc
from util.Func import Json_Updata

class ArgumentFunc(QMainWindow,Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
    
        self.pushButton.clicked.connect(self.Save_Argument)
        
        self.Search() #搜索参数

    def Save_Argument(self):
        self.x = int(self.textEdit.toPlainText())
        self.y = int(self.textEdit_2.toPlainText())
        self.default_z = float(self.textEdit_9.toPlainText())
        self.detect_threshold = int(self.textEdit_3.toPlainText())
        self.gripper_lenght = int(self.textEdit_4.toPlainText())
        self.idVendor = self.textEdit_5.toPlainText()
        self.idProduct = self.textEdit_6.toPlainText()
        self.camera_index = int(self.textEdit_8.toPlainText())
        self.com = self.textEdit_7.toPlainText()
        self.claw_thread = self.comboBox_2.currentText()
        self.model_invoke_pattern = self.comboBox.currentText()
        self.key = self.textEdit_10.toPlainText()
        

        Json_Updata("config/argument.json", "baseX", self.x)
        Json_Updata("config/argument.json", "baseY", self.y)
        Json_Updata("config/argument.json", "default_Z", self.default_z)
        Json_Updata("config/argument.json", "detect_threshold", self.detect_threshold)
        Json_Updata("config/argument.json", "gripper_lenght", self.gripper_lenght)
        Json_Updata("config/argument.json", "idVendor", self.idVendor)
        Json_Updata("config/argument.json", "idProduct", self.idProduct)
        Json_Updata("config/argument.json", "camera_index", self.camera_index)
        Json_Updata("config/argument.json", "com", self.com)
        Json_Updata("config/argument.json", "claw_thread", self.claw_thread)
        Json_Updata("config/argument.json", "model_invoke_pattern", self.model_invoke_pattern)
        Json_Updata("config/argument.json", "key", self.key)
        
        
        messBox = QMessageBox()
        messBox.setWindowTitle(u'提示')
        messBox.setText(u'更新完成，重启软件后生效')
        messBox.exec_()
        
        
        
    def Search(self):
        with open('config/argument.json', 'r', encoding='utf-8') as f:
            self.pose_config = json.load(f)
            
        self.textEdit.setText(str(self.pose_config["baseX"]))
        self.textEdit_2.setText(str(self.pose_config["baseY"]))
        self.textEdit_3.setText(str(self.pose_config["detect_threshold"]))
        self.textEdit_4.setText(str(self.pose_config["gripper_lenght"]))
        self.textEdit_5.setText(str(self.pose_config["idVendor"]))
        self.textEdit_6.setText(str(self.pose_config["idProduct"]))
        self.textEdit_7.setText(str(self.pose_config["com"]))
        self.comboBox_2.setCurrentText(str(self.pose_config["claw_thread"]))
        self.comboBox.setCurrentText(str(self.pose_config["model_invoke_pattern"]))
        self.textEdit_8.setText(str(self.pose_config["camera_index"]))
        self.textEdit_9.setText(str(self.pose_config["default_Z"]))
        self.textEdit_10.setText(str(self.pose_config["key"]))


        

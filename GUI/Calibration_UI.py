# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Calibration_UI.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Calibration(object):
    def setupUi(self, Calibration):
        Calibration.setObjectName("Calibration")
        Calibration.resize(1033, 619)
        Calibration.setMinimumSize(QtCore.QSize(1033, 619))
        Calibration.setMaximumSize(QtCore.QSize(1033, 619))
        Calibration.setStyleSheet("border-width: 1px;border-style: solid;background-color: rgb(51,53,66); border-style: outset;border-radius:6px; ")
        self.centralwidget = QtWidgets.QWidget(Calibration)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setStyleSheet("background-color: rgb(69,71,85)")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_4 = QtWidgets.QFrame(self.frame)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.frame_4)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSpacing(20)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.frame_4)
        self.label_2.setMinimumSize(QtCore.QSize(320, 240))
        self.label_2.setMaximumSize(QtCore.QSize(320, 240))
        self.label_2.setStyleSheet("border-width: 1px;border-style: solid;background-color: rgb(221,221,221)")
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.label = QtWidgets.QLabel(self.frame_4)
        self.label.setMinimumSize(QtCore.QSize(320, 240))
        self.label.setMaximumSize(QtCore.QSize(320, 240))
        self.label.setStyleSheet("border-width: 1px;border-style: solid;background-color: rgb(221,221,221);width:640px;")
        self.label.setText("")
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.horizontalLayout_6.addLayout(self.horizontalLayout_2)
        self.verticalLayout_2.addWidget(self.frame_4)
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.frame_3)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(15, 0, -1, -1)
        self.horizontalLayout_3.setSpacing(27)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.textEdit = QtWidgets.QTextEdit(self.frame_3)
        self.textEdit.setStyleSheet("background: white;\n"
"font: 11pt \"Arial\";\n"
"")
        self.textEdit.setObjectName("textEdit")
        self.horizontalLayout_3.addWidget(self.textEdit)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setContentsMargins(50, -1, 50, -1)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton = QtWidgets.QPushButton(self.frame_3)
        self.pushButton.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.frame_3)
        self.pushButton_2.setEnabled(False)
        self.pushButton_2.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.horizontalLayout_3.addLayout(self.verticalLayout)
        self.horizontalLayout_3.setStretch(0, 2)
        self.horizontalLayout_3.setStretch(1, 1)
        self.horizontalLayout_5.addLayout(self.horizontalLayout_3)
        self.verticalLayout_2.addWidget(self.frame_3)
        self.verticalLayout_2.setStretch(0, 7)
        self.verticalLayout_2.setStretch(1, 3)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setMinimumSize(QtCore.QSize(300, 0))
        self.frame_2.setMaximumSize(QtCore.QSize(350, 16777215))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_5 = QtWidgets.QFrame(self.frame_2)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_5)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_3 = QtWidgets.QLabel(self.frame_5)
        self.label_3.setStyleSheet("font: 87 14pt \"Arial\";\n"
"color: white;\n"
"border: none;\n"
"margin-bottom: 5px;")
        self.label_3.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_4.addWidget(self.label_3)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.comboBox = QtWidgets.QComboBox(self.frame_5)
        self.comboBox.setMinimumSize(QtCore.QSize(0, 40))
        self.comboBox.setStyleSheet("background: white;\n"
"font: 9pt \"Arial\";")
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.horizontalLayout_7.addWidget(self.comboBox)
        self.pushButton_3 = QtWidgets.QPushButton(self.frame_5)
        self.pushButton_3.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton_3.setObjectName("pushButton_3")
        self.horizontalLayout_7.addWidget(self.pushButton_3)
        self.verticalLayout_4.addLayout(self.horizontalLayout_7)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem)
        self.verticalLayout_3.addWidget(self.frame_5)
        self.frame_6 = QtWidgets.QFrame(self.frame_2)
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_6)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_4 = QtWidgets.QLabel(self.frame_6)
        self.label_4.setStyleSheet("font: 87 14pt \"Arial\";\n"
"color: white;\n"
"border: none;")
        self.label_4.setObjectName("label_4")
        self.verticalLayout_5.addWidget(self.label_4)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_5 = QtWidgets.QLabel(self.frame_6)
        self.label_5.setStyleSheet("color: white;\n"
"border: none;\n"
"margin-bottom: 5px;\n"
"font: 9pt \"Arial\";")
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.frame_6)
        self.label_6.setStyleSheet("color: white;\n"
"border: none;\n"
"margin-bottom: 5px;\n"
"font: 9pt \"Arial\";")
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 0, 1, 1, 1)
        self.textEdit_2 = QtWidgets.QTextEdit(self.frame_6)
        self.textEdit_2.setMaximumSize(QtCore.QSize(70, 28))
        self.textEdit_2.setStyleSheet("background: white;\n"
"font: 9pt \"Arial\";\n"
"")
        self.textEdit_2.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
        self.textEdit_2.setPlaceholderText("")
        self.textEdit_2.setObjectName("textEdit_2")
        self.gridLayout.addWidget(self.textEdit_2, 1, 0, 1, 1)
        self.textEdit_3 = QtWidgets.QTextEdit(self.frame_6)
        self.textEdit_3.setMaximumSize(QtCore.QSize(70, 28))
        self.textEdit_3.setStyleSheet("background: white;\n"
"font: 9pt \"Arial\";\n"
"")
        self.textEdit_3.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
        self.textEdit_3.setPlaceholderText("")
        self.textEdit_3.setObjectName("textEdit_3")
        self.gridLayout.addWidget(self.textEdit_3, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame_6)
        self.label_7.setStyleSheet("color: white;\n"
"border: none;\n"
"margin-bottom: 5px;\n"
"font: 9pt \"Arial\";")
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 2, 0, 1, 2)
        self.textEdit_4 = QtWidgets.QTextEdit(self.frame_6)
        self.textEdit_4.setMaximumSize(QtCore.QSize(145, 28))
        self.textEdit_4.setStyleSheet("background: white;\n"
"font: 9pt \"Arial\";\n"
"")
        self.textEdit_4.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
        self.textEdit_4.setPlaceholderText("")
        self.textEdit_4.setObjectName("textEdit_4")
        self.gridLayout.addWidget(self.textEdit_4, 3, 0, 1, 2)
        self.horizontalLayout_8.addLayout(self.gridLayout)
        self.pushButton_4 = QtWidgets.QPushButton(self.frame_6)
        self.pushButton_4.setEnabled(False)
        self.pushButton_4.setMinimumSize(QtCore.QSize(32, 0))
        self.pushButton_4.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton_4.setObjectName("pushButton_4")
        self.horizontalLayout_8.addWidget(self.pushButton_4)
        self.verticalLayout_5.addLayout(self.horizontalLayout_8)
        self.verticalLayout_3.addWidget(self.frame_6)
        self.frame_7 = QtWidgets.QFrame(self.frame_2)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.frame_7)
        self.verticalLayout_7.setContentsMargins(50, -1, 50, -1)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.pushButton_5 = QtWidgets.QPushButton(self.frame_7)
        self.pushButton_5.setEnabled(False)
        self.pushButton_5.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton_5.setObjectName("pushButton_5")
        self.verticalLayout_7.addWidget(self.pushButton_5)
        self.pushButton_6 = QtWidgets.QPushButton(self.frame_7)
        self.pushButton_6.setEnabled(False)
        self.pushButton_6.setStyleSheet("\n"
"QPushButton\n"
"{\n"
"    height: 40px;\n"
"    background-color: rgb(104,225,235); /*背景色*/ \n"
"    border-style: outset;    /* 边界内凹 */\n"
"    border-width: 1px;     /* 边边界宽度 */\n"
"    font: bold 15px;     /* 字体大小 */\n"
"    min-width:2em;\n"
"    color:rgb(69,71,85); /* 字体颜色 */\n"
"    \n"
"}\n"
"/* 鼠标经过改变按钮颜色 */\n"
"QPushButton:hover\n"
"{\n"
"    background-color: rgb(102,255,104);\n"
"}\n"
"")
        self.pushButton_6.setObjectName("pushButton_6")
        self.verticalLayout_7.addWidget(self.pushButton_6)
        self.verticalLayout_3.addWidget(self.frame_7)
        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 1)
        self.verticalLayout_3.setStretch(2, 1)
        self.horizontalLayout_4.addWidget(self.frame_2)
        self.horizontalLayout_4.setStretch(0, 8)
        self.horizontalLayout_4.setStretch(1, 2)
        self.horizontalLayout.addWidget(self.frame)
        Calibration.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(Calibration)
        self.statusbar.setObjectName("statusbar")
        Calibration.setStatusBar(self.statusbar)

        self.retranslateUi(Calibration)
        QtCore.QMetaObject.connectSlotsByName(Calibration)

    def retranslateUi(self, Calibration):
        _translate = QtCore.QCoreApplication.translate
        Calibration.setWindowTitle(_translate("Calibration", "相机标定"))
        self.pushButton.setText(_translate("Calibration", "开启摄像头"))
        self.pushButton_2.setText(_translate("Calibration", "拍照"))
        self.label_3.setText(_translate("Calibration", "模式"))
        self.comboBox.setItemText(0, _translate("Calibration", "单目标定"))
        self.comboBox.setItemText(1, _translate("Calibration", "双目标定"))
        self.comboBox.setItemText(2, _translate("Calibration", "手眼标定"))
        self.pushButton_3.setText(_translate("Calibration", "选择"))
        self.label_4.setText(_translate("Calibration", "参数"))
        self.label_5.setText(_translate("Calibration", "列："))
        self.label_6.setText(_translate("Calibration", "行："))
        self.label_7.setText(_translate("Calibration", "宽度（单位：mm)："))
        self.pushButton_4.setText(_translate("Calibration", "确定"))
        self.pushButton_5.setText(_translate("Calibration", "标定"))
        self.pushButton_6.setText(_translate("Calibration", "保存"))

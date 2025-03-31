# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'robot_motion.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QVBoxLayout, QWidget)

class Ui_Motion(object):
    def setupUi(self, Motion):
        if not Motion.objectName():
            Motion.setObjectName(u"Motion")
        Motion.resize(720, 360)
        Motion.setStyleSheet(u"background-color: rgb(255, 250, 247);")
        self.verticalLayout = QVBoxLayout(Motion)
        self.verticalLayout.setSpacing(5)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setSpacing(10)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_fixed_rob = QLabel(Motion)
        self.label_fixed_rob.setObjectName(u"label_fixed_rob")
        self.label_fixed_rob.setMinimumSize(QSize(0, 0))
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.label_fixed_rob.setFont(font)

        self.horizontalLayout_3.addWidget(self.label_fixed_rob)

        self.robot_pose = QLabel(Motion)
        self.robot_pose.setObjectName(u"robot_pose")
        font1 = QFont()
        font1.setPointSize(11)
        font1.setBold(False)
        self.robot_pose.setFont(font1)
        self.robot_pose.setStyleSheet(u"background-color: rgb(243, 255, 245);")

        self.horizontalLayout_3.addWidget(self.robot_pose)

        self.horizontalLayout_3.setStretch(0, 1)
        self.horizontalLayout_3.setStretch(1, 5)

        self.verticalLayout_2.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_2 = QLabel(Motion)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setFont(font)

        self.horizontalLayout_4.addWidget(self.label_2)

        self.desired_pose = QLabel(Motion)
        self.desired_pose.setObjectName(u"desired_pose")
        font2 = QFont()
        font2.setPointSize(11)
        self.desired_pose.setFont(font2)
        self.desired_pose.setStyleSheet(u"background-color: rgb(238, 238, 236);")

        self.horizontalLayout_4.addWidget(self.desired_pose)

        self.horizontalLayout_4.setStretch(0, 1)
        self.horizontalLayout_4.setStretch(1, 5)

        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_8 = QLabel(Motion)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setMinimumSize(QSize(0, 0))
        self.label_8.setFont(font)

        self.verticalLayout_3.addWidget(self.label_8)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.start_point = QLabel(Motion)
        self.start_point.setObjectName(u"start_point")
        self.start_point.setStyleSheet(u"background-color: rgb(220, 247, 252)")

        self.horizontalLayout.addWidget(self.start_point)

        self.sample_start = QPushButton(Motion)
        self.sample_start.setObjectName(u"sample_start")
        self.sample_start.setFont(font2)

        self.horizontalLayout.addWidget(self.sample_start)

        self.horizontalLayout.setStretch(0, 9)
        self.horizontalLayout.setStretch(1, 2)

        self.verticalLayout_3.addLayout(self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.end_point = QLabel(Motion)
        self.end_point.setObjectName(u"end_point")
        self.end_point.setStyleSheet(u"background-color: rgb(220, 247, 252)")

        self.horizontalLayout_2.addWidget(self.end_point)

        self.sample_end = QPushButton(Motion)
        self.sample_end.setObjectName(u"sample_end")
        self.sample_end.setFont(font2)

        self.horizontalLayout_2.addWidget(self.sample_end)

        self.horizontalLayout_2.setStretch(0, 9)
        self.horizontalLayout_2.setStretch(1, 2)

        self.verticalLayout_3.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label = QLabel(Motion)
        self.label.setObjectName(u"label")

        self.horizontalLayout_6.addWidget(self.label)

        self.number = QLineEdit(Motion)
        self.number.setObjectName(u"number")

        self.horizontalLayout_6.addWidget(self.number)

        self.label_3 = QLabel(Motion)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_6.addWidget(self.label_3)

        self.left = QLabel(Motion)
        self.left.setObjectName(u"left")

        self.horizontalLayout_6.addWidget(self.left)

        self.generate = QPushButton(Motion)
        self.generate.setObjectName(u"generate")

        self.horizontalLayout_6.addWidget(self.generate)

        self.start_rob = QPushButton(Motion)
        self.start_rob.setObjectName(u"start_rob")

        self.horizontalLayout_6.addWidget(self.start_rob)

        self.pause = QPushButton(Motion)
        self.pause.setObjectName(u"pause")

        self.horizontalLayout_6.addWidget(self.pause)

        self.stop_rob = QPushButton(Motion)
        self.stop_rob.setObjectName(u"stop_rob")
        self.stop_rob.setFont(font2)

        self.horizontalLayout_6.addWidget(self.stop_rob)

        self.horizontalLayout_6.setStretch(0, 1)
        self.horizontalLayout_6.setStretch(1, 1)
        self.horizontalLayout_6.setStretch(2, 1)
        self.horizontalLayout_6.setStretch(3, 1)
        self.horizontalLayout_6.setStretch(4, 3)
        self.horizontalLayout_6.setStretch(5, 3)
        self.horizontalLayout_6.setStretch(6, 3)
        self.horizontalLayout_6.setStretch(7, 3)

        self.verticalLayout_3.addLayout(self.horizontalLayout_6)


        self.verticalLayout_2.addLayout(self.verticalLayout_3)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setSpacing(5)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(-1, -1, 0, -1)
        self.label_7 = QLabel(Motion)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setMinimumSize(QSize(0, 0))
        font3 = QFont()
        font3.setPointSize(12)
        self.label_7.setFont(font3)

        self.horizontalLayout_5.addWidget(self.label_7)

        self.warning = QLabel(Motion)
        self.warning.setObjectName(u"warning")
        self.warning.setFont(font3)

        self.horizontalLayout_5.addWidget(self.warning)

        self.reset = QPushButton(Motion)
        self.reset.setObjectName(u"reset")

        self.horizontalLayout_5.addWidget(self.reset)

        self.quit = QPushButton(Motion)
        self.quit.setObjectName(u"quit")
        self.quit.setFont(font2)

        self.horizontalLayout_5.addWidget(self.quit)

        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 6)
        self.horizontalLayout_5.setStretch(2, 2)
        self.horizontalLayout_5.setStretch(3, 2)

        self.verticalLayout_2.addLayout(self.horizontalLayout_5)

        self.verticalLayout_2.setStretch(0, 1)
        self.verticalLayout_2.setStretch(1, 1)
        self.verticalLayout_2.setStretch(2, 4)
        self.verticalLayout_2.setStretch(3, 1)

        self.verticalLayout.addLayout(self.verticalLayout_2)


        self.retranslateUi(Motion)

        QMetaObject.connectSlotsByName(Motion)
    # setupUi

    def retranslateUi(self, Motion):
        Motion.setWindowTitle(QCoreApplication.translate("Motion", u"Motion", None))
        self.label_fixed_rob.setText(QCoreApplication.translate("Motion", u"    iiwa pose:", None))
        self.robot_pose.setText("")
        self.label_2.setText(QCoreApplication.translate("Motion", u"Desired pose:", None))
        self.desired_pose.setText("")
        self.label_8.setText(QCoreApplication.translate("Motion", u"Robot control:", None))
        self.start_point.setText("")
        self.sample_start.setText(QCoreApplication.translate("Motion", u"Sample start", None))
        self.end_point.setText("")
        self.sample_end.setText(QCoreApplication.translate("Motion", u"Sample end", None))
        self.label.setText(QCoreApplication.translate("Motion", u"Total:", None))
        self.label_3.setText(QCoreApplication.translate("Motion", u"Left:", None))
        self.left.setText("")
        self.generate.setText(QCoreApplication.translate("Motion", u"Generate", None))
        self.start_rob.setText(QCoreApplication.translate("Motion", u"Start", None))
        self.pause.setText(QCoreApplication.translate("Motion", u"Pause", None))
        self.stop_rob.setText(QCoreApplication.translate("Motion", u"Stop", None))
        self.label_7.setText(QCoreApplication.translate("Motion", u"Note: ", None))
        self.warning.setText("")
        self.reset.setText(QCoreApplication.translate("Motion", u"Reset", None))
        self.quit.setText(QCoreApplication.translate("Motion", u"Quit", None))
    # retranslateUi


#!/usr/bin/env python3

"""
Coffee Life Test Fixture Application

Vikram Seshadri
December 4, 2020

"""
import os
import sys
import serial
import logging
import time
import re
import csv

# GUI
from PyQt5 import QtWidgets, QtCore, QtGui, uic
# from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtWidgets import QMessageBox
# from PyQt5.QtGui import*

# email
import smtplib
import ssl

# from email.mime.text import MIMEText
# from email.mime.image import MIMEImage
# from email.mime.multipart import MIMEMultipart
# import socket

# GUI defaults
station = 0
unit = 0
sku = " "
mode = " "
size = " "
style = " "
macro = 0
start_cycle = 1
current_cycle = start_cycle  # current cycle number. Will increment with each brew
number_of_cycles = 0
auto_shutoff_temp = 150  # in degC
boiler_cool_temp = 35  # in degC
vessel_cool_temp = 35  # in degC
cool_time = 5  # in minutes
max_brew_time = 10  # in minutes
filename_extra = " "
receiver_email = []

# file manager
file_dir = ' '
subfile_dir = ' '
filename = ' '
data_dir = ' '
summary_file = ' '

raw_data = ' '
summary_data = ' '

SKU_list = ['CFP300', 'CFP200', 'CM400']
Build_list = ['P0', 'P1', 'P2', 'P3', 'P4', 'FOT', 'EB0', 'EB1', 'EB1.5', 'EB2', 'MP']

# defaults
brew_weight = 0  # in g

# Serial ports
arduino_ser = ""
scale_ser = ""
unit_ser = ""

# test_param = "Station: {}, Unit: {}, Mode: {}, Size: {}, Style: {}, Macro: {}, Start Cycle: {}, " \
#              "Current Cycle: {}, Number of cycles: {}, Auto shutoff temp.: {}C, Boiler Cool temp.: {}C, " \
#              "Vessel Cool temp: {}C, Cool time: {}min., Max Brew time: {}min, Filename Extra: {}" \
#     .format(station, unit, mode, size, style, macro, start_cycle, current_cycle, number_of_cycles,
#             auto_shutoff_temp, boiler_cool_temp, vessel_cool_temp, cool_time, max_brew_time, filename_extra)

# flags
unit_fan_flag = 0  # unit fans off
vessel_fan_flag = 0  # vessel fans off
vessel_drain_flag = 0  # vessel drain pump off
arduino_connect_flag = 0  # Arduino disconnected
unit_connect_flag = 0  # Unit disconnected
scale_connect_flag = 0  # Scale disconnected
start_stop_flag = 0  # Stopped

VESSEL_PUMP_ON = b'$0010&\n'
POWER_OFF = b'$0000&\n'
VESSEL_FANS_ON = b'$0100&\n'
UNIT_FANS_ON = b'$0001&\n'


# Styling
class Styling:
    aux_button = (
        "background-color: transparent;"
        "height: 60px;"
        "border: 1px solid darkgray;"
        "font: bold;"
        "padding: 0px;"
        "font-size: 25px;"
    )

    connect_button = (
        "background-color: #1ED760;"
        "color:#ffffff;"
        "height: 30px;"
    )

    disconnect_button = (
        "background-color: #F44336;"
        "height: 30px;"
    )

    start_stop_exit_button = (
        "background-color: transparent;"
        "height: 60px;"
        "width: 20px;"
        "border: 0px solid darkgray;"
        "margin: 5px 5px 5px 5px;"
        "padding:-15px;"
    )


# Arduino communication
class ArduinoComm(object):
    # Commands - Output
    RESERVOIR_PUMP_ON = "A"
    # ESERVOIR_PUMP_OFF = "B"

    # VESSEL_PUMP_OFF = "D"

    # UNIT_FANS_OFF = "F"

    # VESSEL_FANS_OFF = "H"

    STOP_ALL = "X"

    def __init__(self):
        self.threadpool = QtCore.QThreadPool()
        self.do_init = QtCore.QEvent.registerEventType()

    @staticmethod
    def vessel_fan_toggle():
        global vessel_fan_flag
        global VESSEL_FANS_ON
        global POWER_OFF
        global cool_time
        try:
            if vessel_fan_flag == 0:
                # arduino_ser.write(b'$C&\n')
                arduino_ser.write(VESSEL_FANS_ON)
                logging.info('Vessel fans are on')
                vessel_fan_flag = 1
            else:
                logging.info('Vessel fans are off')
                vessel_fan_flag = 0
                # arduino_ser.write(b'$C&\n')
                arduino_ser.write(POWER_OFF)

        except Exception:
            logging.exception("Exception occurred", exc_info=True)

    @staticmethod
    def unit_fan_toggle():
        global unit_fan_flag
        global POWER_OFF
        global UNIT_FANS_ON
        global cool_time
        try:
            if unit_fan_flag == 0:
                arduino_ser.write(b'$C&\n')
                arduino_ser.write(UNIT_FANS_ON)
                logging.info('Unit fans are on')
                unit_fan_flag = 1
            else:
                logging.info('Unit fans are off')
                unit_fan_flag = 0
                arduino_ser.write(b'$C&\n')
                arduino_ser.write(POWER_OFF)

        except Exception:
            logging.exception("Exception occurred", exc_info=True)

    @staticmethod
    def vessel_drain_toggle():
        global vessel_drain_flag
        global VESSEL_PUMP_ON
        global POWER_OFF
        global cool_time

        try:
            if vessel_drain_flag == 0:
                # arduino_ser.write(b'$C&\n')
                arduino_ser.write(VESSEL_PUMP_ON)
                logging.info('Vessel Draining')
                vessel_drain_flag = 1
            else:
                logging.info('Vessel stopped draining')
                vessel_drain_flag = 0
                # arduino_ser.write(b'$C&\n')
                arduino_ser.write(POWER_OFF)

        except Exception:
            logging.exception("Exception occurred", exc_info=True)

    @staticmethod
    def arduino_pre_brew():
        # This function checks for all water float status (Reservoir, Vessel and Drain drum), power
        try:
            # arduino_ser.open()

            t_end = time.time() + 5  # 30 seconds
            arduino_ser.write(b'$1000&\n')
            while time.time() < t_end:
                # arduino_read_serial = arduino_ser.readline()
                # print(read_serial)
                print(time.time())
            logging.info("Pre-brew process has begun")

        except Exception:
            logging.exception("Exception occurred", exc_info=True)

    @staticmethod
    def send_command():
        global POWER_OFF
        arduino_ser.write(POWER_OFF)


# Weight information from the scale
def scale_data():
    global scale_ser, brew_weight
    scale_output = scale_ser.readline()
    scale_output.strip()
    y = scale_output.decode("utf-8", "replace")
    scale_pattern = r"([\w]+)(,)([\+|-])([\d\.]+)(\s+)([\w]+)"

    match = re.match(scale_pattern, y)

    if match is not None:
        # print(match.group(3) + match.group(4))
        brew_weight = match.group(3) + match.group(4)
        print("Scale weight: {}".format(brew_weight))
        return brew_weight


class UnitComm:
    # CFP communication
    if sku == "CFP300":
        pass


# GUI
class MainWindow(QtWidgets.QMainWindow):
    global station, unit, sku, mode, size, style, macro, start_cycle, current_cycle, number_of_cycles, \
        auto_shutoff_temp, boiler_cool_temp, vessel_cool_temp, cool_time, max_brew_time, filename_extra, \
        receiver_email, file_dir, data_dir, subfile_dir, filename, summary_file

    def __init__(self):

        self.threadpool = QtCore.QThreadPool()
        self.do_init = QtCore.QEvent.registerEventType()
        QtWidgets.QMainWindow.__init__(self)
        super(MainWindow, self).__init__()

        # parent_dir = os.path.dirname(os.path.realpath(__file__))
        ui_dir = os.path.join(parent_dir, "GUI.ui")
        self.ui = uic.loadUi(ui_dir, self)

        # update comboBox
        self.ui.CB_SKU.currentIndexChanged.connect(self.update_mode_combo)
        self.ui.CB_Mode.currentIndexChanged.connect(self.update_style_combo)
        self.ui.CB_Style.currentIndexChanged.connect(self.update_size_combo)

        # adds initial Text into SKU combobox
        self.ui.CB_SKU.clear()
        self.ui.CB_SKU.addItems(SKU_list)
        self.ui.CB_Build.clear()
        self.ui.CB_Build.addItems(Build_list)

        # Quit button and shortcut
        quit_action = QtWidgets.QAction('Quit', self)
        quit_action.setShortcuts(['Ctrl+Q', 'Ctrl+W'])
        quit_action.triggered.connect(self.quit)
        self.addAction(quit_action)
        self.ui.PB_Quit.clicked.connect(self.quit)

        # Push button styling
        self.ui.PB_UnitFans.setStyleSheet(Styling.aux_button)
        self.ui.PB_VesselFans.setStyleSheet(Styling.aux_button)
        self.ui.PB_VesselDrain.setStyleSheet(Styling.aux_button)
        self.ui.PB_ReservoirDrain.setStyleSheet(Styling.aux_button)
        self.ui.PB_Arduino_Connect.setStyleSheet(Styling.connect_button)
        self.ui.PB_Unit_Connect.setStyleSheet(Styling.connect_button)
        self.ui.PB_Scale_Connect.setStyleSheet(Styling.connect_button)
        self.ui.PB_Start_Stop.setStyleSheet(Styling.start_stop_exit_button)
        self.ui.PB_Quit.setStyleSheet(Styling.start_stop_exit_button)

        # Aux functions
        self.ui.PB_VesselFans.clicked.connect(ArduinoComm.vessel_fan_toggle)
        self.ui.PB_UnitFans.clicked.connect(ArduinoComm.unit_fan_toggle)
        self.ui.PB_VesselDrain.clicked.connect(ArduinoComm.vessel_drain_toggle)

        # Serial connections
        self.ui.PB_Arduino_Connect.clicked.connect(self.arduino_connect)
        self.ui.PB_Unit_Connect.clicked.connect(self.unit_connect)
        self.ui.PB_Scale_Connect.clicked.connect(self.scale_connect)

        # Start and Stop buttons
        self.ui.PB_Start_Stop.setEnabled(True)
        self.ui.PB_Start_Stop.clicked.connect(self.test_param_import)

        icon = QtGui.QIcon()
        icon2 = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icon/connect.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon2.addPixmap(QtGui.QPixmap("icon/disconnect.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon.addPixmap(QtGui.QPixmap("icon/connect.png"), QtGui.QIcon.Selected, QtGui.QIcon.Off)
        icon2.addPixmap(QtGui.QPixmap("icon/disconnect.png"), QtGui.QIcon.Selected, QtGui.QIcon.On)

        # Oulling Data from GUI
        # self.updateCoolDownTime()
        self.ui.DSB_CoolTime.valueChanged.connect(self.update_cool_down_time)

    def update_mode_combo(self):
        self.ui.CB_Mode.clear()
        if self.ui.CB_SKU.currentText() == "CFP300":
            self.ui.CB_Mode.addItems(['Coffee', 'K-Cup', 'Hot Water', 'Clean'])
        elif self.ui.CB_SKU.currentText() == "CFP200":  # CP300
            self.ui.CB_Mode.addItems(['Coffee', 'K-Cup', 'Clean'])
        elif self.ui.CB_SKU.currentText() == "CM400":  # CM400
            self.ui.CB_Mode.addItems(['Coffee', 'Clean'])

    # Update styles based on SKU and Mode
    def update_style_combo(self):
        self.ui.CB_Style.clear()

        # Coffee
        if self.ui.CB_Mode.currentText() == "Coffee":
            if self.ui.CB_SKU.currentText() == "CFP300":
                self.ui.CB_Style.addItems(
                    ['Classic', 'Rich', 'Over Ice', 'Specialty'])
            elif self.ui.CB_SKU.currentText() == "CFP200":
                self.ui.CB_Style.addItems(['Classic', 'Over Ice', 'Rich'])
            elif self.ui.CB_SKU.currentText() == "CM400":
                self.ui.CB_Style.addItems(['Classic', 'Over Ice', 'Rich', 'Specialty'])

        # K-cup
        elif self.ui.CB_Mode.currentText() == "K-Cup":
            if self.ui.CB_SKU.currentText() == "CFP300":
                self.ui.CB_Style.addItems(['Classic', 'Rich', 'Over Ice', 'Specialty'])
            elif self.ui.CB_SKU.currentText() == "CFP200":
                self.ui.CB_Style.addItems(['Classic', 'Rich', 'Over Ice'])

        # Hot water
        elif self.ui.CB_Mode.currentText() == "Hot Water":
            self.ui.CB_Style.addItems(
                ['Hot', 'Boil'])

        # Clean
        elif self.ui.CB_Mode.currentText() == "Clean":
            self.ui.CB_Size.addItems(['Full Carafe'])

    # Update sizes based on SKU, mode and style
    def update_size_combo(self):
        self.ui.CB_Size.clear()
        style = self.ui.CB_Style.currentText()

        # CFP300 and CFP200
        if self.ui.CB_SKU.currentText() == "CFP300" or "CFP200":
            # Coffee
            if self.ui.CB_Mode.currentText() == "Coffee":
                if style == "Classic" or "Over Ice":
                    # print("Classic: Classic")
                    self.ui.CB_Size.addItems(['8oz', '10oz', '12oz', '14oz', '18oz', '28oz', '35oz', '45oz', '56oz'])
                if style == "Rich":
                    self.ui.CB_Size.clear()
                    # print("Rich: RICH")
                    self.ui.CB_Size.addItems(['7oz', '10oz', '13oz', '16oz', '25oz', '32oz', '40oz', '49oz'])
                if style == "Specialty":
                    # print("Specialty: Specialty")
                    self.ui.CB_Size.clear()
                    self.ui.CB_Size.addItems(['4oz'])
            # K-Cup
            elif self.ui.CB_Mode.currentText() == "K-Cup":
                if self.ui.CB_Style.currentText() == "Classic" or "Over Ice":
                    self.ui.CB_Size.clear()

                    self.ui.CB_Size.addItems(['6oz', '8oz', '10oz', '12oz'])
                if self.ui.CB_Style.currentText() == "Rich":
                    self.ui.CB_Size.clear()
                    self.ui.CB_Size.addItems(['5oz', '7oz', '9oz', '11oz'])
                if self.ui.CB_Style.currentText() == "Specialty":
                    self.ui.CB_Size.clear()
                    self.ui.CB_Size.addItem('4oz')

            # Hot Water
            elif self.ui.CB_Mode.currentText() == "Hot Water":
                if self.ui.CB_Style.currentText() == 'Hot' or 'Boil':
                    self.ui.CB_Size.addItems(
                        ['4oz', '6oz', '8oz', '10oz', '12oz', '16oz', '32oz', '40oz', '50oz', '57oz'])
            # clean
            elif self.ui.CB_Mode.currentText() == "Clean":
                self.ui.CB_Size.clear()
                self.ui.CB_Size.addItems(["Full Carafe"])

        # CM400
        elif self.ui.CB_SKU.currentText() == "CM400":
            # Coffee
            if self.ui.CB_Mode.currentText() == "Coffee":
                if self.ui.CB_Style.currentText() == "Classic" or "Rich" or "Over Ice":
                    self.ui.CB_Size.clear()
                    self.ui.CB_Size.addItems(['Cup', 'Cup XL', 'Travel', 'Travel XL', 'Half Carafe', 'Full Carafe'])

                elif self.ui.CB_Style.currentText() == "Specialty":
                    self.ui.CB_Size.clear()
                    self.ui.CB_Size.addItem('Cup')

            elif self.ui.CB_Mode.currentText() == "Clean":
                self.ui.CB_Size.clear()
                self.ui.CB_Size.addItems(["Full Carafe"])

    # Serial connections
    def arduino_connect(self):
        global arduino_connect_flag, arduino_ser

        if arduino_connect_flag == 0:
            try:
                # serial communication between Pi and Arduino
                arduino_ser = serial.Serial(self.ui.LE_Arduino_Port.text(), 9600)
                arduino_ser.close()
                arduino_ser.open()

                logging.info('Arduino Connected: Port ' + self.ui.LE_Arduino_Port.text())
                self.ui.PB_Arduino_Connect.setText("Disconnect")
                self.ui.PB_Arduino_Connect.setStyleSheet(Styling.disconnect_button)
                arduino_connect_flag = 1
                self.ui.GB_Aux.setEnabled(True)

            except Exception as e:
                self.ui.PB_Arduino_Connect.setCheckable(False)
                QMessageBox.critical(self, "Error", str(e))
                logging.exception("Exception occurred", exc_info=True)

        else:
            try:
                logging.info('Arduino Disconnected: Port ' + self.ui.LE_Arduino_Port.text())
                self.ui.PB_Arduino_Connect.setText("Connect")
                self.ui.PB_Arduino_Connect.setStyleSheet(Styling.connect_button)
                arduino_connect_flag = 0
                self.ui.GB_Aux.setEnabled(False)

            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))
                logging.exception("Exception occurred", exc_info=True)

    def unit_connect(self):
        global unit_connect_flag, unit_ser

        if unit_connect_flag == 0:
            try:
                if self.ui.CB_SKU.currentText() == "CFP300":
                    self.ui.PB_Unit_Connect.setText("Disconnect")
                    self.ui.PB_Unit_Connect.setStyleSheet(Styling.disconnect_button)
                    logging.info(self.ui.CB_SKU.currentText() + ' Connected')
                    unit_connect_flag = 1
                    self.ui.PB_ReservoirDrain.setEnabled(True)
                    self.ui.PB_Start_Stop.setEnabled(True)
                    self.ui.GB_TestParam.setEnabled(True)
                else:
                    QMessageBox.critical(self, "Error", "Unit serial communication not configured")
                    self.ui.PB_Unit_Connect.setCheckable(False)

            except Exception as e:
                self.ui.PB_Unit_Connect.setCheckable(False)
                logging.exception("Exception occurred", exc_info=True)
                QMessageBox.critical(self, "Error", str(e))
        else:
            try:
                logging.info(self.ui.CB_SKU.currentText() + ' Disconnected')
                self.ui.PB_Unit_Connect.setStyleSheet(Styling.connect_button)
                self.ui.PB_Unit_Connect.setText("Connect")
                unit_connect_flag = 0
                self.ui.PB_ReservoirDrain.setEnabled(False)
                self.ui.PB_Start_Stop.setEnabled(False)
                self.ui.GB_TestParam.setEnabled(False)

            except Exception as e:
                logging.exception("Exception occurred", exc_info=True)
                QMessageBox.critical(self, "Error", str(e))

    def scale_connect(self):
        global scale_connect_flag, scale_ser
        if scale_connect_flag == 0:
            try:
                self.ui.PB_Scale_Connect.setText("Disconnect")
                self.ui.PB_Scale_Connect.setStyleSheet(Styling.disconnect_button)
                logging.info('Scale Connected')
                scale_connect_flag = 1

            except Exception as e:
                self.ui.PB_Scale_Connect.setCheckable(False)
                logging.exception("Exception occurred", exc_info=True)
                QMessageBox.critical(self, "Error", str(e))
        else:
            try:
                logging.info('Scale Disconnected')
                self.ui.PB_Scale_Connect.setStyleSheet(Styling.connect_button)
                self.ui.PB_Scale_Connect.setText("Connect")
                scale_connect_flag = 0

            except Exception as e:
                logging.exception("Exception occurred", exc_info=True)
                QMessageBox.critical(self, "Error", str(e))

    # File directory and Filename creation
    def file_manager(self, station, unit, current_cycle, filename_extra):
        # SKU and station info
        global summary_file, filename
        file_dir = os.path.join(data_dir, "{} {} Station {}".format(self.ui.CB_SKU.currentText(),
                                                                    self.ui.CB_Build.currentText(),
                                                                    str(int((self.ui.DSB_Station.value())))))

        if not os.path.exists(file_dir):
            os.makedirs(file_dir)

        # Macro and Unit info
        subfile_dir = os.path.join(file_dir, "#{} Macro {}".format(str(int((self.ui.DSB_Unit.value()))), str(
            int((self.ui.DSB_Macro.value())))))

        if not os.path.exists(subfile_dir):
            os.makedirs(subfile_dir)

        """ 
        Remove characters that don't follow file naming convention
        Do not use the following characters: / \: * ? " < > |
        """
        filename_extra = re.sub(r"([^\w])", "_", filename_extra)

        if filename_extra == "":
            # summary file name
            summary_file = os.path.join(file_dir, "{}_{}_Station{}_Unit{}.csv".format(self.ui.CB_SKU.currentText(),
                                                                                      self.ui.CB_Build.currentText(),
                                                                                      station, unit))

            # data file name
            filename = os.path.join(subfile_dir, "{}_{}_{}_Brew{}.csv".format(self.ui.CB_Mode.currentText(),
                                                                              self.ui.CB_Size.currentText(),
                                                                              self.ui.CB_Style.currentText(),
                                                                              current_cycle))

        else:
            # summary file name
            summary_file = os.path.join(file_dir, "{}_{}_Station{}_Unit{}_{}.csv".format(self.ui.CB_SKU.currentText(),
                                                                                         self.ui.CB_Build.currentText(),
                                                                                         station, unit, filename_extra))

            # data file name
            filename = os.path.join(subfile_dir, "{}_{}_{}_Brew{}_{}.csv".format(self.ui.CB_Mode.currentText(),
                                                                                 self.ui.CB_Size.currentText(),
                                                                                 self.ui.CB_Style.currentText(),
                                                                                 current_cycle, filename_extra))

        print('Data directory: {}'.format(data_dir))
        print('File directory: {}'.format(file_dir))
        print('Sub File directory: {}'.format(subfile_dir))
        print('Filename: {}'.format(filename))
        print('Summary Filename: {}'.format(summary_file))

        b = DataLogging()

        b.summary_file_log("Hello")
        b.raw_file_log("Hello")

    # Data Import - Get's the initial values from the Test parameters in the GUI
    def test_param_import(self):
        global sku, current_cycle, start_cycle
        sku = self.ui.CB_SKU.currentText()
        build = self.ui.CB_Build.currentText()
        station = int(self.ui.DSB_Station.value())
        unit = int(self.ui.DSB_Unit.value())
        mode = self.ui.CB_Mode.currentText()
        size = self.ui.CB_Size.currentText()
        style = self.ui.CB_Style.currentText()
        macro = self.ui.DSB_Macro.value()
        start_cycle = int(self.ui.DSB_StartCycle.value())
        number_of_cycles = int(self.ui.DSB_CycleCount.value())
        auto_shutoff_temp = self.ui.DSB_AutoshutoffTemp.value()
        boiler_cool_temp = self.ui.DSB_BoilerTemp.value()
        vessel_cool_temp = self.ui.DSB_VesselTemp.value()
        cool_time = self.ui.DSB_CoolTime.value()
        max_brew_time = self.ui.DSB_MaxBrewTime.value()
        filename_extra = self.ui.LE_Filename.text()
        receiver_email = self.ui.LE_Email.text().split(",")
        current_cycle = start_cycle

        test_param = "SKU: {},Build:{},Station:{}, Unit:{}, Mode:{}, Size:{}, Style:{}, Macro:{}, Start Cycle:{}," \
                     "Current Cycle:{}, Number of cycles:{}, Auto shutoff temp.:{}C, Boiler Cool temp.:{}C, " \
                     "Vessel Cool temp:{}C, Cool time:{}min., Max Brew time:{}min., Filename Extra:{}" \
            .format(sku, build, station, unit, mode, size, style,
                    macro, start_cycle, current_cycle, number_of_cycles, auto_shutoff_temp, boiler_cool_temp,
                    vessel_cool_temp, cool_time, max_brew_time, filename_extra)

        logging.info(test_param)
        self.file_manager(station, unit, current_cycle, filename_extra)
        self.start_end_brew()

    def update_cool_down_time(self):
        global cool_time
        cool_time = self.ui.DSB_CoolTime.value()
        print(cool_time)

    def start_end_brew(self):
        global start_stop_flag, current_cycle, start_cycle

        try:
            if start_stop_flag == 0:
                self.ui.GB_Serial.setEnabled(False)
                self.ui.GB_TestParam.setEnabled(False)
                self.ui.GB_Aux.setEnabled(False)
                self.ui.GB_Misc.setEnabled(False)
                a = ArduinoComm()
                a.arduino_pre_brew()
                print(start_cycle)
                logging.info('Brew {} Started'.format(current_cycle))
                start_stop_flag = 1

            else:
                logging.info('Brew {} Ended'.format(current_cycle))
                self.ui.GB_Serial.setEnabled(True)
                self.ui.GB_TestParam.setEnabled(True)
                self.ui.GB_Aux.setEnabled(True)
                self.ui.GB_Misc.setEnabled(True)
                start_stop_flag = 0

        except Exception as e:
            logging.exception("Exception occurred", exc_info=True)
            QMessageBox.critical(self, "Error", str(e))

    # Sends stop signal to the unit and the arduino to stop/disable all functions
    def stop_everything(self):
        QMessageBox.critical(self, "Application stopped", "Application stopped")
        logging.info("Application stopped")

    # Quit button - Close application
    @staticmethod
    def quit():
        # self.stop_everything()
        logging.info('Application closed')
        QtWidgets.qApp.closeAllWindows()


# Data logging
class DataLogging(object):
    @staticmethod
    def raw_file_log(data):
        global filename, filename_extra
        try:
            print(filename)
            # data = data.split(',')
            if os.path.exists(filename):
                with open(filename, 'a', newline='') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter=',',
                                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    filewriter.writerow(data)

            else:
                # Create and append
                with open(filename, 'w', newline='') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter=',',
                                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    filewriter.writerow(['Time', 'Boiler temp.', 'Outlet temp.', 'PTC temp.', 'Flow rate', '1 cup temp',
                                         '3 cup temp', '5 cup temp', '7 cup temp', 'Brew weight', 'Error code'])
                    filewriter.writerow(data)

        except Exception:
            logging.exception("Exception occurred", exc_info=True)

    @staticmethod
    def summary_file_log(data):
        global summary_file, filename_extra
        try:
            print(summary_file)
            # data = data.split(',')
            if os.path.exists(summary_file):
                with open(summary_file, 'a', newline='') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter=',',
                                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    filewriter.writerow(data)

            else:
                # Create and append
                with open(summary_file, 'w', newline='') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter=',',
                                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    filewriter.writerow(
                        ['Brew Start time', 'Brew end time', 'Mode', 'Size', 'Style', 'Macro', 'Brew cycle',
                         'Max. Boiler temp (degC)', 'Max. Outlet temp (degC)', 'Initial weight (g)', 'Final weight (g)',
                         'Brew Volume (g)', 'Brew Volume (oz.)', 'Max. Ambient temperature (degC)',
                         'Max. Ambient humidity', 'Brew Status'])
                    filewriter.writerow(data)

        except Exception:
            logging.exception("Exception occurred", exc_info=True)


# Email
def email_send(email_message):
    try:
        port = 465  # For SSL
        smtp_server = "smtp.gmail.com"
        sender_email = "sninja.test@gmail.com"
        password = '$#!N*&!0'

        message = email_message
        context = ssl.create_default_context()
        with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
            server.login(sender_email, password)
            for i in range(0, len(receiver_email)):
                server.sendmail(sender_email, receiver_email[i], message)
                print(receiver_email[i])
    except KeyboardInterrupt:
        print("1. Program Stopped - Keyboard Interrupt")
        sys.exit(1)

    except Exception:
        logging.exception("Exception occurred", exc_info=True)


if __name__ == "__main__":

    if sys.flags.interactive != 1:

        parent_dir = os.path.dirname(__file__)  # Current directory
        data_dir = os.path.join(parent_dir, "RAW")  # RAW data folder
        log_dir = os.path.join(parent_dir, "Log")  # Log files folder

        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Logging
        log_file = os.path.join(log_dir, "log" + str(time.strftime("-%m-%d-%Y--%I-%M-%S %p")) + ".log")
        logging.basicConfig(filename=log_file, filemode='w', level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')
        logging.getLogger().addHandler(logging.StreamHandler())

        app = QtWidgets.QApplication(sys.argv)
        app.processEvents()
        program = MainWindow()

        # Stylesheet
        style_dir = os.path.join(parent_dir, "style")
        style_file = os.path.join(style_dir, "style.qss")
        file = open(style_file)
        with file:
            qss = file.read()
            app.setStyleSheet(qss)

        program.setMinimumSize(810, 850)
        program.show()
        app.exec_()

# energy_monitor_pyqt.py
# Version: 3.2.12

import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QLabel, QComboBox,
                             QPushButton, QCheckBox, QTextEdit, QGroupBox,
                             QLineEdit, QRadioButton, QSplitter, QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
import pyqtgraph as pg
import time
import numpy as np
import qdarkstyle
from collections import deque

VERSION = "3.2.12"
VREF_VOLTS = 2.5
ADS1256_MAX_VALUE = 8388607.0

class SerialReader(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, serial_instance):
        super().__init__()
        self.serial_instance = serial_instance
        self._is_running = True

    def run(self):
        while self._is_running and self.serial_instance and self.serial_instance.is_open:
            try:
                if self.serial_instance.in_waiting > 0:
                    line = self.serial_instance.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                break
            time.sleep(0.01)

    def stop(self):
        self._is_running = False

class ADS1256Interface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS1256 AC Waveform Analyzer")
        self.setGeometry(100, 100, 1400, 800)

        self.serial_port = None
        self.serial_thread = None
        self.is_receiving_samples = False
        self.raw_sample_data = ""
        self.offsets = []

        self.command_queue = deque()
        self.is_applying_settings = False

        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        main_splitter = QSplitter(Qt.Horizontal)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(5, 5, 5, 5)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.right_splitter = QSplitter(Qt.Vertical)

        pg.setConfigOption('background', None)
        pg.setConfigOption('foreground', '#f8f8f2')
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.getAxis('left').setPen(pg.mkPen(color='#f8f8f2'))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen(color='#f8f8f2'))
        self.plot_widget.setLabel('left', 'Voltage', units='V')
        self.plot_widget.setLabel('bottom', 'Time', units='s')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        self.waveform_plot = self.plot_widget.plot(pen=pg.mkPen('#8be9fd', width=2))
        self.mean_line = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('#50fa7b', style=Qt.DashLine))
        self.mean_line.setVisible(False)
        self.plot_widget.addItem(self.mean_line)
        self.crosshair_v = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('yellow', style=Qt.DashLine))
        self.crosshair_h = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('yellow', style=Qt.DashLine))
        self.value_label = pg.TextItem(anchor=(0, 1))
        self.plot_widget.addItem(self.crosshair_v, ignoreBounds=True)
        self.plot_widget.addItem(self.crosshair_h, ignoreBounds=True)
        self.plot_widget.addItem(self.value_label, ignoreBounds=True)
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_move)

        self.right_splitter.addWidget(self.plot_widget)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setVisible(True)
        self.right_splitter.addWidget(self.console)

        right_layout.addWidget(self.right_splitter)
        self.right_splitter.setStretchFactor(0, 2)
        self.right_splitter.setStretchFactor(1, 2)

        left_layout.addWidget(QLabel(f"Version {VERSION}"))
        self.create_connection_group(left_layout)
        self.create_status_group(left_layout)
        self.create_settings_group(left_layout)
        self.create_commands_group(left_layout)
        self.create_view_group(left_layout)
        left_layout.addStretch()

        main_splitter.addWidget(left_panel)
        main_splitter.addWidget(right_panel)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 4)

        main_layout.addWidget(main_splitter)

        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.update_countdown)

    def on_mouse_move(self, pos):
        if self.plot_widget.sceneBoundingRect().contains(pos):
            mouse_point = self.plot_widget.getPlotItem().vb.mapSceneToView(pos)
            x, y = mouse_point.x(), mouse_point.y()

            self.crosshair_v.setPos(x)
            self.crosshair_h.setPos(y)

            x_data, y_data = self.waveform_plot.getData()
            if x_data is not None and len(x_data) > 0:
                index = np.abs(x_data - x).argmin()
                px, py = x_data[index], y_data[index]

                view_range = self.plot_widget.getPlotItem().vb.viewRange()
                y_span = view_range[1][1] - view_range[1][0]
                if abs(py - y) < y_span * 0.05:
                    html = f"<div style='background-color: #44475a; border: 1px solid #6272a4; padding: 5px; color: #f8f8f2;'>"
                    html += f"Time: {px:.4f} s<br>"
                    html += f"Voltage: {py:.4f} V"
                    html += "</div>"
                    self.value_label.setHtml(html)
                    self.value_label.setPos(px, py)
                else:
                    self.value_label.setText("")
            else:
                self.value_label.setText("")

    def create_connection_group(self, layout):
        group = QGroupBox("Connection")
        grid = QGridLayout()
        self.port_combo = QComboBox()
        grid.addWidget(self.port_combo, 0, 0, 1, 2)
        refresh_btn = QPushButton("🔄")
        refresh_btn.setFixedWidth(40)
        refresh_btn.clicked.connect(self.refresh_ports)
        grid.addWidget(refresh_btn, 0, 2)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        grid.addWidget(self.connect_btn, 1, 0)
        self.led = QLabel("●")
        self.led.setStyleSheet("color: #ff5555; font-size: 24px;")
        grid.addWidget(self.led, 1, 1, Qt.AlignCenter)
        group.setLayout(grid)
        layout.addWidget(group)

    def create_status_group(self, layout):
        group = QGroupBox("Status")
        vbox = QVBoxLayout()
        self.status_label = QLabel("Not connected.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #8be9fd;")
        vbox.addWidget(self.status_label)

        self.countdown_label = QLabel("")
        self.countdown_label.setStyleSheet("font-weight: bold; color: #ffb86c;")
        self.countdown_label.setVisible(False)
        vbox.addWidget(self.countdown_label)

        group.setLayout(vbox)
        layout.addWidget(group)

    def create_settings_group(self, layout):
        group = QGroupBox("ADS Settings")
        grid = QGridLayout()
        grid.addWidget(QLabel("Gain:"), 0, 0)
        self.gain_combo = QComboBox()
        self.gain_combo.addItems(["1", "2", "4", "8", "16", "32", "64"])
        grid.addWidget(self.gain_combo, 0, 1)

        grid.addWidget(QLabel("Data Rate (SPS):"), 1, 0)
        self.datarate_combo = QComboBox()
        self.datarate_combo.addItems(["30000", "15000", "7500", "3750", "2000", "1000", "500", "100", "60", "50", "30", "25", "15", "10", "5", "2.5"])
        self.datarate_combo.setCurrentText("30000")
        grid.addWidget(self.datarate_combo, 1, 1)

        self.buffer_check = QCheckBox("Buffer ON")
        self.buffer_check.setChecked(True)
        grid.addWidget(self.buffer_check, 2, 0, 1, 2)
        
        self.calibration_check = QCheckBox("Calibration")
        self.calibration_check.setChecked(True)
        grid.addWidget(self.calibration_check, 3, 0, 1, 2)

        self.apply_settings_btn = QPushButton("Apply Settings")
        self.apply_settings_btn.clicked.connect(self.apply_settings)
        grid.addWidget(self.apply_settings_btn, 4, 0, 1, 2)
        
        group.setLayout(grid)
        layout.addWidget(group)

    def create_commands_group(self, layout):
        group = QGroupBox("Commands")
        vbox = QVBoxLayout()
        self.ac_measure_btn = QPushButton("Start Monitoring")
        self.ac_measure_btn.setStyleSheet("color: #50fa7b;")
        self.ac_measure_btn.setEnabled(False)
        self.ac_measure_btn.clicked.connect(self.request_ac_measurement)
        vbox.addWidget(self.ac_measure_btn)

        self.reset_btn = QPushButton("Reset ADS")
        self.reset_btn.clicked.connect(lambda: self.send_command("RESET"))
        vbox.addWidget(self.reset_btn)

        self.clear_graph_btn = QPushButton("Clear Graph")
        self.clear_graph_btn.clicked.connect(self.clear_graph)
        vbox.addWidget(self.clear_graph_btn)

        clear_console_btn = QPushButton("Clear Console")
        clear_console_btn.clicked.connect(self.clear_console)
        vbox.addWidget(clear_console_btn)

        group.setLayout(vbox)
        layout.addWidget(group)

    def create_view_group(self, layout):
        group = QGroupBox("View")
        vbox = QVBoxLayout()
        self.console_check = QCheckBox("Show Console")
        self.console_check.setChecked(True)
        self.console_check.stateChanged.connect(lambda state: self.console.setVisible(state == Qt.Checked))
        vbox.addWidget(self.console_check)

        mean_layout = QHBoxLayout()
        self.show_mean_check = QCheckBox("Show Mean Value")
        self.show_mean_check.setChecked(False)
        self.show_mean_check.stateChanged.connect(self.toggle_mean_line)
        mean_layout.addWidget(self.show_mean_check)

        self.mean_value_label = QLabel("")
        self.mean_value_label.setStyleSheet("color: #50fa7b;")
        mean_layout.addWidget(self.mean_value_label)
        mean_layout.addStretch()
        
        vbox.addLayout(mean_layout)

        group.setLayout(vbox)
        layout.addWidget(group)

    def toggle_mean_line(self, state):
        self.mean_line.setVisible(state == Qt.Checked)
    
    def clear_console(self):
        self.console.clear()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread.wait()
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.setText("Connect")
            self.led.setStyleSheet("color: #ff5555; font-size: 24px;")
            self.set_ui_for_ready_state(False)
            self.status_label.setText("Disconnected.")
        else:
            port = self.port_combo.currentText()
            if not port: return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.serial_thread = SerialReader(self.serial_port)
                self.serial_thread.data_received.connect(self.process_serial_data)
                self.serial_thread.start()
                self.connect_btn.setText("Disconnect")
                self.led.setStyleSheet("color: #50fa7b; font-size: 24px;")
                self.status_label.setText("Redémarrage de l'ESP requis...")
            except serial.SerialException as e:
                QMessageBox.critical(self, "Connection Error", f"Failed to connect to {port}:\n{e}")

    def request_ac_measurement(self):
        self.send_command("START")
        self.status_label.setText("Mesure de l'offset...")
        self.clear_graph()
        QTimer.singleShot(500, self.start_main_countdown)
        
    def start_main_countdown(self):
        self.status_label.setText("Mesure AC en cours...")
        self.countdown_value = 10
        self.countdown_label.setText(f"Temps restant: {self.countdown_value}s")
        self.countdown_label.setVisible(True)
        self.countdown_timer.start(1000)

    def update_countdown(self):
        self.countdown_value -= 1
        self.countdown_label.setText(f"Temps restant: {self.countdown_value}s")
        if self.countdown_value <= 0:
            self.countdown_timer.stop()

    def process_serial_data(self, line):
        self.console.append(line)

        if "waiting for download" in line or "Connecting..." in line:
            self.set_ui_for_ready_state(False)
            self.status_label.setText("Redémarrer l'ESP")
            return

        if line == "START_DATA":
            self.is_receiving_samples = True
            self.raw_sample_data = ""
            self.offsets = []
            return

        if self.is_receiving_samples:
            if line.startswith("OFFSET"):
                self.offsets.append(int(line.split(':')[1]))
            elif line == "END_DATA":
                self.is_receiving_samples = False
                self.process_and_plot_samples()
                self.countdown_timer.stop()
                self.countdown_label.setVisible(False)
            else:
                self.raw_sample_data += line
        
        elif "ESP32 Ready" in line:
            if self.is_applying_settings:
                if self.command_queue:
                    self.send_next_command()
                else:
                    self.status_label.setText("Paramètres appliqués. Finalisation...")
                    QTimer.singleShot(1000, self.finalize_settings_application)
            else:
                self.set_ui_for_ready_state(True)
                self.status_label.setText("ESP32 Prêt.")

    def finalize_settings_application(self):
        """Finalise le processus d'application des paramètres après le délai."""
        self.is_applying_settings = False
        self.set_ui_for_ready_state(True)
        self.status_label.setText("ESP32 Prêt. Paramètres appliqués.")

    def process_and_plot_samples(self):
        try:
            samples_str = self.raw_sample_data.split(',')
            raw_samples = np.array([int(s) for s in samples_str if s])
            
            if not self.offsets:
                raise ValueError("Aucune valeur d'offset reçue.")
            final_offset = np.mean(self.offsets)

            gain = float(self.gain_combo.currentText())

            voltages = (raw_samples - final_offset) / ADS1256_MAX_VALUE * (VREF_VOLTS * 2.0) / gain
            
            sum_sq = np.sum(np.square(raw_samples - final_offset))
            mean_sq = sum_sq / len(raw_samples)
            rms_raw = np.sqrt(mean_sq)
            rms_voltage = (rms_raw / ADS1256_MAX_VALUE) * (VREF_VOLTS * 2.0) / gain
            
            self.status_label.setText(f"Mesure terminée. Tension RMS: {rms_voltage:.4f} V")
            
            time_axis = np.linspace(0, 10, len(voltages))
            
            self.waveform_plot.setData(time_axis, voltages)
            
            mean_voltage = np.mean(voltages)
            max_deviation = np.max(np.abs(voltages - mean_voltage))
            padding = max_deviation * 0.05
            
            self.plot_widget.setYRange(mean_voltage - max_deviation - padding, mean_voltage + max_deviation + padding)
            self.plot_widget.setXRange(0, 2, padding=0)

            self.mean_value_label.setText(f"({mean_voltage:.4f} V)")
            self.mean_line.setPos(mean_voltage)

        except (ValueError, IndexError) as e:
            self.status_label.setText(f"Erreur de traitement des données: {e}")
            print(f"Error processing data: {e}")
            
    def send_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            self.set_ui_for_ready_state(False)
            self.status_label.setText(f"Commande '{cmd}' envoyée...")
            self.send_raw_command(cmd)

    def send_raw_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            self.console.append(f"-> {cmd}")
            self.serial_port.write((cmd + '\n').encode('utf-8'))

    def send_next_command(self):
        if self.command_queue:
            command = self.command_queue.popleft()
            self.status_label.setText(f"Application de: '{command}'...")
            self.send_raw_command(command)
        
    def apply_settings(self):
        self.command_queue.clear()
        
        gain_cmd = f"GAIN:{self.gain_combo.currentText()}"
        self.command_queue.append(gain_cmd)

        drate_text = self.datarate_combo.currentText()
        sps_to_send = "2" if drate_text == "2.5" else drate_text
        drate_cmd = f"DRATE:{sps_to_send}"
        self.command_queue.append(drate_cmd)

        buffer_state = "ON" if self.buffer_check.isChecked() else "OFF"
        buffer_cmd = f"BUFFER:{buffer_state}"
        self.command_queue.append(buffer_cmd)

        if self.calibration_check.isChecked():
            self.command_queue.append("CAL:SELF")

        self.is_applying_settings = True
        self.set_ui_for_ready_state(False)
        self.send_next_command()

    def clear_graph(self):
        self.waveform_plot.clear()
        self.mean_line.setVisible(False)
        self.mean_value_label.setText("")

    def set_ui_for_ready_state(self, is_ready):
        self.ac_measure_btn.setEnabled(is_ready)
        self.apply_settings_btn.setEnabled(is_ready)
        self.reset_btn.setEnabled(is_ready)

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.toggle_connection()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    window = ADS1256Interface()
    window.show()
    sys.exit(app.exec_())
import json
import sys
import time

from PyQt5 import QtCore, QtNetwork, QtWidgets


TELEMETRY_PORT = 5005


class GroundStation(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cube Ground Station — Level 0")
        self.resize(520, 300)

        self.last_packet_time = None
        self.values = {}

        self.socket = QtNetwork.QUdpSocket(self)
        ok = self.socket.bind(
            QtNetwork.QHostAddress.LocalHost,
            TELEMETRY_PORT,
        )
        if not ok:
            raise RuntimeError(f"Could not listen on UDP port {TELEMETRY_PORT}")

        self.socket.readyRead.connect(self.receive_telemetry)

        self.status_label = QtWidgets.QLabel("Waiting for telemetry…")
        self.status_label.setStyleSheet("font-weight: bold; color: #b00020;")

        form = QtWidgets.QFormLayout()
        self.labels = {}

        for key, title in [
            ("sequence", "Sequence"),
            ("timestamp", "Timestamp"),
            ("mode", "Mode"),
            ("system_state", "System state"),
            ("alive", "Heartbeat"),
        ]:
            label = QtWidgets.QLabel("—")
            self.labels[key] = label
            form.addRow(title + ":", label)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel(
            f"Listening for UDP telemetry on 127.0.0.1:{TELEMETRY_PORT}"
        ))
        layout.addWidget(self.status_label)
        layout.addLayout(form)
        layout.addStretch()

        central = QtWidgets.QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self.timeout_timer = QtCore.QTimer(self)
        self.timeout_timer.timeout.connect(self.check_connection)
        self.timeout_timer.start(250)

    def receive_telemetry(self):
        while self.socket.hasPendingDatagrams():
            data, _, _ = self.socket.readDatagram(
                self.socket.pendingDatagramSize()
            )

            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            self.last_packet_time = time.monotonic()

            for key, label in self.labels.items():
                label.setText(str(packet.get(key, "—")))

            self.status_label.setText("Telemetry link: CONNECTED")
            self.status_label.setStyleSheet(
                "font-weight: bold; color: #137333;"
            )

    def check_connection(self):
        if (
            self.last_packet_time is None
            or time.monotonic() - self.last_packet_time > 2.0
        ):
            self.status_label.setText("Telemetry link: NO DATA")
            self.status_label.setStyleSheet(
                "font-weight: bold; color: #b00020;"
            )


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = GroundStation()
    window.show()
    sys.exit(app.exec_())
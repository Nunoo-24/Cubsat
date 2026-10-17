import json
import sys
import time
from datetime import datetime, timezone

from PyQt5 import QtCore, QtNetwork, QtWidgets

TELEMETRY_PORT = 5005
COMMAND_PORT = 5006


class GroundStation(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cube Ground Station — Level 0")
        self.resize(540, 330)

        self.last_packet_time = None
        self.command_count = 0

        self.socket = QtNetwork.QUdpSocket(self)
        if not self.socket.bind(
            QtNetwork.QHostAddress.LocalHost, TELEMETRY_PORT
        ):
            raise RuntimeError(f"Could not listen on UDP port {TELEMETRY_PORT}")

        self.socket.readyRead.connect(self.receive_telemetry)

        self.status_label = QtWidgets.QLabel("Telemetry link: NO DATA")
        self.status_label.setStyleSheet("font-weight: bold; color: #b00020;")

        form = QtWidgets.QFormLayout()
        self.labels = {}

        for key, title in [
            ("sequence", "Sequence"),
            ("timestamp", "Timestamp"),
            ("mode", "Mode"),
            ("system_state", "System state"),
            ("alive", "Heartbeat"),
            ("last_command", "Last command"),
            ("command_status", "Command status"),
        ]:
            self.labels[key] = QtWidgets.QLabel("—")
            form.addRow(title + ":", self.labels[key])

        button = QtWidgets.QPushButton("Send PING command")
        button.clicked.connect(self.send_ping)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel(
            f"Telemetry: UDP 127.0.0.1:{TELEMETRY_PORT} | "
            f"Commands: UDP 127.0.0.1:{COMMAND_PORT}"
        ))
        layout.addWidget(self.status_label)
        layout.addLayout(form)
        layout.addWidget(button)
        layout.addStretch()

        central = QtWidgets.QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.check_connection)
        timer.start(250)

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

    def send_ping(self):
        self.command_count += 1

        command = {
            "type": "telecommand",
            "command_id": self.command_count,
            "command": "PING",
            "timestamp": datetime.now(timezone.utc).isoformat(
                timespec="seconds"
            ),
        }

        self.socket.writeDatagram(
            json.dumps(command).encode("utf-8"),
            QtNetwork.QHostAddress.LocalHost,
            COMMAND_PORT,
        )

        self.labels["last_command"].setText("PING sent")
        self.labels["command_status"].setText("Awaiting acknowledgement")

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
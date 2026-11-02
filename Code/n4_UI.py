"""Exactly two image panels, status bar and keyboard control (focus window)."""
import cv2
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QPushButton

KEY_COMMANDS = {
    Qt.Key.Key_W:'W', Qt.Key.Key_S:'S', Qt.Key.Key_A:'A', Qt.Key.Key_D:'D'
}
class RobotDashboard(QMainWindow):
    def __init__(self, bus, state):
        super().__init__()
        self.state = state
        self.setWindowTitle('Smoke Robot — Pi Camera and Smoke Center')
        self.resize(1400, 720)
        self.raw = self._view('WAITING FOR PI CAMERA')
        self.marked = self._view('WAITING FOR DETECTION')
        root = QWidget()
        layout = QHBoxLayout(root)
        for name, label in [('CAMERA — ORIGINAL', self.raw), ('SMOKE CANDIDATE — CENTER', self.marked)]:
            panel = QVBoxLayout()
            title = QLabel(name)
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setStyleSheet('font-size:16px;font-weight:bold')
            panel.addWidget(title)
            panel.addWidget(label, 1)
            layout.addLayout(panel, 1)
        self.setCentralWidget(root)
        bar = QWidget()
        bar_layout = QHBoxLayout(bar)
        bar_layout.addWidget(QLabel('Roboflow API Key:'))
        self.key_field = QLineEdit()
        self.key_field.setPlaceholderText('Paste key here (hidden)')
        self.key_field.setEchoMode(QLineEdit.EchoMode.Password)
        bar_layout.addWidget(self.key_field)
        self.apply_button = QPushButton('Connect model')
        self.apply_button.clicked.connect(self.apply_key)
        bar_layout.addWidget(self.apply_button)
        layout_host = QWidget()
        outer = QVBoxLayout(layout_host)
        outer.addWidget(bar)
        outer.addWidget(root, 1)
        self.setCentralWidget(layout_host)
        self.last_raw = self.last_marked = None
        self.preview = 'NO TARGET'
        self.message = 'Starting...'
        bus.frame_ready.connect(self.frames)
        bus.control_preview.connect(self.preview_changed)
        bus.status.connect(self.status_changed)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(200)
        self.refresh_status()

    def apply_key(self):
        self.state.set_api_key(self.key_field.text())
        self.key_field.clear()
        self.message = 'Roboflow key submitted (not saved to disk)'

    @staticmethod
    def _view(text):
        label = QLabel(text)
        label.setMinimumSize(300, 220)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet('background:#050505;color:white')
        return label

    def status_changed(self, text): self.message = text
    def preview_changed(self, text): self.preview = text

    def refresh_status(self):
        s = self.state
        with s.lock:
            arm, auto, forward, connected = s.armed, s.auto, s.forward_allowed, s.connected
        cmd = s.command()
        self.statusBar().showMessage(
            f'{self.message} | Link:{"OK" if connected else "DOWN"} '
            f'| ARM:{"ON" if arm else "OFF"} | AUTO:{"ON" if auto else "OFF"} '
            f'| FORWARD:{"UNLOCKED" if forward else "LOCKED"} '
            f'| CLOUD AUTO:LATCHED (NO TIMER) | CMD:{cmd} | WHY:{s.block_reason} | {self.preview} | M:Arm  T:Auto  F:Forward  WASD:Manual  Space/X:STOP'
        )

    def frames(self, raw, marked, data):
        self.last_raw, self.last_marked = raw, marked
        self.draw(self.raw, raw)
        self.draw(self.marked, marked)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        if self.last_raw is not None:
            self.draw(self.raw, self.last_raw)
            self.draw(self.marked, self.last_marked)

    @staticmethod
    def draw(label, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(image)
        label.setPixmap(pix.scaled(label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                    Qt.TransformationMode.SmoothTransformation))

    def keyPressEvent(self, ev):
        k = ev.key()
        if ev.isAutoRepeat(): return
        if k in (Qt.Key.Key_Space, Qt.Key.Key_X):
            self.state.estop()
        elif k == Qt.Key.Key_M:
            self.state.toggle_arm()
        elif k == Qt.Key.Key_T:
            self.state.toggle_auto()
        elif k == Qt.Key.Key_F:
            self.state.toggle_forward()
        elif k in KEY_COMMANDS:
            self.state.set_manual(KEY_COMMANDS[k])
        else:
            super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev):
        if ev.isAutoRepeat(): return
        if ev.key() in KEY_COMMANDS:
            self.state.set_manual('X')
        else:
            super().keyReleaseEvent(ev)

    def closeEvent(self, ev):
        self.state.estop()
        super().closeEvent(ev)

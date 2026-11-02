from PyQt6.QtCore import QObject, pyqtSignal

class EventBus(QObject):
    frame_ready = pyqtSignal(object, object, object)
    status = pyqtSignal(str)
    control_preview = pyqtSignal(str)

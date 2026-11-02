import sys
from PyQt6.QtWidgets import QApplication
from event_bus import EventBus
from n1_main import RobotState, CameraWorker, ControlWorker
from n4_UI import RobotDashboard

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.137.250'
    app = QApplication(sys.argv)
    bus = EventBus()
    state = RobotState()
    ui = RobotDashboard(bus, state)
    camera = CameraWorker(bus, state, host)
    control = ControlWorker(bus, state, host)
    def shutdown():
        state.estop()
        control.close()
        camera.close()
    app.aboutToQuit.connect(shutdown)
    ui.show()
    control.start()
    camera.start()
    return app.exec()

if __name__ == '__main__':
    sys.exit(main())

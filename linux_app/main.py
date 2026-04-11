import sys
import os

# Add project dir to path so imports work
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt6.QtWidgets import QApplication
from audio_controller import AudioController
from config import ConfigManager
from ui.main_window import MainWindow
from ui.tray_icon import TrayIcon
from constants import APP_NAME


def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setQuitOnLastWindowClosed(False)

    config_manager = ConfigManager()
    audio_controller = AudioController()

    window = MainWindow(audio_controller, config_manager)
    tray = TrayIcon(window)
    tray.show()
    window.show()

    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

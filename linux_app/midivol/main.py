import sys

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QIcon
from importlib.resources import files

from .audio_controller import AudioController
from .config import ConfigManager
from .ui.main_window import MainWindow
from .ui.tray_icon import TrayIcon
from .constants import APP_NAME


def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setQuitOnLastWindowClosed(False)

    # Set app icon (window titlebar + taskbar)
    icon_path = str(files("midivol.resources").joinpath("midivol.svg"))
    app.setWindowIcon(QIcon(icon_path))

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

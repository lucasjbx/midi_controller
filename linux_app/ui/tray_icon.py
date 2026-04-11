from PyQt6.QtWidgets import QSystemTrayIcon, QMenu
from PyQt6.QtGui import QIcon, QAction
from PyQt6.QtWidgets import QApplication


class TrayIcon(QSystemTrayIcon):
    def __init__(self, parent_window):
        super().__init__(parent_window)
        self._window = parent_window
        self.setIcon(QIcon.fromTheme("audio-volume-medium"))
        self.setToolTip("MIDIVol")
        self._build_menu()
        self.activated.connect(self._on_activated)

    def _build_menu(self):
        menu = QMenu()
        show_action = QAction("Show", menu)
        show_action.triggered.connect(self._show_window)
        menu.addAction(show_action)

        menu.addSeparator()

        quit_action = QAction("Quit", menu)
        quit_action.triggered.connect(self._quit)
        menu.addAction(quit_action)

        self.setContextMenu(menu)

    def _show_window(self):
        self._window.show()
        self._window.activateWindow()

    def _quit(self):
        self._window.cleanup()
        QApplication.quit()

    def _on_activated(self, reason):
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self._show_window()

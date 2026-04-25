import os
import sys
import subprocess

from PyQt6.QtWidgets import QDialog, QVBoxLayout, QTextEdit, QPushButton, QHBoxLayout
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QTextCursor


PACKAGE_URL = "git+https://github.com/lucasjbx/midi_controller.git#subdirectory=linux_app"


class _UpdateWorker(QThread):
    output = pyqtSignal(str)
    finished = pyqtSignal(bool)

    def run(self):
        pip = os.path.join(os.path.dirname(sys.executable), "pip")
        cmd = [pip, "install", "--upgrade", PACKAGE_URL]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.output.emit(line.rstrip())
            proc.wait()
            self.finished.emit(proc.returncode == 0)
        except Exception as e:
            self.output.emit(f"Error: {e}")
            self.finished.emit(False)


class UpdateDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Update MIDIVol")
        self.setMinimumSize(520, 320)

        layout = QVBoxLayout(self)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFontFamily("monospace")
        layout.addWidget(self._log)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self._close_btn = QPushButton("Close")
        self._close_btn.clicked.connect(self.reject)
        self._restart_btn = QPushButton("Restart now")
        self._restart_btn.setVisible(False)
        self._restart_btn.clicked.connect(self._restart)
        btn_row.addWidget(self._close_btn)
        btn_row.addWidget(self._restart_btn)
        layout.addLayout(btn_row)

        self._worker = _UpdateWorker()
        self._worker.output.connect(self._append)
        self._worker.finished.connect(self._on_finished)
        self._append("Updating MIDIVol...")
        self._worker.start()

    def _append(self, line):
        self._log.append(line)
        self._log.moveCursor(QTextCursor.MoveOperation.End)

    def _on_finished(self, success):
        if success:
            self._append("\nDone! Restart to apply the update.")
            self._restart_btn.setVisible(True)
        else:
            self._append("\nUpdate failed.")

    def _restart(self):
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def closeEvent(self, event):
        if self._worker.isRunning():
            self._worker.wait()
        super().closeEvent(event)

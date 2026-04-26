import os
import re
import sys
import shutil
import subprocess
import urllib.request

from PyQt6.QtWidgets import QDialog, QVBoxLayout, QTextEdit, QPushButton, QHBoxLayout
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QTextCursor

from ..constants import APP_VERSION

PACKAGE_URL = "git+https://github.com/lucasjbx/midi_controller.git#subdirectory=linux_app"
REMOTE_TOML = "https://raw.githubusercontent.com/lucasjbx/midi_controller/main/linux_app/pyproject.toml"


def _fetch_remote_version():
    try:
        with urllib.request.urlopen(REMOTE_TOML, timeout=10) as r:
            content = r.read().decode()
        m = re.search(r'version\s*=\s*"([^"]+)"', content)
        return m.group(1) if m else None
    except Exception:
        return None


class _UpdateWorker(QThread):
    output = pyqtSignal(str)
    finished = pyqtSignal(bool)
    up_to_date = pyqtSignal()

    def run(self):
        remote = _fetch_remote_version()
        if remote is None:
            self.output.emit("Could not reach GitHub.")
            self.finished.emit(False)
            return
        if remote == APP_VERSION:
            self.up_to_date.emit()
            return
        self.output.emit(f"New version available: {remote} (current: {APP_VERSION})")
        self.output.emit("Installing update...")
        pipx = shutil.which("pipx")
        if not pipx:
            self.output.emit("Error: pipx not found.")
            self.finished.emit(False)
            return
        cmd = [pipx, "install", "--force", "--system-site-packages", PACKAGE_URL]
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
        self.setMinimumSize(520, 280)

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
        self._worker.up_to_date.connect(self._on_up_to_date)
        self._worker.finished.connect(self._on_finished)
        self._append(f"Current version: {APP_VERSION}\nChecking for updates...")
        self._worker.start()

    def _append(self, line):
        self._log.append(line)
        self._log.moveCursor(QTextCursor.MoveOperation.End)

    def _on_up_to_date(self):
        self._append(f"Already up to date ({APP_VERSION}).")

    def _on_finished(self, success):
        if success:
            self._append("\nUpdate complete! Restart to apply.")
            self._restart_btn.setVisible(True)
        else:
            self._append("\nUpdate failed.")

    def _restart(self):
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def closeEvent(self, event):
        if self._worker.isRunning():
            self._worker.wait()
        super().closeEvent(event)

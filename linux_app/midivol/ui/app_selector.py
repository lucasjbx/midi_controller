from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QListWidget, QListWidgetItem,
    QPushButton, QHBoxLayout, QLabel,
)
from PyQt6.QtCore import Qt


class AppSelectorDialog(QDialog):
    def __init__(self, available_apps, already_assigned, audio_controller, parent=None):
        super().__init__(parent)
        self._audio_controller = audio_controller
        self._already_assigned = [a.lower() for a in already_assigned]
        self.setWindowTitle("Select Apps")
        self.setMinimumWidth(350)
        self.setMinimumHeight(300)
        self._build_ui()
        self._populate(available_apps)

    def _build_ui(self):
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Running audio apps:"))

        self._list = QListWidget()
        layout.addWidget(self._list)

        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh)
        layout.addWidget(self._refresh_btn)

        btn_layout = QHBoxLayout()
        ok_btn = QPushButton("OK")
        cancel_btn = QPushButton("Cancel")
        ok_btn.clicked.connect(self.accept)
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)

    def _populate(self, apps):
        self._list.clear()
        for app in apps:
            item = QListWidgetItem(app.name)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            if app.name.lower() in self._already_assigned:
                item.setCheckState(Qt.CheckState.Checked)
            else:
                item.setCheckState(Qt.CheckState.Unchecked)
            self._list.addItem(item)

    def _refresh(self):
        apps = self._audio_controller.get_running_apps()
        selected = self.get_selected_apps()
        self._already_assigned = [a.lower() for a in selected]
        self._populate(apps)

    def get_selected_apps(self):
        result = []
        for i in range(self._list.count()):
            item = self._list.item(i)
            if item.checkState() == Qt.CheckState.Checked:
                result.append(item.text())
        return result

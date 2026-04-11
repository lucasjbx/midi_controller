from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QProgressBar,
    QListWidget, QPushButton, QHBoxLayout,
)
from PyQt6.QtCore import pyqtSignal, Qt


class PotColumn(QWidget):
    apps_changed = pyqtSignal(int, list)
    mute_toggled = pyqtSignal(int, bool)    # (cc_number, is_muted)
    learn_clicked = pyqtSignal(int)          # (cc_number)

    def __init__(self, cc_number, label_text, parent=None):
        super().__init__(parent)
        self.cc_number = cc_number
        self._muted = False
        self._mute_note = None
        self._build_ui(label_text)

    def _build_ui(self, label_text):
        layout = QVBoxLayout(self)

        self._title = QLabel(label_text)
        self._title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._title.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._title)

        self._bar = QProgressBar()
        self._bar.setRange(0, 127)
        self._bar.setValue(0)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(24)
        layout.addWidget(self._bar)

        self._pct_label = QLabel("0%")
        self._pct_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._pct_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self._pct_label)

        self._app_list = QListWidget()
        self._app_list.setMaximumHeight(120)
        layout.addWidget(self._app_list)

        btn_layout = QHBoxLayout()
        self._add_btn = QPushButton("Add Apps...")
        self._remove_btn = QPushButton("Remove")
        btn_layout.addWidget(self._add_btn)
        btn_layout.addWidget(self._remove_btn)
        layout.addLayout(btn_layout)

        # Mute button
        self._mute_btn = QPushButton("MUTE")
        self._mute_btn.setCheckable(True)
        self._mute_btn.setStyleSheet(
            "QPushButton { font-weight: bold; padding: 6px; }"
            "QPushButton:checked { background-color: #cc0000; color: white; }"
        )
        layout.addWidget(self._mute_btn)

        # Learn mode
        learn_layout = QHBoxLayout()
        self._note_label = QLabel("Btn: --")
        self._note_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        learn_layout.addWidget(self._note_label)
        self._learn_btn = QPushButton("Learn")
        learn_layout.addWidget(self._learn_btn)
        layout.addLayout(learn_layout)

        # Connections
        self._add_btn.clicked.connect(self._on_add_clicked)
        self._remove_btn.clicked.connect(self._on_remove_clicked)
        self._mute_btn.clicked.connect(self._on_mute_clicked)
        self._learn_btn.clicked.connect(self._on_learn_clicked)

    def set_add_callback(self, callback):
        self._add_callback = callback

    def _on_add_clicked(self):
        if hasattr(self, "_add_callback"):
            self._add_callback(self)

    def _on_remove_clicked(self):
        selected = self._app_list.currentRow()
        if selected >= 0:
            self._app_list.takeItem(selected)
            self.apps_changed.emit(self.cc_number, self.get_apps())

    def _on_mute_clicked(self):
        self._muted = self._mute_btn.isChecked()
        self.mute_toggled.emit(self.cc_number, self._muted)

    def _on_learn_clicked(self):
        self._learn_btn.setText("Press button...")
        self._learn_btn.setEnabled(False)
        self.learn_clicked.emit(self.cc_number)

    def finish_learn(self, note):
        self._mute_note = note
        self._note_label.setText(f"Btn: Note {note}")
        self._learn_btn.setText("Learn")
        self._learn_btn.setEnabled(True)

    def cancel_learn(self):
        self._learn_btn.setText("Learn")
        self._learn_btn.setEnabled(True)

    def set_muted(self, muted):
        self._muted = muted
        self._mute_btn.setChecked(muted)

    def is_muted(self):
        return self._muted

    def set_mute_note(self, note):
        self._mute_note = note
        if note is not None:
            self._note_label.setText(f"Btn: Note {note}")
        else:
            self._note_label.setText("Btn: --")

    def get_mute_note(self):
        return self._mute_note

    def update_volume(self, midi_value):
        self._bar.setValue(midi_value)
        pct = round(midi_value / 127 * 100)
        self._pct_label.setText(f"{pct}%")

    def set_apps(self, app_names):
        self._app_list.clear()
        for name in app_names:
            self._app_list.addItem(name)

    def get_apps(self):
        return [
            self._app_list.item(i).text()
            for i in range(self._app_list.count())
        ]

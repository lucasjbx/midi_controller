import os
import shutil

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QComboBox, QPushButton, QLabel, QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer

from ..constants import POT_CC_NUMBERS, POT_LABELS, POT_LED_CC, MUTE_ALL_CC, MIDI_MAX, APP_VERSION
from ..midi_worker import MidiWorker
from .pot_column import PotColumn
from .app_selector import AppSelectorDialog
from .update_dialog import UpdateDialog


class MainWindow(QMainWindow):
    def __init__(self, audio_controller, config_manager):
        super().__init__()
        self._audio = audio_controller
        self._config = config_manager
        self._midi_worker = None
        self._pot_columns = {}
        self._learn_target = None           # cc_number waiting for NoteOn
        self._last_volume = {}              # {cc: last_midi_value} for mute restore
        self._prev_master_vol = -1
        self._prev_master_mute = None
        self._master_timer = QTimer()
        self._master_timer.setInterval(500)
        self._master_timer.timeout.connect(self._poll_master_volume)
        self._build_ui()
        self._migrate_autostart_file()
        self._load_config()

    def _build_ui(self):
        self.setWindowTitle(f"MIDIVol v{APP_VERSION}")
        self.setMinimumSize(650, 500)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Top bar: MIDI port selector
        top = QHBoxLayout()
        top.addWidget(QLabel("MIDI Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(250)
        top.addWidget(self._port_combo)

        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        top.addWidget(self._refresh_btn)

        self._status_label = QLabel("Disconnected")
        self._status_label.setStyleSheet("color: red; font-weight: bold;")
        top.addWidget(self._status_label)
        top.addStretch()

        self._autostart_cb = QCheckBox("Start on login")
        self._autostart_cb.setChecked(os.path.exists(self._autostart_path()))
        self._autostart_cb.toggled.connect(self._on_autostart_toggled)
        top.addWidget(self._autostart_cb)

        update_btn = QPushButton("Update")
        update_btn.clicked.connect(self._open_update_dialog)
        top.addWidget(update_btn)

        main_layout.addLayout(top)

        # 3 pot columns
        cols_layout = QHBoxLayout()
        for cc, label in zip(POT_CC_NUMBERS, POT_LABELS):
            col = PotColumn(cc, label)
            col.set_add_callback(self._open_app_selector)
            col.apps_changed.connect(self._on_apps_changed)
            col.mute_toggled.connect(self._on_mute_toggled)
            col.learn_clicked.connect(self._on_learn_clicked)
            self._pot_columns[cc] = col
            self._last_volume[cc] = 0
            cols_layout.addWidget(col)
        main_layout.addLayout(cols_layout)

        # Connect port selector
        self._port_combo.currentTextChanged.connect(self._on_port_selected)
        self._refresh_ports()

    def _refresh_ports(self):
        current = self._port_combo.currentText()
        self._port_combo.blockSignals(True)
        self._port_combo.clear()
        self._port_combo.addItem("")
        ports = MidiWorker.list_ports()
        self._port_combo.addItems(ports)
        idx = self._port_combo.findText(current)
        if idx < 0:
            # Try prefix match if exact match fails
            idx = self._find_port_by_prefix(current)
        if idx >= 0:
            self._port_combo.setCurrentIndex(idx)
        self._port_combo.blockSignals(False)

    def _find_port_by_prefix(self, port_name):
        """Find combo index for port by prefix (ignoring ALSA jack number)."""
        if not port_name:
            return -1
        prefix = self._get_port_prefix(port_name)
        if not prefix:
            return -1
        for i in range(self._port_combo.count()):
            combo_text = self._port_combo.itemText(i)
            if self._get_port_prefix(combo_text) == prefix:
                return i
        return -1

    def _get_port_prefix(self, port_name):
        """Extract port name prefix without ALSA jack number (e.g., '28:0')."""
        if not port_name:
            return None
        # ALSA jack format is " XX:Y" at the end, e.g., " 28:0"
        parts = port_name.rsplit(" ", 1)
        if len(parts) == 2 and ":" in parts[1]:
            return parts[0]
        return port_name

    def _on_port_selected(self, port_name, save_config=True):
        self._stop_midi()
        if not port_name:
            self._set_status(False)
            return
        self._midi_worker = MidiWorker(port_name)
        self._midi_worker.cc_received.connect(self._on_cc_received)
        self._midi_worker.note_received.connect(self._on_note_received)
        self._midi_worker.connection_ok.connect(lambda: self._set_status(True))
        self._midi_worker.connection_lost.connect(lambda: self._set_status(False))
        self._midi_worker.start()
        if save_config:
            self._save_config()

    def _stop_midi(self):
        if self._midi_worker:
            self._midi_worker.stop()
            self._midi_worker = None

    def _set_status(self, connected):
        if connected:
            self._status_label.setText("Connected")
            self._status_label.setStyleSheet("color: green; font-weight: bold;")
            self._master_timer.start()
        else:
            self._status_label.setText("Disconnected")
            self._status_label.setStyleSheet("color: red; font-weight: bold;")
            self._master_timer.stop()

    # --- Master volume monitor ---

    def _poll_master_volume(self):
        if not self._midi_worker:
            return
        muted = self._audio.is_master_muted()
        if muted != self._prev_master_mute:
            self._prev_master_mute = muted
            self._midi_worker.send_cc(MUTE_ALL_CC, 1 if muted else 0)
        if not muted:
            vol = self._audio.get_master_volume()
            midi_val = int(min(vol, 1.0) * MIDI_MAX)
            if midi_val != self._prev_master_vol:
                self._prev_master_vol = midi_val
                self._midi_worker.send_cc(8, midi_val)

    # --- Volume ---

    def _on_cc_received(self, cc_number, value):
        col = self._pot_columns.get(cc_number)
        if not col:
            return
        self._last_volume[cc_number] = value
        if col.is_muted():
            # Pot moved while muted -> unmute
            self._toggle_mute(cc_number, False)
            return
        col.update_volume(value)
        volume = value / MIDI_MAX
        for app_name in col.get_apps():
            self._audio.set_volume(app_name, volume)

    # --- Mute ---

    def _on_mute_toggled(self, cc_number, is_muted):
        self._toggle_mute(cc_number, is_muted)

    def _toggle_mute(self, cc_number, mute):
        col = self._pot_columns.get(cc_number)
        if not col:
            return
        col.set_muted(mute)
        led_cc = POT_LED_CC.get(cc_number)

        if mute:
            # Mute: set volume 0, LED red
            for app_name in col.get_apps():
                self._audio.set_volume(app_name, 0.0)
            if self._midi_worker and led_cc is not None:
                self._midi_worker.send_cc(led_cc, 1)
        else:
            # Unmute: restore last volume, LED off
            restored = self._last_volume.get(cc_number, 0)
            col.update_volume(restored)
            volume = restored / MIDI_MAX
            for app_name in col.get_apps():
                self._audio.set_volume(app_name, volume)
            if self._midi_worker and led_cc is not None:
                self._midi_worker.send_cc(led_cc, 0)

    # --- Learn mode ---

    def _on_learn_clicked(self, cc_number):
        # Cancel any previous learn
        if self._learn_target is not None:
            old_col = self._pot_columns.get(self._learn_target)
            if old_col:
                old_col.cancel_learn()
        self._learn_target = cc_number

    def _on_note_received(self, note, velocity):
        if self._learn_target is not None:
            # Assign note to pot
            col = self._pot_columns.get(self._learn_target)
            if col:
                col.finish_learn(note)
            self._learn_target = None
            self._save_config()
        else:
            # Find pot with matching mute_note -> toggle
            for cc, col in self._pot_columns.items():
                if col.get_mute_note() == note:
                    self._toggle_mute(cc, not col.is_muted())
                    break

    # --- App selector ---

    def _open_app_selector(self, pot_column):
        apps = self._audio.get_running_apps()
        already = pot_column.get_apps()
        dialog = AppSelectorDialog(apps, already, self._audio, self)
        if dialog.exec():
            selected = dialog.get_selected_apps()
            pot_column.set_apps(selected)
            pot_column.apps_changed.emit(pot_column.cc_number, selected)

    def _on_apps_changed(self, cc_number, app_names):
        self._save_config()

    # --- Config ---

    def _save_config(self):
        port = self._port_combo.currentText()
        mappings = {}
        mute_notes = {}
        for cc, col in self._pot_columns.items():
            mappings[str(cc)] = col.get_apps()
            note = col.get_mute_note()
            if note is not None:
                mute_notes[str(cc)] = note
        self._config.save(port, mappings, mute_notes)

    def _load_config(self):
        data = self._config.load()

        # Block signals to prevent _on_port_selected from triggering _save_config
        # before mappings are loaded
        self._port_combo.blockSignals(True)

        saved_port = data.get("midi_port", "")
        port_found = False
        if saved_port:
            # Try exact match first
            idx = self._port_combo.findText(saved_port)
            if idx < 0:
                # Try prefix match (ALSA jack number may change)
                idx = self._find_port_by_prefix(saved_port)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)
                port_found = True

        mappings = data.get("mappings", {})
        for cc_str, apps in mappings.items():
            cc = int(cc_str)
            col = self._pot_columns.get(cc)
            if col:
                col.set_apps(apps)

        mute_notes = data.get("mute_notes", {})
        for cc_str, note in mute_notes.items():
            cc = int(cc_str)
            col = self._pot_columns.get(cc)
            if col:
                col.set_mute_note(note)

        self._port_combo.blockSignals(False)

        # Try to connect to the saved port
        if port_found:
            # Port is in combo, connect with config save
            self._on_port_selected(saved_port, save_config=True)
        elif saved_port:
            # Port not yet available (device not ready at boot), but try to connect anyway
            # Don't save config to avoid overwriting with empty port string
            # MidiWorker will retry connection attempts until device is ready
            self._on_port_selected(saved_port, save_config=False)

    # --- Autostart ---

    def _migrate_autostart_file(self):
        """Auto-migrate old .desktop files to use --autostart CLI arg."""
        path = self._autostart_path()
        if not os.path.exists(path):
            return
        with open(path, "r") as f:
            content = f.read()
        if "--autostart" not in content:
            # Rewrite with correct Exec line using --autostart arg
            self._on_autostart_toggled(True)

    @staticmethod
    def _autostart_path():
        return os.path.expanduser("~/.config/autostart/midivol.desktop")

    def _on_autostart_toggled(self, checked):
        path = self._autostart_path()
        if checked:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            midivol_bin = shutil.which("midivol") or "midivol"
            with open(path, "w") as f:
                f.write(
                    "[Desktop Entry]\n"
                    "Name=MIDIVol\n"
                    "Comment=MIDI volume controller for Linux\n"
                    f"Exec={midivol_bin} --autostart\n"
                    "Icon=midivol\n"
                    "Type=Application\n"
                    "X-GNOME-Autostart-enabled=true\n"
                )
        else:
            if os.path.exists(path):
                os.remove(path)

    def _open_update_dialog(self):
        dlg = UpdateDialog(self)
        dlg.exec()

    def closeEvent(self, event):
        event.ignore()
        self.hide()

    def cleanup(self):
        self._master_timer.stop()
        self._stop_midi()
        self._audio.close()

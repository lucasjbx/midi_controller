import time
import threading
import rtmidi
from PyQt6.QtCore import QThread, pyqtSignal

from .constants import POT_CC_NUMBERS, MIDI_CHANNEL


class MidiWorker(QThread):
    cc_received = pyqtSignal(int, int)      # (cc_number, value)
    note_received = pyqtSignal(int, int)    # (note, velocity)
    connection_lost = pyqtSignal()
    connection_ok = pyqtSignal()

    def __init__(self, port_name=None):
        super().__init__()
        self._port_name = port_name
        self._running = False
        self._midi_out = None
        self._out_lock = threading.Lock()

    @staticmethod
    def list_ports():
        midi_in = rtmidi.MidiIn()
        ports = midi_in.get_ports()
        del midi_in
        return ports

    def set_port(self, port_name):
        self._port_name = port_name

    def send_cc(self, cc_number, value):
        with self._out_lock:
            if self._midi_out:
                self._midi_out.send_message([0xB0 | MIDI_CHANNEL, cc_number, value])

    def run(self):
        self._running = True
        while self._running:
            midi_in = rtmidi.MidiIn()
            port_index = self._find_port(midi_in)
            if port_index is None:
                self.connection_lost.emit()
                self._wait(3.0)
                del midi_in
                continue

            try:
                midi_in.open_port(port_index)
                self._open_output()
                self.connection_ok.emit()
                while self._running:
                    msg = midi_in.get_message()
                    if msg:
                        data, _ = msg
                        self._parse(data)
                    else:
                        time.sleep(0.001)
            except Exception:
                self.connection_lost.emit()
            finally:
                midi_in.close_port()
                del midi_in
                self._close_output()

            if self._running:
                self._wait(3.0)

    def _open_output(self):
        with self._out_lock:
            try:
                self._midi_out = rtmidi.MidiOut()
                ports = self._midi_out.get_ports()
                if not self._port_name:
                    self._midi_out = None
                    return
                # Try exact match first
                for i, name in enumerate(ports):
                    if self._port_name == name:
                        self._midi_out.open_port(i)
                        return
                # If exact match fails, try prefix match (ALSA jack number may change)
                port_prefix = self._get_port_prefix(self._port_name)
                if port_prefix:
                    for i, name in enumerate(ports):
                        if self._get_port_prefix(name) == port_prefix:
                            self._midi_out.open_port(i)
                            return
                # No matching port found
                self._midi_out = None
            except Exception:
                self._midi_out = None

    def _close_output(self):
        with self._out_lock:
            if self._midi_out:
                self._midi_out.close_port()
                self._midi_out = None

    def _find_port(self, midi_in):
        if not self._port_name:
            return None
        ports = midi_in.get_ports()
        # Try exact match first
        for i, name in enumerate(ports):
            if self._port_name == name:
                return i
        # If exact match fails, try prefix match (ALSA jack number may change)
        # Extract prefix by removing the ALSA jack number at the end (e.g., " 28:0")
        port_prefix = self._get_port_prefix(self._port_name)
        if port_prefix:
            for i, name in enumerate(ports):
                if self._get_port_prefix(name) == port_prefix:
                    return i
        return None

    def _get_port_prefix(self, port_name):
        """Extract port name prefix without ALSA jack number (e.g., '28:0')."""
        if not port_name:
            return None
        # ALSA jack format is " XX:Y" at the end, e.g., " 28:0"
        parts = port_name.rsplit(" ", 1)
        if len(parts) == 2 and ":" in parts[1]:
            return parts[0]
        return port_name

    def _parse(self, data):
        if len(data) < 3:
            return
        status = data[0] & 0xF0
        channel = data[0] & 0x0F
        if channel != MIDI_CHANNEL:
            return
        if status == 0xB0:
            cc_number = data[1]
            value = data[2]
            if cc_number in POT_CC_NUMBERS:
                self.cc_received.emit(cc_number, value)
        elif status == 0x90 and data[2] > 0:
            self.note_received.emit(data[1], data[2])

    def _wait(self, seconds):
        end = time.time() + seconds
        while self._running and time.time() < end:
            time.sleep(0.1)

    def stop(self):
        self._running = False
        self.wait()

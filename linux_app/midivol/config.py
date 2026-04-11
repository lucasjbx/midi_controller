import json
import os

from .constants import DEFAULT_CONFIG_DIR, DEFAULT_CONFIG_PATH


class ConfigManager:
    def __init__(self, path=None):
        self._path = os.path.expanduser(path or DEFAULT_CONFIG_PATH)
        self._dir = os.path.expanduser(
            os.path.dirname(path) if path else DEFAULT_CONFIG_DIR
        )

    def load(self):
        if not os.path.exists(self._path):
            return {"midi_port": "", "mappings": {}, "mute_notes": {}}
        try:
            with open(self._path, "r") as f:
                data = json.load(f)
            data.setdefault("mute_notes", {})
            return data
        except (json.JSONDecodeError, IOError):
            return {"midi_port": "", "mappings": {}, "mute_notes": {}}

    def save(self, midi_port, mappings, mute_notes=None):
        os.makedirs(self._dir, exist_ok=True)
        data = {
            "midi_port": midi_port,
            "mappings": mappings,
            "mute_notes": mute_notes or {},
        }
        with open(self._path, "w") as f:
            json.dump(data, f, indent=2)

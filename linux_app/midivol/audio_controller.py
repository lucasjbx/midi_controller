from dataclasses import dataclass
from pulsectl import Pulse, PulseError


@dataclass
class AudioApp:
    sink_input_index: int
    name: str
    binary: str
    volume: float


class AudioController:
    def __init__(self):
        self._pulse = Pulse("midivol")

    def get_running_apps(self):
        apps = []
        seen = set()
        for si in self._pulse.sink_input_list():
            name = si.proplist.get("application.name", "Unknown")
            binary = si.proplist.get("application.process.binary", "")
            key = name.lower()
            if key in seen:
                continue
            seen.add(key)
            apps.append(AudioApp(
                sink_input_index=si.index,
                name=name,
                binary=binary,
                volume=si.volume.value_flat,
            ))
        return apps

    def set_volume(self, app_name, volume):
        volume = max(0.0, min(1.0, volume))
        for si in self._pulse.sink_input_list():
            si_name = si.proplist.get("application.name", "")
            si_binary = si.proplist.get("application.process.binary", "")
            if (app_name.lower() in si_name.lower()
                    or app_name.lower() in si_binary.lower()):
                self._pulse.volume_set_all_chans(si, volume)

    def get_master_volume(self):
        sink = self._pulse.server_info().default_sink_name
        for s in self._pulse.sink_list():
            if s.name == sink:
                return s.volume.value_flat
        return 0.0

    def is_master_muted(self):
        sink = self._pulse.server_info().default_sink_name
        for s in self._pulse.sink_list():
            if s.name == sink:
                return bool(s.mute)
        return False

    def close(self):
        self._pulse.close()

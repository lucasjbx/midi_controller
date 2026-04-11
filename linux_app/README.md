# MIDIVol

Linux desktop application for controlling per-application audio volume using a USB MIDI controller. Works with PipeWire and PulseAudio.

## Features

- **Per-app volume control**: Assign one or more applications to each potentiometer
- **Mute toggle**: Mute/unmute apps via physical buttons or the UI
- **MIDI Learn**: Click "Learn", press a button on the controller, done
- **LED feedback**: Muted channels light up red on the controller
- **Master volume display**: System volume shown on the controller's OLED display
- **System tray**: Runs in the background, minimizes to tray on close
- **Persistent config**: All settings saved to `~/.config/midivol/config.json`

## Requirements

### System packages

```bash
sudo apt install libasound2-dev python3-pyqt6
```

### Python packages (installed automatically)

- `python-rtmidi` (MIDI I/O)
- `pulsectl` (PipeWire/PulseAudio volume control)

PyQt6 must be installed system-wide (too large for pip in a venv).

## Installation

### Quick install

```bash
git clone https://github.com/lucasjbx/midi_controller.git
cd midi_controller/linux_app
chmod +x install.sh
./install.sh
```

This creates a virtual environment, installs MIDIVol, and adds a desktop entry.

### pip install (from cloned repo)

```bash
cd linux_app
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install .
```

### pipx install (directly from GitHub)

**Recommended for users** — no venv management, automatic PATH setup:

```bash
pipx install "midivol @ git+https://github.com/lucasjbx/midi_controller.git#subdirectory=linux_app"
```

If you don't have `pipx`: `sudo apt install pipx`

### Manual venv (alternative)

```bash
cd linux_app
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install .
```

## Usage

### Running

```bash
midivol
```

Or use the desktop entry after running `install.sh` (search for "MIDIVol" in your app menu).

### How to use

1. **Select MIDI port**: Pick your controller from the dropdown (look for "midi lucas")
2. **Assign apps**: Click "Add Apps..." on a pot column, check the apps you want
3. **Set mute button**: Click "Learn", then press a button on the controller
4. **Control volume**: Move the physical potentiometer to change volume
5. **Mute/Unmute**: Press the assigned button or click "MUTE" in the UI

### Closing

Clicking the window close button hides it to the system tray. Right-click the tray icon and select "Quit" to exit completely.

## Configuration

Settings are saved automatically to `~/.config/midivol/config.json`:

```json
{
  "midi_port": "midi lucas:midi lucas MIDI 1 20:0",
  "mappings": {
    "58": ["Spotify", "VLC media player"],
    "59": ["Firefox"],
    "60": ["Discord"]
  },
  "mute_notes": {
    "58": 3,
    "59": 5,
    "60": 6
  }
}
```

| Field | Description |
|---|---|
| `midi_port` | Selected MIDI port name |
| `mappings` | App names assigned to each pot (keyed by CC number) |
| `mute_notes` | Button NoteOn number assigned as mute toggle per pot |

## Troubleshooting

**No MIDI ports listed**: Make sure the controller is plugged in and recognized. Check `aconnect -l` or `amidi -l`.

**No audio apps in the selector**: An app must be actively playing audio to appear in the list. Start playback first, then click "Refresh".

**Volume not changing**: Verify the app name matches. MIDIVol uses case-insensitive substring matching on the PulseAudio application name.

**"Cannot connect to PulseAudio"**: Make sure PipeWire or PulseAudio is running. Check with `pactl info`.

APP_NAME = "MIDIVol"
APP_VERSION = "1.0.0"

# MIDI CC numbers from Pico W firmware (main.py lines 373-375)
POT_CC_NUMBERS = [58, 59, 60]
POT_LABELS = ["Pot 1 (CC 58)", "Pot 2 (CC 59)", "Pot 3 (CC 60)"]
MIDI_CHANNEL = 0
MIDI_MAX = 127

# LED feedback CC: pot CC -> controller LED CC (from firmware main.py:343-349)
POT_LED_CC = {58: 2, 59: 1, 60: 0}
MUTE_ALL_CC = 7

# Config
DEFAULT_CONFIG_DIR = "~/.config/midivol"
DEFAULT_CONFIG_PATH = "~/.config/midivol/config.json"

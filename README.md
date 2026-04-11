# MIDI Controller

A custom USB MIDI controller built with a Raspberry Pi Pico W and CircuitPython, featuring per-application volume control on Linux via **MIDIVol**.

## Features

**Hardware:**
- 3 slide potentiometers for volume control
- 11 programmable buttons (media keys, mute toggles, keyboard shortcuts)
- 2 rotary encoders (system volume, navigation)
- 9 NeoPixel LEDs (3 for pots, 6 for buttons) with visual feedback
- SSD1306 OLED display (128x64) showing real-time control values
- Boot splash animation

**Software (MIDIVol - Linux):**
- Per-application volume control via PipeWire/PulseAudio
- Assign multiple apps to each potentiometer
- Mute toggle per channel with LED feedback on the controller
- MIDI Learn mode: press a physical button to assign it as mute toggle
- System master volume displayed on the controller's OLED
- System tray integration (runs in background)
- Persistent configuration (auto-saved, restored on startup)

## Hardware

### Components

| Component | Specification |
|---|---|
| MCU | Raspberry Pi Pico W |
| ADC | ADS1015 (I2C, 12-bit) |
| Display | SSD1306 OLED 128x64 (I2C) |
| LEDs | 9x NeoPixel (WS2812B) |
| Potentiometers | 3x linear slide |
| Encoders | 2x rotary with push button |
| Buttons | 11x momentary push |

### Pin Mapping

| Component | GPIO |
|---|---|
| I2C (ADS1015 + SSD1306) | SCL=GP1, SDA=GP0 |
| Pot LEDs (3x NeoPixel) | GP6 |
| Button LEDs (6x NeoPixel) | GP5 |
| Encoder 1 (Volume) | GP17, GP18 |
| Encoder 2 (Navigation) | GP20, GP21 |
| Buttons | GP7, GP8, GP9, GP10, GP11, GP16, GP19, GP22, GP26, GP27, GP28 |

## MIDI Protocol

### Controller Output (Pico W -> PC)

| Message | Data | Description |
|---|---|---|
| CC 58 | 0-127 | Potentiometer 0 value |
| CC 59 | 0-127 | Potentiometer 1 value |
| CC 60 | 0-127 | Potentiometer 2 value |
| NoteOn N | velocity 100 | Button press (N = button key number) |

All messages sent on MIDI Channel 0.

### Feedback Input (PC -> Pico W)

| Message | Value | Effect |
|---|---|---|
| CC 0 | 1/0 | Pot 2 LED on (red) / off |
| CC 1 | 1/0 | Pot 1 LED on (red) / off |
| CC 2 | 1/0 | Pot 0 LED on (red) / off |
| CC 7 | 1/0 | Global mute: all LEDs red / all off |
| CC 8 | 0-127 | Display master volume (0-100%) on OLED |

### HID Output

| Button | Action |
|---|---|
| Button 3 | Consumer Control: Mute |
| Button 5 | Consumer Control: Play/Pause |
| Button 7 | Consumer Control: Next Track |
| Button 10 | Consumer Control: Previous Track |
| Button 8 | Keyboard: F13 |
| Button 6 | Keyboard: F14 |
| Button 9 | Keyboard: F15 |
| Encoder 1 | Consumer Control: Volume Up/Down |

## Firmware

The firmware runs CircuitPython 9.2.7 on the Raspberry Pi Pico W.

### Setup

1. Flash CircuitPython 9.2.7 on the Pico W ([download](https://circuitpython.org/board/raspberry_pi_pico_w/))
2. Copy all files from `firmware/` to the CIRCUITPY drive
3. Install required Adafruit libraries to `lib/` on the CIRCUITPY drive:
   - `adafruit_ads1x15`
   - `adafruit_midi`
   - `adafruit_hid`
   - `adafruit_displayio_ssd1306`
   - `adafruit_display_text`
   - `adafruit_display_shapes`
   - `adafruit_bitmap_font`
   - `adafruit_imageload`
   - `adafruit_simplemath`
   - `neopixel`
   - `asyncio`

### Boot Safety

GPIO22 acts as a safety button during boot:
- **Pressed**: USB drive + serial console enabled (for development)
- **Released**: Both disabled (prevents accidental file edits)

## Linux App (MIDIVol)

See [linux_app/README.md](linux_app/README.md) for installation and usage instructions.

## Windows

On Windows, this controller works with [MIDI Mixer](https://www.midi-mixer.com/).

## License

MIT

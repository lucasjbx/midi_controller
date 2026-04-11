#!/bin/bash
set -e

REPO="https://github.com/lucasjbx/midi_controller"
RAW="https://raw.githubusercontent.com/lucasjbx/midi_controller/main"
DESKTOP_DIR="$HOME/.local/share/applications"
ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
AUTOSTART_DIR="$HOME/.config/autostart"

echo "=== MIDIVol Installer ==="

# System dependencies
echo "[1/4] Installing system dependencies..."
sudo apt install -y python3-pyqt6 libasound2-dev pipx

# Install MIDIVol via pipx
echo "[2/4] Installing MIDIVol..."
pipx install --system-site-packages \
  "midivol @ git+$REPO.git#subdirectory=linux_app"

# Install icon
echo "[3/4] Installing icon..."
mkdir -p "$ICON_DIR"
curl -fsSL "$RAW/linux_app/midivol/resources/midivol.svg" \
  -o "$ICON_DIR/midivol.svg"

# Desktop entry + autostart
echo "[4/4] Creating desktop entries..."
MIDIVOL_BIN="$(which midivol)"
mkdir -p "$DESKTOP_DIR" "$AUTOSTART_DIR"

cat > "$DESKTOP_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=$MIDIVOL_BIN
Icon=midivol
Type=Application
Categories=AudioVideo;Audio;
Keywords=midi;volume;mixer;audio;
EOF

cat > "$AUTOSTART_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=env DESKTOP_AUTOSTART_ID=midivol $MIDIVOL_BIN
Icon=midivol
Type=Application
X-GNOME-Autostart-enabled=true
EOF

echo ""
echo "Done! Run: midivol"
echo "Or search for 'MIDIVol' in the app menu."
echo "MIDIVol will start automatically on login (disable in app settings)."

#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DESKTOP_DIR="$HOME/.local/share/applications"
ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"

echo "=== MIDIVol Installer ==="

# Create virtual environment
echo "[1/4] Creating virtual environment..."
python3 -m venv --system-site-packages "$SCRIPT_DIR/.venv"

# Install package
echo "[2/4] Installing MIDIVol..."
"$SCRIPT_DIR/.venv/bin/pip" install --quiet "$SCRIPT_DIR"

# Install icon
echo "[3/4] Installing icon..."
mkdir -p "$ICON_DIR"
cp "$SCRIPT_DIR/midivol/resources/midivol.svg" "$ICON_DIR/midivol.svg"

# Create desktop entry
echo "[4/4] Creating desktop entry..."
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=$SCRIPT_DIR/.venv/bin/midivol
Icon=midivol
Type=Application
Categories=AudioVideo;Audio;
Keywords=midi;volume;mixer;audio;
EOF

echo ""
echo "Done! You can now:"
echo "  - Run from terminal:  midivol  (if venv is active)"
echo "  - Run from app menu:  Search for 'MIDIVol'"

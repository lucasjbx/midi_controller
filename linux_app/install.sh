#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DESKTOP_DIR="$HOME/.local/share/applications"

echo "=== MIDIVol Installer ==="

# Create virtual environment
echo "[1/3] Creating virtual environment..."
python3 -m venv --system-site-packages "$SCRIPT_DIR/.venv"

# Install Python dependencies
echo "[2/3] Installing dependencies..."
"$SCRIPT_DIR/.venv/bin/pip" install --quiet python-rtmidi pulsectl

# Create desktop entry
echo "[3/3] Creating desktop entry..."
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=$SCRIPT_DIR/.venv/bin/python $SCRIPT_DIR/main.py
Icon=audio-volume-medium
Type=Application
Categories=AudioVideo;Audio;
Keywords=midi;volume;mixer;audio;
EOF

echo ""
echo "Done! You can now:"
echo "  - Run from terminal:  cd $SCRIPT_DIR && source .venv/bin/activate && python main.py"
echo "  - Run from app menu:  Search for 'MIDIVol'"

#!/bin/bash
set -e

INSTALL_DIR="$HOME/.local/share/midivol"
DESKTOP_DIR="$HOME/.local/share/applications"
ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
REPO_URL="https://github.com/lucasjbx/midi_controller.git"

echo "=== MIDIVol Installer ==="

# If run via curl|bash, clone the repo; otherwise use script location
if [ -f "$0" ] && [ "$(dirname "$0")" != "/dev/fd" ]; then
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    # If pyproject.toml is in linux_app/, point there
    if [ -f "$SCRIPT_DIR/linux_app/pyproject.toml" ]; then
        SOURCE_DIR="$SCRIPT_DIR/linux_app"
    elif [ -f "$SCRIPT_DIR/pyproject.toml" ]; then
        SOURCE_DIR="$SCRIPT_DIR"
    else
        SOURCE_DIR=""
    fi
fi

if [ -z "$SOURCE_DIR" ]; then
    echo "[0/5] Cloning repository..."
    TMP_DIR=$(mktemp -d)
    trap "rm -rf $TMP_DIR" EXIT
    git clone --depth=1 "$REPO_URL" "$TMP_DIR/repo"
    SOURCE_DIR="$TMP_DIR/repo/linux_app"
fi

# Create install dir and venv
echo "[1/5] Creating virtual environment..."
mkdir -p "$INSTALL_DIR"
python3 -m venv --system-site-packages "$INSTALL_DIR/.venv"

# Install package
echo "[2/5] Installing MIDIVol..."
"$INSTALL_DIR/.venv/bin/pip" install --quiet "$SOURCE_DIR"

# Install icon
echo "[3/5] Installing icon..."
mkdir -p "$ICON_DIR"
cp "$SOURCE_DIR/midivol/resources/midivol.svg" "$ICON_DIR/midivol.svg"

# Create desktop entry
echo "[4/5] Creating desktop entry..."
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=$INSTALL_DIR/.venv/bin/midivol
Icon=midivol
Type=Application
Categories=AudioVideo;Audio;
Keywords=midi;volume;mixer;audio;
EOF

# Enable autostart
echo "[5/5] Enabling autostart..."
AUTOSTART_DIR="$HOME/.config/autostart"
mkdir -p "$AUTOSTART_DIR"
cat > "$AUTOSTART_DIR/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=$INSTALL_DIR/.venv/bin/midivol
Icon=midivol
Type=Application
X-GNOME-Autostart-enabled=true
EOF

echo ""
echo "Done! You can now:"
echo "  - Run from terminal:  $INSTALL_DIR/.venv/bin/midivol"
echo "  - Run from app menu:  Search for 'MIDIVol'"
echo "  - MIDIVol will start automatically on login (disable in app settings)"

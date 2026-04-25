#!/bin/bash
set -e

INSTALL_DIR="$HOME/.local/share/midivol"
DESKTOP_DIR="$HOME/.local/share/applications"
ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
REPO_URL="https://github.com/lucasjbx/midi_controller.git"

echo "=== MIDIVol Installer ==="

echo "[1/5] Downloading MIDIVol..."
TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT
git clone --depth=1 "$REPO_URL" "$TMP_DIR/repo" --quiet
SOURCE_DIR="$TMP_DIR/repo/linux_app"

echo "[2/5] Creating virtual environment..."
mkdir -p "$INSTALL_DIR"
python3 -m venv --system-site-packages "$INSTALL_DIR/.venv"

echo "[3/5] Installing MIDIVol..."
"$INSTALL_DIR/.venv/bin/pip" install --quiet "$SOURCE_DIR"

echo "[4/5] Installing icon and desktop entry..."
mkdir -p "$ICON_DIR" "$DESKTOP_DIR"
cp "$SOURCE_DIR/midivol/resources/midivol.svg" "$ICON_DIR/midivol.svg"
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

echo "[5/5] Enabling autostart..."
AUTOSTART_DIR="$HOME/.config/autostart"
mkdir -p "$AUTOSTART_DIR"
cp "$DESKTOP_DIR/midivol.desktop" "$AUTOSTART_DIR/midivol.desktop"

# Refresh icon and desktop caches so Plasma picks up the icon immediately
update-desktop-database "$DESKTOP_DIR" 2>/dev/null || true
gtk-update-icon-cache -f -t ~/.local/share/icons/hicolor 2>/dev/null || true
kbuildsycoca6 --noincremental 2>/dev/null || true

echo ""
echo "Done! Run with: $INSTALL_DIR/.venv/bin/midivol"
echo "Or search 'MIDIVol' in the app menu."

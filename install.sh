#!/bin/bash
set -e

DESKTOP_DIR="$HOME/.local/share/applications"
ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
PACKAGE_URL="git+https://github.com/lucasjbx/midi_controller.git#subdirectory=linux_app"
ICON_URL="https://raw.githubusercontent.com/lucasjbx/midi_controller/main/linux_app/midivol/resources/midivol.svg"

echo "=== MIDIVol Installer ==="

if ! command -v pipx &>/dev/null; then
    echo "Error: pipx not found. Install it with:"
    echo "  sudo pacman -S python-pipx   (Arch)"
    echo "  sudo apt install pipx         (Ubuntu/Debian)"
    exit 1
fi

echo "[1/3] Installing MIDIVol via pipx..."
pipx install --force --system-site-packages "$PACKAGE_URL"

echo "[2/3] Installing icon and desktop entry..."
mkdir -p "$ICON_DIR" "$DESKTOP_DIR"
curl -fsSL "$ICON_URL" -o "$ICON_DIR/midivol.svg"

MIDIVOL_BIN="$HOME/.local/bin/midivol"

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

echo "[3/3] Enabling autostart..."
mkdir -p "$HOME/.config/autostart"
cat > "$HOME/.config/autostart/midivol.desktop" << EOF
[Desktop Entry]
Name=MIDIVol
Comment=MIDI volume controller for Linux
Exec=bash -c "sleep 5 && $MIDIVOL_BIN --autostart"
Icon=midivol
Type=Application
X-GNOME-Autostart-enabled=true
X-KDE-autostart-enabled=true
X-KDE-autostart-phase=2
EOF

update-desktop-database "$DESKTOP_DIR" 2>/dev/null || true
gtk-update-icon-cache -f -t ~/.local/share/icons/hicolor 2>/dev/null || true
kbuildsycoca6 --noincremental 2>/dev/null || true

echo ""
echo "Done! Run with: midivol"
echo "Or search 'MIDIVol' in the app menu."

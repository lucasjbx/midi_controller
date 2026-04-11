import usb_midi

import storage, usb_cdc
import board, digitalio

usb_midi.enable()
usb_midi.set_names(
    streaming_interface_name="interface name", audio_control_interface_name="midi lucas"
)


button = digitalio.DigitalInOut(board.GP22)
button.pull = digitalio.Pull.UP

# Disable devices only if button is not pressed.
if button.value:
    usb_cdc.disable()
    storage.disable_usb_drive()

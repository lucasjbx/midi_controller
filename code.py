# SPDX-FileCopyrightText: 2021 John Park for Adafruit Industries
# SPDX-License-Identifier: MIT

#  Grand Central MIDI Knobs
#  for USB MIDI
#  Reads analog inputs, sends out MIDI CC values
#   with Kattni Rembor and Jan Goolsbey for range and hysteresis code

import time
import board
import busio
from simpleio import map_range
from analogio import AnalogIn
from digitalio import DigitalInOut, Direction, Pull
import digitalio
import usb_midi
import adafruit_midi  # MIDI protocol encoder/decoder library
from adafruit_midi.control_change import ControlChange
from adafruit_midi.note_on import NoteOn
from adafruit_midi.note_off import NoteOff
#from machine import Pin
import neopixel
import usb_hid
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
import rotaryio

ws_pin = board.GP22
led_num = 16
BRIGHTNESS = 0.1  # Adjust the brightness (0.0 - 1.0)

ring = neopixel.NeoPixel(ws_pin, led_num, brightness = 0.1, auto_write=True)



USB_MIDI_channel = 1  # pick your USB MIDI out channel here, 1-16
# pick your classic MIDI channel for sending over UART serial TX/RX
#CLASSIC_MIDI_channel = 2

usb_midi = adafruit_midi.MIDI(
    midi_in=usb_midi.ports[0], in_channel=0,
    midi_out=usb_midi.ports[1], out_channel=0
)


#  use DIN-5 or TRS MIDI jack on TX/RX for classic MIDI
#uart = busio.UART(board.TX, board.RX, baudrate=31250, timeout=0.001)  # initialize UART
#classic_midi = adafruit_midi.MIDI(
#    midi_out=uart, midi_in=uart, out_channel=CLASSIC_MIDI_channel - 1, debug=False
#)

led = DigitalInOut(board.LED)  # activity indicator
led.direction = Direction.OUTPUT

knob_count = 1  # Set the total number of potentiometers used

btn = DigitalInOut(board.GP14)
btn.direction = Direction.INPUT
btn.pull = Pull.UP

prev_state = btn.value


# Create the input objects list for potentiometers
knob = []
for k in range(knob_count):
    knobs = AnalogIn(
        getattr(board, "A0".format(k))
    )  # get pin # attribute, use string formatting
    knob.append(knobs)

#  assignment of knobs to cc numbers
cc_number = [
    1,  # knob 0, mod wheel
    2,  # knob 1, breath control
    7,  # knob 2, volume
    10,  # knob 3 pan
    11,  # knob 4, expression
    53,  # knob 5
    54,  # knob 6
    74,  # knob 7
    74,  # knob 8, Filter frequency cutoff
    71,  # knob 9, Filter resonance
    58,  # knob 10
    59,  # knob 11
    60,  # knob 12
    61,  # knob 13
    62,  # knob 14
    63,  # knob 15
]

# CC range list defines the characteristics of the potentiometers
#  This list contains the input object, minimum value, and maximum value for each knob.
#   example ranges:
#   0 min, 127 max: full range control voltage
#   36 (C2) min, 84 (B5) max: 49-note keyboard
#   21 (A0) min, 108 (C8) max: 88-note grand piano
cc_range = [
    (0, 127),  # knob 15
]

print("---Grand Central MIDI Knobs---")
print("   USB MIDI channel: {}".format(USB_MIDI_channel))
#print("   TRS MIDI channel: {}".format(CLASSIC_MIDI_channel))

# Initialize cc_value list with current value and offset placeholders
cc_value = []
for _ in range(knob_count):
    cc_value.append((0, 0))
last_cc_value = []
for _ in range(knob_count):
    last_cc_value.append((0, 0))

#  range_index converts an analog value (ctl) to an indexed integer
#  Input is masked to 8 bits to reduce noise then a scaled hysteresis offset
#  is applied. The helper returns new index value (idx) and input
#  hysteresis offset (offset) based on the number of control slices (ctrl_max).
def range_index(ctl, ctrl_max, old_idx, offset):
    if (ctl + offset > 65535) or (ctl + offset < 0):
        offset = 0
    idx = int(map_range((ctl + offset) & 0xFF00, 1200, 65500, 0, ctrl_max))
    if idx != old_idx:  # if index changed, adjust hysteresis offset
        # offset is 25% of the control slice (65536/ctrl_max)
        offset = int(
            0.25 * sign(idx - old_idx) * (65535 / ctrl_max)
        )  # edit 0.25 to adjust slices
    return idx, offset


def sign(x):  # determine the sign of x
    if x >= 0:
        return 1
    else:
        return -1

flag = 1
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

##############################rotary encoder##################################
# Rotary encoder inputs with pullup on D3 & D4
# Rotary encoder
enc = rotaryio.IncrementalEncoder(board.GP19, board.GP20)
#encSw = digitalio.DigitalInOut(board.GP15)
#encSw.direction = digitalio.Direction.INPUT
#encSw.pull = digitalio.Pull.UP
lastPosition = 0
# USB device
consumer = ConsumerControl(usb_hid.devices)
##############################################################################

while True:
#############################rotary encoder-volume############################
    # poll encoder position
    position = enc.position
    if position != lastPosition:
        led.value = True
        if lastPosition < position:
            consumer.send(ConsumerControlCode.VOLUME_INCREMENT)
        else:
            consumer.send(ConsumerControlCode.VOLUME_DECREMENT)
        lastPosition = position
        led.value = False


##############################################################################
    msg = usb_midi.receive()
    if msg != None:
        if str(msg)[0] != "M":
            print(msg)
##light
            leds_to_light = round(msg.value/(127/led_num))
            if leds_to_light == 0:
                ring.fill((0, 0, 0))
            else:
                ring[0:leds_to_light] = BLUE * leds_to_light
                ring[leds_to_light:led_num] = BLACK * (led_num-leds_to_light)
##button            
    cur_state = btn.value
    if cur_state != prev_state:
        if not cur_state:
            usb_midi.send(NoteOn(12, 100))
            print("BTN is down")
        else:
            usb_midi.send(NoteOff(12, 60))
            print("BTN is up")
##pot
    prev_state = cur_state
    # read all the knob values
    for i in range(knob_count):
        cc_value[i] = range_index(
            knob[i].value,
            (cc_range[i][1] - cc_range[i][0] + 1),
            cc_value[i][0],
            cc_value[i][1],
        )
        if cc_value[i] != last_cc_value[i]:  # only send if it changed
            # Form a MIDI CC message and send it:
            usb_midi.send(ControlChange(58, cc_value[i][0] + cc_range[i][0]))
#            classic_midi.send(
#                ControlChange(cc_number[i], cc_value[i][0] + cc_range[i][0])
#            )
            last_cc_value[i] = cc_value[i]
            led.value = True
            print(ControlChange(7, cc_value[i][0] + cc_range[i][0]))
            

    time.sleep(0.01)
    led.value = False

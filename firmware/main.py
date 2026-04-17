import board
import neopixel
import time
import busio
import asyncio
import digitalio
from collections import deque
from adafruit_simplemath import map_range

# ads
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# midi
import usb_midi
import adafruit_midi
from adafruit_midi.control_change import ControlChange
from adafruit_midi.note_on import NoteOn
from adafruit_midi.note_off import NoteOff

# HID
import usb_hid
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode

keyboard = Keyboard(usb_hid.devices)

# display
import displayio
import terminalio
import adafruit_displayio_ssd1306
from adafruit_display_text import label
from adafruit_display_shapes.rect import Rect

# buttons
from buttons import keys

# encoders
import rotaryio

# time keeper, so we know when to turn off the LED
led_timestamps = [0, 0, 0]
led_timestamps_6 = [0, 0, 0, 0, 0, 0]
display_timestamp_1 = 0
LIT_TIMEOUT = 2  # after n seconds, turn off ring, led or display

# mute crt
MUTE = False

# leds
ws_pin = board.GP6
led_num = 3
BRIGHTNESS = 0.5  # Adjust the brightness (0.0 - 1.0)
p_leds = neopixel.NeoPixel(ws_pin, led_num, brightness=0.3, auto_write=True)

ws_pin_1 = board.GP5
led_num_1 = 6
BRIGHTNESS = 0.5  # Adjust the brightness (0.0 - 1.0)
b_leds = neopixel.NeoPixel(ws_pin_1, led_num_1, brightness=0.3, auto_write=True)

# Define el color RGB (Rojo, Verde, Azul)
# Los valores van de 0 a 255
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
purple = (255, 0, 255)
black = (0, 0, 0, 0)
cyan = (0, 255, 255)
yellow = (255, 255, 0)


# Create the ADC object using the I2C bus
displayio.release_displays()
i2c = busio.I2C(scl=board.GP1, sda=board.GP0)
ads = ADS.ADS1015(i2c)


# MIDI
USB_MIDI_channel = 2  # pick your USB MIDI out channel here, 1-16

usb_midi = adafruit_midi.MIDI(
    midi_in=usb_midi.ports[0], in_channel=0, midi_out=usb_midi.ports[1], out_channel=0
)

# HID
consumer = ConsumerControl(usb_hid.devices)

# display
display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)

WIDTH = 128
HEIGHT = 64

display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=WIDTH, height=HEIGHT)

# Crear un grupo para contener el texto
text_group = displayio.Group()

# Crear un objeto Label para mostrar texto
text = label.Label(terminalio.FONT, text="", color=0xFFFFFF, x=1, y=40, scale=3)
text1 = label.Label(terminalio.FONT, text="", color=0xFFFFFF, x=1, y=10, scale=2)
text_group.append(text)
text_group.append(text1)

display.root_group = text_group


# monitor potentiometers
async def pot(pin, name, control, color, led_n):
    analog = AnalogIn(ads, pin)
    pot_prev_state = -1
    readings = deque(maxlen=8)  # Moving average filter: 8 samples
    while True:
        # Collect sample
        readings.append(analog.value)

        # Only process if buffer is full
        if len(readings) == 8:
            # Average the readings for smooth response
            pot_val = sum(readings) // len(readings)
            val = int(map_range(pot_val, 0, 26250, 0, 127))

            # Only send if change is significant (deadzone = 1)
            if abs(val - pot_prev_state) > 1:
                val_0_100 = map_range(val, 0, 127, 0, 100)
                if name == "Pot0":
                    display_line_1("   | - -")
                elif name == "Pot1":
                    display_line_1("   - | -")
                elif name == "Pot2":
                    display_line_1("   - - |")
                display_line_2(str(round(val_0_100)))
                print(f"{name} - {val}\r {control, val}\r {pot_val}")
                usb_midi.send(ControlChange(control, val))
                pot_prev_state = val
        await asyncio.sleep(0.005)


# boot led animation
br = 0
colors = [red, green, blue]
for c in range(3):
    for i in range(10):
        b_leds.brightness = br
        p_leds.brightness = br
        b_leds.fill(colors[c])
        p_leds.fill(colors[c])
        br += 0.1
        time.sleep(0.05)
    for i in range(10):
        b_leds.brightness = br
        p_leds.brightness = br
        b_leds.fill(colors[c])
        p_leds.fill(colors[c])
        br -= 0.1
        time.sleep(0.05)
    b_leds.fill(black)
    p_leds.fill(black)
b_leds.brightness = 0.3
p_leds.brightness = 0.3


# display control
def display_line_1(text_):
    global display_timestamp
    display_timestamp = time.monotonic()
    text1.text = text_


def display_line_2(text_):
    global display_timestamp
    display_timestamp = time.monotonic()
    text.text = text_


# display off
async def display_off():
    global display_timestamp
    global MUTE
    while True:
        if MUTE == False:
            if time.monotonic() - display_timestamp > 2:
                text.text = ""
                text1.text = ""

        await asyncio.sleep(0.1)


# monitor buttons
async def buttons():
    while True:
        event = keys.events.get()
        if event:
            if event.pressed:
                print(f"button pressed {event}")
                usb_midi.send(NoteOn(event.key_number, 100))
                if event.key_number == 3:
                    consumer.send(ConsumerControlCode.MUTE)
                    display_line_1("MUTE")
                    display_line_2("")
                if event.key_number == 5:
                    consumer.send(ConsumerControlCode.PLAY_PAUSE)
                    led_6(val=127, color=blue, led_n=5)
                    display_line_1("PLAY/PAUSE")
                    display_line_2("")
                if event.key_number == 10:
                    consumer.send(ConsumerControlCode.SCAN_PREVIOUS_TRACK)
                    led_6(val=127, color=blue, led_n=4)
                    display_line_1("Prev <-")
                    display_line_2("")
                if event.key_number == 7:
                    consumer.send(ConsumerControlCode.SCAN_NEXT_TRACK)
                    led_6(val=127, color=blue, led_n=3)
                    display_line_1("Next ->")
                    display_line_2("")
                if event.key_number == 8:
                    keyboard.send(Keycode.F13)
                    led_6(val=127, color=yellow, led_n=2)
                    display_line_1("F13")
                    display_line_2("")
                if event.key_number == 6:
                    keyboard.send(Keycode.F14)
                    led_6(val=127, color=cyan, led_n=1)
                    display_line_1("F14")
                    display_line_2("")
                if event.key_number == 9:
                    keyboard.send(Keycode.F15)
                    led_6(val=127, color=purple, led_n=0)
                    display_line_1("F15")
                    display_line_2("")
                    text_val = "F15"

        await asyncio.sleep(0.005)


# monitor encoders
async def encoders():
    # encoder
    encoder1 = rotaryio.IncrementalEncoder(board.GP17, board.GP18)
    enc1_last_position = 0
    encoder2 = rotaryio.IncrementalEncoder(board.GP20, board.GP21)
    enc2_last_position = 0
    while True:
        # rotary encoder1
        position1 = encoder1.position
        if position1 != enc1_last_position:
            if enc1_last_position < position1:
                consumer.send(ConsumerControlCode.VOLUME_INCREMENT)
            else:
                consumer.send(ConsumerControlCode.VOLUME_DECREMENT)
            enc1_last_position = position1
        position2 = encoder2.position
        # rotary encoder2
        if position2 != enc2_last_position:
            if enc2_last_position < position2:
                keyboard.send(79)
                display_line_2("")
                display_line_1(">")
                await asyncio.sleep(0.03)
                display_line_1(" >")
                await asyncio.sleep(0.03)
                display_line_1("  >")
            else:
                keyboard.send(80)
                display_line_2("")
                display_line_1("  <")
                await asyncio.sleep(0.03)
                display_line_1(" <")
                await asyncio.sleep(0.03)
                display_line_1("<")
            enc2_last_position = position2
        await asyncio.sleep(0)


# 3leds group ################################################
def led(color, led_n):
    # Asegúrate de declarar que vas a usar la lista global
    global led_timestamps

    # 🌟 ¡Actualiza solo el timestamp de este LED!
    led_timestamps[led_n] = time.monotonic()

    p_leds[led_n] = color
    p_leds.show()


# apaga led despues de led_timestamps
async def timekeeper_task():
    global led_timestamps  # Accede a la lista de tiempos

    while True:
        # Revisa CADA LED en el grupo p_leds (índices 0, 1, 2)
        for i in range(len(p_leds)):

            # Si el tiempo transcurrido para este LED es mayor que el límite
            if time.monotonic() - led_timestamps[i] > LIT_TIMEOUT:

                # 🌟 ¡Apaga SOLO este LED usando su índice 'i'!
                p_leds[i] = black
                p_leds.show()  # Notifica a los NeoPixels del cambio

        await asyncio.sleep(0.1)


# 6leds group ################################################
def led_6(val, color, led_n):
    # Asegúrate de declarar que vas a usar la lista global
    global led_timestamps_6

    # 🌟 ¡Actualiza solo el timestamp de este LED!
    led_timestamps_6[led_n] = time.monotonic()

    n_val = map_range(val, 0, 127, 0, 0.2)
    brightness_val = map_range(val, 0, 127, 0, 255)
    print(f"Brightness: {n_val}")

    # Esta parte de tu lógica está bien para encender el LED:
    b_leds[led_n] = color
    b_leds.show()


# apaga led despues de led_timestamps
async def timekeeper_task_6():
    global led_timestamps_6  # Accede a la lista de tiempos
    global MUTE

    while True:
        if MUTE == False:
            # Revisa CADA LED en el grupo p_leds (índices 0, 1, 2)
            for i in range(len(b_leds)):

                # Si el tiempo transcurrido para este LED es mayor que el límite
                if time.monotonic() - led_timestamps_6[i] > LIT_TIMEOUT:

                    # 🌟 ¡Apaga SOLO este LED usando su índice 'i'!
                    b_leds[i] = black
                    b_leds.show()  # Notifica a los NeoPixels del cambio

        await asyncio.sleep(0.1)


# monitor midi input
async def midi_in():
    global MUTE
    while True:
        msg = usb_midi.receive()
        if msg != None:
            if isinstance(msg, ControlChange):
                # display_ctr(str(msg), 1)
                print(msg)
                if msg.control <= 2:
                    for i in range(3):
                        if msg.control == i:
                            if msg.value == 1:
                                led(red, i)
                            elif msg.value == 0:
                                led(black, i)
                if msg.control == 8:
                    vol = map_range(msg.value, 0, 127, 0, 100)
                    display_line_1("Volumen")
                    display_line_2(str(round(vol)))
                if msg.control == 7:
                    if msg.value == 1:
                        MUTE = True
                        b_leds.fill(red)
                        p_leds.fill(red)
                        display_line_1("")
                        display_line_2("MUTE")
                    if msg.value == 0:
                        MUTE = False
                        b_leds.fill(black)
                        p_leds.fill(black)
                        # display_line_1("")
                        # display_line_2("")

        await asyncio.sleep(0)


###################################
async def main():
    pot0 = asyncio.create_task(pot(ADS.P0, "Pot0", 58, red, 2))
    pot1 = asyncio.create_task(pot(ADS.P1, "Pot1", 59, blue, 1))
    pot2 = asyncio.create_task(pot(ADS.P2, "Pot2", 60, green, 0))
    button_0 = asyncio.create_task(buttons())
    encoders_ = asyncio.create_task(encoders())
    # timekeeper = asyncio.create_task(timekeeper_task())
    timekeeper_6 = asyncio.create_task(timekeeper_task_6())
    display_off_ = asyncio.create_task(display_off())
    midi_input = asyncio.create_task(midi_in())
    await asyncio.gather(
        pot0,
        pot1,
        pot2,
        button_0,
        encoders_,
        timekeeper_6,
        display_off_,
        midi_input,
    )


asyncio.run(main())

import board
import keypad

KEY_PINS = (
    board.GP9,
    board.GP10,
    board.GP11,
    board.GP16,
    board.GP19,
    board.GP22,
    board.GP26,
    board.GP27,
    board.GP28,
    board.GP7,
    board.GP8,
)

keys = keypad.Keys(KEY_PINS, value_when_pressed=False, pull=True)

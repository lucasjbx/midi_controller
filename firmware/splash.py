# Raspberry Pi Pico: http://educ8s.tv/part/RaspberryPiPico
# OLED DISPLAY: https://educ8s.tv/part/OLED096

import board, busio, displayio, time
import adafruit_displayio_ssd1306
import adafruit_imageload

IMAGE_FILE = "icon_30_frames.bmp"
SPRITE_SIZE = (132, 64)
FRAMES = 30


def invert_colors():
    temp = icon_pal[0]
    icon_pal[0] = icon_pal[1]
    icon_pal[1] = temp


displayio.release_displays()

# sda, scl = board.A4, board.A5
# i2c = busio.I2C(scl=board.GP1, sda=board.GP0)

display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=132, height=80)

group = displayio.Group()

#  load the spritesheet
icon_bit, icon_pal = adafruit_imageload.load(
    IMAGE_FILE, bitmap=displayio.Bitmap, palette=displayio.Palette
)
invert_colors()

icon_grid = displayio.TileGrid(
    icon_bit,
    pixel_shader=icon_pal,
    width=1,
    height=1,
    tile_height=SPRITE_SIZE[1],
    tile_width=SPRITE_SIZE[0],
    default_tile=0,
    x=0,
    y=0,
)

group.append(icon_grid)

display.root_group = group

timer = 0
pointer = 0

n = 60


def splash():
    while n > 0:
        if (timer + 0.1) < time.monotonic():
            icon_grid[0] = pointer
            pointer += 1
            timer = time.monotonic()
            if pointer > FRAMES - 1:
                pointer = 0
            n -= 1

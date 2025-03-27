from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from time import sleep

class LEDMatrixAnimator:
    def __init__(self, width=8, height=8, port=0, device=0):
        self.serial = spi(port=port, device=device, gpio=noop())
        self.device = max7219(self.serial, width=width, height=height)

    def display_animation(self, frames, delay=0.5, repeat=4):
        """Display a custom animation."""
        for _ in range(repeat):
            for frame in frames:
                with canvas(self.device) as draw:
                    for y, row in enumerate(frame):
                        for x in range(8):
                            if (row >> (7 - x)) & 1:
                                draw.point((x, y), fill="white")
                sleep(delay)

    def play_preset(self, name, delay=0.5, repeat=4):
        """Play a predefined animation."""
        if name in self.animations:
            self.display_animation(self.animations[name], delay, repeat)
        else:
            raise ValueError(f"Preset animation '{name}' not found!")

    animations = {
        "Happy": [
            [
                0b00000000,
                0b00110100,
                0b00110010,
                0b00000010,
                0b00000010,
                0b00110010,
                0b00110100,
                0b00000000
            ],
            [
                0b00000000,
                0b01101000,
                0b01100100,
                0b00000100,
                0b00000100,
                0b01100100,
                0b01101000,
                0b00000000
            ],
        ],
        "Confused": [
            [
                0b00000000,
                0b00110000,
                0b01111000,
                0b01001101,
                0b01001101,
                0b01100000,
                0b00100000,
                0b00000000
            ],
            [
                0b00000000,
                0b01100000,
                0b11110000,
                0b10011010,
                0b10011010,
                0b11000000,
                0b01000000,
                0b00000000
            ],
        ],
        "Expecting": [
            [
                0b00000000,
                0b00110100,
                0b00110010,
                0b00000010,
                0b00000010,
                0b00110010,
                0b00110100,
                0b00000000
            ],
            [
                0b00000000,
                0b00110100,
                0b00000010,
                0b00000010,
                0b00000010,
                0b00000010,
                0b00110100,
                0b00000000
            ],
        ],
        "Sad": [
            [
                0b00000000,
                0b00110010,
                0b00110100,
                0b00000100,
                0b00000100,
                0b00110100,
                0b00110010,
                0b00000000
            ],
            [
                0b00000000,
                0b01100100,
                0b01101000,
                0b00001000,
                0b00001000,
                0b01101000,
                0b01100100,
                0b00000000
            ],
        ]
    }

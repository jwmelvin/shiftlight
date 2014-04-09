shiftlight
==========

An Arduino-based shift light using Adafruit's NeoPixel strip

v2 now allows live editing of configuration variables

Display segments:
----------------
- dot
  - a single dot
- bar
  - a growing bar
- wipe
  - either a solid bar, a new growing bar, or a wipe from the "bar" color (grow/wipe are from the rpm for "wipe" to the rpm for "shift")
- shift
  - a flashing bar

User Interface (implemented with a pushbutton encoder):
------------------------------------------------------

- in RUN mode:
  - holding button enters SET mode
  - turning knob adjusts brightness
    - pressing button during brightness display saves the brightness setting
    - waiting for timeout retains but does not save the brightness setting
  - tapping button toggles RUN/STOP modes

- in SET mode:
  - holding button enters ADJUST mode
  - turning knob adjusts which of the five sets of parameters is active
    - pressing button during the color display saves the current parameter set
  - tapping button saves current parameter set
  - waiting for timeout retains but does not save the parameters

- in ADJUST mode:
  - holding button saves parameters and returns to RUN mode
  - turning knob adjusts the current parameter (color, rpm, wipe mode, or brake light)
  - tapping button moves to the next parameter (and will cycle back)
    - first is the color for a segment, shown when exceeding the rpm for the segment
    - next is the rpm for the segment, displaying in two 4-bit digits representing the thousands (red) and hundreds (blue) places, so 7800 rpm is displayed as these lights: (0111 1000)
    - after all four segments (dot, bar, wipe, shift), the next two parameters are wipe mode and the brake light
    - for wipe mode, choices are solid (flashing), grow (oscillating w/ bar segment blank), or wipe (oscillating)
    - for brake light, the brake light will be displayed when it will illuminate during the shift period
  - waiting for timeout retains but does not save the parameters

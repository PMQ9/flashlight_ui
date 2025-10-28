# Flashlight UI - TI Tiva C LaunchPad

Simple LED control application for the EK-TM4C1294XL LaunchPad using Arduino framework.

## RTOS Compatibility Warning

**This board has poor RTOS support**. After extensive testing:
- **FreeRTOS**: Arduino-FreeRTOS only supports AVR (not ARM)
- **libopencm3**: Firmware doesn't execute (gets stuck at reset_handler)
- **Zephyr**: TM4C1294 not in supported boards list

**Recommendation**: Use Arduino framework or consider [alternative boards](#better-alternatives-for-rtos-development) with proper RTOS support.

## Hardware
- Board: TI Tiva C Connected LaunchPad (EK-TM4C1294XL)
- MCU: TM4C1294NCPDT (ARM Cortex-M4F, 120MHz)
- Framework: Arduino (Energia)

## Features
- Blue LED (D3) heartbeat - blinks every 1 second
- SW1 button toggles red LED (D1)
- Serial output at 115200 baud

## Build and Upload
```bash
# PlatformIO is not in PATH, use full path:
~/.platformio/penv/Scripts/platformio.exe run --target upload

# Then manually press RESET button on board
# Monitor serial output:
~/.platformio/penv/Scripts/platformio.exe device monitor
```

**Note**: After upload, you'll see "SRST error" but if you see "**Verified OK**", the upload succeeded. Manually press the RESET button on the board.

## Pin Mapping
- Red LED (D1): PN_1
- Green LED (D2): PN_0
- Blue LED (D3): PF_0
- Button SW1: PJ_0


## Original Project Goal

This project was originally intended to implement an Anduril-2 based flashlight UI by ToyKeeper for testing on Texas Instrument Tiva development boards.

- Anduril-2 GitHub: https://github.com/ToyKeeper/anduril
- UI Chart: https://www.reddit.com/r/flashlight/comments/sk1upj/and%C3%BAril_2_ui_chart/

For Anduril-2 implementation, an RTOS-capable board is recommended (see alternatives above).

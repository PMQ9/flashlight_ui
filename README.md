# Flashlight UI - Distributed FreeRTOS System

This project was intended to implement an Anduril-2 based flashlight UI by ToyKeeper for testing on Texas Instrument Tiva development boards.

- Anduril-2 GitHub: https://github.com/ToyKeeper/anduril
- UI Chart: https://www.reddit.com/r/flashlight/comments/sk1upj/and%C3%BAril_2_ui_chart/

The system combining Tiva TM4C1294 (button & LED control) with Raspberry Pi 4 (FreeRTOS task scheduler). Button press → Tiva sends event → Pi4 processes with 3 tasks → Pi4 sends command → Tiva toggles LED.

## Quick Start

### 1. Upload Tiva Firmware
```bash
cd ~\flashlight_ui
~/.platformio/penv/Scripts/platformio.exe run --target upload
# Press RESET button - blue LED should blink
```

### 2. Run Pi4 Relay
```bash
ssh raspberrypiuser@raspberrypiaddress
cd pi4-freertos
./flashlight_relay
```

### 3. Test
Press button SW1 on Tiva → Red LED toggles → Check Pi4 shows "LED_OK"

## Hardware

**Connection**: Tiva ICDI Micro USB → Pi4 USB-A Port

**Pin Mapping**:
- Red LED (D1): PN_1 (toggled by Pi4)
- Green LED (D2): PN_0 (unused)
- Blue LED (D3): PF_0 (heartbeat - blinks every 1 sec)
- Button SW1: PJ_0 (sends event to Pi4)

**Serial**: 115200 baud, /dev/ttyACM0 (CDC ACM protocol)

## System Architecture

```
Tiva Board           Pi4 FreeRTOS Relay
├─ Button Input   → ├─ Serial Receiver Task
├─ LED Control   ← ├─ LED Controller Task
└─ Heartbeat       └─ Heartbeat Task
      ║ USB Serial ║
    /dev/ttyACM0
```

## Features

- 3 concurrent FreeRTOS-style tasks on Pi4
- Mutex locks + Semaphore synchronization
- Pre-compiled Pi4 binary ready to run (71KB)
- Simple text protocol: BTN_PRESSED, LED_TOGGLE, LED_OK
- Auto buffer-clearing for clean serial communication
- Deploy to multiple Pi4s (just copy pi4-freertos/ directory)

## Build Commands

```bash
# Tiva firmware
~/.platformio/penv/Scripts/platformio.exe run --target upload

# Pi4 relay (build from source)
cd pi4-freertos && make clean && make && ./flashlight_relay

# Pi4 relay (run pre-compiled)
cd pi4-freertos && ./flashlight_relay
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Blue LED not blinking | Re-upload firmware, press RESET button |
| Red LED won't toggle | Check relay is running, try holding button longer |
| Serial data corrupted | Normal on startup, relay auto-clears buffer |
| Permission denied | Run with sudo or: `sudo chmod 666 /dev/ttyACM0` |

## Files

- `src/main.cpp` - Tiva firmware
- `pi4-freertos/flashlight_relay` - Pre-compiled Pi4 binary
- `pi4-freertos/flashlight_relay.c` - Pi4 source code
- `pi4-freertos/Makefile` - Pi4 build config



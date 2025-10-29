# Pi4 FreeRTOS Relay

Complete deployment package for Raspberry Pi 4. Contains pre-compiled binary, source code, and build system.

## Quick Start

### Run Pre-Compiled Binary (Fastest)
```bash
./flashlight_relay
```
Binary prebuilt is ready to run on Pi4 (ARM Cortex-A72, 64-bit).

### Build from Source
```bash
make clean && make
./flashlight_relay
```

## Files

- **flashlight_relay** - Pre-compiled binary (71KB)
- **flashlight_relay.c** - Source code (254 lines)
- **Makefile** - Build configuration
- **README.md** - This file

## System Requirements

- OS: Raspberry Pi OS (any recent version)
- CPU: ARM 64-bit (Pi4, Pi5, etc.)
- Tools: GCC (usually pre-installed)

## Deployment to Another Pi4

### Option 1: Copy Binary

TBD

### Option 2: Copy and Build

TBD


### Option 3: Git Clone
```bash
git clone <repo>
cd flashlight_ui/pi4-freertos
make clean && make
./flashlight_relay
```

## Build Commands

```bash
# Clean build artifacts
make clean

# Build from source
make

# Run the relay
./flashlight_relay

# All in one
make clean && make && ./flashlight_relay
```

## Manual Compilation

If make doesn't work:
```bash
gcc -pthread -Wall -O2 -o flashlight_relay flashlight_relay.c
```

## Serial Port Setup

The relay needs access to `/dev/ttyACM0`:

```bash
# Check if it exists
ls /dev/ttyACM0

# Fix permissions if needed
sudo chmod 666 /dev/ttyACM0

# Or add to group (permanent)
sudo usermod -aG plugdev $USER
# Then log out and back in
```

## Running the Relay

### Basic
```bash
./flashlight_relay
```

### With Logging
```bash
./flashlight_relay > relay.log 2>&1 &
tail -f relay.log
```

### As Background Service
```bash
nohup ./flashlight_relay &
```

## Architecture

3 concurrent FreeRTOS-style tasks:

1. **Serial Receiver Task** - Reads from /dev/ttyACM0
2. **LED Controller Task** - Processes button events
3. **Heartbeat Task** - Monitors system health

All with proper mutex locks and semaphore synchronization.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't open /dev/ttyACM0 | Reconnect USB, wait 5 sec, or: `sudo chmod 666 /dev/ttyACM0` |
| Permission denied | Run with `sudo` or add to plugdev group |
| Compile error | Install: `sudo apt-get install build-essential` |
| No heartbeat | Check Tiva is connected and firmware running |

## Files to Know

- **flashlight_relay.c** - Main source code
- **flashlight_relay** - Compiled binary
- **Makefile** - Build system

For hardware/system details, see main README in parent directory.

## Status

- Source: 7.4KB (254 lines)
- Binary: 71KB (pre-compiled)
- Status: Production ready

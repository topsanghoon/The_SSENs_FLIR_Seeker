# IR Capture Test

This test program validates the IR capture and transmission pipeline by capturing thermal frames from the FLIR Lepton 2.5 camera via SPI and streaming them over UDP.

## Overview

The test creates two threads:
- **IR_CaptureThread**: Captures raw thermal frames from `/dev/spidev0.0` using the VoSPI protocol
- **IR_TxThread**: Streams captured frames via GStreamer to `192.168.0.15:5000`

## Hardware Requirements

- FLIR Lepton 2.5 thermal camera on Breakout Board v1.3
- Connected to SPI bus 0, device 0 (`/dev/spidev0.0`)
- SPI configured at 20 MHz
- Network connection to PC at 192.168.0.15

## Building

From the build directory:

```bash
cd /home/user/THESSENS_TEST/The_SSENs_FLIR_Seeker/src/software
cmake --build build/debug --target ir_capture_test
```

Or for Zynq release build:
```bash
cmake --build build/zynq-release --target ir_capture_test
```

## Running

```bash
./build/debug/app/tools/ir_capture_test/ir_capture_test
```

The program will:
1. Initialize SPI communication with the FLIR Lepton 2.5 camera
2. Start capturing 80x60 thermal frames at ~9 fps
3. Stream frames via UDP to 192.168.0.15:5000
4. Display statistics every 2 seconds

Press `Ctrl+C` to stop.

## Viewing the Stream

On the receiving PC (192.168.0.15), use GStreamer to view the stream:

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  videoparse width=80 height=60 framerate=9/1 format=gray16-le ! \
  videoconvert ! autovideosink
```

Or use a more advanced visualization with false color mapping:

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  videoparse width=80 height=60 framerate=9/1 format=gray16-le ! \
  videoconvert ! videobalance saturation=2.0 ! autovideosink
```

## Output Statistics

The program displays real-time statistics:
- **Frames**: Total number of successfully captured frames
- **Errors**: Number of capture errors
- **Discards**: Number of discard packets received (camera not ready)

Example output:
```
=== IR Capture & Stream Test ===
Capturing from /dev/spidev0.0 (FLIR Lepton 2.5) and streaming to udp://192.168.0.15:5000
Press Ctrl+C to stop...

[IR_Capture] SPI initialized: /dev/spidev0.0 @ 20000000 Hz
[INFO] Pipeline started. Capturing from FLIR Lepton 2.5 and streaming...

[STATS @ 2s] Frames: 18, Errors: 0, Discards: 5
[STATS @ 4s] Frames: 36, Errors: 0, Discards: 8
[STATS @ 6s] Frames: 54, Errors: 0, Discards: 12
```

## Troubleshooting

### No SPI device found
```
[IR_Capture] Failed to open SPI device: /dev/spidev0.0
```
- Check if SPI is enabled in device tree
- Verify permissions: `ls -l /dev/spidev0.0`
- May need to run with sudo or add user to spi group

### High discard count
- This is normal during camera warmup
- Discard packets indicate the camera is performing internal calibration
- Should stabilize after a few seconds

### No video on receiving PC
- Verify network connectivity: `ping 192.168.0.15`
- Check if port 5000 is not blocked by firewall
- Ensure GStreamer is installed on receiving PC
- Verify the PC is listening on the correct port

### Permission denied for SPI
```bash
sudo usermod -a -G spi $USER
# or run with sudo
sudo ./ir_capture_test
```

## VoSPI Protocol Details

The FLIR Lepton 2.5 uses VoSPI (Video over SPI) protocol:
- Frame size: 80x60 pixels (16-bit grayscale)
- 1 segment per frame, 60 lines per segment
- 164-byte packets (4-byte header + 160-byte payload)
- Discard packets (0x0F00) indicate camera busy
- Frame rate: ~9 fps typical

## See Also

- `IR_CaptureThread.cpp/hpp` - SPI capture implementation
- `IR_TxThread.cpp/hpp` - GStreamer streaming implementation
- `eo_capture_test` - Similar test for EO camera

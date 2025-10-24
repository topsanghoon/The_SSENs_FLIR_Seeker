# Net_RxThread Simple Test

This is a focused test for only the `Net_RxThread` component, which receives click commands from a Windows PC via UDP.

## What This Test Does

- **Linux Side**: Runs `Net_RxThread` to listen for UDP packets on port 5001
- **Windows Side**: Sends click commands via UDP when you click in a test window
- **No GStreamer**: Simple UDP communication only
- **No Video**: Just click command testing

## How to Use

### 1. Build and Run Linux Test

```bash
cd /home/user/THESSENS/The_SSENs_FLIR_Seeker/src/software/build/debug
./app/tools/net_rx_test/net_rx_test
```

You should see:
```
=== Net_RxThread Test ===
[CONFIG] Listening on UDP port: 5001
[START] Starting Net_RxThread...
[INFO] Net_RxThread is running and listening for clicks
[INFO] Send clicks from Windows client to see them here
```

### 2. Run Windows Client

On your Windows PC:

1. Copy the `simple_click_sender.py` file
2. Edit the `LINUX_PC_IP` variable to your Ubuntu machine's IP address
3. Install requirements: `pip install opencv-python numpy`
4. Run: `python simple_click_sender.py`

### 3. Test the Connection

1. Click anywhere in the Windows test window
2. You should see click messages appear on the Linux console:

```
[CLICK #1] seq=1 type=0 box=(245.0, 156.0, 10.0, 10.0) elapsed=1234ms
[CLICK #2] seq=2 type=0 box=(312.0, 203.0, 10.0, 10.0) elapsed=2456ms
```

### 4. Stop the Test

- Linux: Press `Ctrl+C` to stop
- Windows: Press 'q' or ESC to close the window

## Protocol Details

The Windows client sends UDP packets with this format:
- Byte 0: Command type (0 = CLICK)
- Bytes 1-4: X coordinate (float, network byte order)
- Bytes 5-8: Y coordinate (float, network byte order)
- Bytes 9-12: Width (float, network byte order) 
- Bytes 13-16: Height (float, network byte order)

Total: 17 bytes per click command

## Troubleshooting

### "No clicks received"
1. Check firewall on Linux (port 5001)
2. Verify IP address in Windows client
3. Make sure both machines are on same network

### "Connection failed"
1. Check if the Linux test is running first
2. Verify the IP address is correct
3. Test with `ping` between machines

### "Build errors"
1. Make sure you're using `/usr/bin/cmake` (not Xilinx version)
2. Clean build: `rm -rf build/debug && mkdir -p build/debug`
3. Configure: `cd build/debug && /usr/bin/cmake -G Ninja ../..`
4. Build: `ninja net_rx_test`
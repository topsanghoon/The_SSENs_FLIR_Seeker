"""
Simple Windows Click Sender for Net_RxThread Test

This script sends click coordinates to Net_RxThread. The Linux system converts
the click point to a bounding box for object tracking initialization.

Protocol: Sends only (x,y) coordinates - Net_RxThread creates tracking box automatically.

Requirements:
- Python 3.x
- pip install opencv-python numpy

Usage:
1. Change LINUX_PC_IP to your Ubuntu system's IP address
2. Run: python simple_click_sender.py
3. Click anywhere in the window to send coordinates
4. Press 'q' or ESC to quit
"""

import cv2
import socket
import struct
import time
import numpy as np

# ============================================================
# CONFIGURATION - CHANGE THESE TO MATCH YOUR LINUX PC
# ============================================================
LINUX_PC_IP = "192.168.176.128" 
CLICK_PORT = 5001  # Port where Net_RxThread is listening
# ============================================================

class ClickSender:
    def __init__(self, linux_ip, port):
        self.linux_ip = linux_ip
        self.port = port
        self.sock = None
        self.click_seq = 1
        self.connected = False
        
        # Create a simple test window
        self.window_name = "Click Test - Send clicks to Linux"
        self.image = np.zeros((400, 600, 3), dtype=np.uint8)
        self.setup_window()
        
    def setup_window(self):
        """Setup the test window with instructions"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # Draw instructions on the image
        self.image.fill(50)  # Dark gray background
        
        # Title
        cv2.putText(self.image, "Net_RxThread Click Test", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        
        # Instructions
        instructions = [
            f"Target: {self.linux_ip}:{self.port}",
            "",
            "Click anywhere in this window",
            "to send click commands to Linux",
            "",
            "Press 'q' or ESC to quit"
        ]
        
        y_pos = 100
        for instruction in instructions:
            cv2.putText(self.image, instruction, (50, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
            y_pos += 40
            
        # Status area
        self.update_status("Ready - Click to start sending")
        
    def update_status(self, message, color=(0, 255, 0)):
        """Update status message in the window"""
        # Clear status area
        cv2.rectangle(self.image, (50, 320), (550, 380), (50, 50, 50), -1)
        
        # Add new status
        cv2.putText(self.image, f"Status: {message}", (50, 350), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow(self.window_name, self.image)
        
    def connect_to_linux(self):
        """Establish UDP connection to Linux PC"""
        try:
            if self.sock:
                self.sock.close()
                
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Test connection by sending a small packet
            test_data = b"test"
            self.sock.sendto(test_data, (self.linux_ip, self.port))
            self.connected = True
            self.update_status(f"Connected to {self.linux_ip}:{self.port}")
            return True
            
        except Exception as e:
            self.connected = False
            self.update_status(f"Connection failed: {str(e)}", (0, 0, 255))
            return False
            
    def send_click(self, x, y):
        """Send click command to Linux PC"""
        if not self.connected:
            if not self.connect_to_linux():
                return False
                
        try:
            # Create click command (Net_RxThread updated format - just coordinates)
            # Byte 0: Command type (0 = CLICK)
            # Bytes 1-4: x coordinate (float, network byte order)
            # Bytes 5-8: y coordinate (float, network byte order)
            # Note: Net_RxThread will create bounding box automatically
            
            # Pack command: type(1 byte) + x(4 bytes) + y(4 bytes)
            command_data = struct.pack('!Bff', 
                                     0,          # Command type: 0 = CLICK
                                     float(x),   # X position (network byte order)
                                     float(y))   # Y position (network byte order)
            
            # Send to Linux
            self.sock.sendto(command_data, (self.linux_ip, self.port))
            
            self.update_status(f"Sent click #{self.click_seq}: ({x}, {y})")
            self.click_seq += 1
            return True
            
        except Exception as e:
            self.connected = False
            self.update_status(f"Send failed: {str(e)}", (0, 0, 255))
            return False
            
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks in the window"""
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Click at ({x}, {y}) - sending to Linux...")
            success = self.send_click(x, y)
            if success:
                # Visual feedback - draw a circle at click position
                cv2.circle(self.image, (x, y), 5, (0, 255, 0), -1)
                cv2.imshow(self.window_name, self.image)
                
    def run(self):
        """Main loop"""
        print(f"Starting Click Sender for {self.linux_ip}:{self.port}")
        print("Click in the window to send commands to Linux")
        print("Press 'q' or ESC to quit")
        
        cv2.imshow(self.window_name, self.image)
        
        while True:
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
                
        # Cleanup
        if self.sock:
            self.sock.close()
        cv2.destroyAllWindows()
        print("Click sender stopped")

def main():
    print("=== Simple Click Sender for Net_RxThread Test ===")
    print(f"Target Linux PC: {LINUX_PC_IP}:{CLICK_PORT}")
    print()
    
    # Validate IP address format
    try:
        socket.inet_aton(LINUX_PC_IP)
    except socket.error:
        print(f"ERROR: Invalid IP address: {LINUX_PC_IP}")
        print("Please edit the LINUX_PC_IP variable in this script")
        input("Press Enter to exit...")
        return
        
    sender = ClickSender(LINUX_PC_IP, CLICK_PORT)
    sender.run()

if __name__ == "__main__":
    main()
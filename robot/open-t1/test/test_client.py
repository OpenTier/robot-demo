#!/usr/bin/env python3
import socket
import time

# Update this with your ESP32's IP address
ESP32_IP = "192.168.100.231"
ESP32_PORT = 5000

def ping_pong():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {ESP32_IP}:{ESP32_PORT}...")
        s.connect((ESP32_IP, ESP32_PORT))
        print("Connected!")
        try:
            while True:
                s.sendall(b"ping")
                print("Sent: ping")
                data = s.recv(1024)
                if not data:
                    print("Connection closed by server.")
                    break
                print("Received:", data.decode())
                time.sleep(1)
        except KeyboardInterrupt:
            print("Exiting ping-pong test.")

if __name__ == "__main__":
    ping_pong()

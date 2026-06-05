import time
import busio
import board
import digitalio
import adafruit_rfm9x
import threading
import sys

# --- Hardware Setup (Keeping your specific pins) ---
CS = digitalio.DigitalInOut(board.D8) 
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    rfm9x.tx_power = 17
    print("RFM9x LoRa Radio Found!")
except Exception as e:
    print(f"Error: {e}")
    sys.exit()

def receive_loop():
    """Background thread to constantly listen for LoRa packets."""
    print("Receiver thread started...")
    while True:
        # Check for packet (shorter timeout for responsiveness)
        packet = rfm9x.receive(timeout=0.1)
        if packet is not None:
            try:
                packet_text = str(packet, "utf-8").strip()
                print(f"\n[RX]: {packet_text} | RSSI: {rfm9x.last_rssi} dBm")
                print("Message to send: ", end="", flush=True)
            except Exception:
                print(f"\n[RX Binary/Hex]: {packet}")
        time.sleep(0.01)

# Start the receiver thread
rx_thread = threading.Thread(target=receive_loop, daemon=True)
rx_thread.start()

print("-------------------------------------------")
print("Simultaneous LoRa Terminal")
print("Type a message anytime and press Enter.")
print("-------------------------------------------")

# Main Loop: Handle User Input
while True:
    send_text = input("Message to send: ").strip()
    if send_text:
        # Remove the "\r\n" to stop the unknown characters on Arduino
        # Convert to bytes for the radio
        rfm9x.send(bytes(send_text, "utf-8"))
        print("Sent!")

import time
import busio
from digitalio import DigitalInOut, Direction
import board
import adafruit_rfm9x

# --- Hardware Configuration ---
# You are using CE1 (Physical Pin 26) for Chip Select
CS = DigitalInOut(board.D8) 
RESET = DigitalInOut(board.D25)

# Initialize SPI
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize RFM9x
# Ensure 915.0 matches your radio's frequency (915 or 433)
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    rfm9x.tx_power = 23
    print("RFM9x LoRa Radio Found!")
except Exception as e:
    print(f"Error initializing Radio: {e}")
    exit()

print("-------------------------------------------")
print("LoRa Terminal Chat")
print("Type a message and press Enter to send.")
print("The Pi will also listen for incoming packets.")
print("-------------------------------------------")

while True:
    # 1. Check for incoming packets (non-blocking)
    packet = rfm9x.receive(timeout=0.5) # Wait 0.5s for a packet
    
    if packet is not None:
        try:
            packet_text = str(packet, "utf-8")
            rssi = rfm9x.last_rssi
            print(f"\n[RX]: {packet_text} | RSSI: {rssi} dBm")
        except Exception:
            print(f"\n[RX Hex]: {packet}")

    # 2. Get User Input to Send
    # We use a short timeout above so the script doesn't hang forever
    # To keep it simple, we'll ask for a message every loop
    # If you want to send, type it. If not, just press Enter to keep listening.
    
    send_text = input("Message to send (or Enter to listen): ").strip()
    
    if send_text:
        print(f"Sending: {send_text}...")
        # Convert string to bytes and send
        rfm9x.send(bytes(send_text + "\r\n", "utf-8"))
        print("Sent!")
    
    time.sleep(0.1)
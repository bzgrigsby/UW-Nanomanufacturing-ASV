import time
import busio
import board
import digitalio
import adafruit_rfm9x
from intercomm import Intercomm
import threading
import sys
import telem
import comm
import struct

class LoRa_rfm9x:
    def __init__(self, inter : Intercomm):
        self.rfm9x = None
        self.inter = inter
        self.init_radio()

    def init_radio(self):
        CS = digitalio.DigitalInOut(board.D8) 
        RESET = digitalio.DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        try:
            self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915)
            self.rfm9x.tx_power = 23
            self.rfm9x.bandwidth = 125000
            self.rfm9x.preamble_length = 8
            self.rfm9x.coding_rate = 5
            self.rfm9x.auto_agc = True
            print("RFM9x LoRa Radio Found!")
        except Exception as e:
            print(f"Error: {e}")

    def transmit(self, packet, important):
        if (packet is not None and packet != -1 and self.rfm9x is not None):
            self.rfm9x.send(packet)
            if important:
                self.inter.ack_pending = True
            return True
        return False
        
        



        


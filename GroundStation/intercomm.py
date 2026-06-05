import time

class Intercomm:
    def __init__(self):
        self.ack_pending = False
        self.ack_pend_start_time = 0.0

    def ack_pend_time(self):
        if self.ack_pending == True:
            return time.perf_counter() - self.ack_pend_start_time
        return 0.0
    
    def set_pend_ack(self):
        self.ack_pending = True
        self.ack_pend_start_time = time.perf_counter()
# Interface with the fluxgate box
# Derek Fujimoto
# July 2026

import zmq
import struct
import numpy as np
import threading
import time

VOLTAGE_REFERENCE = 2.5

class FluxgateBox(object):
    """Read/write from fluxagate DAQ box"""

    def __init__(self, ip="142.90.151.5", poll_ms=1):
        """
        Args:
            ip (str): ip address of DAQ box
            poll_ms (int): duration between message polls
        """

        # connect over ethernet
        context = zmq.Context()
        self.fcontext = context
        server = f"tcp://{ip}:5555"        

        # connect for polling
        print(f"Connecting via ZMQ to {ip}...", end='')
        socket = context.socket(zmq.SUB)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.connect(server)
        self.socket_sub = socket
        print("failed" if context.closed else "success")

        # socket for request 
        self.socket_req = context.socket(zmq.REQ)
        self.socket_req.connect(server)

        # init
        self.fmessage = None
        self.number_packets = 0
        self.data = {}

        # multithreaded polling
        self.interval = poll_ms/1000
        self._stop_event = threading.Event() # Signals the loop to exit.
        self._thread = None # Holds the background thread once started.

    def poll(self):
        """ask for latest message, unpack and save"""

        # loop until the queue is fully drained - ensure that data is newest
        while True:
            try:
                message = self.socket_sub.recv_multipart(flags=zmq.NOBLOCK)
            except (zmq.ZMQError, zmq.Again):
                break

            # unpack data
            (
                tx_count,
                time_sec,
                time_nsec,
                flags,
                mag_id,
                mag_x,
                mag_y,
                mag_z,
            ) = struct.unpack_from("<IIIIIIII", message[0], 0)

            # save data
            self.data[mag_id] = [tx_count, 
                                 time_sec+time_nsec*1e-9, 
                                 flags, 
                                 raw_to_voltage(mag_x), 
                                 raw_to_voltage(mag_y), 
                                 raw_to_voltage(mag_z)]
            # if mag_id == 0 :
            #     print(f'{time.time()-time_sec+time_nsec*1e-9:.3f} {time_sec+time_nsec*1e-9:.3f} {mag_id}: {self.get_volts(mag_id)*10} uT', flush=True)

    def _run(self):
        """Loop that calls `poll` every `interval` seconds until stopped.

        Using `wait` (instead of `sleep`) lets `stop` interrupt the interval
        immediately, so shutdown does not block for up to `interval` seconds.
        """
        while not self._stop_event.is_set():
            self.poll()
            # Returns True if stopped during the wait, False on timeout.
            self._stop_event.wait(self.interval)

        # stop communication
        self.socket_sub.close()

    def start(self):
        """Start polling in a background daemon thread (non-blocking)."""
        assert self._thread is None, "Service is already running."
        self._stop_event.clear()
        # daemon=True lets the program exit even if the thread is alive.
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("Starting polling")

    def stop(self):
        """Signal the background thread to stop and wait for it to finish."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def get_volts(self, mag_id):
        """Get latest fluxgate message in volts"""
        try:
            return np.array([self.data[mag_id][3],
                             self.data[mag_id][4],
                             self.data[mag_id][5]])
        except KeyError:
            return np.full(3, np.nan)

    def get_time(self, mag_id):
        try:
            return self.data[mag_id][1]
        except KeyError:
            return np.nan

    @property
    def enhanced_filter(self):  return self.read_msg(2)
    @property
    def rate(self):             return self.read_msg(3) 
    @property
    def delay(self):            return self.read_msg(4)
    @property
    def bitmask(self):          return self.read_msg(5) & 0xFFFF # Magnetometer bitmask
    @property
    def sinc3(self):            return bool(self.read_msg(3) & 0x80000000)

    def set_filter(self, val):  self.write_msg(2, val)
    def set_rate(self, val):    self.write_msg(3, val)
    def set_delay(self, val):   self.write_msg(4, val)
    def set_bitmask(self, val): self.write_msg(5, val)
    def toggle_reset(self):  
        self.write_msg(0, 1)
        self.write_msg(0, 0)
              
    def write_msg(self, addr, data):
        msg = struct.pack("<III", ord("w"), addr, data)
        self.socket_req.send(msg, 0)
        resp = self.socket_req.recv()
        msg = struct.unpack_from("<I", resp, 0)
        if msg[0] == 114:
            msg = struct.unpack_from("<I", resp, 4)
            return msg[0]
        else:
            raise IOError("Write Error")
        
    def read_msg(self, addr):
        msg = struct.pack("<III", ord('r'), addr, 0)
        self.socket_req.send(msg, 0)
        resp = self.socket_req.recv()
        msg = struct.unpack_from("<I", resp, 0)
        if msg[0] == 114:
            return struct.unpack_from("<I", resp, 4)[0]
        else:
            raise IOError("Read Error") 

# HELPER FUNCTIONS =======================================================
def raw_to_voltage(val):
    # When the ADC is configured for bipolar operation, the output           
    # code is offset binary with a negative full-scale voltage resulting    
    # in a code of 000 … 000, a zero differential input voltage resulting in 
    # a code of 100 … 000, and a positive full-scale input voltage          
    # resulting in a code of 111 … 111. The output code for any              
    # analog input voltage can be represented as                            
    # Code = 2^(N – 1) × ((VIN × 0.1)/VREF) + 1)                    
    # N = 24, Vin = input voltage, Vref = reference voltage (internal 2.5V) 
    return (((val / 8388608) - 1) * VOLTAGE_REFERENCE) / 0.1

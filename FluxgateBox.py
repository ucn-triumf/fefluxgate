# Interface with the fluxgate box
# Derek Fujimoto
# July 2026

import zmq
import struct
import numpy as np
import threading
import time # for debugging

VOLTAGE_REFERENCE = 2.5

class FluxgateBox(object):
    """Read/write from fluxagate DAQ box

    Talks to the TUCAN fluxgate magnetometer readout (DE10-Nano) over ZeroMQ:
    a SUB socket on port 5555 receives the data stream, and a REQ socket on
    port 5556 reads and writes the control registers. See ETHERNET_INTERFACE.md.

    Control register writes (filter, rate, delay, bitmask) are not applied to
    the ADCs until `toggle_reset` is called.
    """

    def __init__(self, ip="142.90.151.5", poll_ms=1):
        """
        Args:
            ip (str): ip address of DAQ box
            poll_ms (int): duration between message polls, in milliseconds

        Attributes:
            data (dict): latest message per magnetometer. Key: mag_id (0-based).
                Value: [tx_counter, time (s since device boot), status,
                Vx, Vy, Vz (V)].
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
        print("done")

        # socket for request (command channel is on its own port)
        self.socket_req = context.socket(zmq.REQ)
        self.socket_req.connect(f"tcp://{ip}:5556")

        # init
        self.fmessage = None
        self.number_packets = 0
        self.data = {}

        # multithreaded polling
        self.interval = poll_ms/1000
        self._stop_event = threading.Event() # Signals the loop to exit.
        self._thread = None # Holds the background thread once started.

    def poll(self):
        """ask for latest message, unpack and save

        Drains all queued messages without blocking and keeps only the latest
        message for each magnetometer in `self.data`. Each message is 32 bytes,
        `<IIIIIIII` = (tx_counter, sec, nsec, status, mag_id, x, y, z).

        The timestamp is the device CLOCK_MONOTONIC_RAW (time since boot, not
        wall-clock time) at the moment the message was sent, not when the
        samples were taken.
        """

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
        """Get latest fluxgate message in volts

        Convert to field with B [uT] = V / S, where S is the Mag690 scale
        (e.g. 0.1 V/uT for the +/-100 uT range).

        Args:
            mag_id (int): magnetometer index, 0-based (0 = magnetometer 1)

        Returns:
            np.ndarray: (x, y, z) voltages at the ADC input, or NaN if no data
                has been received for this magnetometer.
        """
        try:
            return np.array([self.data[mag_id][3],
                             self.data[mag_id][4],
                             self.data[mag_id][5]])
        except KeyError:
            return np.full(3, np.nan)

    def get_time(self, mag_id):
        """Get the timestamp of the latest message.

        Args:
            mag_id (int): magnetometer index, 0-based (0 = magnetometer 1)

        Returns:
            float: device time since boot in seconds, or NaN if no data has
                been received for this magnetometer.
        """
        try:
            return self.data[mag_id][1]
        except KeyError:
            return np.nan

    @property
    def enhanced_filter(self):
        """int: raw register 2 (50/60 Hz post-filter). Bit 31 = enable, bits [2:0] = filter 2, 3, 5 or 6."""
        return self.read_msg(2)
    @property
    def rate(self):
        """int: output data rate code, register 3 bits [4:0] (0-22)."""
        return self.read_msg(3) & 0x1F
    @property
    def delay(self):
        """int: settle delay code, register 4 (0-7 -> 0 us to 8 ms after each channel switch)."""
        return self.read_msg(4)
    @property
    def bitmask(self):
        """int: magnetometer enable mask, register 5. Bit k enables magnetometer k+1."""
        return self.read_msg(5) & 0xFFFF
    @property
    def sinc3(self):
        """bool: True if the sinc3 filter is selected (register 3 bit 31), else sinc5+sinc1."""
        return bool(self.read_msg(3) & 0x80000000)

    def set_filter(self, val):
        """Set the 50/60 Hz enhanced post-filter (register 2). Apply with `toggle_reset`.

        Requires sinc5+sinc1 (sinc3 off).

        Args:
            val (int): 0 to disable, or 2, 3, 5, 6 (27.27, 25, 20, 16.67 SPS)
        """
        # bit 31 enables the filter, bits [2:0] select it
        self.write_msg(2, 0 if val == 0 else 0x80000000 | (val & 0x7))
    def set_rate(self, val):
        """Set the output data rate code (register 3). Apply with `toggle_reset`.

        This overwrites the sinc3 bit (31): pass `0x80000000 | code` to select sinc3.

        Args:
            val (int): rate code 0-22 (see ETHERNET_INTERFACE.md section 5.1)
        """
        self.write_msg(3, val)
    def set_delay(self, val):
        """Set the post-channel-switch settle delay (register 4). Apply with `toggle_reset`.

        Args:
            val (int): 0-7 -> 0, 32, 128, 320, 800 us, 1.6, 4, 8 ms
        """
        self.write_msg(4, val)
    def set_bitmask(self, val):
        """Set the magnetometer enable mask (register 5). Apply with `toggle_reset`.

        Args:
            val (int): 0x0000-0xFFFF, bit k enables magnetometer k+1
        """
        self.write_msg(5, val)
    def toggle_reset(self):
        """Pulse ADC_SOFT_RST (register 0) to latch registers 1-5 into the ADCs.

        The first message per magnetometer after a reset may combine values
        from before and after the reset, so it should be discarded.
        """
        self.write_msg(0, 1)
        self.write_msg(0, 0)

    def write_msg(self, addr, data):
        """Write a control register.

        Args:
            addr (int): control register address (0-5)
            data (int): unsigned 32-bit value to write

        Returns:
            int: value read back from the register after the write

        Raises:
            IOError: if the device replies with an error
        """
        # request: (command, address, data)
        msg = struct.pack("<III", ord("w"), addr, data)
        self.socket_req.send(msg, 0)
        resp = self.socket_req.recv()

        # reply: ('r' = 114, value) on success, ('e',) on error
        msg = struct.unpack_from("<I", resp, 0)
        if msg[0] == 114:
            msg = struct.unpack_from("<I", resp, 4)
            return msg[0]
        else:
            raise IOError("Write Error")

    def read_msg(self, addr):
        """Read a control register.

        Args:
            addr (int): control register address (0-5)

        Returns:
            int: unsigned 32-bit register value

        Raises:
            IOError: if the device replies with an error
        """
        # request: (command, address, unused)
        msg = struct.pack("<III", ord('r'), addr, 0)
        self.socket_req.send(msg, 0)
        resp = self.socket_req.recv()

        # reply: ('r' = 114, value) on success, ('e',) on error
        msg = struct.unpack_from("<I", resp, 0)
        if msg[0] == 114:
            return struct.unpack_from("<I", resp, 4)[0]
        else:
            raise IOError("Read Error")

# HELPER FUNCTIONS =======================================================
def raw_to_voltage(val):
    """Convert a 24-bit offset-binary ADC code to input voltage.

    Args:
        val (int): ADC code in the low 24 bits (0x800000 = 0 V)

    Returns:
        float: voltage at the AD4112 input in V (code range spans +/-25 V
            because of the on-chip /10 divider)
    """
    # When the ADC is configured for bipolar operation, the output
    # code is offset binary with a negative full-scale voltage resulting
    # in a code of 000 … 000, a zero differential input voltage resulting in
    # a code of 100 … 000, and a positive full-scale input voltage
    # resulting in a code of 111 … 111. The output code for any
    # analog input voltage can be represented as
    # Code = 2^(N – 1) × ((VIN × 0.1)/VREF) + 1)
    # N = 24, Vin = input voltage, Vref = reference voltage (internal 2.5V)
    return (((val / 8388608) - 1) * VOLTAGE_REFERENCE) / 0.1

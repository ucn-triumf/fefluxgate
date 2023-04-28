#!/bin/python

import zmq
import time
import struct
import signal
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import configparser
import os
import logging
import pkg_resources
import datetime
from threading import Thread
#from . import discovery
import discovery

logger = logging.getLogger(__name__)

RATE = 2  # Rate in seconds
DEFAULT_PLOT_SAMPLES = 2000
VOLTAGE_REFERENCE = 2.5
VOLT_MAX = 12
VOLT_MIN = -12
CFG_DEFAULTS = {
    "; Allowed rate values [ 0 - 22 ]": None,
    "; Fastest rate is 0": None,
    "rate": 0,
    "; Allowed settle_delay values [ 0 - 7 ]": None,
    "settle_delay": 0,
    "; Allowed enhanced_filter values: [ 2, 3, 5, 6 ]": None,
    "enhanced_filter": 0,
    "; Allowed use_sinc3 choices: [ True | False ]": None,
    "use_sinc3": False,
    "; Allowed magnetometer enable bitmask: [ 0x0000 - 0xFFFF ]": None,
    "; 0x8000 is Mag 16, 0x0001 is Mag 1, 0x0002 is Mag 2 etc": None,
    "use_magnetometer": 0xFFFF
}

"""
CTRL+C Interrupt Handler
"""






class DuplicateFilter(logging.Filter):
    def filter(self, record):
        # add other fields if you need more granular comparison, depends on your app
        current_log = (record.module, record.levelno, record.msg)
        if current_log != getattr(self, "last_log", None):
            self.last_log = current_log
            return True
        return False


graph_data = []
for n in range(16):
    graph_data.append({"ts": [], "xs": [], "ys": [], "zs": []})


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

def sign_extend(value, bits):
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

class FG_Args:
  ip = "1.1.1.1"


def main():
    prog = "fluxgate-ctl"


    # Make my own dictionary
    myargs = FG_Args
    myargs.ip = "142.90.151.5"

    print("Got here")
    arg_data(myargs)

    print("Rate = " + str(myargs.rate) + " Mask= " + str(myargs.mag_mask))
    print("Got here4")




def arg_data(args):

    #    if not os.path.exists(args.ini):
    get_config(args)
    #else:
    #    set_config(args)

    #return
    context = zmq.Context()
    server = "tcp://" + args.ip + ":5555"
    bit_count = 0
    msg_count = 0

    #  Socket to talk to server
    print("Connecting via ZMQ to " + args.ip + ", writing to screen ")
    socket = context.socket(zmq.SUB)
    socket.connect(server)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    start_time = time.time()

    mag_count = [0] * 16

    first_calc_time = 0
    delta_calc_time = 0
    prev_tx_counter = False

    total_reads = 0

    for i in range(500000):
        try:
            message = socket.recv_multipart(flags=zmq.NOBLOCK)
            
            # Increment message count
            msg_count = msg_count + 1
            bit_count = bit_count + (len(message[0]) * 8)
            
            # Unpack message
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
            
            if prev_tx_counter != False:
                if tx_count != (prev_tx_counter + 1):
                    print("Dropped")

            prev_tx_counter = tx_count

            calc_time = time_sec + (time_nsec / 1000000000)
            
            if first_calc_time == 0:
                first_calc_time = calc_time

            delta_calc_time = calc_time - first_calc_time

            # Skip writing this if it isn't in the list
            # Can happen right after ini file gets written to module
            if(((1 << mag_id) & args.mag_mask) == 0):
                continue

            mag_count[mag_id] = mag_count[mag_id] + 1
            
            volt_x = raw_to_voltage(mag_x)
            volt_y = raw_to_voltage(mag_y)
            volt_z = raw_to_voltage(mag_z)
            
            mag_record = (
                str(tx_count)
                + ","
                + str(calc_time)
                + ","
                + str(mag_id + 1)
                + ","
                + str(mag_count[mag_id])
                + ","
                + str(mag_x)
                + ","
                + str(mag_y)
                + ","
                + str(mag_z)
                + ","
                + f"{volt_x:.6f}"
                + ","
                + f"{volt_y:.6f}"
                + ","
                + f"{volt_z:.6f}"
            )

            print(mag_record)

        except zmq.ZMQError:
            #print("No data!")
            # No message received, keep looping
            pass

    end_time = time.time()
    delta=1
    rate=0
    print("Got rate? " + str(end_time-start_time) + " " + str(start_time) + " " + str(RATE))
    if (end_time - start_time) >= RATE:
        print("rated!")
        delta = end_time - start_time
        start_time = end_time
        rate = msg_count/delta
    print(
        "Rate "
        + f"{(bit_count / delta / 1000000):.3f}"
        + " mbit/s ["
        + f"{int(msg_count / delta):d}"
        + " mag/s]",
        end="\r",
    )

    print("\n______ Rate " + str(rate) + "\n")
    

    bit_count = 0
    msg_count = 0
    first_calc_time = 0
    
    print("Magnetometer Counts")
    for n in range(16):
        print(str(n + 1) + ": " + str(mag_count[n]))




def write_msg(socket, addr, data):
    msg = struct.pack("<III", ord("w"), addr, data)
    socket.send(msg, 0)
    resp = socket.recv()
    msg = struct.unpack_from("<I", resp, 0)
    if msg[0] == 114:
        msg = struct.unpack_from("<I", resp, 4)
        return msg[0]
    else:
        raise ("Write Error")


def read_msg(socket, bus, addr):
    msg = struct.pack("<III", ord(bus), addr, 0)
    socket.send(msg, 0)
    resp = socket.recv()
    msg = struct.unpack_from("<I", resp, 0)
    if msg[0] == 114:
        return struct.unpack_from("<I", resp, 4)[0]
    else:
        raise ValueError("Read Error")

def arg_get_config(args):
    get_config(args)
    write_config(args)


def get_config(args):
    # Get config from module
    cfg = configparser.ConfigParser(CFG_DEFAULTS, allow_no_value=True)
    cfg.default_section = "default"
    context = zmq.Context()
    server = "tcp://" + args.ip + ":5556"
    socket = context.socket(zmq.REQ)
    socket.connect(server)

    args.enhfilter = read_msg(socket, "r", 2)  # enhanced_filter
    args.rate = read_msg(socket, "r", 3)  # rate
    args.delay = read_msg(socket, "r", 4)  # delay
    args.mag_mask = read_msg(socket, "r", 5)  # Magnetometer bitmask

    if(args.rate & 0x80000000):
        args.use_sinc3 = True
    else:
        args.use_sinc3 = False

    args.mag_mask = args.mag_mask & 0xFFFF
    args.rate = args.rate & 0x7FFFFFFF
    print("Rate = " + str(args.rate) + " Mask= " + str( args.mag_mask))

def write_config(args):
    cfg = configparser.ConfigParser(CFG_DEFAULTS, allow_no_value=True)
    cfg.default_section = "default"

    if (args.enhfilter == 0) and ((args.rate & 0x80000000) != 0):
        cfg.set("default", "use_sinc3", "True")
    else:
        cfg.set("default", "use_sinc3", "False")

    args.mag_mask = args.mag_mask & 0xFFFF
    args.rate = args.rate & 0x7FFFFFFF

    cfg.set("default", "enhanced_filter", str(args.enhfilter))
    cfg.set("default", "rate", str(args.rate))
    cfg.set("default", "settle_delay", str(args.delay))
    cfg.set("default", "use_magnetometer", hex(args.mag_mask))

    with open(args.ini, "w") as configfile:
        cfg.write(configfile)

    print("Wrote config to " + args.ini)

def set_config(args):
    cfg = configparser.ConfigParser(CFG_DEFAULTS, allow_no_value=True)
    cfg.default_section = "default"

    print("Using INI File " + args.ini)
    cfg.read(args.ini)

    context = zmq.Context()
    server = "tcp://" + args.ip + ":5556"
    socket = context.socket(zmq.REQ)
    socket.connect(server)

    args.rate = cfg.getint("default", "rate")
    args.delay = cfg.getint("default", "settle_delay")
    args.enhanced_filter = cfg.getint("default", "enhanced_filter")
    args.use_sinc3 = cfg.getboolean("default", "use_sinc3")
    args.mag_mask = int(cfg.get("default", "use_magnetometer"), 0) & 0xFFFF

    if args.enhanced_filter != 0:
        args.use_sinc3 = False
        args.rate = args.rate & 0x7FFFFFFF

    if args.use_sinc3:
        args.rate = args.rate | 0x80000000

    write_msg(socket, 2, args.enhanced_filter)
    write_msg(socket, 3, args.rate)
    write_msg(socket, 4, args.delay)
    write_msg(socket, 5, args.mag_mask)
    # toggle adc_soft_reset to start SPI load of values
    write_msg(socket, 0, 1)
    write_msg(socket, 0, 0)



def arg_write(args):
    try:
        context = zmq.Context()
        server = "tcp://" + args.ip + ":5556"

        #  Socket to talk to server
        socket = context.socket(zmq.REQ)
        socket.connect(server)
        resp = write_msg(socket, int(args.addr, 0), int(args.data, 0))
        print("0x{0:08X}".format(int(resp)) + " [" + str(int(resp)) + "]")

    except zmq.ZMQError:
        # No message received, keep looping
        pass
    pass


def arg_read(args):

    try:
        context = zmq.Context()
        server = "tcp://" + args.ip + ":5556"

        #  Socket to talk to server
        socket = context.socket(zmq.REQ)
        socket.connect(server)
        resp = read_msg(socket, "r", int(args.addr, 0))
        print("0x{0:08X}".format(int(resp)) + " [" + str(int(resp)) + "]")

    except ValueError:
        print("Bad address")
        pass

    except zmq.ZMQError:
        # No message received, keep looping
        pass


def arg_status(args):

    try:
        context = zmq.Context()
        server = "tcp://" + args.ip + ":5556"

        args.addr = int(args.addr, 0)

        #  Socket to talk to server
        socket = context.socket(zmq.REQ)
        socket.connect(server)
        resp = read_msg(socket, "s", args.addr)

        if(args.addr == 0):
            print("Hardware revision: " + str(resp))
        elif(args.addr == 1):
            print("Build timestamp: "  + str(datetime.datetime.fromtimestamp(resp)) + " (" + '0x{:08x}'.format(resp) + ")")
        else:
            print("0x{0:08X}".format(int(resp)) + "[" + str(int(resp)) + "]")


    except ValueError:
        print("Bad address")
        pass

    except zmq.ZMQError:
        # No message received, keep looping
        pass


if __name__ == "__main__":
    main()

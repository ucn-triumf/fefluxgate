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
class GracefulExiter:
    def __init__(self):
        self.state = False
        signal.signal(signal.SIGINT, self.change_state)

    def change_state(self, signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.state = True

    def exit(self):
        return self.state


flag = GracefulExiter()


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


def main():
    prog = "fluxgate-ctl"
    parser = argparse.ArgumentParser(prog=prog)
    #parser.add_argument("--version", action="version", version="%(prog)s " + pkg_resources.get_distribution('fluxgate-ctl').version)
    parser.add_argument("-i", "--ini", default="fluxgate.ini", help="Ini file to use")
    parser.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="Increase logging verbosity, can be repeated",
    )
    parser.add_argument("-l", "--log", metavar="file", help="Log to output file")
    parser.add_argument("ip", help="IP address / discover")

    cmd_parser = parser.add_subparsers(dest="command")

    data_parser = cmd_parser.add_parser("data")
    data_parser.set_defaults(func=arg_data)
    data_parser.add_argument("file", nargs="?", default="fluxgate_data.txt", help="File to write to")

    plot_parser = cmd_parser.add_parser("plot")
    plot_parser.set_defaults(func=arg_plot)
    plot_parser.add_argument("magnetometer", nargs="?", default='all', help="Magnetometer to plot")
    plot_parser.add_argument(
        "samples",
        nargs="?",
        default=DEFAULT_PLOT_SAMPLES,
        help="Number samples to plot",
    )

    status_parser = cmd_parser.add_parser("status")
    status_parser.set_defaults(func=arg_status)
    status_parser.add_argument("addr")

    read_parser = cmd_parser.add_parser("read")
    read_parser.set_defaults(func=arg_read)
    read_parser.add_argument("addr")

    write_parser = cmd_parser.add_parser("write")
    write_parser.set_defaults(func=arg_write)
    write_parser.add_argument("addr")
    write_parser.add_argument("data")

    set_cfg_parser = cmd_parser.add_parser("set-config")
    set_cfg_parser.set_defaults(func=set_config)

    get_cfg_parser = cmd_parser.add_parser("get-config")
    get_cfg_parser.set_defaults(func=arg_get_config)

    args = parser.parse_args()

    logger.setLevel(logging.DEBUG)
    logger.addFilter(DuplicateFilter())  # add the filter to it

    # Default logging is WARNING, -v gives INFO, -vv gives DEBUG
    levels = [logging.WARNING, logging.INFO, logging.DEBUG]
    # capped to number of levels
    level = levels[min(len(levels) - 1, args.verbose)]

    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    if args.log:
        handler = logging.FileHandler(args.log, mode="w")
    else:
        handler = logging.StreamHandler()

    handler.setLevel(level)
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    if args.ip.lower() == "discover":
        args.func = arg_discover

    if(hasattr(args,'func')):
        args.func(args)
    else:
        parser.print_help()


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

def arg_discover(args):
    # Load config
    config = discovery.DiscoveryConfig()

    # Override config with command line arguments

    controller = discovery.DiscoveryController(config)
    controller.start()

    flag = GracefulExiter()

    # while(1):
    #    if flag.exit() or not controller.is_alive():
    #        break
    time.sleep(2)

    controller.stop()


def arg_plot(args):

    global graph_data

    lines = []

    def get_data(graph_data):

        while not flag.exit():
            try:
                message = socket.recv_multipart(flags=zmq.NOBLOCK)

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

                # Skip writing this if it isn't in the list
                # Can happen right after ini file gets written to module
                if(((1 << mag_id) & args.mag_mask) == 0):
                    continue

                volt_x = raw_to_voltage(mag_x)
                volt_y = raw_to_voltage(mag_y)
                volt_z = raw_to_voltage(mag_z)

                # graph_data['ts'].append(time_sec + (time_nsec/1000000000))
                graph_data[mag_id]["xs"].append(volt_x)
                graph_data[mag_id]["ys"].append(volt_y)
                graph_data[mag_id]["zs"].append(volt_z)

                graph_data[mag_id]["xs"] = graph_data[mag_id]["xs"][-max_samples:]
                graph_data[mag_id]["ys"] = graph_data[mag_id]["ys"][-max_samples:]
                graph_data[mag_id]["zs"] = graph_data[mag_id]["zs"][-max_samples:]

            except zmq.ZMQError as e:
                # No message received, keep looping
                time.sleep(0.001)

    # This function is called periodically from FuncAnimation
    def animate(i, graph_data):
        # Draw x and y lists
        if args.magnetometer == "all":
            for i in range(16):
                lines[(i * 3) + 0].set_ydata(graph_data[i]["xs"])
                lines[(i * 3) + 1].set_ydata(graph_data[i]["ys"])
                lines[(i * 3) + 2].set_ydata(graph_data[i]["zs"])
        else:
            mag_id = int(args.magnetometer) - 1
            lines[0].set_ydata(graph_data[mag_id]["xs"])
            lines[1].set_ydata(graph_data[mag_id]["ys"])
            lines[2].set_ydata(graph_data[mag_id]["zs"])

        return lines

    def init_plot():
        max_samples = int(args.samples)
        for idx, a in enumerate(ax):
            if args.magnetometer == "all":
                a.set_title(str(idx + 1), x=0.1, y=0.9)
                a.set_xticks([0, max_samples / 2], rotation=315)
            else:
                a.set_title(str(idx + 1))
            a.set_xlim([0, max_samples])
            a.set_ylim(VOLT_MAX, VOLT_MIN)
            a.grid(True)

    if not os.path.exists(args.ini):
        get_config(args)
    else:
        set_config(args)

    #  Socket to talk to server
    context = zmq.Context()
    server = "tcp://" + args.ip + ":5555"

    socket = context.socket(zmq.SUB)
    socket.connect(server)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    max_samples = int(args.samples)

    fig = plt.figure(figsize=(160, 90))

    if args.magnetometer == "all":
        gs = fig.add_gridspec(2, 8, wspace=0.0, hspace=0.0)
        px = gs.subplots(sharex=True)
        ax = [
            px[0, 0],
            px[0, 1],
            px[0, 2],
            px[0, 3],
            px[0, 4],
            px[0, 5],
            px[0, 6],
            px[0, 7],
            px[1, 0],
            px[1, 1],
            px[1, 2],
            px[1, 3],
            px[1, 4],
            px[1, 5],
            px[1, 6],
            px[1, 7],
        ]
        fig.canvas.manager.set_window_title("All Magnetometers - Voltage over Time")
        fig.text(0.5, 0.04, "Sample #", ha="center", va="center")
        fig.text(
            0.06, 0.5, "Voltage (V)", ha="center", va="center", rotation="vertical"
        )
    else:
        if int(args.magnetometer) < 1:
            args.magnetometer = 1

        elif int(args.magnetometer) > 16:
            args.magnetometer = 16

        gs = fig.add_gridspec(1, hspace=0)
        ax = [gs.subplots(sharex=True)]
        fig.canvas.manager.set_window_title(
            "Magnetometer #" + str(int(args.magnetometer)) + " Voltage over Time"
        )

        plt.ylabel("Voltage (V)")
        plt.xlabel("Sample #")

    plt.ioff()

    for i in range(16):
        for n in range(max_samples):
            graph_data[i]["ts"].append(n)
            graph_data[i]["xs"].append(None)
            graph_data[i]["ys"].append(None)
            graph_data[i]["zs"].append(None)

    # Draw x and y lists
    if args.magnetometer == "all":
        for i in range(16):
            (line0,) = ax[i].plot(graph_data[i]["ts"], graph_data[i]["xs"], label="X")
            (line1,) = ax[i].plot(graph_data[i]["ts"], graph_data[i]["ys"], label="Y")
            (line2,) = ax[i].plot(graph_data[i]["ts"], graph_data[i]["zs"], label="Z")

            lines.append(line0)
            lines.append(line1)
            lines.append(line2)

            # Format plot
            for axs in ax:
                axs.label_outer()

        fig.legend(fig.axes[0].lines, ["X", "Y", "Z"], loc="upper center", ncols=3)

    else:
        mag_id = int(args.magnetometer) - 1

        (line0,) = ax[0].plot(
            graph_data[mag_id]["ts"], graph_data[mag_id]["xs"], label="X"
        )
        (line1,) = ax[0].plot(
            graph_data[mag_id]["ts"], graph_data[mag_id]["ys"], label="Y"
        )
        (line2,) = ax[0].plot(
            graph_data[mag_id]["ts"], graph_data[mag_id]["zs"], label="Z"
        )

        lines.append(line0)
        lines.append(line1)
        lines.append(line2)
        ax[0].legend(loc="upper left")

        # Format plot
        plt.xlim([0, max_samples])
        plt.grid(True)

    t = Thread(target=get_data, args=(graph_data,))
    t.start()

    # Set up plot to call animate() function periodically
    if args.magnetometer == "all":
        interval = 200
    else:
        interval = 50

    ani = animation.FuncAnimation(
        fig,
        animate,
        init_func=init_plot,
        fargs=(graph_data,),
        interval=interval,
        blit=True,
    )

    # Blocks until closed
    plt.show()

    # Tell thread to end
    flag.change_state(0, 0)


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


def arg_data(args):

    if not os.path.exists(args.ini):
        get_config(args)
    else:
        set_config(args)

    context = zmq.Context()
    server = "tcp://" + args.ip + ":5555"
    bit_count = 0
    msg_count = 0

    #  Socket to talk to server
    print("Connecting via ZMQ to " + args.ip + ", writing to " + args.file)
    socket = context.socket(zmq.SUB)
    socket.connect(server)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    start_time = time.time()

    mag_count = [0] * 16

    first_calc_time = 0
    delta_calc_time = 0
    prev_tx_counter = False

    with open(args.file, "wt") as run_file:
        run_file.write("tx_counter,timestamp,magnetometer,x,y,z,volt_x,volt_y,volt_z\n")

        while not flag.exit():
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

                if args.verbose > 0:
                    print(mag_record)

                # Write data to file as text
                run_file.write(mag_record + "\n")
            except zmq.ZMQError:
                # No message received, keep looping
                pass

            end_time = time.time()
            if (end_time - start_time) >= RATE:
                delta = end_time - start_time
                start_time = end_time

                print(
                    "Rate "
                    + f"{(bit_count / delta / 1000000):.3f}"
                    + " mbit/s ["
                    + f"{int(msg_count / delta):d}"
                    + " mag/s]",
                    end="\r",
                )

                bit_count = 0
                msg_count = 0
                first_calc_time = 0

            if flag.exit():
                if args.verbose > 0:
                    print("Magnetometer Counts")
                    for n in range(16):
                        print(str(n + 1) + ": " + str(mag_count[n]))
                break


if __name__ == "__main__":
    main()

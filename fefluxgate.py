"""
Example of a basic midas frontend that has one periodic equipment.

See `examples/multi_frontend.py` for an example that uses more
features (frontend index, polled equipment, ODB settings etc). 
"""

import midas
import midas.frontend
import midas.event

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
import collections
import pkg_resources
import datetime
from threading import Thread
#from . import discovery                                                                                                                        
import discovery

last_voltage = [[0.0 for x in range(16)] for y in range(3)] 
number_packets = 0
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


class FluxGate(midas.frontend.EquipmentBase):
    fcontext = None
    fsocket = None
    fpoller = None
    fmessage = None
    """
    We define an "equipment" for each logically distinct task that this frontend
    performs. For example, you may have one equipment for reading data from a
    device and sending it to a midas buffer, and another equipment that updates
    summary statistics every 10s.
    
    Each equipment class you define should inherit from 
    `midas.frontend.EquipmentBase`, and should define a `readout_func` function.
    If you're creating a "polled" equipment (rather than a periodic one), you
    should also define a `poll_func` function in addition to `readout_func`.
    """
    def __init__(self, client):
        # The name of our equipment. This name will be used on the midas status
        # page, and our info will appear in /Equipment/MyPeriodicEquipment in
        # the ODB.
        equip_name = "FluxGate"
        
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 2323
        default_common.period_ms = 5000
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 5
        
        # Settings
        default_settings = collections.OrderedDict([  
            ("host_ip", "142.90.151.5"),
            ("rate", 0),
            ("settle_delay",0),
            ("enhanced_filter", 0),
            ("use_sinc3", False),
            ("use_magnetometer", 0xffff)
        ])    


        # You MUST call midas.frontend.EquipmentBase.__init__ in your equipment's __init__ method!
        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common,default_settings)
        
        # Get IP address
        self.host_ip = self.client.odb_get(self.odb_settings_dir + "/host_ip")

        # Setup callback on port enable ODB keys
        self.client.odb_watch(self.odb_settings_dir, self.settings_callback)

        global number_packets
        number_packets = 0

        # You can set the status of the equipment (appears in the midas status page)
        self.set_status("Initialized")
        


    def readout_func(self):
        """
        For a periodic equipment, this function will be called periodically
        (every 100ms in this case). It should return either a `cdms.event.Event`
        or None (if we shouldn't write an event).
        """
        
        # Get the current configuration
        cfg = configparser.ConfigParser(CFG_DEFAULTS, allow_no_value=True)
        cfg.default_section = "default"
        context = zmq.Context()
        server = "tcp://"+self.host_ip+":5556"
        socket = context.socket(zmq.REQ)
        socket.connect(server)

        enhfilter = read_msg(socket, "r", 2)  # enhanced_filter
        rate = read_msg(socket, "r", 3)  # rate
        delay = read_msg(socket, "r", 4)  # delay
        mag_mask = read_msg(socket, "r", 5)  # Magnetometer bitmask
        
        if(rate & 0x80000000):
            use_sinc3 = True
        else:
            use_sinc3 = False

        mag_mask = mag_mask & 0xFFFF
        rate = rate & 0x7FFFFFFF

        print("Rate : " + str(rate) + "  mask: " + str(mag_mask) + "  enhfilter: " + str(enhfilter))

        # get a sample flux gate value

        event = midas.event.Event()
        
        voltage_updated = False
        
        # Only update flux gate voltage bank once we have good data;
        # Wait for 100 packets at least
        if number_packets > 100: 

        
            # Create a bank (called "MYBK") which in this case will store 8 ints.
            # data can be a list, a tuple or a numpy array.
            #data = [[0 for x in range(16)] for y in range(3)]
            data = []
            for i in range(16):
                for j in range(3):
                    data.append(last_voltage[j][i])
 
            event.create_bank("FG00", midas.TID_FLOAT, data)

            #print(last_voltage[0][0])
        
        
        return event


    def settings_callback(self,client, path, odb_value):
        """
        Callback function when setting tree is change
        """
        print("New ODB content at %s is %s" % (path, odb_value))
        
        #print(str(odb_value['host_ip']))
        context = zmq.Context()
        server = "tcp://" +self.host_ip + ":5556"
        socket = context.socket(zmq.REQ)
        socket.connect(server)

        rate = odb_value['rate']
        delay = odb_value['settle_delay']
        enhanced_filter = odb_value['enhanced_filter']
        use_sinc3 = odb_value['use_sinc3']
        mag_mask = odb_value['use_magnetometer'] & 0xFFFF
        

        # do checks to make sure values are sensible
        if rate > 22 or rate < 0:
            self.client.msg("Invalid value for rate="+str(rate)+"; must be between 0-22.",True)
            rate = 0;
        if delay > 7 or delay < 0:
            self.client.msg("Invalid value for settle_delay="+str(delay)+"; must be between 0-7.",True)
            rate = 0;

        filter_list = [0,2,3,5,6]
        if  not enhanced_filter in filter_list:
            self.client.msg("Invalid value for enhanced_list="+str(enhanced_filter)+".  Must be in list="+str(filter_list),True)
          
            enhanced_filter = 0

        self.client.msg("New flux_gate settings: rate="+str(rate)+" settle_delay="+str(delay)+"; enhanced_filter="+str(enhanced_filter)+"mag_mask="+str(mag_mask))
            

        if enhanced_filter != 0:
            use_sinc3 = False
            rate = rate & 0x7FFFFFFF
            
        if use_sinc3:
            rate = rate | 0x80000000

      

        write_msg(socket, 2, enhanced_filter)
        write_msg(socket, 3, rate)
        write_msg(socket, 4, delay)
        write_msg(socket, 5, mag_mask)
        # toggle adc_soft_reset to start SPI load of values
        write_msg(socket, 0, 1)
        write_msg(socket, 0, 0)



class FluxGateData(midas.frontend.EquipmentBase):
    fcontext = None
    fsocket = None
    fpoller = None
    fmessage = None
    """
    We define an "equipment" for each logically distinct task that this frontend
    performs. For example, you may have one equipment for reading data from a
    device and sending it to a midas buffer, and another equipment that updates
    summary statistics every 10s.
    
    Each equipment class you define should inherit from 
    `midas.frontend.EquipmentBase`, and should define a `readout_func` function.
    If you're creating a "polled" equipment (rather than a periodic one), you
    should also define a `poll_func` function in addition to `readout_func`.
    """
    def __init__(self, client):
        # The name of our equipment. This name will be used on the midas status
        # page, and our info will appear in /Equipment/MyPeriodicEquipment in
        # the ODB.
        equip_name = "FluxGateData"
        
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_POLLED
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 1
        default_common.period_ms = 0
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 0
        
        # You MUST call midas.frontend.EquipmentBase.__init__ in your equipment's __init__ method!
        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common)
        

        # Setup connection to ZMQ socket
        context = zmq.Context()
        self.fcontext = context
        ip = "142.90.151.5"
        server = "tcp://" + ip + ":5555"
        bit_count = 0
        msg_count = 0
        
        #  Socket to talk to server                                                                                                                 
        print("Connecting via ZMQ to " + ip + ", writing to screen ")
        socket = context.socket(zmq.SUB)
        socket.connect(server)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.fsocket = socket
        print("Is socket closed? " + str(context.closed))

        # Setup polling
        #poller = zmq.Poller()
        #poller.register(socket, zmq.POLLIN)
        #self.fpoller = poller

        # You can set the status of the equipment (appears in the midas status page)
        self.set_status("Initialized")
        
    def poll_func(self):
        """
        This function is called very frequently and should return True/False
        depending on whether an event is ready to be read out. It should be
        a quick function. 
        
        In this case we're just choosing random number as we don't have any
        actual device hooked up.
        """
        #gotdata = False
        #socks = dict(self.fpoller.poll(1))                                                                                                
        #if self.fsocket in socks and socks[self.fsocket] == zmq.POLLIN:                                                                    
        #    gotdata=True
        gotdata = False
        try:
            message = self.fsocket.recv_multipart(flags=zmq.NOBLOCK)
            self.fmessage = message
            gotdata = True 
        except zmq.ZMQError:
            gotdata = False
            #print("No data!")



        return gotdata


    def readout_func(self):
        
        # read first event
        message = self.fmessage
        
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
        
        calc_time = time_sec + (time_nsec / 1000000000)

        volt_x = raw_to_voltage(mag_x)
        volt_y = raw_to_voltage(mag_y)
        volt_z = raw_to_voltage(mag_z)

        # cache the values to use in history
        last_voltage[0][mag_id] = volt_x
        last_voltage[1][mag_id] = volt_y
        last_voltage[2][mag_id] = volt_z

        global number_packets
        number_packets = number_packets + 1

        # In this example, we just make a simple event with one bank.
        event = midas.event.Event()
        
        # Create a bank (called "MYBK") which in this case will store 8 ints.
        # data can be a list, a tuple or a numpy array.
        data = []
        data.append(calc_time)
        data.append(tx_count)
        data.append(mag_id)
        data.append(volt_x)
        data.append(volt_y)
        data.append(volt_z)

        event.create_bank("FGD0", midas.TID_FLOAT, data)
        
        list_events = []
        list_events.append(event)

        # Now try to quickly get 15 more events... (16 channels)
        for x in range(15):
            try:
                message = self.fsocket.recv_multipart(flags=zmq.NOBLOCK)
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
                
                calc_time = time_sec + (time_nsec / 1000000000)
                
                volt_x = raw_to_voltage(mag_x)
                volt_y = raw_to_voltage(mag_y)
                volt_z = raw_to_voltage(mag_z)
                number_packets = number_packets + 1
                event = midas.event.Event()
        
                data = []
                data.append(calc_time)
                data.append(tx_count)
                data.append(mag_id)
                data.append(volt_x)
                data.append(volt_y)
                data.append(volt_z)
                
                event.create_bank("FGD0", midas.TID_FLOAT, data)
                
                list_events.append(event)

                
            except zmq.ZMQError:
                total_bad = 0

        
        
        return list_events

class MyFrontend(midas.frontend.FrontendBase):
    """
    A frontend contains a collection of equipment.
    You can access self.client to access the ODB etc (see `midas.client.MidasClient`).
    """
    def __init__(self):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "fefluxgate")
        
        self.add_equipment(FluxGateData(self.client))
        self.add_equipment(FluxGate(self.client))

        self.client.msg("Fluxgate frontend initialized.")
        
    def begin_of_run(self, run_number):
        """
        This function will be called at the beginning of the run.
        You don't have to define it, but you probably should.
        You can access individual equipment classes through the `self.equipment`
        dict if needed.
        """
        self.set_all_equipment_status("Running", "greenLight")
        self.client.msg("Frontend has seen start of run number %d" % run_number)
        return midas.status_codes["SUCCESS"]
        
    def end_of_run(self, run_number):
        self.set_all_equipment_status("Finished", "greenLight")
        self.client.msg("Frontend has seen end of run number %d" % run_number)
        return midas.status_codes["SUCCESS"]
    
    def frontend_exit(self):
        """
        Most people won't need to define this function, but you can use
        it for final cleanup if needed.
        """
        self.client.msg("Fluxgate frontend stopped.")
        
if __name__ == "__main__":
    # The main executable is very simple - just create the frontend object,
    # and call run() on it.
    with MyFrontend() as my_fe:
        my_fe.run()

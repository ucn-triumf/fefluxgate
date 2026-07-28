"""
Poll the orange TRIUMF Fluxgate DAQ box

Allowed rate values [ 0 - 22 ], fastest rate is 0
Allowed settle_delay values [ 0 - 7 ]
Allowed enhanced_filter values: [ 2, 3, 5, 6 ]
Allowed use_sinc3 choices: [ True | False ]
Allowed magnetometer enable bitmask: [ 0x0000 - 0xFFFF ]
    0x8000 is Mag 16, 0x0001 is Mag 1, 0x0002 is Mag 2 etc

"""

import midas
import midas.frontend
import midas.event
import numpy as np
import collections
from FluxgateBox import FluxgateBox
                                                                                            
VOLTAGE_REFERENCE = 2.5

class FluxGate(midas.frontend.EquipmentBase):
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
        default_common.period_ms = 1000
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 10
        
        # Settings
        default_settings = collections.OrderedDict([  
            ("host_ip", "142.90.151.5"),
            ("rate", 0),
            ("settle_delay",0),
            ("enhanced_filter", 0),
            ("use_sinc3", False),
            ("use_magnetometer", 0xffff),
            ("poll_ms", 10),
        ])    

        # You MUST call midas.frontend.EquipmentBase.__init__ in your equipment's __init__ method!
        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common,default_settings)
        
        # Create inteface with fluxgate box
        self.fluxbox = FluxgateBox(self.settings['host_ip'], poll_ms=self.settings['poll_ms'])
        self.fluxbox.start() # start polling

        # You can set the status of the equipment (appears in the midas status page)
        self.set_status("Initialized")
        
    def readout_func(self):
        """
        For a periodic equipment, this function will be called periodically
        (every 100ms in this case). It should return either a `cdms.event.Event`
        or None (if we shouldn't write an event).
        """
        
        # get a sample fluxgate value
        event = midas.event.Event()    
    
        data = np.zeros(16*3, dtype=float)
        idx = 0
        for i in range(16):
            for j in range(3):
                data[idx] = self.fluxbox.data[i][j+3]
                idx += 1

        event.create_bank("FG00", midas.TID_FLOAT, data)

        return event

    def stop(self):
        """Stop polling"""
        self.fluxbox.stop()

    def settings_changed_func(self):
        """
        Callback function when setting tree is changed
        """
        
        rate = self.settings['rate']
        delay = self.settings['settle_delay']
        enhanced_filter = self.settings['enhanced_filter']
        use_sinc3 = self.settings['use_sinc3']
        mag_mask = self.settings['use_magnetometer'] & 0xFFFF
        
        # do checks to make sure values are sensible
        if rate > 22 or rate < 0:
            self.client.msg(f"Invalid value for rate={rate}; must be between 0-22.", True)
            rate = 0;
        if delay > 7 or delay < 0:
            self.client.msg(f"Invalid value for settle_delay={delay}; must be between 0-7.",True)
            delay = 0;

        filter_list = [0,2,3,5,6]
        if  not enhanced_filter in filter_list:
            self.client.msg(f"Invalid value for enhanced_list={enhanced_filter}.  Must be in list={filter_list}",True)
            enhanced_filter = 0

        self.client.msg(f"New FluxGate settings: rate={rate} settle_delay={delay}; enhanced_filter={enhanced_filter} mag_mask={mag_mask}")
            
        if enhanced_filter != 0:
            use_sinc3 = False
            rate = rate & 0x7FFFFFFF
            
        if use_sinc3:
            rate = rate | 0x80000000

        self.fluxbox.set_filter(enhanced_filter)
        self.fluxbox.set_rate(rate)
        self.fluxbox.set_delay(delay)
        self.fluxbox.set_bitmask(mag_mask)

        # toggle adc_soft_reset to start SPI load of values
        self.fluxbox.toggle_reset()

class MyFrontend(midas.frontend.FrontendBase):
    """
    A frontend contains a collection of equipment.
    You can access self.client to access the ODB etc (see `midas.client.MidasClient`).
    """
    def __init__(self):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "fefluxgate")
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
        self.equipment['FluxGate'].stop() # stop polling
        self.client.msg("Fluxgate frontend stopped.")
        
if __name__ == "__main__":
    # The main executable is very simple - just create the frontend object,
    # and call run() on it.
    with MyFrontend() as my_fe:
        my_fe.run()

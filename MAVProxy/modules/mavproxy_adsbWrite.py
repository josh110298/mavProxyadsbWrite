#!/usr/bin/env python
'''
Example Module
Peter barker, September 2016

This module simply serves as a starting point for your own MAVProxy module.

1. copy this module sidewise (e.g. "cp mavproxy_example.py mavproxy_coolfeature.py"
2. replace all instances of "example" with whatever your module should be called
(e.g. "coolfeature")

3. trim (or comment) out any functionality you do not need
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from mavproxy_adsb import ADSBVehicle

class adsbWrite(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(adsbWrite, self).__init__(mpstate, "adsbWrite", "")
        self.active_threat_ids = []  # holds all threat ids the vehicle is evading
        self.threat_vehicles = {}
        self.adsbWrite_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('adsbWrite', self.cmd_adsbWrite, "adsbWrite module", ['status','set (LOGSETTING)'])

    def usage(self):
        '''write adsb to a file'''
        return "Usage: adsbWrite <status|set>"

    def cmd_adsbWrite(self, args):
        '''control behaviour of the module'''
        f = open('adsbThreats.txt', 'w')
        for id in self.threat_vehicles:
            if self.threat_vehicles[id].distance <= 100:
                f.write('id: %s lat: %s long: %s distance: %s \n' % (id,  self.threat_vehicles[id].state['lat']*1e-7,  self.threat_vehicles[id].state['lon']*1e-7,  self.threat_vehicles[id].distance))
        f.close

    def update_threat_distances(self, latlonalt):
        '''update the distance between threats and vehicle'''
        for id in self.threat_vehicles.keys():
            threat_latlonalt = (self.threat_vehicles[id].state['lat'] * 1e-7,
                                self.threat_vehicles[id].state['lon'] * 1e-7,
                                self.threat_vehicles[id].state['altitude'])

            self.threat_vehicles[id].h_distance = self.get_h_distance(latlonalt, threat_latlonalt)
            self.threat_vehicles[id].v_distance = self.get_v_distance(latlonalt, threat_latlonalt)

            # calculate and set the total distance between threat and vehicle
            self.threat_vehicles[id].distance = math.sqrt(self.threat_vehicles[id].h_distance**2 + (self.threat_vehicles[id].v_distance)**2)
    def get_h_distance(self, latlonalt1, latlonalt2):
        '''get the horizontal distance between threat and vehicle'''
        (lat1, lon1, alt1) = latlonalt1
        (lat2, lon2, alt2) = latlonalt2

        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dLat = lat2 - lat1
        dLon = lon2 - lon1

        # math as per mavextra.distance_two()
        a = math.sin(0.5 * dLat)**2 + math.sin(0.5 * dLon)**2 * math.cos(lat1) * math.cos(lat2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return 6371 * 1000 * c

    def get_v_distance(self, latlonalt1, latlonalt2):
        '''get the horizontal distance between threat and vehicle'''
        (lat1, lon1, alt1) = latlonalt1
        (lat2, lon2, alt2) = latlonalt2
        return alt2 - alt1

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == "ADSB_VEHICLE":
            id = 'ADSB-' + str(m.ICAO_address)
            if id not in self.threat_vehicles.keys():  # check to see if the vehicle is in the dict
                    # if not then add it
                self.threat_vehicles[id] = ADSBVehicle(id=id, state=m.to_dict())
            else:  # the vehicle is in the dict
                    # update the dict entry
                self.threat_vehicles[id].update(m.to_dict())
        if m.get_type() == "GLOBAL_POSITION_INT":
            self.update_threat_distances((m.lat*1e-7,  m.lon*1e-7,  m.alt*1e-3))
def init(mpstate):
    '''initialise module'''
    return writeADSB(mpstate)

def init(mpstate):
    '''initialise module'''
    return adsbWrite(mpstate)

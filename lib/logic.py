
import struct

import pycom
import utime
import machine

from pytrack import Pytrack
from L76GNSS import L76GNSS
from LIS2HH12 import LIS2HH12

# Boards and Sensors:
_py = Pytrack()
_acc = LIS2HH12(_py)
_gps = L76GNSS(_py)

class Application:
    """
    Pytrack basic application for GPS logging
    """

    def __init__(self, sock=None, lora=None, board=_py, gps=_gps, acc=_acc):
        """
        Initialiaze application
        """
        
        # Components:
        self._board = board
        self._gps = gps
        self._acc = acc
        self._sock = sock
        self._lora = lora

        # Timers:
        self._measure_clock = machine.Timer.Chrono()
        self._lora_clock = machine.Timer.Chrono()

        # States:
        self._id = 0
        self._last = None

        print("APP: Initialiazed")

    @property
    def board(self):
        return self._board

    @property
    def gps(self):
        return self._gps

    @property
    def acc(self):
        return self._acc

    @property
    def sock(self):
        return self._sock

    @property
    def lora(self):
        return self._lora

    def measure(self, timeout=5.0, debug=False):
        """
        Measure available data on the board.
        """
        data = dict()

        # Tag measure:
        data['id'] = self._id
        self._id += 1
        
        # Read GPS (heavy duty cycle):
        self.gps.read(timeout=timeout, targets=['GPGGA', 'GPVTG'], debug=debug)
        data['coords'] = self.gps.coords(debug=debug)
        data['speed'] = self.gps.speed(debug=debug)

        # Read Sensor (quicker):
        data['acceleration'] = self.acc.acceleration()
        data['roll'] = self.acc.roll()
        data['pitch'] = self.acc.pitch()

        if debug:
            print("APP-MEASURE [id={}]: {}".format(data['id'], data))

        return data

    def send(self, payload, save=False):
        """
        Send payload through socket if defined (Uplink)
        """
        if self.sock is None:
            print("APP-SEND ERROR: Cannot send, no socket defined")
        else:
            # https://forum.pycom.io/topic/3780/lorawan-frames-counter-does-not-work-after-deepsleep-if-socket-is-set-to-non-blocking/2
            #self.sock.setblocking(True)
            n = self.sock.send(payload)
            #self.sock.setblocking(False) # Maybe linked in Downlin issue
            print("APP-SENT [size={}]: {}".format(n, payload))
            # Save LoRa state:
            if save:
                self.lora.nvram_save()
            return n

    def recv(self, size=64, debug=False, save=False):
        """
        Receive payload through socket if defined (Downlink)
        """
        if self.sock is None:
            print("APP-RECV ERROR: Cannot receive, no socket defined")
        else:
            # Checkout for Downlink:
            payload, port = self.sock.recvfrom(size)
            if (port > 0) or debug:
                print("APP-RECV [size={}, port={}]: {} ".format(len(payload), port, payload))
            # Save LoRa state:
            if save:
                self.lora.nvram_save()
            return payload, port

    @staticmethod
    def encode(data):
        """
        Encode data into application payload:
        """
        rep = data.copy()
        # Convert Metrics:
        t = data['coords']['time']
        s = int(data["coords"]["lat"] is None) | (data['coords']['fix'] << 1)
        d = ((t[0]*60+t[1])*60+t[2])
        lon = round((data['coords']['lon'] or 1e3)*1e6)
        lat = round((data['coords']['lat'] or 1e3)*1e6)
        h = round((data['coords']['height'] or 0))
        dop = round((data["coords"]["hdop"] or 0)*10)
        # Encode Metrics:
        payload = b''
        payload += struct.pack("<b", s)
        payload += struct.pack("<I", d)
        payload += struct.pack("<I", lon)
        payload += struct.pack("<I", lat)
        payload += struct.pack("<h", h)
        payload += struct.pack("<h", dop)
        rep['payload'] = payload
        print("APP-ENCODE [size={}]: {}".format(len(rep['payload']), rep))
        return rep

    def emit(self, timeout=15.0, debug=False):
        """
        Emit a measure through socket
        """
        try:
            m = self.measure(timeout=timeout)
            rep = self.encode(m)
            n = self.send(rep['payload'])
            if debug:
                print("APP-EMIT [size={}]: {}".format(n, rep))
        except OSError as err:
            print("APP-EMIT ERROR: {}".format(err))

    def start(self, measure_period=1, lora_period=20, gps_timeout=5*60, gps_retry=5,
              mode='eco', dryrun=False, debug=False, color=0x007f00):
        """
        Start application
        """

        assert mode in ('eco', 'power')
        print("APP [mode={}]: Started".format(mode))

        # Get a Fix from GPS:
        if not dryrun:
            self.gps.fix(timeout=gps_timeout, retry=gps_retry)

        # Mode Eco, measure, send data and go to deepsleep
        if mode == 'eco':

            #self.measure(debug=False)
            if not dryrun:
                self.emit(timeout=30.0)
            
            # More or less safely shutdown the device:
            utime.sleep(1.)
            machine.deepsleep(1000*lora_period)

        # Mode Power:
        if mode == 'power':

            self._measure_clock.reset()
            self._measure_clock.start()

            self._lora_clock.reset()
            self._lora_clock.start()

            # Application Loop
            while True:

                # Class C: Get Downlink Payload if any (eg.: TWljcm9QeXRob24=)
                downlink, port = self.recv(debug=debug)
                if downlink:
                    # Branch command logic here...
                    pass

                # LoRa Cycle:
                if self._lora_clock.read() >= lora_period:

                    # Blink (light on):
                    pycom.rgbled(color)
                    
                    if dryrun:
                        # Send a heavy dummy packet:
                        self.send(b'\x02'*56)
                    else:
                        self.emit(timeout=90.)

                    self._lora_clock.reset()

                    # Blink (light off):
                    pycom.rgbled(0x000000)

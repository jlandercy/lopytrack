
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

        print("APPLICATION: Initialiazed")

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

    def measure(self, timeout=5.0, debug=False, show=False):
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

        if show:
            print("MEASURES [{}]: {}".format(data['id'], data))

        return data

    def send(self, payload):
        """
        Send payload through socket if defined
        """
        if self.sock is None:
            print("ERROR: Cannot send, no socket defined")
        else:
            n = self.sock.send(payload)
            print("SENT [{}]: {}".format(n, payload))
            ack = self.sock.recv(64)
            print("ACK [{}]: {}".format(len(ack), ack))
            # Save LoRa state:
            utime.sleep(1.)
            self.lora.nvram_save()
            utime.sleep(1.)
            return ack

    @staticmethod
    def encode(data):
        """
        Encode data into LoRa physical payload:
        """
        rep = data.copy()
        # Test payload not compliant with ISM allowed duty cycle:
        vec = (
            float(data['coords']['time']), data['coords']['lat'], data['coords']['lon'], data['coords']['height'], data['coords']['hdop'],
            data['acceleration'][0], data['acceleration'][1], data['acceleration'][2],
            data['roll'], data['pitch']
        )
        vec = tuple([float('nan') if x is None else x for x in vec])
        rep['data'] = vec
        rep['payload'] = struct.pack("%uf" % len(vec), *vec)
        """
        # Minimalist Payload:
        # https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude
        # len(bin(-180*10000*4))-2 # 24
        # int(-180*10000)<<24 + int(90*10000) makes the board crash because + takes precedence to <<
        payload = b''
        payload += struct.pack("<h", data['coords']['time'])
        rep['payload'] = payload
        """
        print("LORA-DATA [{}]: {}".format(len(rep['payload']), rep))
        return rep

    def emit(self, timeout=5.0):
        """
        Emit a measure through LoRaWAN
        """
        #try:
        m = self.measure()
        rep = self.encode(m)
        ack = self.send(rep['payload'])
        #except:
        #    pass

    def start(self, measure_period=1, lora_period=20, mode='eco', dryrun=False, debug=False, show=True, color=0x007f00):
        """
        Start application, mainly sample measures, aggregate and send through LoRa
        """

        assert mode in ('eco', 'power')
        print("APPLICATION: Started in mode '{}'".format(mode))

        # Mode Eco, measure, send data and go to deepsleep
        if mode == 'eco':

            #self.measure(debug=False, show=False)
            self.emit(timeout=30.0)
            utime.sleep(1.)
            machine.deepsleep(1000*lora_period)

        # Mode power
        if mode == 'power':

            self._measure_clock.reset()
            self._measure_clock.start()

            self._lora_clock.reset()
            self._lora_clock.start()

            # Application Loop
            while True:

                # Measure:
                # if self._measure_clock.read() >= measure_period:

                #     self.measure(debug=debug, show=show)
                #     self._measure_clock.reset()

                if self._lora_clock.read() >= lora_period:

                    # Blink (light on):
                    pycom.rgbled(color)
                    
                    self.emit()

                    self._lora_clock.reset()

                    # Blink (light off):
                    pycom.rgbled(0x000000)


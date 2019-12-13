
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

    def __init__(self, sock=None, board=_py, gps=_gps, acc=_acc):
        """
        Initialiaze application
        """
        
        # Components:
        self._board = board
        self._gps = gps
        self._acc = acc
        self._sock = sock

        # Timers:
        self._measure_clock = machine.Timer.Chrono()
        self._lora_clock = machine.Timer.Chrono()

        # States:
        self._id = 0
        self._last = None

        print("Application initialiazed")

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

    def measure(self, debug=False):
        """
        Measure available data on the board.
        """
        data = dict()

        # Tag measure:
        data['id'] = self._id
        self._id += 1
        
        # Read Sensors:
        
        self.gps.read(timeout=5.0, targets=['GPGGA', 'GPVTG'])
        data['coords'] = self.gps.coords()
        data['speed'] = self.gps.speed()

        # data['acceleration'] = self.acc.acceleration()
        # data['roll'] = self.acc.roll()
        # data['pitch'] = self.acc.pitch()

        print("Measure [{}]: {}".format(data['id'], data))
        return data

    def send(self, payload):
        """
        Send payload through socket if defined
        """
        if self.sock is None:
            print("ERROR: Cannot send, no socket defined")
        else:
            n = self.sock.send(payload)
            print("Sent Frame [{}]: {}".format(n, payload))
            ack = self.sock.recv(64)
            print("Ack [{}]: {}".format(len(ack), ack))

    @staticmethod
    def encode(data):
        """
        Encode data into LoRa physical payload:
        """
        rep = data.copy()
        vec = (
            data['coords'][0], data['coords'][1],
            data['acceleration'][0], data['acceleration'][1], data['acceleration'][2],
            data['roll'], data['pitch'],
            float(data.get('count', 'nan')), float(data.get('missed', 'nan'))
        )
        rep['data'] = vec
        rep['payload'] = struct.pack("%uf" % len(vec), *vec)
        print("Data Encoded: {}".format(rep))
        return rep

    def start(self, measure_period=1, lora_period=20, mode='eco', dryrun=False, debug=False, color=0x007f00):
        """
        Start application, mainly sample measures, aggregate and send through LoRa
        """

        assert mode in ('eco', 'power')
        
        if mode == 'eco':
            self.measure(debug=debug)
            machine.deepsleep(1000*lora_period)

        if mode == 'power':
            self._measure_clock.reset()
            self._measure_clock.start()

            self._lora_clock.reset()
            self._lora_clock.start()

            while True:

                print(self._lora_clock.read(), self._measure_clock.read())

                # Measure:
                if self._measure_clock.read() >= measure_period:
                    self.measure(debug=debug)
                    self._measure_clock.reset()

                if self._lora_clock.read() >= lora_period:

                    # Blink (light on):
                    pycom.rgbled(color)
                    
                    self._lora_clock.reset()

                    # Blink (light off):
                    pycom.rgbled(0x000000)

                


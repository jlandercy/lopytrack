#!/usr/bin/env python
#
# Copyright (c) 2019, Jean Landercy
#

import gc

from micropython import const
import uio
import ure

from machine import Timer


class L76GNSS:
    """
    PyTrack Quectel L76 GPS Module

    Heavily refactored from:

       - https://github.com/pycom/pycom-libraries/blob/master/pytrack/lib/L76GNSS.py
    
    Constructor documentations:

       - https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Hardware_Design_V1.1.pdf
       - https://www.quectel.com/UploadImage/Downlad/L76_GNSS_Protocol_Specification_V1.3.pdf

    NMEA Sentences:
       - Usual received sentences: {'GNRMC', 'GLGSV', 'GNGSA', 'GPVTG', 'GNGLL', 'GPGSV', 'GPGGA'}
       - https://www.gpsinformation.org/dale/nmea.htm
       - https://en.wikipedia.org/wiki/NMEA_0183
       - http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
       - http://aprs.gids.nl/nmea/
    """

    # GPS L76 I2C Bus Id:
    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21', debug=False):
        """
        Initialize Quectel L76 GPS Class
        """
        
        # Bind I2C component:
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        # Options:
        self.debug = debug

        # GPS Storage:
        self._set_buffer()
        #self._lastfix = None
        self._lastframes = dict()
        self._satellites = dict()
        
        # NMEA Regular Expression:
        self._NMEA = ure.compile(b"\$(G....)(.*)\*([0-9A-F]*)\r")

        # Time Management:
        self._watchdog = Timer.Chrono()
        self._fixtime = Timer.Chrono()
        #self._fixtime.reset()
        #self._fixtime.start()

        # L76 Initialization:
        self.reg = bytearray(1)
        self.i2c.writeto(L76GNSS.GPS_I2CADDR, self.reg)

    def _read(self, n=64):
        """
        Read and buffer n bytes from the L76 over the I2C bus:
        L76 claims to be abvle to read up to n 255 characters from the I2C
        """
        self.reg = self.i2c.readfrom(L76GNSS.GPS_I2CADDR, n)
        return self.reg

    def convert_coords(self, coord, head):
        """
        Convert NMEA coordinates to decimal Lon/Lat coordinates
        """
        if coord:
            coord = (float(coord) // 100) + ((float(coord) % 100) / 60)
            if head in ('S', 'W'):
                coord *= coord
            return coord

    def _set_buffer(self, line=b''):
        """
        Truncate buffer (Missing truncate function in uio.bytesIO object)
        """
        self._buffer = uio.BytesIO()
        self._buffer.write(line)
        gc.collect()

    def checksum(self, payload):
        """
        Compute NMEA Checksum
        """
        checksum = 0
        for s in payload:
            checksum ^= s
        return checksum

    def _safe_float(self, s):
        """
        Safely create float:
        """
        try:
            return float(s)
        except:
            return None

    def _safe_int(self, s):
        """
        Safely create int:
        """
        try:
            return int(s)
        except:
            return None

    def _GPGGA(self, payload):
        """
        Decode NMEA GPGCA Type (essential fix data which provide 3D location and accuracy data)

        $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

        Where:
                    GGA          Global Positioning System Fix Data
            0)      123519       Fix taken at 12:35:19 UTC
            1-2)    4807.038,N   Latitude 48 deg 07.038' N
            3-4)    01131.000,E  Longitude 11 deg 31.000' E
            5)                 Fix quality: 0 = invalid
                                    1 = GPS fix (SPS)
                                    2 = DGPS fix
                                    3 = PPS fix
                                    4 = Real Time Kinematic
                                    5 = Float RTK
                                    6 = estimated (dead reckoning) (2.3 feature)
                                    7 = Manual input mode
                                    8 = Simulation mode
            6)      08           Number of satellites being tracked
            7)      0.9          Horizontal dilution of position
            8-9)    545.4,M      Altitude, Meters, above mean sea level
            10-11)  46.9,M       Height of geoid (mean sea level) above WGS84
                                ellipsoid
            12)     (empty field) time in seconds since last DGPS update
            13)     (empty field) DGPS station ID number
            CS       *47          the checksum data, always begins with *
        """
        fields = payload.split(",")[1:]
        result = {
            "time": (int(fields[0][:2])*60 + int(fields[0][2:4]))*60 + int(fields[0][4:6]),
            "lat": self.convert_coords(*fields[1:3]),
            "lon": self.convert_coords(*fields[3:5]),
            "fix": int(fields[5]),
            "sat": int(fields[6]),
            "hdop": self._safe_float(fields[7]),
            "height": self._safe_float(fields[8]),
            "hog": self._safe_float(fields[10]),
            "units": fields[9]
        }
        return result

    def _GPVTG(self, payload):
        """
        Decode NMEA GPVTG Type ()

        $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

        where:
                    VTG          Track made good and ground speed
            0-1)    054.7,T      True track made good (degrees)
            2-3)    034.4,M      Magnetic track made good
            4-5)    005.5,N      Ground speed, knots
            6-7)    010.2,K      Ground speed, Kilometers per hour
            CS      *48          Checksum
        """
        fields = payload.split(",")[1:]
        result = {
            "track": self._safe_float(fields[0]),
            "magnetic": self._safe_float(fields[2]),
            "speed": self._safe_float(fields[6])
        }
        return result

    def _GLGSV(self, payload):
        """
        Decode NMEA GLGSV Type ()
        """
        fields = payload.split(",")[1:]
        result = {
        }
        return result

    def _GNGSA(self, payload):
        """
        Decode NMEA GNGSA Type (GPS Dilution Of Precision and active satellites)

        $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

        Where:
                    GSA      Satellite status
            0)      A        Auto selection of 2D or 3D fix (M = manual) 
            1)      3        3D fix - values include: 1 = no fix
                                                      2 = 2D fix
                                                      3 = 3D fix
            2-13)   04,05... PRNs of satellites used for fix (space for 12) 
            14)     2.5      PDOP (dilution of precision) 
            15)     1.3      Horizontal dilution of precision (HDOP) 
            16)     2.1      Vertical dilution of precision (VDOP)
            CS      *39      the checksum data, always begins with *
        """
        fields = payload.split(",")[1:]
        result = {
        }
        return result

    def _GNGLL(self, payload):
        """
        Decode NMEA GNGLL Type (Geographic Latitude and Longitude)

        $GPGLL,4916.45,N,12311.12,W,225444,A,*1D

        Where:
                    GLL          Geographic position, Latitude and Longitude
            0-1)    4916.46,N    Latitude 49 deg. 16.45 min. North
            2-3)    12311.12,W   Longitude 123 deg. 11.12 min. West
            4)      225444       Fix taken at 22:54:44 UTC
            5)      A            Data Active or V (void)
            CS      *iD          checksum data
        """
        fields = payload.split(",")[1:]
        result = {
        }
        return result

    def _GPGSV(self, payload, mode='GPGSV'):
        """
        Decode NMEA GPGSV Type (Detailed data on Satelites)

        $GPGSV,2,1,08, 01,40,083,46, 02,17,308,41, 12,07,344,39, 14,22,228,45 *75

        Where:
                GSV          Satellites in view
            0)    2            Number of sentences for full data
            1)    1            sentence 1 of 2
            2)    08           Number of satellites in view
            3)    01           Satellite PRN number
            4)    40           Elevation, degrees
            5)    083          Azimuth, degrees
            6)    46           SNR - higher is better
            x)   +4x3          for up to 4 satellites per sentence
            CS    *75          the checksum data, always begins with *
        """
        def _sat(seq):
            if seq[0]:
                return {
                    "PRN": seq[0],
                    "elevation": self._safe_float(seq[1]),
                    "azimuth": self._safe_float(seq[2]),
                    "SNR": self._safe_float(seq[3]),
                    "mode": mode
                }
        fields = payload.split(",")[1:]
        sats = list(filter(None, [_sat(fields[i*4+3:(i+1)*4+3]) for i in range(4)]))
        result = {
            "count": self._safe_int(fields[2]),
            "satelites": sats
        }
        self._satellites.update({sat['PRN']: sat for sat in sats})
        return result


    def _GLGSV(self, payload):
        """
        Decode NMEA GLGSV Type (Synonym for GPGSV)
        """
        return self._GPGSV(payload, mode='GLGSV')

    def _GPRMC(self, payload):
        """
        Decode NMEA GPVTG Type (Essential GPS pvt position, velocity, time data)

        $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

        Where:
                    RMC          Recommended Minimum sentence C
            0)      123519       Fix taken at 12:35:19 UTC
            1)      A            Status A=active or V=Void.
            2-3)    4807.038,N   Latitude 48 deg 07.038' N
            4-5)    01131.000,E  Longitude 11 deg 31.000' E
            6)      022.4        Speed over the ground in knots
            7)      084.4        Track angle in degrees True
            8)      230394       Date - 23rd of March 1994
            9-10)   003.1,W      Magnetic Variation
            CS      *6A          The checksum data, always begins with *
        """
        fields = payload.split(",")[1:]
        result = {
        }
        return result

    def parse(self, sentence):
        """
        Parse NMEA Sentence
        """
        m = self._NMEA.match(sentence)
        if m:

            # Map frame & perform checksum:
            data = dict()
            data['raw'] = m.group(0)
            data['type'] = m.group(1).decode()
            data['payload'] = m.group(2).decode()
            data['checksum'] = int(m.group(3).decode(), 16)
            data['checked'] = self.checksum(m.group(0).split(b'*')[0].replace(b'$', b''))
            data['integrity'] = (data['checksum'] == data['checked'])

            # Parse payload:
            try:
                key = "_{}".format(data['type'])
                parser = getattr(self, key)
                data['result'] = parser(data['payload'])

            except (KeyError, AttributeError):
                data['result'] = None

            return data

    def read(self, timeout=1., debug=False, show=True, targets=None, mode='all'):
        """
        Read data from L76 chipset and parse NMEA protocol
        """
        
        assert mode in ('any', 'all')

        # Target management:
        if targets is None:
            targets = []

        matches = set()
        targets = set(targets)
        
        # Start watchdog:
        self._watchdog.reset()
        self._watchdog.start()

        # L76 read loop w/ timeout:
        i = 0
        self._set_buffer()
        while (timeout is None) or (self._watchdog.read() <= timeout):

            # Read from L76 and buffer frames no carriage return strip as it prevent some sentence to be detected:
            s = self._read()
            if s:
                self._buffer.write(s)
                if self.debug or debug:
                    print("STREAM [{:.6f},{}/{}]: {}".format(self._watchdog.read(), len(s), len(self._buffer.getvalue()), s))

            # Iterate buffered lines:
            line = b''
            self._buffer.seek(0)
            for line in self._buffer:
                i += 1                
                # Parse Line:
                try:
                    res = self.parse(line)

                except Exception as err:
                    print("ERROR [{}]: {}({})".format(i, err, line))

                else:
                    # Line is a valid NMEA sentence:
                    if res:

                        if self.debug or debug:
                            print("NMEA [{},{type:},{count:}]: {raw:}".format(i, count=len(line), **res))
                        
                        if res['result'] and show:
                            print("GPS-DATA [{}]: {}".format(i, res['result']))
                        
                        # NMEA Sentence has correct check sum:
                        if res['integrity']:

                            # Store Results:
                            self._lastframes[res['type']] = res
                            matches.update([res['type']])

                        else:
                            print("CHECKSUM [{},{checked:X}/{checksum:X}]: {raw:}".format(i, **res))

            # Reset buffer with trailing data:
            self._set_buffer(line)

            # Break read loop (any mode):
            if mode == 'any' and len(matches.intersection(targets))>0:
                break
            
            # Break read loop (all mode):
            if (mode == 'all') and matches.issuperset(targets):
                break
        
        # Timeout reason:
        else:
            print("TIMEOUT [{}]: {} {} in {}, missing {}".format(self._watchdog.read(), mode, targets, matches, targets.difference(matches)))


    def start(self, debug=False, show=True):
        """
        Start GPS in deamon mode (not threadable at the moment)
        """
        self.read(timeout=None, targets=None, debug=debug, show=show)

    def coords(self, timeout=1., debug=False, refresh=False):
        """
        Read coordinates from L76
        """
        if refresh or self._lastframes.get('GPGGA', {}).get('result') is None:
            self.read(timeout=timeout, debug=debug, targets=['GPGGA'])
        return self._lastframes.get('GPGGA', {}).get('result')

    def speed(self, timeout=1., debug=False, refresh=False):
        """
        Read speed from L76
        """
        if refresh or self._lastframes.get('GPVTG', {}).get('result') is None:
            self.read(timeout=timeout, debug=debug, targets=['GPVTG'])
        return self._lastframes.get('GPVTG', {}).get('result')

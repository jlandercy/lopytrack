#!/usr/bin/env python
#
# Copyright (c) 2019, Jean Landercy
#

import gc

from micropython import const
import uio
import ure
import utime

import machine


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
        self._RTC = machine.RTC()
        self._lastfixon = None
        self._lastframes = dict()
        self._satellites = dict()

        # NMEA Regular Expression:
        self._NMEA = ure.compile(b"\$(G....)(.*)\*([0-9A-F]*)\r")

        # Time Management:
        self._watchdog = machine.Timer.Chrono()

        # L76 Initialization:
        self.reg = bytearray(1)
        self.i2c.writeto(L76GNSS.GPS_I2CADDR, self.reg)

        # NMEA Synonym registration:
        oNMEA = [key for key in dir(self) if key.startswith("_GP")]
        for okey in oNMEA:
            for ntype in ['N', 'L']:
                nkey = okey[:2] + ntype + okey[3:]
                ofunc = getattr(self, okey)
                # The safe python way makes this closure fails, eg. GLGSV is mapped to PGRMC
                #setattr(self, nkey, lambda s: ofunc(s))
                # Does not work because function pointed by okey is not yet defined:
                #self.__dict__[nkey] = self.__dict__[okey]
                # This command seems to do the job, we should investigate this issue!
                # Dynamic programming looks like it is working but may not be accurate 
                self.__dict__[nkey] = ofunc 
                print("GPS-NMEA [sentence={}]: Synonym created for {}".format(okey[1:], nkey[1:]))
        print("GPS-NMEA: Registred sentences are {}".format([key[1:] for key in dir(self) if key.startswith("_G")]))

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
                coord *= -1
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

    def _convert_time(self, s):
        """
        Convert Time:
        """
        if len(s) == 6:
            return (int(s[0:2]), int(s[2:4]), int(s[4:6]))
        else:
            return (int(s[0:2]), int(s[2:4]), int(s[4:6]), int(s[7:10])*1000)

    def _convert_date(self, s):
        """
        Convert Date:
        """
        return (2000 + int(s[4:6]), int(s[2:4]), int(s[0:2]))

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
            "time": self._convert_time(fields[0]),
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

    def _GPGSA(self, payload):
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

    def _GPGLL(self, payload):
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
        n = (len(fields) - 3) // 4
        # If no satellites, it sends a 0 or 1 after number of satellites:
        # $GLGSV,1,1,00,1*78 
        # $GPGSV,1,1,00,0*65
        #assert (len(fields) - 3) % 4 == 0
        sats = list(filter(None, [_sat(fields[i*4+3:(i+1)*4+3]) for i in range(n)]))
        result = {
            "count": self._safe_int(fields[2]),
            "satelites": sats
        }
        self._satellites.update({sat['PRN']: sat for sat in sats})
        return result

    # def _GLGSV(self, payload):
    #     """
    #     Decode NMEA GLGSV Type (Synonym for GPGSV)
    #     """
    #     return self._GPGSV(payload, mode='GLGSV')

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
            "status": fields[1],
            "lat": self.convert_coords(*fields[2:4]),
            "lon": self.convert_coords(*fields[4:6]),
            "time": self._convert_time(fields[0]),
            "date": self._convert_date(fields[8]),
            "speed": self._safe_float(fields[6]),
            "track": self._safe_float(fields[7]),
            "magentic": self._safe_float(fields[9]),
            "direction": fields[10]
        }
        if result['speed']:
            result['speed'] *= 1.852
        return result

    # def _GNRMC(self, payload):
    #     """
    #     Synonym for GPRMC
    #     """
    #     return self._GPRMC(payload)

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

    @staticmethod
    def _to_utime(rtctime):
        """
        Signature argument conversion from RTC class and utime.mktime method
        RTC:        (year, month, day, hour, minute, second, microsecond, None)
        utime:      (year, month, day, hour, minute, second, dayofweek, dayofyear)
        compliance: (year, month, day, hour, minute, second, None, None)
        Resolution is limited to second which is not a limitation for GPS of Class 1
        """
        return list(rtctime[0:7]) + [None, None]

    def _set_RTC(self, data, eps=20):
        """
        Set Real Time Clock based on G*RMC sentence
        """
        # Date vector:
        vec = list(data['date']) + list(data['time'])
        # Time arithmetic:
        tnow = utime.mktime(L76GNSS._to_utime(vec))
        trtc = utime.mktime(L76GNSS._to_utime(self._RTC.now()))
        dt = tnow - trtc
        if abs(dt) > eps:
            self._RTC.init(vec)
            print("DEVICE-RTC [dt={}s]: Time sync w/ GPS {}".format(dt, self._RTC.now()))  

    def read(self, timeout=1., debug=False, targets=None, mode='all', fix=False):
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
                    print("GPS-STREAM [{:.6f},{}/{}]: {}".format(self._watchdog.read(), len(s), len(self._buffer.getvalue()), s))

            # Iterate buffered lines:
            line = b''
            self._buffer.seek(0)
            for line in self._buffer:
                i += 1                
                # Parse Line:
                #res = self.parse(line) # Make it crash because MicroPython cannot reraise
                try:
                    res = self.parse(line)

                except Exception as err:
                    print("GPS-STREAM ERROR [{}]: {}({})".format(i, err, line))

                else:
                    # Line is a valid NMEA sentence:
                    if res:

                        if self.debug or debug:
                            print("GPS-NMEA [{},{type:},{count:}]: {raw:}".format(i, count=len(line), **res))
                        
                        # NMEA Sentence has correct check sum:
                        if res['integrity']:

                            if (self.debug or debug) and res['result']:
                                print("GPS-DATA [{}]: {}".format(i, res['result']))

                            # Store Results:
                            self._lastframes[res['type']] = res
                            matches.update([res['type']])

                            # Is time fixed?
                            if res['type'] in ('GPRMC', 'GNRMC') and res['result'] is not None:
                                self._set_RTC(res['result'])

                            # Is position fixed?
                            if res['type'] in ('GPGGA',) and res['result']['lat'] is not None:
                                self._lastfixon = self._RTC.now()

                        else:
                            print("GPS-NMEA CHECKSUM [{},{checked:X}/{checksum:X}]: {raw:}".format(i, **res))

            # Reset buffer with trailing data:
            self._set_buffer(line)

            if fix and ('GPGGA' in matches) and not self._lastframes['GPGGA']['result']['lon'] is None:
                print("GPS-FIX: {}".format(self._lastframes['GPGGA']['result']))
                break

            # Break read loop (any mode):
            if not fix and (mode == 'any') and len(matches.intersection(targets))>0:
                break
            
            # Break read loop (all mode):
            if not fix and (mode == 'all') and matches.issuperset(targets):
                break
        
        # Timeout reason:
        else:
            print("GPS-FIX [timeout={}s]: {} {} in {}, missing {}".format(self._watchdog.read(), mode, targets, matches, targets.difference(matches)))

    def start(self, debug=False):
        """
        Start GPS in deamon mode (not threadable at the moment)
        """
        self.read(timeout=None, targets=None, debug=debug)

    def is_fixed(self, eps=5*60):
        """
        Check if position is fixed ()
        """
        if self._lastfixon is None:
            return False
        else:
            tnow = utime.mktime(self._to_utime(self._RTC.now()))
            tlf = utime.mktime(self._to_utime(self._lastfixon))
            return abs(tnow - tlf) < eps

    def fix(self, debug=False, timeout=300.0, retry=5):
        """
        Get a GPS fix in a given timeout with retries
        """
        for i in range(retry):
            self.read(timeout=timeout, targets=['GPGGA', 'GPRMC', 'GNRMC'], mode='all', fix=True, debug=debug)
            print("GPS-FIX [try={}/{}]: Fixed = {}".format(i+1, retry, self.is_fixed()))
            if self.is_fixed():
                break
            
        return self._lastframes['GPGGA']['result']

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


#!/usr/bin/env python

import pycom

pycom.wifi_on_boot(False)
print("DEVICE-BOOT: WiFi on Boot set to {}".format(pycom.wifi_on_boot()))

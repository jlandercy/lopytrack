
#!/usr/bin/env python

import pycom

print("DEVICE [BOOT]: WiFi [{}]".format(pycom.wifi_on_boot()))
pycom.wifi_on_boot(False)
print("DEVICE [BOOT]: WiFi [{}]".format(pycom.wifi_on_boot()))

import json
import binascii

import pycom
import machine
import network

import lora
import logic

# Start to blink:
pycom.heartbeat(True)

# Setup network & sensors
if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    print('POWER-ON [DEEPSLEEP]')
else:
    print('POWER-ON [RESET]')

# Node Connection:
with open('./data/nodes.json') as fh:
    creds = json.load(fh)

# Detect device:
eid = binascii.hexlify(network.LoRa().mac()).decode().upper()
print("Device [Device-EUI={}] detected".format(eid))

# Read credentials:
keys = creds.get(eid)
sock = None

# Create Socket:
#if keys:
#    sock = lora.connect(**keys)
print("Socket: {}".format(sock))

# Stop to blink:
pycom.heartbeat(False)

# Create and start application:
app = logic.Application(sock=sock)
app.start(dryrun=True, debug=False, mode='power')

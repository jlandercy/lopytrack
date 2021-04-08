#!/usr/bin/env python

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
    mode = 'eco'
    print('DEVICE [POWER]: Woke up from deepsleep')
else:
    mode = 'power'
    print('DEVICE [POWER]: Started after a reset')

# Detect device:
eid = binascii.hexlify(network.LoRa().mac()).decode().upper()
print("DEVICE [EUI={}]: EUI detected".format(eid))


# Node/Application Key:
target = './data/lora.json'
try:
    with open(target) as fh:
        creds = json.load(fh)
except OSError:
    creds = {eid: lora.generate_keys()}
    print("CREDENTIALS: {}".format(json.dumps(creds)))
    with open(target, 'w') as fh:
        json.dump(creds, fh)
keys = creds.get(eid)

# Create Socket:
sock = None
if keys:
    sock, _lora = lora.connect(**keys, force=True)
print("SOCKET: {}".format(sock))

# Stop to blink:
pycom.heartbeat(False)

# Create and start application:
app = logic.Application(sock=sock, lora=_lora)
app.start(lora_period=60*1, gps_timeout=10, debug=False, show=True, mode='power', dryrun=False)

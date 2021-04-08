#!/usr/bin/env python

import os
import time
import socket
import binascii

from network import LoRa
import ubinascii


def generate_keys():
    """"
    Generate random Application EUI and Key:
    """
    appeui = ubinascii.hexlify(os.urandom(8)).decode().upper()
    appkey = ubinascii.hexlify(os.urandom(16)).decode().upper()
    print("LORA [EUI={}]: Application keys generated".format(appeui))
    return {"appeui": appeui, "appkey": appkey}

def connect(appeui, appkey, force=False, max_retry=20, grace_period=2.5):
    """
    Create and connect Socket for LoRa application using OTAA mechanism
    """

    # Initialise LoRa in LORAWAN mode.
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868)
    #lora.nvram_erase()
    lora.nvram_restore()

    if not lora.has_joined() or force:

        # create an OTAA authentication parameters
        app_eui = ubinascii.unhexlify(appeui)
        app_key = ubinascii.unhexlify(appkey)

        # join a network using OTAA (Over the Air Activation)
        lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)
    
    # overwrite the default three channels to just sent on one channel
    #lora.add_channel(0,frequency=868100000, dr_min=5, dr_max=5)
    #lora.add_channel(1,frequency=868100000, dr_min=5, dr_max=5)
    #lora.add_channel(2,frequency=868100000, dr_min=5, dr_max=5)

    # wait until the module has joined the network
    i = 0
    while not lora.has_joined():
        time.sleep(grace_period)
        i += 1
        print('LORA/OTAA [EUI={}]: Application Join request pending ({}/{})...'.format(appeui, i, max_retry))
        if i >= max_retry:
            break
    else:
        print('LORA/OTAA [EUI={}]: Application Join request accepted'.format(appeui))

    # create a LoRa socket
    sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

    # set the LoRaWAN data rate
    sock.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

    # make the socket blocking
    # (waits for the data to be sent and for the 2 receive windows to expire)
    sock.setblocking(True)
    #sock.send(b'')
    
    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    sock.setblocking(False)
    #sock.recv(64)

    # Save LoRa State:
    # https://forum.pycom.io/topic/1668/has-anyone-successfully-used-lora-nvram_save-and-restore/16
    lora.nvram_save()
    print('LORA/OTAA [EUI={}]: LoRa state saved'.format(appeui))

    return sock, lora

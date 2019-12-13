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

def connect(appeui, appkey):
    """
    Create and connect Socket for LoRa application using OTAA mechanism
    """

    # Initialise LoRa in LORAWAN mode.
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868)

    # create an OTAA authentication parameters
    app_eui = ubinascii.unhexlify(appeui)
    app_key = ubinascii.unhexlify(appkey)

    # join a network using OTAA (Over the Air Activation)
    lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)
    
    # wait until the module has joined the network
    while not lora.has_joined():
        time.sleep(1.0)
        print('LORA/OTAA [EUI={}]: Application Join request pending...'.format(appeui))

    print('LORA/OTAA [EUI={}]: Application Join request accepted'.format(appeui))
    
    # Save configuration:
    #lora.nvram_save()

    # create a LoRa socket
    sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

    # set the LoRaWAN data rate
    sock.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

    # make the socket blocking
    # (waits for the data to be sent and for the 2 receive windows to expire)
    sock.setblocking(True)

    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    sock.setblocking(False)

    return sock

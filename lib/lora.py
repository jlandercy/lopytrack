import os
import time
import socket
import binascii

from network import LoRa
import ubinascii


def generate_keys(show=False):
    """"
    Generate random Application EUI and Key:
    """
    appeui = ubinascii.hexlify(os.urandom(8)).decode().upper()
    appkey = ubinascii.hexlify(os.urandom(16)).decode().upper()
    if show:
        print("App-EUI=".format(appeui))
        print("App-Key=".format(appkey))
    return (appeui, appkey)

def connect(appeui=None, appkey=None):
    """
    Create anc connect Socket for LoRa technology using OTAA mechanism
    """

    if appeui is None:
        appeui, appkey = generate_keys()
        print("Generated [App-EUI={}]".format(appeui))

    # Initialise LoRa in LORAWAN mode.
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868)

    # create an OTAA authentication parameters
    app_eui = ubinascii.unhexlify(appeui)
    app_key = ubinascii.unhexlify(appkey)

    # join a network using OTAA (Over the Air Activation)
    lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)
    
    # wait until the module has joined the network
    while not lora.has_joined():
        time.sleep(2.5)
        print('Join [App-EUI={}] pending...'.format(appeui))

    print('Joined [App-EUI={}]'.format(appeui))
    
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

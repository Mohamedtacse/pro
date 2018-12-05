import socket
import struct
from network import LoRa
from network import WLAN
from network import Bluetooth
import time
from crypto import AES
import crypto
import machine
from machine import UART
import pycom
from pycoproc import Pycoproc
import utime
from machine import Pin
import config
from machine import Timer
from network import LTE




uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1, timeout_chars=1, pins=('P11', 'P10'))
uart.deinit()
uart = None
uart = UART(1, baudrate=9600, bits=8, parity=None, stop=1, timeout_chars=1, pins=('P11', 'P10'))

pycom.heartbeat(False)
wlan = WLAN()
wlan.deinit()
bluetooth = Bluetooth()
bluetooth.deinit()
#lte = LTE()
#lte.deinit()
#py = Pytrack()


class lora:
    master = False
    color = 0x000020
    shift = 0
    key=b'notsuchsecretkey'
    #lora = LoRa()
    #lora_sock = None
    #en = False

    def __init__(self, opc=False, master=False, uart=None, shift=0):
        self.config(opc=opc, master=master, uart=uart, shift=shift)

    def config(self, opc=False, master=False, uart=None, shift=0):
        self.uart = uart
        self.master = master
        self.shift = shift
        self.color = self.color << self.shift
        pycom.rgbled( self.color )

        if master is True:
            self.lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868)
        else:
            self.lora = LoRa(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868)

        self.lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
        self.lora_sock.setblocking(False)
        print('LoRa connection done!!')
        self.en = opc;

        if self.en is True:
            #self.iv = self.key #crypto.getrandbits(128)
            #self.cipher = AES(self.key, AES.MODE_CFB, self.iv)
            print('conexion con mensaje encriptado')
        else:
            print('conexion sin mensaje encriptado')

        self.lora.callback(trigger=(LoRa.RX_PACKET_EVENT), handler=self.cb)

    def send(self,data):
        if data is not None:
            if self.en is True:
                self.iv=crypto.getrandbits(128)
                #self.iv = self.key
                self.cipher = AES(self.key, AES.MODE_CFB, self.iv)
                data = self.iv + self.cipher.encrypt(data)
            self.lora_sock.send(data)

            #while True:
            #if uart.any():
            #    data = uart.read(50)
            #    data = data.decode()
            #    print(data,end='')
            #    self.lora_sock.send(data)
            #    #print(msg)

    def cb(self,l):
        events = l.events()
        if events & LoRa.RX_PACKET_EVENT:
            #print('Lora packet received: ')
            #print(self.lora_sock.recv(1024))
            recv = self.lora_sock.recv(1024)

            if len(recv) > 16:
                if self.en is True:
                    self.cipher=AES(self.key, AES.MODE_CFB, recv[:16])
                    recv = self.cipher.decrypt(recv[16:])
                    num = len(recv)
                else:
                    num = len(recv)
                #R = self.R
                #G = self.G
                #B = self.B
                if  num > 0:
                    color = (self.color >> self.shift) & 0xFF
                    color = (color + num) & 0xFF
                    mask = ~(0xFF << self.shift)
                    #pycom.rgbled(R<<16 | G<<8 | B)
                    self.color = (self.color & mask) | (color << self.shift)
                    pycom.rgbled( self.color  )
                    recv=recv.decode()
                    if self.master is True:
                        print(recv,end='')
                        print(self.estado())
                    else:
                        self.uart.write(recv)
                        #print(recv,end='')

    def estado(self):
        a = self.lora.stats()
        #print(self.lora.stats())
        print("rssi: {}\r\n".format(a[1]))



def pin_handler(arg):
    print(arg.id());
    b = Pycoproc()
    nivell = b.read_battery_voltage();
    dada = 'pulsador: {0}\r\n'.format(nivell)
    l.send(dada);

pin = machine.Pin('P14', machine.Pin.IN, machine.Pin.PULL_UP)

if pin.value() == 0:
    print('Master')
    l=lora(opc=True, master=True, uart = uart, shift = 0)
else:
    print('Slave')
    l=lora(opc=True, master=False, uart = uart, shift = 16)
    pin.callback(machine.Pin.IRQ_FALLING, pin_handler);

    chrono = Timer.Chrono()
    chrono.start()

    l.send("up!\r\n")
    while chrono.read() < 15:
        if uart.any():
            data = uart.readall() #(50)
            data = data.decode()
            l.send(data)

    machine.pin_deepsleep_wakeup(pins = ['P14'], mode = machine.WAKEUP_ALL_LOW, enable_pull = True)
    #lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868)
    #lora.power_mode(LoRa.SLEEP)
    #print('Device: ' + uos.uname().machine)
    #if (uos.uname().sysname == 'FiPy'):
    #print('Switching off LTE')
    #lte =LTE()
    #quit = False
    #while quit == False:
    #    try:
    #        lte.deinit()
    #    except OSError:
    #        print('  Exception occured, retrying...')
    #        time.sleep(2)
    #        pass
    #    else:
    #        quit = True
    #lte = LTE()
    #lte.deinit()
    machine.deepsleep(10000) #this does not restart a new interval!!
            #print(data,end='')


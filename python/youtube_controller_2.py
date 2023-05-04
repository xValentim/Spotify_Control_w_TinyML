import pyautogui
import serial
import argparse
import time
import logging
import numpy as np



TS = 16000
START_CHAR = 'A'
END_CHAR = 'X'

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'J','B':'Space','C':'F'} # Fast forward (10 seg) pro Youtube
class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'
    def handshake(self):
        print("handshake")

        self.incoming = b''
        while self.incoming != b'H':
            print("Entrou no while")
            self.incoming = self.ser.read()
            print(self.incoming)
            logging.debug("Received INCOMING: {}".format(self.incoming))
            print("waiting for H (Hello) from uC")

        self.incoming = b''
        while self.incoming != b'S':
            self.ser.write(b'O')
            print("sent O (OK)")
            print(self.incoming)
            self.incoming = self.ser.read()

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay

    
    def update(self):

        ## Sync protocol    
        while self.incoming != b'X':

            self.incoming = self.ser.read()
            print('To aq')

            logging.debug("Received INCOMING: {}".format(self.incoming))

        sended = False
        
        data = self.ser.read()
        # data = self.incoming
        print(data, self.incoming)
        logging.debug("Received DATA: {}".format(data))
        
        #BACK
        if data == b'1':
            print('data1')
            logging.info("KEYDOWN A")
            with pyautogui.hold('ctrl'):
                pyautogui.press('left')
            sended = True
            
            #pyautogui.keyDown(self.mapping.button['A'])
        elif data == b'0':
            print('data0')
            logging.info("KEYUP A")
            pyautogui.keyUp(self.mapping.button['A'])
            sended = True
        # PAUSE    
        elif data == b'3':
            print('data3')
            logging.info("KEYDOW B")
            pyautogui.keyDown(self.mapping.button['B'])
            sended = True
        elif data == b'2':
            print('data2')
            logging.info("KEYUP B")
            pyautogui.keyUp(self.mapping.button['B'])
            sended = True
        
        # NEXT  
        elif data == b'5':
            print('data5')
            logging.info("KEYDOW C")
            with pyautogui.hold('ctrl'):
                pyautogui.press('right')
            sended = True
            
            # pyautogui.keyDown(self.mapping.button['C'])
        elif data == b'4':
            print('data4')
            logging.info("KEYUP C")
            pyautogui.keyUp(self.mapping.button['C'])
            sended = True
        
        elif data == b'V':
            data = self.ser.read()
            abs_volume = int.from_bytes(data, "big")
            print("abs_volume", abs_volume)
            sended = True
        
            for i in range(17):
                with pyautogui.hold('ctrl'):
                    pyautogui.press('down')
        
            for i in range(abs_volume):
                with pyautogui.hold('ctrl'):
                    pyautogui.press('up')

        if sended:
            self.ser.write(b'K')

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        pyautogui.keyDown(self.mapping.button['B'])
        pyautogui.keyDown(self.mapping.button['C'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        pyautogui.keyUp(self.mapping.button['B'])
        pyautogui.keyUp(self.mapping.button['C'])
        logging.info("[Dummy] Pressed A button")
        logging.info("[Dummy] Pressed B button")
        logging.info("[Dummy] Pressed C button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)
    controller.handshake()
    while True:
        controller.update()

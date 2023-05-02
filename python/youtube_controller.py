import pyautogui
import serial
import argparse
import time
import logging

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'J','B':'K','C':'L'} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        print("update")
        while self.incoming != b'X':
            print('travei')
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))
        #print('nao chego aqui')
        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))

        #BACK
        if data == b'1':
            print('data1')
            logging.info("KEYDOWN A")
            pyautogui.keyDown(self.mapping.button['A'])
        elif data == b'0':
            print('data0')
            logging.info("KEYUP A")
            pyautogui.keyUp(self.mapping.button['A'])
        # PAUSE    
        elif data == b'3':
            print('data3')
            logging.info("KEYDOW B")
            pyautogui.keyDown(self.mapping.button['B'])
        elif data == b'2':
            print('data2')
            logging.info("KEYUP B")
            pyautogui.keyUp(self.mapping.button['B'])
        
        # NEXT  
        elif data == b'5':
            print('data5')
            logging.info("KEYDOW C")
            pyautogui.keyDown(self.mapping.button['C'])
        elif data == b'4':
            print('data4')
            logging.info("KEYUP C")
            pyautogui.keyUp(self.mapping.button['C'])

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

    while True:
        controller.update()

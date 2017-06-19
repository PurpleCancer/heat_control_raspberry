import RPi.GPIO as GPIO
import time
import threading
import requests
GPIO.setwarnings(False)

dataPin = 12
clockPin = 5
shiftPin = 6
digitPins = [22, 23, 24, 25]

iters_per_display_cycle = 100

byte_lookup = [252, 96, 218, 242, 102, 182, 190, 224, 254, 246]

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        if PID > 100.0:
            self.P_value *= (100/PID)
            self.I_value *= (100/PID)
            self.D_value *= (100/PID)
            return 100.0
        else:
            return max(PID, 0)

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

class four_dig_display_thread (threading.Thread):
    def __init__(self, iters, bts):
        threading.Thread.__init__(self)
        self.iters = iters
        self.bts = bts
    def run(self):
        for i in range(self.iters):
            for i, byte in enumerate(self.bts):
                print_digit(byte, i)
            time.sleep(0.00025)

class display_stuff (threading.Thread):
    def __init__(self, temp, target, pid, repeats):
        threading.Thread.__init__(self)
        self.temp = [0, byte_lookup[int(temp[0])],
                     byte_lookup[int(temp[1])] + 1,
                     byte_lookup[round((int(temp[2])*10 + int(temp[3]))/10)]]
        self.target = [0, byte_lookup[int(target/10)],
                       byte_lookup[int(target)%10] + 1,
                       byte_lookup[round((round(target, 1) - int(target))*10)%10]]

        self.pid = [byte_lookup[int(pid/100)],
                    byte_lookup[int((int(pid)%100)/10)],
                    byte_lookup[int(pid)%10] + 1,
                    byte_lookup[round((round(pid, 1) - int(pid))*10)%10]]
        if self.pid[0] == byte_lookup[0]:
            self.pid[0] = 0

        self.displays = [self.temp, self.target, self.pid]
                       
        self.repeats = repeats

        self.thread = \
            four_dig_display_thread(iters_per_display_cycle, self.temp)
        self.first = True
    def run(self):
        self.thread.start()
        for i in range(self.repeats):
            for disp in self.displays:
                if self.first:
                    self.first = False
                    continue
                self.new_thread = \
                    four_dig_display_thread(iters_per_display_cycle, disp)
                self.thread.join()
                self.new_thread.start()
                self.thread = self.new_thread
        self.thread.join()


def get_current_target():
    try:
        r = requests.get('http://194.28.50.219:8888/api/temperature/' ,timeout=1)
    except:
        return 10
    json = r.json()
    if 'temperature' in json:
        return json['temperature']
    else:
        return 10

def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clockPin, GPIO.OUT)
    GPIO.setup(shiftPin, GPIO.OUT)
    GPIO.setup(dataPin, GPIO.OUT)
    for digitPin in digitPins:
        GPIO.setup(digitPin, GPIO.OUT)
        GPIO.output(digitPin, True)

def print_digit(byte, digitNo):
    bit_list = byte2bit_list(byte)
    # print(bit_list)
    for bit in bit_list:
        GPIO.output(dataPin, bit)
        GPIO.output(clockPin, True)
        GPIO.output(clockPin, False)
        time.sleep(0.00025)
        GPIO.output(dataPin, False)

    GPIO.output(digitPins[digitNo], False)
    GPIO.output(shiftPin, True)
    GPIO.output(shiftPin, False)
    time.sleep(0.00025)
    GPIO.output(digitPins[digitNo], True)

def byte2bit_list(byte):
    return [(byte >> bit) & 1 for bit in range(7, -1, -1)]

init()

thread = four_dig_display_thread(iters_per_display_cycle, [1,1,1,1])
thread.start()

p = PID(7.0, 0.3, 1.2)
current_target = 10

while(True):
    target = get_current_target()
    # TODO get target from server
    if target != current_target:
        current_target = target
        p.setPoint(current_target)
        
    f = open('/sys/bus/w1/devices/28-0416857b1aff/w1_slave', 'r')
    content = f.read()
    f.close()

    correct_read = content.find('YES')
    if correct_read != -1:
        t_index = content.find('t=')
        temp = content[t_index+2:t_index+7]

        pid = 0
        if current_target > int(temp)/1000:
            pid = p.update(int(temp)/1000)
        # print(pid)

        new_thread = display_stuff(temp, current_target, pid, 1)
        thread.join()
        new_thread.start()
        thread = new_thread
        
        
##    print_digit(byte_lookup[2], 1)
##    print_digit(byte_lookup[4], 2)
##    print_digit(byte_lookup[7], 3)

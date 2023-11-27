import time
import datetime
import random

from LeServo import PWM_Servo
import pigpio


COUNT_DATA = (
    # TIGHT FIST
    (0, 420, 2500, 500, 500, 500, 500, 1500),
    # ONE
    (0, 420, 500, 500, 500, 500, 500, 1500),
    # TWO
    (0, 420, 500, 2500, 500, 500, 500, 1500),
    # THREE
    (0, 420, 500, 2500, 2500, 500, 500, 1500),
    # FOUR
    (0, 420, 500, 2500, 2500, 2500, 500, 1500),
    # FIVE
    (0, 420, 500, 2500, 2500, 2500, 2500, 1500),
)

MUDRA_DATA = (
    # Arthapathaka
    (0, 420, 2500, 2500, 2500, 1200, 1200, 1500),
    # Tripathaka
    (0, 420, 2000, 2500, 2500, 1400, 2500, 1500),
    # Chandrakala
    (0, 420, 500, 2500, 500, 500, 500, 1500),
    # Sarpasirsha
    (0, 420, 2500, 1650, 1650, 1650, 1650, 1500)
)



Servos = ()


def runActionGroup(action_group):
    if not action_group:
        return

    for action_data in action_group:
        for i in range(0, 6, 1):
            Servos[i].setPosition(action_data[2 + i], action_data[1])
        time.sleep(float(action_data[1])/1000.0)


def initLeArm(d):
    global Servos
    global pi
    pi = pigpio.pi()
    servo1 = PWM_Servo(pi, 12,  deviation=d[0], control_speed=True)
    servo2 = PWM_Servo(pi, 16, deviation=d[1], control_speed=True)
    servo3 = PWM_Servo(pi, 20, deviation=d[2], control_speed=True)
    servo4 = PWM_Servo(pi, 21, deviation=d[3], control_speed=True)
    servo5 = PWM_Servo(pi, 19, deviation=d[4], control_speed=True)
    servo6 = PWM_Servo(pi, 13, deviation=d[5], control_speed=True)
    Servos = (servo1, servo2, servo3, servo4, servo5, servo6)

if __name__ == "__main__":
    initLeArm([0, 0, 0, 0, 0, 0])

    start  = datetime.datetime.now()
    mudra_index = 0
    while True:
        time_elapsed = datetime.datetime.now() - start
        if time_elapsed.seconds > 80:
            break
        
        # RUN COUNTS
        for count_data in COUNT_DATA:
            runActionGroup(((count_data),))
            time.sleep(0.429)
        
        # RUN MUDRAS
        runActionGroup(((MUDRA_DATA[mudra_index]),))
        time.sleep(0.429)
        mudra_index = (mudra_index + 1) % len(MUDRA_DATA)

        # runActionGroup(((0, 420,
        #                 random.randint(500, 2500),
        #                 random.randint(500, 2500), 
        #                 random.randint(500, 2500),
        #                 random.randint(500, 2500), 
        #                 random.randint(500, 2500),
        #                 1500),))
    

# Arthapathaka
# Tripathaka
# Chandrakala
# Sarpasirsha
# SHAKA
# SWAG
# SCISSORS 
# ROCK
# PAPER
# FIVE
# FOUR
# THREE
# TWO
# ONE
# FIVE

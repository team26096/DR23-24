from hub import button
from hub import port
import motor

# ----------------------------------------
# HELPER FUNCTIONS

def gyro_follow_angle_for_distance(angle, distance_to_cover):
    # TODO
    pass


def run_motor(port_name, velocity=1000):
    motor.run(port_name, velocity)


def stop_motor(port_name):
    motor.stop(port_name)


def run_motor_while_button_pressed(motor_port, velocity = 1000):
    
    while not button.pressed(button.LEFT) and not button.pressed(button.RIGHT):
        pass
    
    if button.pressed(button.LEFT) or button.pressed(button.RIGHT):
        # Determine which button is pressed
        target_button = button.LEFT if button.pressed(button.LEFT) else button.RIGHT
        
        # Reverse velocity (to Anti-clockwise) if left button is pressed
        if target_button == button.LEFT:
            velocity = velocity * -1
        
        run_motor(motor_port, velocity)
        # Run motor until same button is released
        
        while button.pressed(target_button):
            pass
        
    stop_motor(motor_port)

# ----------------------------------------
# RUN FUNCTIONS


# ----------------------------------------
# MASTER PROGRAM


LEFT_MOTOR_PORT = port.C
RIGHT_MOTOR_PORT = port.B

motor_port = "L"
if motor_port == "L":
    motor_port = LEFT_MOTOR_PORT
elif motor_port == "R":
    motor_port = RIGHT_MOTOR_PORT

while True:
    run_motor_while_button_pressed(motor_port)

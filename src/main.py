# Library imports
from vex import *

# Brain
brain = Brain()

# Controller
controller = Controller(PRIMARY)

# Left motors
lmotor_1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
lmotor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
lmotor_3 = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)

lmotors = MotorGroup(lmotor_1, lmotor_2, lmotor_3)

# Right motors
rmotor_1 = Motor(Ports.PORT4, GearSetting.RATIO_18_1)
rmotor_2 = Motor(Ports.PORT5, GearSetting.RATIO_6_1)
rmotor_3 = Motor(Ports.PORT6, GearSetting.RATIO_6_1)

rmotors = MotorGroup(rmotor_1, rmotor_2, rmotor_3)

# Intake motor
intake = Motor(Ports.PORT11, GearSetting.RATIO_6_1)

# Joystick control
while True:
    # Set up variables for easy access
    fwd_back = controller.axis3.position()
    left_right = controller.axis4.position()

    intake_in = controller.buttonL1.pressing()
    intake_out = controller.buttonL2.pressing()
    
    # Backward
    if (fwd_back < -10):
        lmotors.spin(REVERSE, abs(fwd_back), PERCENT)
        rmotors.spin(REVERSE, abs(fwd_back), PERCENT)
    
    # Forward
    elif (fwd_back > 10):
        lmotors.spin(FORWARD, abs(fwd_back), PERCENT)
        rmotors.spin(FORWARD, abs(fwd_back), PERCENT)
    
    # Left
    elif (left_right < -10):
        lmotors.spin(REVERSE, abs(left_right)/2, PERCENT)
        rmotors.spin(FORWARD, abs(left_right)/2, PERCENT)
    
    # Right
    elif (left_right > 10):
        lmotors.spin(FORWARD, abs(left_right)/2, PERCENT)
        rmotors.spin(REVERSE, abs(left_right)/2, PERCENT)

    # Intake
    if (intake_in):
        intake.spin(REVERSE, 60, PERCENT)

    elif (intake_out):
        intake.spin(FORWARD, 60, PERCENT)

    # Safety block
    else:
        lmotors.stop()
        rmotors.stop()
        intake.stop()
    
    # Make sure CPU doesn't overload
    wait(20,MSEC)

"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
### ds = robot.getDevice('dsname')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

### ds.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    ### val = ds.getValue()
    gps_vals = gps.getValues()
    imu_rads = imu.getRollPitchYaw()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    motor_left.setVelocity(-5.0)
    motor_right.setVelocity(-5.0)

    print('Hello World from Python!', gps_vals, [x*180.0/3.14159 for x in imu_rads])

# Enter here exit cleanup code.
print('Bye from Python!')

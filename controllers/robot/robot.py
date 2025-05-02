import math
from controller import Robot

goal = (3.0, 3.0)
speed = 10

robot = Robot()
timestep = int(robot.getBasicTimeStep())

distF = robot.getDevice('distF')
distF.enable(100)
distR = robot.getDevice('distR')
distR.enable(100)
distFR = robot.getDevice('distFR')
distFR.enable(100)
gps = robot.getDevice('gps')
gps.enable(100)
compass = robot.getDevice('compass')
compass.enable(100)

mR1 = robot.getDevice('mR1')
mR2 = robot.getDevice('mR2')
mR3 = robot.getDevice('mR3')
mL1 = robot.getDevice('mL1')
mL2 = robot.getDevice('mL2')
mL3 = robot.getDevice('mL3')

mR1.setPosition(float('inf'))
mR2.setPosition(float('inf'))
mR3.setPosition(float('inf'))
mL1.setPosition(float('inf'))
mL2.setPosition(float('inf'))
mL3.setPosition(float('inf'))


def set_motors(left_v, right_v):
    mL1.setVelocity(left_v)
    mL2.setVelocity(left_v)
    mL3.setVelocity(left_v)
    mR1.setVelocity(right_v)
    mR2.setVelocity(right_v)
    mR3.setVelocity(right_v)


def get_orientation():
    compass_values = compass.getValues()
    return math.atan2(compass_values[0], compass_values[1])


def get_lidar_average(lidar):
    ranges = lidar.getRangeImage()
    return sum(ranges) / len(ranges) * 10000 if ranges else float('inf')


def calculate_line_to_goal():
    current_position = gps.getValues()
    if goal[0] == current_position[0]:
        return float('inf'), goal[0]
    slope = (goal[1] - current_position[1]) / (goal[0] - current_position[0])
    intercept = current_position[1] - slope * current_position[0]
    return slope, intercept


def is_on_line_to_goal(line):
    current_position = gps.getValues()
    if line[0] == float('inf'):
        result = abs(current_position[0] - line[1])
        return result < 0.1
    slope, intercept = line
    expected_y = slope * current_position[0] + intercept
    result = abs(expected_y - current_position[1])
    return result < 0.1


def towards_goal():
    current_position = gps.getValues()
    robot_orientation = get_orientation()

    delta_x = goal[0] - current_position[0]
    delta_y = goal[1] - current_position[1]
    target_angle = math.atan2(delta_y, delta_x)

    angle_difference = (target_angle - robot_orientation + math.pi) % (2 * math.pi) - math.pi

    distance_to_goal = math.sqrt(
        (goal[0] - current_position[0]) ** 2 +
        (goal[1] - current_position[1]) ** 2
    )
    if distance_to_goal < 0.7:
        set_motors(0, 0)
        print("Bug 2 reached the goal!")
        return True

    if abs(angle_difference) > 0.1:
        polarity = 1 if angle_difference > 0 else -1
        set_motors(-speed * polarity, speed * polarity)
    else:
        set_motors(speed, speed)

    return False


def circumnavigate_obstacle():
    follow = False
    while not is_on_line_to_goal(line_to_goal):
        if get_lidar_average(distR) > 3000 and not follow:
            set_motors(-speed, speed)
        elif (get_lidar_average(distF) > 3000
            and get_lidar_average(distR) < 3000):
            follow = True
            set_motors(speed, speed)
        elif get_lidar_average(distF) < 5000 and get_lidar_average(distFR) < 3000:
            while get_lidar_average(distF) < 5000 or get_lidar_average(distFR) < 2000:
                set_motors(-speed, speed)
                robot.step()
                set_motors(speed/3, speed/3)
                robot.step()
        
        else:
        
            
            while get_lidar_average(distFR) > 3000:
                set_motors(speed, -speed)
                robot.step()
                set_motors(speed/3, speed/3)
                robot.step()
                
            while get_lidar_average(distFR) < 2000:
                set_motors(-speed, speed)
                robot.step()
                set_motors(speed/3, speed/3)
                robot.step()
                
        robot.step()
    set_motors(speed/3, speed/3)
    robot.step(500)


saved_line = False
line_to_goal = None
robot.step(500)
while robot.step(timestep) != -1:
    distance = get_lidar_average(distF)
    if math.isnan(distance) or distance == 0:
        robot.step(150)
        continue

    if not saved_line:
        line_to_goal = calculate_line_to_goal()
        print(line_to_goal)
        saved_line = True
     
    if distance > 3000:
        if towards_goal():
            break
    else:
        circumnavigate_obstacle()

import sys,time
sys.path.insert(0, '../src')
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def wall_follow(dist):
    distance_left = min(dist[:4])
    distance_right = min(dist[4:])
    walking.input['distance left'] = distance_left
    walking.input['distance right'] = distance_right
    walking.compute()

    return [walking.output['velocity left'], walking.output['velocity right']]

distance_left = ctrl.Antecedent(np.arange(0.2, 5.01, 0.01), 'distance left')
distance_right = ctrl.Antecedent(np.arange(0.2, 5.01, 0.01), 'distance right')

velocity_left = ctrl.Consequent(np.arange(0.0,5.1,0.1), 'velocity left')
velocity_right = ctrl.Consequent(np.arange(0.0,5.1,0.1), 'velocity right')

distance_left['low'] = fuzz.trimf(distance_left.universe, [0.2, 0.2, 0.3])
distance_left['medium'] = fuzz.trimf(distance_left.universe, [0.2, 0.3, 1.0])
distance_left['high'] = fuzz.trapmf(distance_left.universe, [0.5, 1.0, 5.0, 5.0])

distance_right['low'] = fuzz.trimf(distance_right.universe, [0.2, 0.2, 0.3])
distance_right['medium'] = fuzz.trimf(distance_right.universe, [0.2, 0.3, 1.0])
distance_right['high'] = fuzz.trapmf(distance_right.universe, [0.5, 1.0, 5.0, 5.0])

velocity_left['low'] = fuzz.trimf(velocity_left.universe, [0, 0, 3.0])
velocity_left['medium'] = fuzz.trimf(velocity_left.universe, [0, 3.0, 5.0])
velocity_left['high'] = fuzz.trimf(velocity_left.universe, [3.0, 5.0, 5.0])

velocity_right['low'] = fuzz.trimf(velocity_right.universe, [0, 0, 3.0])
velocity_right['medium'] = fuzz.trimf(velocity_right.universe, [0, 3.0, 5.0])
velocity_right['high'] = fuzz.trimf(velocity_right.universe, [3.0, 5.0, 5.0])

rule0 = ctrl.Rule(distance_left['high'] & distance_right['high'], consequent = (velocity_left['high'],velocity_right['high']))
rule1 = ctrl.Rule((distance_left['low']), consequent = (velocity_right['low'] , velocity_left['high']))
rule2 = ctrl.Rule((distance_right['low'] & (distance_left['medium'] | distance_left['high'])), consequent = (velocity_right['high'] , velocity_left['low']))
rule3 = ctrl.Rule((distance_left['medium'] & distance_right['medium']), consequent = (velocity_right['low'], velocity_left['medium']))
rule4 = ctrl.Rule((distance_left['medium'] & (distance_right['low'] | distance_right['high'])), consequent = (velocity_right['medium'],velocity_left['low']))

velocity_control = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4])

walking = ctrl.ControlSystemSimulation(velocity_control)

robot = Robot()
robot.set_left_velocity(3.0)
robot.set_right_velocity(3.0)
i = 0
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    vel = wall_follow(us_distances[:8])
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
    if(i==1000):
        print(us_distances[:8])
        print("VELOCIDADE", vel)
        i=0
    i+=1

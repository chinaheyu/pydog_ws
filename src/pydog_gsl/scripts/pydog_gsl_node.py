#!/usr/bin/env python3
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import pathlib
import onnxruntime as ort
import numpy as np
import cv2
from olfaction_msgs.msg import gas_sensor, anemometer
import matplotlib.pyplot as plt


# set initial position
initial_position = (2, 14)
source_position = (3, 2)

# initialize path
pkg_path = pathlib.Path(rospkg.RosPack().get_path('pydog_gsl'))
map_path = pkg_path / 'map/occupancy.png'
model_path = pkg_path / 'model/dqn.onnx'
output_path = pkg_path / 'result.csv'

# initialize map_grid
map_grid = cv2.imread(map_path.as_posix())
map_grid = cv2.cvtColor(map_grid, cv2.COLOR_BGR2GRAY)
_, map_grid = cv2.threshold(map_grid, 125, 255, cv2.THRESH_BINARY)
map_grid = map_grid.astype(np.float32)
map_grid /= 255.0


# onnxruntime Policy
class Policy:
    def __init__(self, model_path: str) -> None:
        self.ort_sess = ort.InferenceSession(model_path)
        self.action_name = ['Up', 'Down', 'Left', 'Right']

    def __call__(self, model_input):
        output = self.ort_sess.run(['output'], {'input': model_input})[0]
        return np.argmax(output, axis=1, keepdims=True)[0, 0]
    
    def __len__(self):
        return len(self.action_name)

    def __getitem__(self, key):
        return self.action_name[key]


# observation matrix
class ObservationMatrix:
    def __init__(self, map_grid, initial_position, concentration_limit=200.0) -> None:
        self.trajectory_matrix = np.zeros(map_grid.shape, dtype=np.float32)
        self.obstacle_matrix = map_grid.copy()
        self.concentration_matrix = np.zeros(map_grid.shape, dtype=np.float32)
        self.airflow_x_matrix = np.zeros(map_grid.shape, dtype=np.float32)
        self.airflow_y_matrix = np.zeros(map_grid.shape, dtype=np.float32)
        self.agent_position = [initial_position[0], initial_position[1]]
        self.concentration_limit = concentration_limit
    
    def get_observation(self):
        trajectory_matrix_pad = np.pad(self.trajectory_matrix, (5, 5), 'constant', constant_values=0)
        obstacle_matrix_pad = np.pad(self.obstacle_matrix, (5, 5), 'constant', constant_values=1)
        concentration_matrix_pad = np.pad(self.concentration_matrix, (5, 5), 'constant', constant_values=0)
        airflow_x_matrix_pad = np.pad(self.airflow_x_matrix, (5, 5), 'constant', constant_values=0)
        airflow_y_matrix_pad = np.pad(self.airflow_y_matrix, (5, 5), 'constant', constant_values=0)

        observation_matrix = np.stack((trajectory_matrix_pad,
                                       obstacle_matrix_pad,
                                       concentration_matrix_pad,
                                       airflow_x_matrix_pad,
                                       airflow_y_matrix_pad), axis=0)

        observation_matrix = observation_matrix[:, self.agent_position[0]:self.agent_position[0] + 11, self.agent_position[1]:self.agent_position[1] + 11]

        # shape: (1, 5, 11, 11)
        return np.expand_dims(observation_matrix, 0)
    
    def move_agent(self, action):
        if action == 0:
            self.agent_position[0] -= 1
        if action == 1:
            self.agent_position[0] += 1
        if action == 2:
            self.agent_position[1] -= 1
        if action == 3:
            self.agent_position[1] += 1
    
    def get_agent_position(self):
        return self.agent_position
    
    def set_observation_data(self, concentration, airflow_x, airflow_y):
        if concentration > self.concentration_limit:
            concentration = 1.0
        else:
            concentration = concentration / self.concentration_limit
        self.trajectory_matrix[self.agent_position[0], self.agent_position[1]] += 1
        self.concentration_matrix[self.agent_position[0], self.agent_position[1]] = concentration
        self.airflow_x_matrix[self.agent_position[0], self.agent_position[1]] = airflow_x
        self.airflow_y_matrix[self.agent_position[0], self.agent_position[1]] = airflow_y


# actionlib
class MoveBase:
    def __init__(self) -> None:
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def move_to(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()


# sensors
class Sensors:
    def __init__(self) -> None:
        rospy.Subscriber("/PID/Sensor_reading", gas_sensor, self.gas_sensor_callback)
        rospy.Subscriber("/Anemometer/WindSensor_reading", anemometer, self.anemometer_callback)
        self.concentration = []
        self.airflow_x = []
        self.airflow_y = []
    
    def gas_sensor_callback(self, msg):
        self.concentration.append(msg.raw)

    def anemometer_callback(self, msg):
        self.airflow_x.append(msg.wind_speed * np.cos(msg.wind_direction))
        self.airflow_y.append(-msg.wind_speed * np.sin(msg.wind_direction))
    
    def read_all(self):
        self.concentration.clear()
        self.concentration.clear()
        self.airflow_x.clear()
        self.airflow_y.clear()
        rospy.sleep(1.0)
        return (sum(self.concentration) / len(self.concentration), sum(self.airflow_x) / len(self.airflow_x), sum(self.airflow_y) / len(self.airflow_y))


def to_map(x, y, grid_x=9, grid_y=16, world_x=4.48, world_y=8.08):
    x = (x + 0.5) * world_x / grid_x
    y = (y + 0.5) * world_y / grid_y
    return x, y


class DataLogger:
    def __init__(self) -> None:
        self.concentration = []
        self.airflow_x = []
        self.airflow_y = []
        self.position_x = []
        self.position_y = []

    def log(self, concentration, airflow_x, airflow_y, position_x, position_y):
        self.concentration.append(concentration)
        self.airflow_x.append(airflow_x)
        self.airflow_y.append(airflow_y)
        self.position_x.append(position_x)
        self.position_y.append(position_y)
    
    def plot(self):
        a = np.array([self.position_x, self.position_y, self.concentration, self.airflow_x, self.airflow_y])
        a = a.T
        np.savetxt(output_path.as_posix(), a, delimiter=",")

        plt.figure()
        plt.plot(self.concentration)
        plt.xlabel('Step')
        plt.ylabel('Concentration (ppm)')
        plt.show()


if __name__ == '__main__':
    rospy.init_node('pydog_gsl_node', anonymous=False)
    robot = MoveBase()
    obs = ObservationMatrix(map_grid, initial_position)
    policy = Policy(model_path.as_posix())
    sensors = Sensors()
    logger = DataLogger()

    print(f'Move to initial position.')
    target_position = initial_position
    robot.move_to(*to_map(*target_position))

    print(f'Start gas source localization task.')

    step_count = 0

    while not rospy.is_shutdown():
        step_count += 1

        # read sensors
        data = sensors.read_all()

        # log data
        logger.log(*data, *target_position)
        print(f'Step: {step_count:3d}'.center(30, '*'))
        print(f'Sensors Data: {data}')

        # create observation matrix
        obs.set_observation_data(*data)
        observation_matrix = obs.get_observation()

        # select action
        action = policy(observation_matrix)
        print(f'Selected Action: {policy[action]}')

        # move robot
        obs.move_agent(action)
        target_position = obs.get_agent_position()
        print(f'Target Position: {target_position}')
        robot.move_to(*to_map(*target_position))

        if target_position[0] == source_position[0] and target_position[1] == source_position[1]:
            print(f'Find gas source at step {step_count}.')
            break

    logger.plot()

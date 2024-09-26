import sys, os

from threading import Lock
import numpy as np
from scipy.spatial.transform import Rotation

from carla_msgs.msg import CarlaStatus
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from quad_sim_python_msgs.msg import QuadMotors, QuadWind, QuadState

import rospy
from tf import TransformListener, Exception as TransformException

from quad_sim_python import Quadcopter
from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict

quad_params = {}
# Moments of inertia:
# (e.g. from Bifilar Pendulum experiment https://arc.aiaa.org/doi/abs/10.2514/6.2007-6822)
Ixx = 0.0123
Iyy = 0.0123
Izz = 0.0224
IB  = np.array([[Ixx, 0,   0  ],
                [0,   Iyy, 0  ],
                [0,   0,   Izz]]) # Inertial tensor (kg*m^2)

IRzz = 2.7e-5   # Rotor moment of inertia (kg*m^2)
quad_params["mB"]   = 1.2    # mass (kg)
quad_params["g"]    = 9.81   # gravity (m/s^2)
quad_params["dxm"]  = 0.16   # arm length (m) - between CG and front
quad_params["dym"]  = 0.16   # arm length (m) - between CG and right
quad_params["dzm"]  = 0.05   # motor height (m)
quad_params["IB"]   = IB
quad_params["IRzz"] = IRzz
quad_params["Cd"]         = 0.1      # https://en.wikipedia.org/wiki/Drag_coefficient
quad_params["kTh"]        = 1.076e-5 # thrust coeff (N/(rad/s)^2)  (1.18e-7 N/RPM^2)
quad_params["kTo"]        = 1.632e-7 # torque coeff (Nm/(rad/s)^2)  (1.79e-9 Nm/RPM^2)
quad_params["minThr"]     = 0.1*4    # Minimum total thrust
quad_params["maxThr"]     = 9.18*4   # Maximum total thrust
quad_params["minWmotor"]  = 75       # Minimum motor rotation speed (rad/s)
quad_params["maxWmotor"]  = 925      # Maximum motor rotation speed (rad/s)
quad_params["tau"]        = 0.015    # Value for second order system for Motor dynamics
quad_params["kp"]         = 1.0      # Value for second order system for Motor dynamics
quad_params["damp"]       = 1.0      # Value for second order system for Motor dynamics
quad_params["motorc1"]    = 8.49     # w (rad/s) = cmd*c1 + c0 (cmd in %)
quad_params["motorc0"]    = 74.7
# Select whether to use gyroscopic precession of the rotors in the quadcopter dynamics
# ---------------------------
# Set to False if rotor inertia isn't known (gyro precession has negigeable effect on drone dynamics)
quad_params["usePrecession"] = False

quad_params["Ts"] = 1/200 # state calculation time step (current ode settings run faster using a smaller value)
quad_params["Tp"] = 1/25 # period it publishes the current pose
quad_params["Tfs"] = 1/50 # period it publishes the full state
quad_params["orient"] = "ENU"
quad_params["target_frame"] = 'flying_sensor'
quad_params["map_frame"] = 'map'

class QuadSim:
    def __init__(self):

        self.t = None
        self.res = None
        self.w_cmd_lock = Lock()
        self.wind_lock = Lock()
        self.sim_pub_lock = Lock()

        # pos[3], quat[4], rpy[3], vel[3], vel_dot[3], omega[3], omega_dot[3]
        self.curr_state = np.zeros(22, dtype='float64')
        self.curr_state[6] = 1 # the w of a quaternion

        self.wind = [0,0,0]
        self.prev_wind = [0,0,0]


        self.tf_listener = TransformListener()
        rospy.Subscriber('/carla/flying_sensor', Imu, self.check_flying_sensor_alive_cb)

        
    def check_flying_sensor_alive_cb(self, msg):
        rospy.loginfo("Flying sensor is alive, proceeding with simulation setup.")
        rospy.Subscriber('/carla/flying_sensor', Imu, None)  
        # Stop the subscription we don't need this subscriber anymore...

        # Read ROS2 parameters the user may have set 
        # E.g. (https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html):
        # --ros-args -p init_pose:=[0,0,0,0,0,0])
        # --ros-args --params-file params.yaml
        read_params = ROS2Params2Dict(list(quad_params.keys()) + ["init_pose"])
        for k,v in read_params.items():
            # Update local parameters
            quad_params[k] = v
        
        # Update ROS2 parameters
        Dict2ROS2Params(quad_params) # the controller needs to read some parameters from here

        # Timer for the tf
        # I couldn't find a way to receive it without using a timer 
        # to allow me to call lookup_transform after rclpy.spin(quad_node)
        self.tf_trials = 5
        rospy.Timer(rospy.Duration(1.0), self.on_tf_init_timer)

    def get_tf(self, t=0.0, timeout=1.0):
        try:
            now = rospy.Time.from_sec(t)
            self.tf_listener.waitForTransform(quad_params["map_frame"], quad_params["target_frame"], now, rospy.Duration(timeout))
            (trans, rot) = self.tf_listener.lookupTransform(quad_params["map_frame"], quad_params["target_frame"], now)

            rospy.logdebug(f'TF received: trans {trans}, rot {rot}')
            curr_pos = list(trans)
            curr_quat = list(rot)

            return now.to_sec(), curr_pos, curr_quat

        except TransformException as ex:
            rospy.logerr(f'Could not transform {quad_params["map_frame"]} to {quad_params["target_frame"]}: {ex}')


    def on_tf_init_timer(self):
        res = self.get_tf()
        if res is None:
            return
        
        self.t, init_pos, init_quat = res
        if "init_pose" not in quad_params:
            try:
                init_rpy = Rotation.from_quat(init_quat).as_euler('xyz')
            except Exception as exc:
                rospy.logerr(f'Something went wrong with the tf {res} generating the exception {exc}')
                raise            
            quad_params["init_pose"] = np.concatenate((init_pos,init_rpy))
            # Update ROS2 parameters
            Dict2ROS2Params({"init_pose": quad_params["init_pose"]}) # the controller needs to read some parameters from here
        else:
            self.start_sim()


    def start_sim(self):   
        params = ROS2Params2Dict(quad_params.keys())
        init_pose = np.array(params['init_pose']) # x0, y0, z0, phi0, theta0, psi0
        init_twist = np.array([0,0,0,0,0,0]) # xdot, ydot, zdot, p, q, r
        init_states = np.hstack((init_pose,init_twist))
        self.Ts = params['Ts']
        self.quad = Quadcopter(self.t, init_states, params=params.copy(), orient=params['orient'])
        self.w_cmd = [self.quad.params['w_hover']]*4
        new_params = {key: self.quad.params[key] for key in self.quad.params if key not in params}
        Dict2ROS2Params(new_params) # some parameters are created by the quad object

        self.quadpos_pub = rospy.Publisher(f'/carla/{quad_params["target_frame"]}/control/set_transform', Pose, queue_size=1)
        self.quadstate_pub = rospy.Publisher(f'/quadsim/{quad_params["target_frame"]}/state', QuadState, queue_size=1)

        rospy.Subscriber(f'/quadsim/{quad_params["target_frame"]}/w_cmd', QuadMotors, self.receive_w_cmd_cb)
        rospy.Subscriber(f'/quadsim/{quad_params["target_frame"]}/wind', QuadWind, self.receive_wind_cb)

        rospy.Timer(rospy.Duration(self.Ts), self.on_sim_loop)
        rospy.Timer(rospy.Duration(params['Tfs']), self.on_sim_publish_fs)
        rospy.Timer(rospy.Duration(params['Tp']), self.on_sim_publish_pose)

        rospy.loginfo('Simulator started!')

    def receive_w_cmd_cb(self, motor_msg):
        with self.w_cmd_lock:
            self.w_cmd = [motor_msg.m1, 
                          motor_msg.m2,
                          motor_msg.m3,
                          motor_msg.m4]
        rospy.logdebug(f'Received w_cmd: {self.w_cmd}')


    def receive_wind_cb(self, wind_msg):
        with self.wind_lock:
            self.wind = [wind_msg.vel_w, 
                         wind_msg.head_w,
                         wind_msg.elev_w]
        rospy.logdebug(f'Received wind: {self.wind}')


    def on_sim_loop(self):
        res = self.get_tf()

        if res is None:
            return
        
        new_t, curr_pos, curr_quat = res
        loops = int((new_t - self.t)/self.Ts)


        if self.wind_lock.acquire(blocking=False):
            self.prev_wind[:] = self.wind[:]
            self.wind_lock.release()

        for i in range(loops):
            with self.w_cmd_lock:
                self.quad.update(self.t, self.Ts, self.w_cmd, self.prev_wind)

            if self.sim_pub_lock.acquire(blocking=False):
                self.curr_state[0:3] = self.quad.pos[:]
                self.curr_state[3:7] = self.quad.quat[[1,2,3,0]] # the sim uses w x y z
                self.curr_state[7:10] = self.quad.euler[:]
                self.curr_state[10:13] = self.quad.vel[:]
                self.curr_state[13:16] = self.quad.vel_dot[:]
                self.curr_state[16:19] = self.quad.omega[:]
                self.curr_state[19:22] = self.quad.omega_dot[:]
                self.t += self.Ts
                self.sim_pub_lock.release()

        rospy.logdebug(f'Quad State: {self.curr_state}')


    def on_sim_publish_pose(self):
        if self.t is None:
            return
        pose_msg = Pose()
        with self.sim_pub_lock:
            pose_msg.position.x = float(self.curr_state[0])
            pose_msg.position.y = float(self.curr_state[1])
            pose_msg.position.z = float(self.curr_state[2])
            pose_msg.orientation.x = float(self.curr_state[3])
            pose_msg.orientation.y = float(self.curr_state[4])
            pose_msg.orientation.z = float(self.curr_state[5])
            pose_msg.orientation.w = float(self.curr_state[6])

        self.quadpos_pub.publish(pose_msg)


    def on_sim_publish_fs(self):
        if self.t is None:
            return
        state_msg = QuadState()
        with self.sim_pub_lock:
            now = rospy.Time.from_sec(self.t)
            state_msg.header.stamp = now
            state_msg.t = self.t
            state_msg.pos = self.curr_state[0:3][:]
            state_msg.quat = self.curr_state[3:7][:]
            state_msg.rpy = self.curr_state[7:10][:]
            state_msg.vel = self.curr_state[10:13][:]
            state_msg.vel_dot = self.curr_state[13:16][:]
            state_msg.omega = self.curr_state[16:19][:]
            state_msg.omega_dot = self.curr_state[19:22][:]

        self.quadstate_pub.publish(state_msg)
        rospy.logdebug(f'Quad State: {self.curr_state}')


def main():
    print("Starting QuadSim...")

    quad_node = QuadSim()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


    print("Shutting down QuadSim...")
    rospy.signal_shutdown("Simulation stopped")


if __name__ == '__main__':
   main()

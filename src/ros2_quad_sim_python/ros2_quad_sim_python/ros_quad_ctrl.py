import time
import sys, os
from threading import Lock
from copy import copy
import numpy as np
from numpy.linalg import norm

import rospy  # Sostituito rclpy con rospy
from geometry_msgs.msg import Twist
from quad_sim_python_msgs.msg import QuadMotors, QuadState, QuadControlSetPoint

from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict  # Manteniamo i nomi come richiesto
from quad_sim_python import Controller

# Aggiunto il controllo del percorso corrente
curr_path = os.getcwd()
if os.path.basename(curr_path) not in sys.path:
    sys.path.append(os.path.dirname(os.getcwd()))

# Parametri di controllo
ctrl_params = {
    # Position P gains
    "Px": 2.0,
    "Py": 2.0,
    "Pz": 1.0,

    # Velocity P-D gains
    "Pxdot": 5.0,
    "Dxdot": 0.5,
    "Ixdot": 5.0,

    "Pydot": 5.0,
    "Dydot": 0.5,
    "Iydot": 5.0,

    "Pzdot": 4.0,
    "Dzdot": 0.5,
    "Izdot": 5.0,

    # Attitude P gains
    "Pphi": 8.0,
    "Ptheta": 8.0,
    "Ppsi": 1.5,

    # Rate P-D gains
    "Pp": 1.5,
    "Dp": 0.04,

    "Pq": 1.5,
    "Dq": 0.04,

    "Pr": 1.0,
    "Dr": 0.1,

    # Max Velocities (x,y,z) [m/s]
    "uMax": 50.0,
    "vMax": 50.0,
    "wMax": 50.0,

    "saturateVel_separately": True,

    # Max tilt [degrees]
    "tiltMax": 50.0,

    # Max Rate [rad/s]
    "pMax": 200.0,
    "qMax": 200.0,
    "rMax": 150.0,

    # Minimum velocity for yaw follow to kick in [m/s]
    "minTotalVel_YawFollow": 0.1,

    "useIntegral": True,  # Include integral gains in linear velocity control
}


class QuadCtrl:
    def __init__(self):
        # Inizializzazione del nodo
        rospy.init_node('quad_ctrl', anonymous=True)

        # Pubblicatori e sottoscrittori
        self.w_cmd_pub = rospy.Publisher('/quad_w_cmd', QuadMotors, queue_size=10)
        self.state_sub = rospy.Subscriber('/quad_state', QuadState, self.state_callback)
        self.cmd_sub = rospy.Subscriber('/quad_cmd', QuadControlSetPoint, self.cmd_callback)

        # Blocco per la concorrenza
        self.lock = Lock()

        # Caricamento dei parametri iniziali
        self.ctrl = Controller(Dict2ROS2Params(ctrl_params))
        self.current_state = None
        self.current_time = None
        self.last_time = None

        # Memorizzazione dei messaggi di controllo
        self.cmd = None

        rospy.loginfo("QuadCtrl node initialized")

    def state_callback(self, state_msg):
        with self.lock:
            self.current_state = state_msg
            self.current_time = rospy.Time.now()

            if self.cmd is not None:
                self.process_control()

    def cmd_callback(self, cmd_msg):
        with self.lock:
            self.cmd = cmd_msg
            if self.current_state is not None:
                self.process_control()

    def process_control(self):
        """Elabora il messaggio di controllo e pubblica i comandi per i motori"""
        # Controlla che ci sia stato un aggiornamento del tempo
        if self.last_time is None:
            self.last_time = self.current_time
            return

        # Calcola l'intervallo di tempo
        dt = (self.current_time - self.last_time).to_sec()

        if dt == 0:
            return

        # Calcola il comando di controllo
        motor_msg = self.ctrl.update(self.current_state, self.cmd, dt)

        # Pubblica i comandi ai motori
        self.w_cmd_pub.publish(motor_msg)

        # Aggiorna l'ultimo tempo
        self.last_time = self.current_time

    def spin(self):
        rospy.spin()  # Esegui lo spin del nodo


def main():
    print("Starting QuadCtrl...")
    ctrl_node = QuadCtrl()
    try:
        ctrl_node.spin()  # Esegui lo spin del nodo
    except rospy.ROSInterruptException:
        pass

    print("Shutting down QuadCtrl...")


if __name__ == '__main__':
    main()

# rclpy_param_helper
Convert between Python dictionary and ROS2 parameters

# Usage example
```
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict

class MyNode(Node):
    def __init__(self):
        super().__init__('mynode', 
                         allow_undeclared_parameters=True, # necessary for using set_parameters
                         automatically_declare_parameters_from_overrides=True) # allows command line parameters

        # Read ROS2 parameters the user may have set 
        # E.g. (https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html):
        # --ros-args -p param_key_1:=5 -p param_key_2:="some random string" -p another_param_key:=[1.0,2.0,3.0]
        # --ros-args --params-file params.yaml
        read_params = ROS2Params2Dict(self, 'mynode', ['param_key_1', 'param_key_2', 'another_param_key'])

        # Read parameters from a different node
        read_more_params = ROS2Params2Dict(self, 'differentnode', ['param_key_1', 'param_key_2', 'another_param_key'])

        ...

        new_dict = {'my_mult_dim_numpy_array': np.zeros((10,3))}

        # Update ROS2 parameters
        Dict2ROS2Params(self, new_dict)
        # It will create a parameter 'my_mult_dim_numpy_array___shape' with (10,3)
        # and another parameter called 'my_mult_dim_numpy_array' with a 1D array with length 30.
        # This way it becomes easier to store ROS2 parameters that have matrices, etc.

        ...

        read_params = ROS2Params2Dict(self,'mynode')
        # read_params['my_mult_dim_numpy_array'] will have the original numpy array with shape (10,3).

```

"""Convert between Python dictionary and ROS1 parameters

ROS1 doesn't accept matrices as parameters directly, so we need to handle them using 
nested lists or numpy arrays. This script converts arrays to numpy arrays to maintain
uniformity and processes only basic types: int, float, string, bool.

It uses rospy to get and set parameters.
"""
import rospy
import numpy as np

def Dict2ROS2Params(dictparams):
    """
    Convert a Python dictionary into ROS 1 parameters and set them using rospy.
    """
    for k, v in dictparams.items():
        value_type = str(type(v)).lower()

        if 'none' in value_type:
            rospy.set_param(k, None)
        elif 'bool' in value_type:
            rospy.set_param(k, bool(v))
        elif 'int' in value_type:
            rospy.set_param(k, int(v))
        elif 'float' in value_type:
            rospy.set_param(k, float(v))
        elif 'str' in value_type:
            rospy.set_param(k, str(v))
        elif 'array' in value_type or 'list' in value_type or 'tuple' in value_type:
            v = np.array(v)
            list_value = v.ravel().tolist()
            rospy.set_param(k, list_value)
        else:
            rospy.logerr(f"Type {value_type} is not supported!")
            raise TypeError(f"{value_type} is not supported!")

        rospy.loginfo(f"{k} : {v} set as a parameter!")

def ROS2Params2Dict(parameter_names):
    """
    Retrieve ROS 1 parameters as a dictionary from a list of parameter names.
    """
    param_dict = {}
    for name in parameter_names:
        try:
            param_dict[name] = rospy.get_param(name)
        except KeyError:
            rospy.logwarn(f"Parameter {name} is not available!")

    return param_dict

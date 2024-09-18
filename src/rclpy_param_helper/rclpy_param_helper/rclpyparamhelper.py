"""Convert between Python dictionary and ROS2 parameters
https://docs.ros2.org/latest/api/rclpy/api/parameters.html

ROS2 doesn't accept matrices as parameters, so we declare a parameter named "<name>___shape" with the shape.
Arrays (and matrices) are converted to numpy arrays and they MUST use only one type: int, float, string, bool or byte.
https://numpy.org/doc/stable/reference/generated/numpy.dtype.char.html

It expects the node to set 'allow_undeclared_parameters=True'.
"""
import rclpy
from rclpy.parameter import Parameter
# from rcl_interfaces.msg import ParameterDescriptor
# from rcl_interfaces.srv import GetParameters
from ros2param.api import call_get_parameters
import numpy as np

type2int = {}
int2type = {}
for ti in Parameter.Type:
    type2int[ti.name] = ti.value
    int2type[ti.value] = ti.name

def Dict2ROS2Params(node, dictparams):
    params = []
    for k,v in dictparams.items():
        name = k
        value = v
        value_type = str(type(value)).lower()

        if 'None' in value_type:
            params.append(Parameter(name=name, type_=Parameter.Type.NOT_SET, value=value))
            # node.declare_parameter(name, descriptor=ParameterDescriptor(name=name, type=type2int["NOT_SET"]))
        elif 'bool' in value_type:
            params.append(Parameter(name=name, type_=Parameter.Type.BOOL, value=value))
            # node.declare_parameter(name, value=value, descriptor=ParameterDescriptor(name=name, type=type2int["BOOL"]))
        elif 'int' in value_type:
            params.append(Parameter(name=name, type_=Parameter.Type.INTEGER, value=value))
            # node.declare_parameter(name, value=value, descriptor=ParameterDescriptor(name=name, type=type2int["INTEGER"]))
        elif 'float' in value_type:
            params.append(Parameter(name=name, type_=Parameter.Type.DOUBLE, value=value))
            # node.declare_parameter(name, value=value, descriptor=ParameterDescriptor(name=name, type=type2int["DOUBLE"]))
        elif 'str' in value_type:
            params.append(Parameter(name=name, type_=Parameter.Type.STRING, value=value))
            # node.declare_parameter(name, value=value, descriptor=ParameterDescriptor(name=name, type=type2int["STRING"]))
        elif 'array' in value_type or 'list' in value_type or 'tuple' in value_type:
            value = np.array(value)
            list_value = value.ravel().tolist()
            value_shape = value.shape
            if len(value_shape)>1:
                shape_descr = name + '___shape'
                params.append(Parameter(name=shape_descr, type_=Parameter.Type.INTEGER_ARRAY, value=value_shape))
            # node.declare_parameter(shape_descr, value=value_shape, descriptor=ParameterDescriptor(name=name, type=type2int["INTEGER_ARRAY"]))
            char_dtype = value.dtype.char
            if   char_dtype.lower()=='q' or char_dtype.lower()=='l' or char_dtype.lower()=='i' or char_dtype.lower()=='h'  or char_dtype.lower()=='b':
                # uint becomes int...
                params.append(Parameter(name=name, type_=Parameter.Type.INTEGER_ARRAY, value=list_value))
                # param_descriptor = ParameterDescriptor(name=name, type=type2int["INTEGER_ARRAY"])
            elif char_dtype=='d' or char_dtype=='f' or char_dtype=='e':
                params.append(Parameter(name=name, type_=Parameter.Type.DOUBLE_ARRAY, value=list_value))
                # param_descriptor = ParameterDescriptor(name=name, type=type2int["DOUBLE_ARRAY"])
            elif char_dtype=='U':
                params.append(Parameter(name=name, type_=Parameter.Type.STRING_ARRAY, value=list_value))
                # param_descriptor = ParameterDescriptor(name=name, type=type2int["STRING_ARRAY"])
            elif char_dtype=='?':
                params.append(Parameter(name=name, type_=Parameter.Type.BOOL_ARRAY, value=list_value))
                # param_descriptor = ParameterDescriptor(name=name, type=type2int["BOOL_ARRAY"])
            elif char_dtype=='S':
                params.append(Parameter(name=name, type_=Parameter.Type.BYTE_ARRAY, value=list_value))
                # param_descriptor = ParameterDescriptor(name=name, type=type2int["BYTE_ARRAY"])
            else:
                node.get_logger().error(f"Array of {value.dtype, char_dtype} is not supported!")
                raise TypeError(f"Array of {value.dtype, char_dtype} is not supported!")

            # node.declare_parameter(name, value=list_value, descriptor=param_descriptor)
        else:
            node.get_logger().error(f"{value_type} is not supported!")
            raise TypeError(f"{value_type} is not supported!")
        
        node.get_logger().info(f"{name} : {value} set as a parameter!")

    node.set_parameters(params)



def ROS2Params2Dict(node, node_name, parameter_names):

    if node.get_name() != node_name:
        def get_params(parameter_names):
            response = call_get_parameters(node=node, node_name=node_name, parameter_names=parameter_names)
            # # create client
            # client = node.create_client(
            #     GetParameters,
            #     f'{node_name}/get_parameters')

            # # call as soon as ready
            # ready = client.wait_for_service(timeout_sec=5.0)
            # if not ready:
            #     raise RuntimeError('Wait for service timed out')

            # request = GetParameters.Request()
            # request.names = parameter_names
            # future = client.call_async(request)
            # rclpy.spin_until_future_complete(node, future)

            # # handle response
            # response = future.result()
            # if response is None:
            #     e = future.exception()
            #     raise RuntimeError(
            #         f'Exception while calling service of node '
            #         "'{node_name}': {e}")
            
            # node.destroy_client(client) # ignoring the response since it shouldn't return False, right???

            return response.values
    else:
        def get_params(parameter_names):
            return [param.to_parameter_msg().value for param in node.get_parameters(parameter_names)]

    # response is expected to follow the parameter_names order...
    try:
        response = get_params(parameter_names)
    except RuntimeError:
        node.get_logger().error(f"Parameters from '{node_name}' are not available!")
        raise

    param_dict = {}
    shapes2retrieve = []
    for name, param in zip(parameter_names, response):
        if param.type: # NOT_SET is 0
            param_dict[name] = getattr(param, int2type[param.type].lower()+"_value")
            if "ARRAY" in int2type[param.type]:
                shapes2retrieve.append(name+"___shape")
        else:
            node.get_logger().warn(f"Parameter {name} is not available!")

    if shapes2retrieve:
        response = get_params(shapes2retrieve)
        for name, param in zip(shapes2retrieve, response):
            array_name = name[:len(name)-len("___shape")]
            if param.type: # NOT_SET is 0
                shape = getattr(param, int2type[param.type].lower()+"_value")
                param_dict[array_name] = np.array(param_dict[array_name]).reshape(shape)
            else:
                pass # the shape was not set, so it will be treated as 1D

    return param_dict
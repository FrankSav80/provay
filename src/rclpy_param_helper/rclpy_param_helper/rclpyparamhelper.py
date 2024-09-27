"""Convert between Python dictionary and ROS1 parameters

It uses rospy to get and set parameters.

Dict2ROS2Params(dictparams): Converte i parametri e gestisce correttamente i tipi di dati, inclusi array e matrici, registrando ogni operazione.
ROS2Params2Dict(parameter_names): Recupera i parametri e gestisce le forme associate, ricostruendo correttamente gli array multidimensionali.

"""
import rospy
import numpy as np

def Dict2ROS2Params(dictparams):
    """
    Converte un dizionario Python in parametri ROS1 e li imposta usando rospy.
    """
    for k, v in dictparams.items():
        name = k
        value = v
        value_type = str(type(value)).lower()

        if 'none' in value_type:
            rospy.set_param(name, None)
        elif 'bool' in value_type:
            rospy.set_param(name, bool(value))
        elif 'int' in value_type:
            rospy.set_param(name, int(value))
        elif 'float' in value_type:
            rospy.set_param(name, float(value))
        elif 'str' in value_type:
            rospy.set_param(name, str(value))
        elif 'array' in value_type or 'list' in value_type or 'tuple' in value_type:
            value = np.array(value)
            list_value = value.ravel().tolist()
            rospy.set_param(name, list_value)

            shape_descr = f"{name}___shape"
            rospy.set_param(shape_descr, value.shape)

        else:
            rospy.logerr(f"{value_type} non è supportato!")
            raise TypeError(f"{value_type} non è supportato!")

        rospy.loginfo(f"Parametro {name} : {value} impostato!")  # Logging


def ROS2Params2Dict(parameter_names):
    """
    Retrieve ROS 1 parameters as a dictionary from a list of parameter names.
    Handles both namespaced and non-namespaced parameters.
    """
    param_dict = {}
    quadsim_namespace = "/quadsim/"  # Namespace da usare per i parametri quadsim
    
    for name in parameter_names:
        # Prima cerca il parametro senza il namespace
        try:
            param_value = rospy.get_param(name)
            param_dict[name] = param_value
            rospy.loginfo(f"Retrieved parameter {name}: {param_value}")

            # Controlla se esiste una forma associata per array/matrici
            shape_name = f"{name}___shape"
            if rospy.has_param(shape_name):
                shape = rospy.get_param(shape_name)
                param_dict[name] = np.array(param_value).reshape(shape)
                rospy.loginfo(f"Reshaped parameter {name} to shape {shape}")

        # Se non esiste senza namespace, cerca nel namespace quadsim
        except KeyError:
            rospy.logwarn(f"Parameter {name} not found, trying with namespace quadsim.")
            
            try:
                full_name = quadsim_namespace + name
                param_value = rospy.get_param(full_name)
                param_dict[name] = param_value
                rospy.loginfo(f"Retrieved parameter {full_name}: {param_value}")

                # Controlla se esiste una forma associata per array/matrici
                shape_name = f"{full_name}___shape"
                if rospy.has_param(shape_name):
                    shape = rospy.get_param(shape_name)
                    param_dict[name] = np.array(param_value).reshape(shape)
                    rospy.loginfo(f"Reshaped parameter {full_name} to shape {shape}")

            except KeyError:
                rospy.logwarn(f"Parameter {name} is not available even in quadsim namespace!")
            except Exception as e:
                rospy.logerr(f"Error retrieving parameter {full_name}: {e}")
        except Exception as e:
            rospy.logerr(f"Error retrieving parameter {name}: {e}")

    return param_dict

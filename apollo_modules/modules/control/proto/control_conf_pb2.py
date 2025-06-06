"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
_sym_db = _symbol_database.Default()
from ....modules.canbus.proto import chassis_pb2 as modules_dot_canbus_dot_proto_dot_chassis__pb2
from ....modules.control.proto import pad_msg_pb2 as modules_dot_control_dot_proto_dot_pad__msg__pb2
from ....modules.control.proto import lat_controller_conf_pb2 as modules_dot_control_dot_proto_dot_lat__controller__conf__pb2
from ....modules.control.proto import lon_controller_conf_pb2 as modules_dot_control_dot_proto_dot_lon__controller__conf__pb2
from ....modules.control.proto import mpc_controller_conf_pb2 as modules_dot_control_dot_proto_dot_mpc__controller__conf__pb2
DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n(modules/control/proto/control_conf.proto\x12\x0eapollo.control\x1a"modules/canbus/proto/chassis.proto\x1a#modules/control/proto/pad_msg.proto\x1a/modules/control/proto/lat_controller_conf.proto\x1a/modules/control/proto/lon_controller_conf.proto\x1a/modules/control/proto/mpc_controller_conf.proto"\xae\x0c\n\x0bControlConf\x12!\n\x15control_test_duration\x18\x01 \x01(\x01:\x02-1\x12\x1f\n\x10enable_csv_debug\x18\x02 \x01(\x08:\x05false\x12+\n\x1cenable_speed_station_preview\x18\x03 \x01(\x08:\x05false\x12#\n\x14is_control_test_mode\x18\x04 \x01(\x08:\x05false\x12*\n\x1buse_preview_speed_for_table\x18\x05 \x01(\x08:\x05false\x12+\n\x1cenable_input_timestamp_check\x18\x06 \x01(\x08:\x05false\x12%\n\x19max_localization_miss_num\x18\x07 \x01(\x05:\x0220\x12 \n\x14max_chassis_miss_num\x18\x08 \x01(\x05:\x0220\x12!\n\x15max_planning_miss_num\x18\t \x01(\x05:\x0220\x12+\n\x1dmax_acceleration_when_stopped\x18\n \x01(\x01:\x040.01\x12\x1d\n\x10steer_angle_rate\x18\x0b \x01(\x01:\x03100\x12#\n\x15enable_gain_scheduler\x18\x0c \x01(\x08:\x04true\x12\x1d\n\x0fset_steer_limit\x18\r \x01(\x08:\x04true\x12"\n\x13enable_slope_offset\x18\x0e \x01(\x08:\x05false\x12\x1f\n\x10lock_steer_speed\x18\x0f \x01(\x01:\x050.081\x122\n#enable_navigation_mode_error_filter\x18\x10 \x01(\x08:\x05false\x124\n&enable_navigation_mode_position_update\x18\x11 \x01(\x08:\x04true\x12%\n\x17enable_persistent_estop\x18\x12 \x01(\x08:\x04true\x12\x16\n\x0econtrol_period\x18\x13 \x01(\x01\x12!\n\x19max_planning_interval_sec\x18\x14 \x01(\x01\x12$\n\x1cmax_planning_delay_threshold\x18\x15 \x01(\x01\x128\n\x0cdriving_mode\x18\x16 \x01(\x0e2".apollo.canbus.Chassis.DrivingMode\x12-\n\x06action\x18\x17 \x01(\x0e2\x1d.apollo.control.DrivingAction\x12\x18\n\x10soft_estop_brake\x18\x18 \x01(\x01\x12F\n\x12active_controllers\x18\x19 \x03(\x0e2*.apollo.control.ControlConf.ControllerType\x12\'\n\x1fmax_steering_percentage_allowed\x18\x1a \x01(\x05\x12\x1f\n\x17max_status_interval_sec\x18\x1b \x01(\x01\x12>\n\x13lat_controller_conf\x18\x1c \x01(\x0b2!.apollo.control.LatControllerConf\x12>\n\x13lon_controller_conf\x18\x1d \x01(\x0b2!.apollo.control.LonControllerConf\x12\x19\n\x11trajectory_period\x18\x1e \x01(\x01\x12\x16\n\x0echassis_period\x18\x1f \x01(\x01\x12\x1b\n\x13localization_period\x18  \x01(\x01\x12 \n\x18minimum_speed_resolution\x18! \x01(\x01\x12>\n\x13mpc_controller_conf\x18" \x01(\x0b2!.apollo.control.MPCControllerConf\x12\x1b\n\x13query_relative_time\x18# \x01(\x01\x12 \n\x18minimum_speed_protection\x18$ \x01(\x01\x12)\n\x1cmax_path_remain_when_stopped\x18% \x01(\x01:\x030.3"L\n\x0eControllerType\x12\x12\n\x0eLAT_CONTROLLER\x10\x00\x12\x12\n\x0eLON_CONTROLLER\x10\x01\x12\x12\n\x0eMPC_CONTROLLER\x10\x02')
_CONTROLCONF = DESCRIPTOR.message_types_by_name['ControlConf']
_CONTROLCONF_CONTROLLERTYPE = _CONTROLCONF.enum_types_by_name['ControllerType']
ControlConf = _reflection.GeneratedProtocolMessageType('ControlConf', (_message.Message,), {'DESCRIPTOR': _CONTROLCONF, '__module__': 'modules.control.proto.control_conf_pb2'})
_sym_db.RegisterMessage(ControlConf)
if _descriptor._USE_C_DESCRIPTORS == False:
    DESCRIPTOR._options = None
    _CONTROLCONF._serialized_start = 281
    _CONTROLCONF._serialized_end = 1863
    _CONTROLCONF_CONTROLLERTYPE._serialized_start = 1787
    _CONTROLCONF_CONTROLLERTYPE._serialized_end = 1863
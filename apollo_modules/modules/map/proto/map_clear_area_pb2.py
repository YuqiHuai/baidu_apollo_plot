"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
_sym_db = _symbol_database.Default()
from ....modules.map.proto import map_id_pb2 as modules_dot_map_dot_proto_dot_map__id__pb2
from ....modules.map.proto import map_geometry_pb2 as modules_dot_map_dot_proto_dot_map__geometry__pb2
DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n&modules/map/proto/map_clear_area.proto\x12\x0capollo.hdmap\x1a\x1emodules/map/proto/map_id.proto\x1a$modules/map/proto/map_geometry.proto"w\n\tClearArea\x12\x1c\n\x02id\x18\x01 \x01(\x0b2\x10.apollo.hdmap.Id\x12$\n\noverlap_id\x18\x02 \x03(\x0b2\x10.apollo.hdmap.Id\x12&\n\x07polygon\x18\x03 \x01(\x0b2\x15.apollo.hdmap.Polygon')
_CLEARAREA = DESCRIPTOR.message_types_by_name['ClearArea']
ClearArea = _reflection.GeneratedProtocolMessageType('ClearArea', (_message.Message,), {'DESCRIPTOR': _CLEARAREA, '__module__': 'modules.map.proto.map_clear_area_pb2'})
_sym_db.RegisterMessage(ClearArea)
if _descriptor._USE_C_DESCRIPTORS == False:
    DESCRIPTOR._options = None
    _CLEARAREA._serialized_start = 126
    _CLEARAREA._serialized_end = 245
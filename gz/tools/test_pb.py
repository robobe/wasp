from google.protobuf import text_format
from pygazebo.msg import vector3d_pb2
import io
msg = vector3d_pb2.Vector3d()
msg.x = 1
msg.y = 1
msg.z = 1
string_out = io.StringIO()
string_out.write(text_format.MessageToString(msg))
print(str(string_out.getvalue()))
data = str(string_out.getvalue())
print(data)
msg2 = vector3d_pb2.Vector3d()
text_format.Parse(data, msg2)
print(msg2.x)
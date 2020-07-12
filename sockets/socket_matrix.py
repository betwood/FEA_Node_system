import bpy
from bpy.types import NodeTree, Node, NodeSocket
from .socket_base import SocketBase

# Custom socket type
class MatrixSocket(NodeSocket, SocketBase):
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SocketTypeMatrix'
    # Label for nice name display
    bl_label = "Matrix Node Socket"
    # define color
    color = (0.0, 0.4, 1.0, 0.5)

    default_value: bpy.props.StringProperty(default="[]")
    # default_type: bpy.props.StringProperty(default="NUMBER")

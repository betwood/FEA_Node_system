import bpy
from bpy.types import NodeTree, Node, NodeSocket
from .socket_base import SocketBase

# Custom socket type
class GenericSocket(NodeSocket, SocketBase):
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SocketTypeGeneric'
    # Label for nice name display
    bl_label = "Generic Node Socket"
    # define color
    color = (1.0, 0.4, 1.0, 0.5)

    default_value: bpy.props.StringProperty()
    # default_type: bpy.props.StringProperty(default="NUMBER")

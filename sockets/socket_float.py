import bpy
from bpy.types import NodeTree, Node, NodeSocket
from .socket_base import SocketBase

# Custom socket type
class FloatSocket(NodeSocket, SocketBase):
    # Description string
    '''Custom node socket type'''
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SocketTypeFloat'
    # Label for nice name display
    bl_label = "Float Node Socket"
    # define color
    color = (1.0, 0.4, 0.216, 0.5)

    default_value: bpy.props.FloatProperty(default=0)
    default_type: bpy.props.StringProperty(default="NUMBER")

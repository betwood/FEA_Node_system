import bpy
from bpy.types import NodeTree, Node, NodeSocket, Object
from .socket_base import SocketBase

# Custom socket type
class VectorSocket(NodeSocket, SocketBase):
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SocketTypeVector'
    # Label for nice name display
    bl_label = "Vector Node Socket"
    # define color
    color = (0.0, 1.0, 1.0, 0.5)

    default_value: bpy.props.FloatVectorProperty()
    # default_type: bpy.props.StringProperty(default="NUMBER")

    # def draw_layout(self, context, layout, node, text):
    #     col = layout.column()
    #     col.prop_search(self, "default_value", context.scene, "objects", text="")

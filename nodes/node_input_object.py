import bpy
import numpy as np
import bmesh
import math
import mathutils

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False

class NodeInputObject(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'InputObjectNode'
    # Label for nice name display
    bl_label = "Object input"


    float_value: bpy.props.FloatProperty(default=5)
    object: bpy.props.PointerProperty(type=Object)
    
    
    def init(self, context):
        self.outputs.new('SocketTypeObject', "Object")


    def draw_buttons(self, context, layout):
        col = layout.column()
        col.prop_search(self, "object", context.scene, "objects", text="")
        # self.eval()
        # col = layout.column()
        # col.prop_search(self, "object", context.scene, "objects")
        pass

    def get_object(self):
        print(self.object)
        return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):
        self.outputs[0].set_value(self.object)
        return self.object

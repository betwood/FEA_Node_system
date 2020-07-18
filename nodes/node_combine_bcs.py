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

class BCCombine(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'CombineBCSNode'
    # Label for nice name display
    bl_label = "BC Combine"


    float_value: bpy.props.FloatProperty(default=5)
    vertex_group: bpy.props.StringProperty(name="Vertex Group")
    x: bpy.props.BoolProperty(default=True)
    y: bpy.props.BoolProperty(default=True)
    z: bpy.props.BoolProperty(default=True)
    mom_x: bpy.props.BoolProperty(default=True)
    mom_y: bpy.props.BoolProperty(default=True)
    mom_z: bpy.props.BoolProperty(default=True)
    max_mult: bpy.props.IntProperty(default=3)

    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "boundary conditions")
        self.inputs.new('SocketTypeMatrix', "bc_1")
        self.inputs.new('SocketTypeMatrix', "bc_2")


    def draw_buttons(self, context, layout):
        pass

    # def get_object(self):
    #     print(self.object)
    #     return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):

        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]

        # get inputs from previous nodes
        val1 = np.array(self.get_value(input_socket_1))
        val2 = np.array(self.get_value(input_socket_2))

        out = np.logical_or(val1, val2)

        return out


import bpy
import numpy as np
import bmesh
import math
import mathutils

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base import NodeBase
from .node_output import UpdateObjectNodeOperator
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False

class NodeShapekeyOutput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'OutputShapekeyNode'
    # Label for nice name display
    bl_label = "Shapekey Output"


    sk_index: bpy.props.IntProperty(min=0, max=5)
    
    def init(self, context):
        self.inputs.new('SocketTypeMatrix', "output")


    def draw_buttons(self, context, layout):
        layout.prop(self, "sk_index", text="shape key index")
        # self.eval()
        # col = layout.column()
        # col.prop_search(self, "object", context.scene, "objects")
        pass

    def draw_buttons_ext(self, context, layout):
        layout.operator("node.evaluate", text="evaluate")
        layout.label(text=str(self.object))
        pass

    def get_object(self):
        print(self.object)
        return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):
        # set sockets
        input_socket_1 = self.inputs[0]

        # get inputs from previous nodes
        U = self.get_value(input_socket_1)
        print(U)

        # get basis shape key
        self.object.active_shape_key_index = 0
        sk_basis = self.object.active_shape_key

        # make active shape key the index value and select it
        self.object.active_shape_key_index = self.sk_index
        sk = self.object.active_shape_key


        #apply to mesh
        for i in range(len(self.object.data.vertices)):
            if self.solve_type == "1DFRAME":
                sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[6*i], U[6*i+1], U[6*i+2]))
            else:
                sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[3*i], U[3*i+1], U[3*i+2]))
        return
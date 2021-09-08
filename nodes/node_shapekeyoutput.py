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


    sk_index: bpy.props.IntProperty(min=0, default=1)
    mult: bpy.props.FloatProperty(min=.01, default=1)
    
    def init(self, context):
        self.inputs.new('SocketTypeMatrix', "output")


    def draw_buttons(self, context, layout):
        if(bpy.context.active_node == self):
            layout.operator("node.evaluate", text="evaluate")
        layout.prop(self, "sk_index", text="shape key index")
        layout.prop(self, "mult", text="multiplier")
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
        U *= self.mult
        # print(U)

        if(self.object.data.shape_keys == None):
            self.object.shape_key_add(from_mix=False)

        # get basis shape key
        self.object.active_shape_key_index = 0
        sk_basis = self.object.active_shape_key

        # make active shape key the index value and select it
        self.object.active_shape_key_index = self.sk_index
        sk = self.object.active_shape_key


        #apply to mesh
        for j in range(0, U.shape[1]):
            # adds new shape key if none exist
            test = self.sk_index + j
            test1 = len(self.object.data.shape_keys.key_blocks)
            while self.sk_index + j >= len(self.object.data.shape_keys.key_blocks):
                self.object.shape_key_add(from_mix=False)

            # selects shape key
            self.object.active_shape_key_index = self.sk_index + j
            sk = self.object.active_shape_key

            # applies data to shape key depending on solve type
            for i in range(len(self.object.data.vertices)):
                if self.solve_type == "1DFRAME":
                    sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[6*i, j], U[6*i+1, j], U[6*i+2, j]))
                
                elif self.solve_type == "2DTruss":
                    sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[2*i, j], U[2*i+1, j], 0))

                else:
                    sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[3*i, j], U[3*i+1, j], U[3*i+2, j]))
        return
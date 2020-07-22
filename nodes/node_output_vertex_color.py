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

class NodeVertexColorOutput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'OutputVertexColorNode'
    # Label for nice name display
    bl_label = "Vertex Color Output"


    vc_index: bpy.props.IntProperty(min=1, max=6)
    
    def init(self, context):
        self.inputs.new('SocketTypeMatrix', "output")


    def draw_buttons(self, context, layout):
        layout.prop(self, "vc_index", text="vertex color index")
        # self.eval()
        # col = layout.column()
        # col.prop_search(self, "object", context.scene, "objects")
        pass

    def draw_buttons_ext(self, context, layout):
        super().draw_buttons_ext(context, layout)
        layout.operator("node.evaluate", text="evaluate")

    def get_object(self):
        print(self.object)
        return self.object

    def update_value(self, context):
        print("updated")

    def absmax(self, a, axis=None):
        amax = a.max(axis)
        amin = a.min(axis)
        return np.where(-amin > amax, amin, amax)

    def eval(self):
        # set sockets
        input_socket_1 = self.inputs[0]

        # get inputs from previous nodes
        U = np.matrix(self.get_value(input_socket_1))
        print(U)

        # make base vertex color
        self.object.data.vertex_colors.active_index = 0
        vc_base = self.object.data.vertex_colors.active

        # make active shape key the index value and select it
        self.object.data.vertex_colors.active_index = self.vc_index
        vc = self.object.data.vertex_colors.active

        # create list of dispacements
        print(len(U) / 3)
        print(len(U))

        if self.solve_type == "1DFRAME":
            disp = np.zeros(int(len(U) / 6))
            rot = np.zeros(int(len(U) / 6))
            for i in range(len(disp)):
                disp[i] = math.sqrt(U[6 * i] ** 2 + U[6 * i + 1] ** 2 + U[6 * i + 2] ** 2)
                rot[i] = math.sqrt(U[6 * i + 3] ** 2 + U[6 * i + 4] ** 2 + U[6 * i + 5] ** 2)
        else:
            disp = np.zeros(int(len(U) / 3))
            for i in range(len(disp)):
                disp[i] = math.sqrt(U[3 * i] ** 2 + U[3 * i + 1] ** 2 + U[3 * i + 2] ** 2)

        print("disp", disp)

        i = 0
        #apply to mesh
        for poly in self.object.data.polygons:
            for loop_index in poly.loop_indices:
                if self.solve_type == "1DFRAME":
                    loop = self.object.data.loops[loop_index]
                    v = loop.vertex_index
                    # color_value = (U[v * 2] ^ 2 + U[v * 2 + 1] ^ 2 + U[v * 2 + 2] ^ 2) ^ (1/2) / self.absmax(U)
                    color_value = disp[v] / self.absmax(disp)
                    color = (color_value, color_value, color_value, 1.0)
                    vc.data[loop_index].color = color
                    # sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[6*i], U[6*i+1], U[6*i+2]))
                else:
                    # sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[3*i], U[3*i+1], U[3*i+2]))
                    loop = self.object.data.loops[loop_index]
                    v = loop.vertex_index

                    color_value = disp[v] / self.absmax(disp)
                    
                    color = (color_value, color_value, color_value, 1.0)
                    vc.data[loop_index].color = color
                
                vc_base.data[loop_index].color = (0.0, 0.0, 0.0, 1.0)
        return
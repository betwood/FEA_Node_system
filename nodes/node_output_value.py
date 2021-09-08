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

class NodeValueOutput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'OutputValueNode'
    # Label for nice name display
    bl_label = "Solver Value Output"

    drop_down_items = (
        ('DISP', "Displacement", "Add the 2 values together"),
        ('STRAIN', "Strain", "Subtract the second value from the first"),
        ('STRESS', "Stress", "Subtract the second value from the first"),
    )

    output_type: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'DISP',
    )

    vertex: bpy.props.IntProperty(min=0)
    x: bpy.props.FloatProperty()
    y: bpy.props.FloatProperty()
    z: bpy.props.FloatProperty()

    
    def init(self, context):
        self.inputs.new('SocketTypeMatrix', "output")


    def draw_buttons(self, context, layout):
        if(bpy.context.active_node == self):
            layout.operator("node.evaluate", text="evaluate")
        layout.prop(self, "output_type", text="")
        layout.prop(self, "vertex", text="vertex index")
        layout.label(text=str(self.x))
        layout.label(text=str(self.y))
        layout.label(text=str(self.z))
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
        # print(U)

        # create list of dispacements
        # print(len(U) / 3)
        # print(len(U))

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

        # print("disp", disp)
        self.x = U[self.vertex * 6]
        self.y = U[self.vertex * 6 + 1]
        self.z = U[self.vertex * 6 + 2]

        print(U[self.vertex * 6])
        print(U[self.vertex * 6 + 1])
        print(U[self.vertex * 6 + 2])
        print(U)
        
        return
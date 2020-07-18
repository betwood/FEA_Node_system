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

class NodeVectorMath(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'VectorMathNode'
    # Label for nice name display
    bl_label = "Vector Math"


    vector_value: bpy.props.FloatVectorProperty(default=[0, 0, 0])
    vector_1: bpy.props.FloatVectorProperty(default=[0, 0, 0])
    vector_2: bpy.props.FloatVectorProperty(default=[0, 0, 0])
    object: bpy.props.PointerProperty(type=Object)

    # Setup for drop down list
    drop_down_items = (
        ('ADD', "Add", "Add the 2 values together"),
        ('SUB', "Subtract", "Subtract the second value from the first"),
        ('MUL', "Multiply", "Multiply the 2 values together"),
        ('DIV', "Divide", "Divide the second value by the first"),
    )

    my_enum_prop: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'ADD',
    )
    
    
    def init(self, context):
        self.outputs.new('SocketTypeFloat', "vector_value")
        self.inputs.new('SocketTypeFloat', "Vector_1").init("vector_1")
        self.inputs.new('SocketTypeFloat', "Vector_2").init("vector_2")


    def draw_buttons(self, context, layout):
        layout.prop(self, "my_enum_prop", text="")
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
        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]

        input_socket_1.set_value(self.vector_1)
        input_socket_2.set_value(self.vector_2)

        # get inputs from previous nodes
        val1 = self.get_value(input_socket_1)
        val2 = self.get_value(input_socket_2)

        if self.my_enum_prop == "ADD":
            self.float_value = val1 + val2
        if self.my_enum_prop == "SUB":
            self.float_value = val1 - val2
        if self.my_enum_prop == "MUL":
            self.float_value = val1 * val2
        if self.my_enum_prop == "DIV":
            self.float_value = val1 / val2



        return self.vector_value

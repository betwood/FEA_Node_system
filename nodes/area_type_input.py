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

class NodeAreaInput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'AreaInputNode'
    # Label for nice name display
    bl_label = "Area Input"


    r: bpy.props.FloatProperty(default=.1)
    t: bpy.props.FloatProperty(default=.01)
    b: bpy.props.FloatProperty(default=.1)
    h: bpy.props.FloatProperty(default=.1)

    object: bpy.props.PointerProperty(type=Object)

    # Setup for drop down list
    drop_down_items = (
        ('CYL', "Cylinder", "Add the 2 values together"),
        ('HCYL', "Hollow Cylinder", "Subtract the second value from the first"),
        ('BOX', "Box", "Multiply the 2 values together"),
        # ('HBOX', "Hollow Box", "Divide the second value by the first"),
        # ('IBEAM', "I-Beam", "Divide the second value by the first"),
    )

    my_enum_prop: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'CYL',
    )
    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Value")


    def draw_buttons(self, context, layout):
        layout.prop(self, "my_enum_prop", text="")
        if self.my_enum_prop == 'CYL':
            layout.prop(self, "r", text="r")
        if self.my_enum_prop == 'HCYL':
            layout.prop(self, "r", text="r")
            layout.prop(self, "t", text="t")
        if self.my_enum_prop == 'BOX':
            layout.prop(self, "b", text="r")
            layout.prop(self, "h", text="t")

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
        if self.my_enum_prop == 'CYL':
            A = math.pi * self.r
            Iz = (math.pi / 4) * self.r ** 4
            Iy = (math.pi / 4) * self.r ** 4
            J = (math.pi / 2) * self.r ** 4
        if self.my_enum_prop == 'HCYL':
            A = 2 * math.pi * self.r * self.t
            Iz = math.pi * self.r ** 3 * self.t
            Iy = math.pi * self.r ** 3 * self.t
            J = 2 * math.pi * self.r ** 3 * self.t
        if self.my_enum_prop == 'BOX':
            A = self.b * self.h
            Iz = (self.b * self.h ** 3) / 12
            Iy = (self.h * self.b ** 3) / 12
            J = self.h * self.b ** 3 # check this equation
        return [A, Iz, Iy, J]

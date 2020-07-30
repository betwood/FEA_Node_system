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

class NodeMaterialInput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'InputMaterialNode'
    # Label for nice name display
    bl_label = "Material input"


    float_value: bpy.props.FloatProperty(default=5)
    object: bpy.props.PointerProperty(type=Object)

        # Setup for drop down list
    drop_down_items = (
        ('ALUMINUM', "Aluminum", "material properties of aluminum"),
        ('STEEL', "Steel", "material properties of steel"),
    )

    my_enum_prop: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'ALUMINUM',
    )
    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Value")


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
        if self.my_enum_prop == "ALUMINUM":
            mat = [68900000, 26000000]
        if self.my_enum_prop == "STEEL":
            mat = [200000000, 80000000]
        return mat

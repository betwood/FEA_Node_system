import bpy
import numpy as np
import scipy
import bmesh
import math
import mathutils

from timeit import default_timer as timer
from scipy import sparse
from scipy import linalg
from scipy.stats import uniform
from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False
TIME = True

class NodeSolverBase(NodeBase):

    solver_type: bpy.props.StringProperty(default="")
    E: bpy.props.FloatProperty(default=21000000)
    A: bpy.props.FloatProperty(default=.1)
    G: bpy.props.FloatProperty(default=10000)
    Iy: bpy.props.FloatProperty(default=1000)
    Iz: bpy.props.FloatProperty(default=1000)
    J: bpy.props.FloatProperty(default=1000)

    # setup for drop down list
    drop_down_items = (
        ('VALUE', "Value", "Add the 2 values together"),
        ('NODE', "Node", "Subtract the second value from the first"),
    )

    material_input: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'VALUE',
        update=NodeBase.update_value,
    )

        # setup for drop down list
    drop_down_items_2 = (
        ('VALUE', "Value", "Add the 2 values together"),
        ('NODE', "Node", "Subtract the second value from the first"),
    )

    size_input: bpy.props.EnumProperty(
        name = "area input type",
        description = "The type of operation that will be used",
        items = drop_down_items_2,
        default = 'VALUE',
        update=NodeBase.update_value,
    )

    # setup for drop down list
    buffer: bpy.props.BoolProperty(default=False)


    def draw_buttons(self, context, layout):
        # layout.prop(self, "my_enum_prop", text="")
        # self.eval()
        # col = layout.column()
        # col.prop_search(self, "object", context.scene, "objects")
        pass



    def draw_buttons_ext(self, context, layout):
        layout.label(text="Press to set object to all nodes in tree")
        layout.operator("node.object_update", text="update objects")

        layout.label(text="Material input type:")
        layout.operator("node.test", text="TEST")
        layout.prop(self, "material_input", text="")

        layout.label(text="Area input type:")
        layout.prop(self, "size_input", text="")

        layout.label(text="Keeps previous results if checked:")
        layout.prop(self, "buffer", text="Buffer")
        
        super().draw_buttons_ext(context, layout)

    def update_value(self, context):
        print("yay")

    def test(self):
        print("test")
        pass


    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        self.solve_type = self.solver_type
        print(self.object)
        for input in self.inputs:
            if input.is_linked:
                self.set_object(input, self.object, "in", self.solve_type)
                if DEBUG: print(input)

        for output in self.outputs:
            if output.is_linked:
                self.set_object(output, self.object, "out", self.solve_type)
                if DEBUG: print(output)
        # self.get_object()


    def eval(self):
        pass

    def set_color(self):
        if self.buffer == True:
            self.color = (0, 0, 0)
        else:
            self.color = (1, 1, 1)

class UpdateObjectNodeOperator(Operator):
    """Add a simple box mesh"""
    bl_idname = "node.object_update"
    bl_label = "update object node tree"

    # node_group_name = StringProperty()

    node = None

    def execute(self, context):
        # node = [i for i in bpy.data.node_groups[bpy.context.space_data.node_tree.name].nodes if i.bl_idname == "Outputnode"][0]
        node = context.active_node
        node.update_object()
        return{'FINISHED'}

class TestNodeOperator(Operator):
    """Add a simple box mesh"""
    bl_idname = "node.test"
    bl_label = "Test"

    # node_group_name = StringProperty()

    node = None

    def execute(self, context):
        # node = [i for i in bpy.data.node_groups[bpy.context.space_data.node_tree.name].nodes if i.bl_idname == "Outputnode"][0]
        node = context.active_node
        node.test()
        return{'FINISHED'}
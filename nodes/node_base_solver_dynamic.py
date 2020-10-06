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

class NodeDynamicSolverBase(NodeBase):

    bl_idname = 'BaseDynamicSolver'
    # Label for nice name display
    bl_label = "Base Dynamic Solver"

    solver_type: bpy.props.StringProperty(default="")
    E: bpy.props.FloatProperty(default=21000000)
    A: bpy.props.FloatProperty(default=.1)
    G: bpy.props.FloatProperty(default=10000)
    Iy: bpy.props.FloatProperty(default=1000)
    Iz: bpy.props.FloatProperty(default=1000)
    J: bpy.props.FloatProperty(default=1000)
    object: bpy.props.PointerProperty(type=Object)
    solver_type: bpy.props.StringProperty(default="1DFRAME")
    disp: bpy.props.StringProperty(default="")
    rho: bpy.props.FloatProperty(default=.1)
    num_t: bpy.props.IntProperty(default=20)
    dt: bpy.props.FloatProperty(default=.1)
    d0: bpy.props.FloatProperty(default=0)
    v0: bpy.props.FloatProperty(default=0)
    b: bpy.props.FloatProperty(default=1/6, min=0, max=1/2)
    g: bpy.props.FloatProperty(default=1/2)

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

    drop_down_items_3 = (
        ('IMPULSE', "Impulse", "Force only at first time interval"),
        ('CONSTANT', "Constant", "Force over entire time interval"),
    )

    force_input: bpy.props.EnumProperty(
        name = "Force type",
        description = "The force type",
        items = drop_down_items_3,
        default = 'IMPULSE',
        update=NodeBase.update_value,
    )

    # setup for drop down list
    buffer: bpy.props.BoolProperty(default=True)


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
        layout.prop(self, "material_input", text="")

        layout.label(text="Area input type:")
        layout.prop(self, "size_input", text="")

        layout.label(text="Force input type:")
        layout.prop(self, "force_input", text="")

        layout.label(text="Keeps previous results if checked:")
        layout.prop(self, "buffer", text="Buffer")
        
        layout.label(text="Dynamic solve properties:")
        layout.prop(self, "rho", text="density") # needs to be added to material
        layout.prop(self, "num_t", text="number of time steps")
        layout.prop(self, "dt", text="time step")
        layout.prop(self, "b", text="dynamic solve parameter")
        layout.prop(self, "g", text="dynamic solve parameter")

        super().draw_buttons_ext(context, layout)

    def update_value(self, context):
        print("yay")


    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        self.solve_type = self.solver_type

        for input in self.inputs:
            if input.is_linked:
                self.set_object(input, self.object, "in", self.solve_type)
                if DEBUG: print(input)

        for output in self.outputs:
            if output.is_linked:
                self.set_object(output, self.object, "out", self.solve_type)
                if DEBUG: print(output)

    def get_force(self, F, f):
        if self.force_input == "IMPULSE":
            f[:, 0] = F[:, 0]
        if self.force_input == "CONSTANT":
            for i in range(self.num_t):
                f[:, i] = F[:, 0]
        return f


    def eval(self):
        pass
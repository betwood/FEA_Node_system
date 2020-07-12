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

class NodeOutput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'OutputNode'
    # Label for nice name display
    bl_label = "Generic Output"


    output: bpy.props.StringProperty(default="")
    object: bpy.props.PointerProperty(type=Object)
    
    
    def init(self, context):
        self.inputs.new('SocketTypeGeneric', "print output")


    def draw_buttons(self, context, layout):
        layout.label(text=self.output)
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

        input_socket_1 = self.inputs[0]

        input_socket_1.set_value(self.output)
        # input_socket_2.set_value(self.A)

        # get inputs from previous nodes
        self.output = str(self.get_value(input_socket_1))
        return


class UpdateObjectNodeOperator(Operator):
    """Add a simple box mesh"""
    bl_idname = "node.evaluate"
    bl_label = "evaluate"

    # node_group_name = StringProperty()

    node = None

    def execute(self, context):
        # node = [i for i in bpy.data.node_groups[bpy.context.space_data.node_tree.name].nodes if i.bl_idname == "Outputnode"][0]
        node = context.active_node
        node.eval()
        return{'FINISHED'}
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

class BCInput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'BCNode'
    # Label for nice name display
    bl_label = "BC input"


    float_value: bpy.props.FloatProperty(default=5)
    vertex_group: bpy.props.StringProperty(name="Vertex Group")
    x: bpy.props.BoolProperty(default=True)
    y: bpy.props.BoolProperty(default=True)
    z: bpy.props.BoolProperty(default=True)

    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "boundary conditions")


    def draw_buttons(self, context, layout):
        if not self.object == None:
            col = layout.column()
            col.prop_search(self, "vertex_group", self.object, "vertex_groups")
        row = layout.row()
        row.prop(self, "x", text="x", toggle=True)
        row.prop(self, "y", text="y", toggle=True)
        row.prop(self, "z", text="z", toggle=True)
        pass

    # def get_object(self):
    #     print(self.object)
    #     return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):
        
        # look at vertex group for boundary conditions
        vg_idx = 0
        max = len(self.object.data.vertices) * 3
        vs = np.zeros((max,1))
        i = 0
        gi = self.object.vertex_groups[self.vertex_group].index
        for v in self.object.data.vertices:
            for g in v.groups:
                if g.group == gi:
                    # any vertex that is a fixed boundary condition is set to 1
                    if self.x == True:
                        vs[i] = 1
                    if self.y == True:
                        vs[i + 1] = 1
                    if self.z == True:
                        vs[i + 2] = 1             
            i += 3 # i moves up by 3 because 3 dof per vertex
        bool = ((vs == 0))

        return bool

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
    mom_x: bpy.props.BoolProperty(default=True)
    mom_y: bpy.props.BoolProperty(default=True)
    mom_z: bpy.props.BoolProperty(default=True)
    max_mult: bpy.props.IntProperty(default=3)

    
    
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
        row = layout.row()
        if self.solve_type == "1DFRAME":
            row.prop(self, "mom_x", text="x", toggle=True)
            row.prop(self, "mom_y", text="y", toggle=True)
            row.prop(self, "mom_z", text="z", toggle=True)
        pass

    # def get_object(self):
    #     print(self.object)
    #     return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):
        
        # look at vertex group for boundary conditions
        if self.solve_type == "1DFRAME":
            self.max_mult = 6
        elif self.solve_type == "2DTruss":
            self.max_mult = 2
        else:
            self.max_mult = 3
        
        vg_idx = 0
        max = len(self.object.data.vertices) * self.max_mult
        vs = np.zeros((max,1))
        vs = ((vs == 1))
        i = 0
        
        gi = self.object.vertex_groups[self.vertex_group].index
        vertices = [v for v in self.object.data.vertices if gi in [ vg.group for vg in v.groups]]
        
        for v in self.object.data.vertices:
            print(v.groups)
            for g in v.groups:
                if g.group == gi:
                    # any vertex that is a fixed boundary condition is set to 1
                    vs[i] = self.x
                    vs[i + 1] = self.y
                    
                    if not self.solve_type == "2DTruss":
                        vs[i + 2] = self.z
                    
                    if self.solve_type == "1DFRAME":
                        vs[i + 3] = self.mom_x
                        vs[i + 4] = self.mom_y
                        vs[i + 5] = self.mom_z

            i += self.max_mult # i moves up by 3 because 3 dof per vertex
        return vs

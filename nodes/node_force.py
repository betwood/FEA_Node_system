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

class ForceInput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'ForceNode'
    # Label for nice name display
    bl_label = "Force input"


    float_value: bpy.props.FloatProperty(default=5)
    vertex_group: bpy.props.StringProperty(name="Vertex Group")
    vector: bpy.props.FloatVectorProperty(default=[0,0,-12000])
    mom_vector: bpy.props.FloatVectorProperty(default=[0,0,-12000])
    max_mult: bpy.props.IntProperty(default=3)

    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Forces")
        self.inputs.new('SocketTypeVector', "Forces").init("vector")
        self.inputs.new('SocketTypeVector', "Moment").init("mom_vector")
        self.inputs[1].hide = True
        # self.set_inputs()

    def set_inputs(self):
        # for i in self.inputs:
        #     print(i.bl_rna.identifier)
        #     if not i.bl_rna.identifier == "Moment":
        #         if self.solve_type == "1DFRAME":
        #             self.inputs.new('SocketTypeVector', "Moment").init("mom_vector")
        #             return
        #         else:
        #             return
        #     else:
        #         self.inputs.remove(i)
        if self.solve_type == "1DFRAME":
            self.inputs[1].hide = False
        else:
            self.inputs[1].hide = True
        

    def draw_buttons(self, context, layout):
        if not self.object == None:
            col = layout.column()
            col.prop_search(self, "vertex_group", self.object, "vertex_groups")
            
        # self.eval()
        
        pass

    def set_object(self, socket, object, type, solve_type):
        super().set_object(socket, object, type, solve_type)
        self.set_inputs()
        


    def get_object(self):
        print(self.object)
        return self.object

    def update_value(self, context):
        print("updated")

    def eval(self):
        
        # look at vertex group for boundary conditions
        if self.solve_type == "1DFRAME":
            self.max_mult = 6
        else:
            self.max_mult = 3


        vg_idx = 0
        max = len(self.object.data.vertices) * self.max_mult
        Force=np.zeros((self.max_mult,1))
        F = np.zeros((max,1))
        gi = self.object.vertex_groups[self.vertex_group].index
        i = 0
        for v in self.object.data.vertices:
            for g in v.groups:
                if g.group == gi: # currently using vetex group 0 for fixed boundary conditions
                    # any vertex that is a fixed boundary condition is set to 1
                    F[i] = self.vector[0]
                    F[i + 1] = self.vector[1]
                    F[i + 2] = self.vector[2]
                    
                    if self.solve_type == "1DFRAME": # currently only 2 solver types will need to make this more universal later
                        F[i + 3] = self.mom_vector[0]
                        F[i + 4] = self.mom_vector[1]
                        F[i + 5] = self.mom_vector[2]
            i += self.max_mult # i moves up by 3 because 3 dof per vertex

        return F

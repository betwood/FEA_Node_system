import bpy
import numpy as np
import bmesh
import math
import mathutils
from numpy.linalg import inv

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base_solver_dynamic import NodeDynamicSolverBase
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False

class ContactTesting(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'ContactTesting'
    # Label for nice name display
    bl_label = "ContactTesting"

    # Setup for drop down list
    # drop_down_items = (
    #     ('ADD', "Add", "Add the 2 values together"),
    #     ('SUB', "Subtract", "Subtract the second value from the first"),
    #     ('MUL', "Multiply", "Multiply the 2 values together"),
    #     ('DIV', "Divide", "Divide the second value by the first"),
    # )

    # my_enum_prop: bpy.props.EnumProperty(
    #     name = "Operation type",
    #     description = "The type of operation that will be used",
    #     items = drop_down_items,
    #     default = 'ADD',
    # )

    # E: bpy.props.FloatProperty(default=21000000)
    # v: bpy.props.FloatProperty(default=.3)
    # t: bpy.props.FloatProperty(default=.1)
    # rho: bpy.props.FloatProperty(default=.1)
    # num_t: bpy.props.IntProperty(default=20)
    # dt: bpy.props.FloatProperty(default=.1)
    # d0: bpy.props.FloatProperty(default=0)
    # v0: bpy.props.FloatProperty(default=0)
    # b: bpy.props.FloatProperty(default=1/6, min=0, max=1/2)
    # g: bpy.props.FloatProperty(default=1/2)
    object1: bpy.props.PointerProperty(type=Object)
    object2: bpy.props.PointerProperty(type=Object)
    contact: bpy.props.StringProperty(name="Vertex Group")
    target: bpy.props.StringProperty(name="Vertex Group")

    disp: bpy.props.StringProperty(default="")
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Output")
        # self.inputs.new('SocketTypeFloat', "E").init("E")
        # self.inputs.new('SocketTypeFloat', "v").init("v")
        # # self.inputs.new('SocketTypeMatrix', "Material")
        # self.inputs.new('SocketTypeFloat', "t").init("t")
        # self.inputs.new('SocketTypeMatrix', "t_mat")
        self.inputs.new('SocketTypeObject', "Object").init("object1")
        self.inputs.new('SocketTypeObject', "Object").init("object2")

        

    def draw_buttons(self, context, layout):
        super().draw_buttons(context, layout)
        col = layout.column()
        if self.object1:
            col.prop_search(self, "contact", self.object1, "vertex_groups")
        if self.object2:
            col.prop_search(self, "target", self.object2, "vertex_groups")
        

    def update_draw_buttons():
        col = layout.column()
        if self.object1:
            col.prop_search(self, "contact", self.object1, "vertex_groups")
        if self.object2:
            col.prop_search(self, "target", self.object2, "vertex_groups")

    def draw_buttons_ext(self, context, layout):
        super().draw_buttons_ext(context, layout)
        layout.label(text="Press to set object to all nodes in tree")
        layout.operator("node.object_contact_update", text="update contact objects")


    def update_value(self, context):
        print("updated")
        return None

    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        self.solve_type = "2DTruss"

        for input in self.inputs:
            if input.is_linked:
                self.set_object(input, self.object, "in", self.solve_type)
                if DEBUG: print(input)

        for output in self.outputs:
            if output.is_linked:
                self.set_object(output, self.object, "out", self.solve_type)
                if DEBUG: print(output)
        # self.get_object()

    def update_contact_object(self):
        print("Contact Updated")
        self.object1 = self.get_value(self.inputs[0])
        self.object2 = self.get_value(self.inputs[1])
    


    def eval(self):
        bs = .5

                # create bmesh environment
        bm = bmesh.new()
        bm.from_mesh(self.object1.data)

        # create a matrix of values for edges
        #new row 1
            # column 0 = element number
            # column 1 = node 1
            # column 2 = node 2
        # new row 2
            # column 0 = modulus of elasticity
            # column 1 = area
            # column 2 = G
            # column 3 = y moment of inertia
            # column 4 = z moment of inertia
            # column 5 = J
        edge_matrix = np.zeros((4), dtype=int)
        properties = [0,0,0,0,0,0]
        coormat = np.zeros((12))
        new_row_1 = np.zeros((4), dtype=int)
        new_row_2 = properties
        i = 0
        G = 0

        for face in bm.faces:
            j = 0
            coorelement = np.array([])
            # print(face.index)
            for vert in face.verts:
                # vert = loop.vert
                coordinates = np.array([vert.co[0], vert.co[1], vert.co[2]])
                coorelement = np.hstack([coorelement, coordinates])
                new_row_1[j] = vert.index
                j = j + 1
                # print(vert.index, coordinates)

            if DEBUG: print(new_row_1,new_row_2)
            edge_matrix = np.vstack([edge_matrix, new_row_1])
            # print(coorelement)
            # print(coormat)
            coormat = np.vstack([coormat, coorelement])
            i += 1
        edge_matrix = np.delete(edge_matrix, 0, 0) # find better way to initialize (redundant)
        coormat = np.delete(coormat, 0, 0)
        print(coormat)

        xmax = np.max(coordinates[:,0])
        # solve for min and max of object (might need to change when added to solver)

        Sx = (int) (xmax - xmin)/bs + 1
        return xmax

class UpdateContactObjectNodeOperator(Operator):
    """Add a simple box mesh"""
    bl_idname = "node.object_contact_update"
    bl_label = "update contact object"

    # node_group_name = StringProperty()

    node = None

    def execute(self, context):
        # node = [i for i in bpy.data.node_groups[bpy.context.space_data.node_tree.name].nodes if i.bl_idname == "Outputnode"][0]
        node = context.active_node
        node.update_contact_object()
        return{'FINISHED'}
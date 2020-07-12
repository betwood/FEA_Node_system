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

class NodeSpaceTruss(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SpaceTrussNode'
    # Label for nice name display
    bl_label = "Space Truss node"

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

    E: bpy.props.FloatProperty(default=21000000)
    A: bpy.props.FloatProperty(default=.1)
    object: bpy.props.PointerProperty(type=Object)
    
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Output")
        self.inputs.new('SocketTypeFloat', "E").init("E")
        self.inputs.new('SocketTypeFloat', "A").init("A")
        self.inputs.new('SocketTypeObject', "Object").init("object")
        self.inputs.new('SocketTypeMatrix', "Boundary conditions")
        self.inputs.new('SocketTypeMatrix', "Forces")


    def draw_buttons(self, context, layout):
        # layout.prop(self, "my_enum_prop", text="")
        # self.eval()
        # col = layout.column()
        # col.prop_search(self, "object", context.scene, "objects")
        pass

    def draw_buttons_ext(self, context, layout):
        layout.operator("node.object_update", text="update objects")
        layout.label(text=str(self.object))

    def update_value(self, context):
        print("updated")

    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        print(self.object)
        for input in self.inputs:
            if input.is_linked:
                self.set_object(input, self.object, "in")
                if DEBUG: print(input)

        for output in self.outputs:
            if output.is_linked:
                self.set_object(output, self.object, "out")
                if DEBUG: print(output)
        # self.get_object()


    def SpaceTrussElementStiffness(self, L,E,A,thetax,thetay,thetaz):
        x = thetax * math.pi / 180
        u = thetay * math.pi / 180
        v = thetaz * math.pi / 180
        Cx = thetax
        Cy = thetay    
        Cz = thetaz
        if DEBUG: print('Cz',Cz)
        w = np.array([[Cx * Cx, Cx * Cy, Cx * Cz], [Cy * Cx, Cy * Cy, Cy * Cz], [Cz * Cx, Cz * Cy, Cz * Cz]])
        w_new = E * A / L * np.concatenate((np.concatenate((w, -w), axis=1),np.concatenate((-w, w),axis=1)))
        # w = E * A / L * np.array([[w,-w],[-w,w]])
        # w = np.reshape(w, (6, 6))
        if DEBUG: print(w.shape)
        if DEBUG: print('w', w)
        return w_new

    def SpaceTrussAssemble(self, K,k,i,j):
        # puts together stiffness matrix
            #need to look up if there  is a better way of doing this
        K[3*i][3*i] += k[0][0]
        K[3*i][3*i+1] += k[0][1]
        K[3*i][3*i+2] += k[0][2]
        K[3*i][3*j] += k[0][3]
        K[3*i][3*j+1] += k[0][4]
        K[3*i][3*j+2] += k[0][5]
        K[3*i+1][3*i] += k[1][0]
        K[3*i+1][3*i+1] += k[1][1]
        K[3*i+1][3*i+2] += k[1][2]
        K[3*i+1][3*j] += k[1][3]
        K[3*i+1][3*j+1] += k[1][4]
        K[3*i+1][3*j+2] += k[1][5]
        K[3*i+2][3*i] += k[2][0]
        K[3*i+2][3*i+1] += k[2][1]
        K[3*i+2][3*i+2] += k[2][2]
        K[3*i+2][3*j] += k[2][3]
        K[3*i+2][3*j+1] += k[2][4]
        K[3*i+2][3*j+2] += k[2][5]
        K[3*j][3*i] += k[3][0]
        K[3*j][3*i+1] += k[3][1]
        K[3*j][3*i+2] += k[3][2]
        K[3*j][3*j] += k[3][3]
        K[3*j][3*j+1] += k[3][4]
        K[3*j][3*j+2] += k[3][5]
        K[3*j+1][3*i] += k[4][0]
        K[3*j+1][3*i+1] += k[4][1]
        K[3*j+1][3*i+2] += k[4][2]
        K[3*j+1][3*j] += k[4][3]
        K[3*j+1][3*j+1] += k[4][4]
        K[3*j+1][3*j+2] += k[4][5]
        K[3*j+2][3*i] += k[5][0]
        K[3*j+2][3*i+1] += k[5][1]
        K[3*j+2][3*i+2] += k[5][2]
        K[3*j+2][3*j] += k[5][3]
        K[3*j+2][3*j+1] += k[5][4]
        K[3*j+2][3*j+2] += k[5][5]
        return K

    def eval(self):
        # set input sockets
        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]
        input_socket_3 = self.inputs[2]
        input_socket_4 = self.inputs[3]
        input_socket_5 = self.inputs[4]

        input_socket_1.set_value(self.E)
        input_socket_2.set_value(self.A)

        # get inputs from previous nodes
        E = self.get_value(input_socket_1)
        A = self.get_value(input_socket_2)
        self.object = self.get_value(input_socket_3, "object")
        object = self.object
        ob = object.data
        print(ob)

        

        bm = bmesh.new()
        bm.from_mesh(self.object.data)

        # create a matrix of values for edges
        #new row 1
            # column 0 = element number
            # column 1 = node 1
            # column 2 = node 2
        # new row 2
            # column 0 = length
            # column 1 = modulus of elasticity
            # column 2 = area
            # column 3 = x angle
            # column 4 = y angle
            # column 5 = z angle
        edge_matrix = [0,0,0]
        properties = [0,0,0,0,0,0]
        new_row_1 = edge_matrix
        new_row_2 = properties
        i = 0
        for edge in bm.edges:
            new_row_1[0] = edge.index
            new_row_1[1] = edge.verts[0].index
            new_row_1[2] = edge.verts[1].index
            new_row_2[0] = edge.calc_length()
            new_row_2[1] = E
            new_row_2[2] = A
            new_row_2[3] = ((edge.verts[1].co[0]-edge.verts[0].co[0])/new_row_2[0])
            new_row_2[4] = ((edge.verts[1].co[1]-edge.verts[0].co[1])/new_row_2[0])
            new_row_2[5] = ((edge.verts[1].co[2]-edge.verts[0].co[2])/new_row_2[0])
            if DEBUG: print(new_row_1,new_row_2)
            edge_matrix = np.vstack([edge_matrix, new_row_1])
            properties = np.vstack([properties, new_row_2])
            i += 1
        edge_matrix = np.delete(edge_matrix, 0, 0)
        properties = np.delete(properties, 0, 0)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        max = (edge_matrix[:,1:].max() + 1) * 3
        if DEBUG: print(max)

        # create element stiffness matrices


        # k=SpaceTrussElementStiffness(properties[0, 0],properties[0, 1],properties[0, 2],properties[0, 3],properties[0, 4],properties[0, 5])
        # print(k)

        # k=SpaceTrussElementStiffness(properties[1, 0],properties[1, 1],properties[1, 2],properties[1, 3],properties[1, 4],properties[1, 5])
        # print(k)
        # print(properties[1][:])
        if DEBUG: print('break')

        k = np.zeros((len(edge_matrix),6,6))
        for i in range(len(edge_matrix)):
            k[i, :, :]=self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3],properties[i, 4],properties[i, 5])
            # print(k)
        if DEBUG: print(k.shape)

        # create global stiffness matrix
        K=np.zeros((max,max))
        for i in range(len(edge_matrix)):
            K=self.SpaceTrussAssemble(K,k[i, :, :],edge_matrix[i,1],edge_matrix[i,2])
        # K=SpringAssemble(K,k2,1,2)
        if DEBUG: print(K.shape)
        if DEBUG: print(K)

        # create force vector
        Force=np.zeros((3,1))
        F = np.zeros((max,1))
        Force=np.array([[0],[0],[-1000000]])
        if DEBUG: print(F)


        if DEBUG: print('testing')
        # look at vertex group for boundary conditions
        vg_idx = 0
        vs = np.zeros((max,1))
        if DEBUG: print(vs.shape)
        i = 0



        for v in ob.vertices:
            for g in v.groups:
                if g.group == 0: # currently using vetex group 0 for fixed boundary conditions
                    # any vertex that is a fixed boundary condition is set to 1
                    vs[i] = 1
                    vs[i + 1] = 1
                    vs[i + 2] = 1
                if g.group == 1:
                    # any vetex with a force on it is set to 2
                    F[i] = Force[0]
                    F[i + 1] = Force[1]
                    F[i + 2] = Force[2]
                        
            i += 3 # i moves up by 3 because 3 dof per vertex
        bool = ((vs == 0))
        bool = np.ravel(bool)
        print("bool before", bool)
        bool = ((self.get_value(input_socket_4)))
        
        F = self.get_value(input_socket_5)
        if DEBUG: print(vs)
        print("Force:", F)
        # bool = np.reshape(bool, (12,))
        # bool = np.ix_(bool)
        # bool = np.array([False,False,False,False,False,False,False,False,False,True,True,True])
        # if DEBUG: print(bool.shape)
        bool = np.ravel(bool)
        print("bool after", bool)
        if DEBUG: print(bool.shape)
        boolv,boolh = np.ix_(bool, bool)
        if DEBUG: print(boolv)

        if DEBUG: print(K.shape)
        if DEBUG: print(vs.shape)

        vstest = vs[bool]
        if DEBUG: print(vstest)

        # apply boundary conditions
        Ksolve = K[boolv,boolh]
        F = F[boolv]
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        if DEBUG: print('applying boundary conditions')
        if DEBUG: print(Ksolve)
        if DEBUG: print(F)

        # solve for displacement
        u=np.linalg.solve(Ksolve,F)
        if DEBUG: print(u)
        bound=np.array([0])
        U=np.zeros((max,1))
        j = 0
        for i in range(len(U)):
            if bool[i] == 1:
                U[i] = u[j]
                j = j + 1
        # U[9]=u[0]
        # U[10]=u[1]
        # U[11]=u[2]
        if DEBUG: print(U)
        return U
        # # create shape keys

        # sk_basis = bm.verts.layers.shape.get('Basis')
        # # sk_basis.use_relative = False
        # sk_new = bm.verts.layers.shape.new('MyShape')
        # # sk_new.use_relative = False

        # #sk_basis = obj.shape_key_add('Basis')
        # #sk_basis.interpolation = 'KEY_LINEAR'
        # #obj.data.shape_keys.use_relative = False

        # # Create new shape key
        # #sk = obj.shape_key_add('Deform')
        # #sk.interpolation = 'KEY_LINEAR'

        # #apply to mesh
        # i=0
        # for v in bm.verts:
        #     v[sk_new] = v.co + mathutils.Vector((U[3*i], U[3*i+1], U[3*i+2]))
        #     # v.co.y += U[3*i+1]
        #     # v.co.z += U[3*i+2]
        #     i += 1
                
        # # caclulate force
        # F = K.dot(U)
        # if DEBUG: print(F)

        # sigma = np.zeros([len(edge_matrix)])
        # # calculate stress
        # for i in range(len(edge_matrix)):
        #     store = properties[i,1] / properties[i,0] * np.array([-properties[i,3], -properties[i,4], -properties[i,5], properties[i,3], properties[i,4], properties[i,5]])   
        #     uvect = np.array([U[3 * edge_matrix[i,1]], U[3 * edge_matrix[i,1] + 1], U[3 * edge_matrix[i,1] + 2], U[3 * edge_matrix[i,2]], U[3 * edge_matrix[i,2] + 1], U[3 * edge_matrix[i,2] + 2]])
        #     uvect = uvect.reshape(-1,1)
        #     sigma[i] = store.dot(uvect)
        #     if DEBUG: print(uvect.shape)

        # if DEBUG: print(sigma)

        # # output colors
        #     # currently using displacement as output because this can be done on verticies
        #     # stress output need to find a way to output color to edges
        # # vcol_output = bm.vertex_colors.new()

        # # for v in bm.vertex:
            

        # # apply changes
        # bm.to_mesh(ob)
        # bm.free() # free and prevent further acess



        
        # return object

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
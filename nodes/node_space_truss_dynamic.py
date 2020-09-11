import bpy
import numpy as np
import bmesh
import math
import mathutils
from numpy.linalg import inv

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False

class NodePlaneTrussDynamic(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'PlaneTrussNodeDynamic'
    # Label for nice name display
    bl_label = "Plane Truss node Dynamic"

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
    rho: bpy.props.FloatProperty(default=.1)
    num_t: bpy.props.IntProperty(default=20)
    dt: bpy.props.FloatProperty(default=.1)
    d0: bpy.props.FloatProperty(default=0)
    v0: bpy.props.FloatProperty(default=0)
    b: bpy.props.FloatProperty(default=1/6, min=0, max=1/2)
    g: bpy.props.FloatProperty(default=1/2)
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
        super().draw_buttons_ext(context, layout)
        layout.operator("node.object_update", text="update objects")
        layout.prop(self, "rho", text="density") # needs to be added to material
        layout.prop(self, "num_t", text="number of time steps")
        layout.prop(self, "dt", text="time step")
        layout.prop(self, "b", text="dynamic solve parameter")
        layout.prop(self, "g", text="dynamic solve parameter")

    def update_value(self, context):
        print("updated")
        return None

    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        self.solve_type = "2DTruss"
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


    def SpaceTrussElementStiffness(self, L,E,A,theta):
        C = np.cos(theta)
        S = np.sin(theta)
        k = ((E * A) / L) * np.array([
            [C ** 2, C * S, -C ** 2, - C * S],
            [C * S, S ** 2, -C * S, -S ** 2],
            [-C ** 2, -C * S, C ** 2, C * S],
            [-C * S, -S ** 2, C * S, S ** 2],
        ])
        mhat = ((self.rho * A * L)/6) * np.array([
            [2, 0, 1, 0],
            [0, 2, 0, 1],
            [1, 0, 2, 0],
            [0, 1, 0, 2]
        ])
        T = np.array([
            [C, S, 0, 0],
            [-S, C, 0, 0],
            [0, 0, C, S],
            [0, 0, -S, C]
        ])
        m = np.dot(np.dot(np.transpose(T), mhat), T) 
        return m, k

    def SpaceTrussAssemble(self, K,k,i,j, M, m):
        for val1 in range(4):
            for val2 in range(4):
                if val1 <= 1:
                    index1 = 2 * i + val1
                else:
                    index1 = 2 * j + (val1 - 2)

                if val2 <= 1:
                    index2 = 2 * i + val2
                else:
                    index2 = 2 * j + (val2 - 2)

                K[index1][index2] += k[val1][val2]
                M[index1][index2] += m[val1][val2]
        return M, K

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
            # column 3 = theta
        edge_matrix = [0,0,0]
        properties = [0,0,0,0]
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
            # if edge.verts[1].co[0]-edge.verts[0].co[0] == 0:
            #     new_row_2[3] = 0
            # else:
            new_row_2[3] = np.arctan2((edge.verts[1].co[1]-edge.verts[0].co[1]), (edge.verts[1].co[0]-edge.verts[0].co[0]))

            if DEBUG: print(new_row_1,new_row_2)
            edge_matrix = np.vstack([edge_matrix, new_row_1])
            properties = np.vstack([properties, new_row_2])
            i += 1
        edge_matrix = np.delete(edge_matrix, 0, 0)
        properties = np.delete(properties, 0, 0)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        max = (edge_matrix[:,1:].max() + 1) * 2
        if DEBUG: print(max)

        # create stiffness matrix
        k = np.zeros((4,4))
        m = np.zeros((4,4))
        K = np.zeros((max, max))
        M = np.zeros((max, max))
        for i in range(len(edge_matrix)):
            [m, k] = self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3])
            
            [M, K] = self.SpaceTrussAssemble(K, k, edge_matrix[i,1], edge_matrix[i,2], M, m)
        
        if DEBUG: print(k.shape)

        # create global stiffness matrix
        # K=np.zeros((max,max))
        # for i in range(len(edge_matrix)):
        #     K=self.SpaceTrussAssemble(K,k[i, :, :],edge_matrix[i,1],edge_matrix[i,2])
        # if DEBUG: print(K.shape)
        # if DEBUG: print(K)

        bool = ((self.get_value(input_socket_4)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_5)
        print("Force:", F)
        bool = np.ravel(bool)
        print("bool after", bool)
        if DEBUG: print(bool.shape)
        boolv,boolh = np.ix_(bool, bool)
        if DEBUG: print(boolv)

        if DEBUG: print(K.shape)

        # apply boundary conditions
        K = K[boolv,boolh]
        F = F[boolv]
        M = M[boolv, boolh]
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        if DEBUG: print('applying boundary conditions')
        if DEBUG: print(Ksolve)
        if DEBUG: print(F)

        # algorithm for dynamic solution based on newmarks equations
        
        # initialize displacement, velocity, and acceleration
        d = np.zeros((len(F), self.num_t))
        v = np.zeros((len(F), self.num_t))
        a = np.zeros((len(F), self.num_t))
        f = np.zeros((len(F), self.num_t))

        # initial conditions
        d0 = np.zeros((len(F), 1))
        d[:, 0] = d0[:, 0]
        v0 = np.zeros((len(F), 1))
        v[:, 0] = v0[:, 0]
        f[:, 0] = F[:, 0]
        F = f

        t = 0
        a[:, 0] = np.dot(inv(M), F[:, 0] - np.dot(K, d[:, 0]))
        for i in range(self.num_t - 1):
            Kp = K + (1/(self.b + self.dt ** 2)) * M
            p1 = (1/(self.b + self.dt ** 2)) * M
            p2 = d[:, i] + self.dt * v[:, i] + (.5 - self.b) * self.dt ** 2 * a[:, i]
            test = np.dot(p1, p2.reshape(-1,1))
            print(F[:, i + 1])
            Fp = F[:, i + 1].reshape(-1,1) + test
            
            print(np.linalg.solve(Kp, Fp))
            print("inv:", np.dot(inv(Kp), Fp))
            d[:, i + 1] = np.linalg.solve(Kp, Fp).ravel()

            a[:, i + 1] = (1/(self.b + self.dt ** 2)) * (d[:, i + 1] - d[:, i] - self.dt * v[:, i] - (.5 - self.b) * self.dt ** 2 * a[:, i])
            
            v[:, i + 1] = v[:, i] + self.dt * ((1 - self.g) * a[:, i] + self.g * d[:, i + 1])
        
        # solve for displacement
        # u=np.linalg.solve(Ksolve,F)
        if DEBUG: print(u)
        bound=np.array([0])
        U=np.zeros((max, self.num_t))
        j = 0
        for i in range(len(U)):
            if bool[i] == 1:
                U[i, :] = d[j, :]
                j = j + 1

        if DEBUG: print(U)
        return U
                
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
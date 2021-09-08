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

class NodeSpaceTrussDynamic(Node, NodeDynamicSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SpaceTrussNodeDynamic'
    # Label for nice name display
    bl_label = "Space Truss node Dynamic"


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
        super().draw_buttons(context, layout)

    def draw_buttons_ext(self, context, layout):
        super().draw_buttons_ext(context, layout)

    def update_value(self, context):
        print("updated")
        return None

    def update_object(self):
        input_object = self.inputs["Object"]
        self.object = self.get_value(input_object)
        self.solve_type = "1DTruss"
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


    def SpaceTrussElementStiffness(self, L,E,A,thetax,thetay,thetaz):
        x = thetax * math.pi / 180
        u = thetay * math.pi / 180
        v = thetaz * math.pi / 180

        Cx = thetax
        Cy = thetay    
        Cz = thetaz

        w = np.array([[Cx * Cx, Cx * Cy, Cx * Cz], [Cy * Cx, Cy * Cy, Cy * Cz], [Cz * Cx, Cz * Cy, Cz * Cz]])
        w_new = E * A / L * np.concatenate((np.concatenate((w, -w), axis=1),np.concatenate((-w, w),axis=1)))
        
        m = A * self.rho * L / 6 * np.array([
            [2 * Cx * Cx, 2 * Cx * Cy, 2 * Cx * Cz, Cx * Cx, Cx * Cy, Cx * Cz],
            [2 * Cx * Cy, 2 * Cy * Cy, 2 * Cy * Cz, Cx * Cy, Cy * Cy, Cy * Cz],
            [2 * Cx * Cz, 2 * Cy * Cz, 2 * Cz * Cz, Cx * Cz, Cy * Cz, Cz * Cz],
            [Cx * Cx, Cx * Cy, Cx * Cz, 2 * Cx * Cx, 2 * Cx * Cy, 2 * Cx * Cz],
            [Cx * Cy, Cy * Cy, Cy * Cz, 2 * Cx * Cy, 2 * Cy * Cy, 2 * Cy * Cz],
            [Cx * Cz, Cy * Cz, Cz * Cz, 2 * Cx * Cz, 2 * Cy * Cz, 2 * Cz * Cz]
        ])

        return m, w_new

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

        k = np.zeros((4,4))
        m = np.zeros((4,4))
        K = np.zeros((max, max))
        M = np.zeros((max, max))
        for i in range(len(edge_matrix)):
            [m, k] = self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3],properties[i, 4],properties[i, 5])
            
            K = self.SpaceTrussAssemble(K, k, edge_matrix[i,1], edge_matrix[i,2])
            M = self.SpaceTrussAssemble(M, m, edge_matrix[i,1], edge_matrix[i,2])
        

        # # create stiffness matrix
        # k = np.zeros((len(edge_matrix),6,6))
        # for i in range(len(edge_matrix)):
        #     k[i, :, :]=self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3],properties[i, 4],properties[i, 5])
        # if DEBUG: print(k.shape)

        # # create global stiffness matrix
        # K=np.zeros((max,max))
        # for i in range(len(edge_matrix)):
        #     K=self.SpaceTrussAssemble(K,k[i, :, :],edge_matrix[i,1],edge_matrix[i,2])
        # if DEBUG: print(K.shape)
        # if DEBUG: print(K)

        bool = ((self.get_value(input_socket_4)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_5)
        bool = np.ravel(bool)
        if DEBUG: print(bool.shape)
        boolv,boolh = np.ix_(bool, bool)
        if DEBUG: print(boolv)

        if DEBUG: print(K.shape)

        # apply boundary conditions
        K = K[boolv,boolh]
        M = M[boolv,boolh]
        F = F[boolv]
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        if DEBUG: print('applying boundary conditions')
        if DEBUG: print(Ksolve)
        if DEBUG: print(F)

        # solve for displacement
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
        
        
        F = self.get_force(F, f)

        t = 0
        a[:, 0] = np.dot(inv(M), F[:, 0] - np.dot(K, d[:, 0]))
        for i in range(self.num_t - 1):
            Kp = K + (1/(self.b + self.dt ** 2)) * M
            p1 = (1/(self.b + self.dt ** 2)) * M
            p2 = d[:, i] + self.dt * v[:, i] + (.5 - self.b) * self.dt ** 2 * a[:, i]
            test = np.dot(p1, p2.reshape(-1,1))

            Fp = F[:, i + 1].reshape(-1,1) + test
            
            d[:, i + 1] = np.linalg.solve(Kp, Fp).ravel()

            a[:, i + 1] = (1/(self.b + self.dt ** 2)) * (d[:, i + 1] - d[:, i] - self.dt * v[:, i] - (.5 - self.b) * self.dt ** 2 * a[:, i])
            
            v[:, i + 1] = v[:, i] + self.dt * ((1 - self.g) * a[:, i] + self.g * d[:, i + 1])


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
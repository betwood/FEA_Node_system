import bpy
import numpy as np
import scipy
import bmesh
import math
import mathutils
import json

from timeit import default_timer as timer
from scipy import sparse
from scipy import linalg
from scipy.stats import uniform
from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base_solver import NodeSolverBase
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False
TIME = True

class NodeSpaceFrame(Node, NodeSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'SpaceFrameNode'
    # Label for nice name display
    bl_label = "Space Frame node"

    E: bpy.props.FloatProperty(default=21000000)
    A: bpy.props.FloatProperty(default=.1)
    G: bpy.props.FloatProperty(default=10000)
    Iy: bpy.props.FloatProperty(default=1000)
    Iz: bpy.props.FloatProperty(default=1000)
    J: bpy.props.FloatProperty(default=1000)
    object: bpy.props.PointerProperty(type=Object)
    solver_type: bpy.props.StringProperty(default="1DFRAME")
    disp: bpy.props.StringProperty(default="")

    

    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Output")
        self.inputs.new('SocketTypeFloat', "E").init("E")
        self.inputs.new('SocketTypeFloat', "A").init("A")
        self.inputs.new('SocketTypeFloat', "G").init("G")
        self.inputs.new('SocketTypeFloat', "Iy").init("Iy")
        self.inputs.new('SocketTypeFloat', "Iz").init("Iz")
        self.inputs.new('SocketTypeFloat', "J").init("J")
        self.inputs.new('SocketTypeMatrix', "Material")
        self.inputs.new('SocketTypeObject', "Object").init("object")
        self.inputs.new('SocketTypeMatrix', "Boundary conditions")
        self.inputs.new('SocketTypeMatrix', "Forces")
        self.set_inputs()


    def draw_buttons(self, context, layout):
        pass

    def set_inputs(self):
        if self.material_input == "VALUE":
            self.inputs[0].hide = False
            self.inputs[2].hide = False
            self.inputs[3].hide = False
            self.inputs[4].hide = False
            self.inputs[5].hide = False
            self.inputs[6].hide = True
        else:
            self.inputs[0].hide = True
            self.inputs[2].hide = True
            self.inputs[3].hide = True
            self.inputs[4].hide = True
            self.inputs[5].hide = True
            self.inputs[6].hide = False
    
    def update_value(self, context):
        print("updated")
        return None

    def SpaceTrussElementStiffness(self, E, A, G, Iy, Iz, J, vertex1, vertex2):
        x1 = vertex1.x
        y1 = vertex1.y
        z1 = vertex1.z
        x2 = vertex2.x
        y2 = vertex2.y
        z2 = vertex2.z
        L = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        w1 = E * A / L
        w2 = 12 * E * Iz / (L ** 3)
        w3 = 6 * E * Iz / (L ** 2)
        w4 = 4 * E * Iz / (L)
        w5 = 2 * E * Iz / L
        w6 = 12 * E * Iy / (L ** 3)
        w7 = 6 * E * Iy / (L ** 2)
        w8 = 4 * E * Iy / (L)
        w9 = 2 * E * Iy
        w10 = G * J / L
        kprime = np.array([
            [w1, 0, 0, 0, 0, 0, -w1, 0, 0, 0, 0, 0],
            [0, w2, 0, 0, 0, w3, 0, -w2, 0, 0, 0, w3],
            [0, 0, w6, 0, -w7, 0, 0, 0, -w6, 0, -w7, 0],
            [0, 0, 0, w10, 0, 0, 0, 0, 0, -w10, 0, 0],
            [0, 0, -w7, 0, w8, 0, 0, 0, w7, 0, w9, 0],
            [0, w3, 0, 0, 0, w4, 0, -w3, 0, 0, 0, w5],
            [-w1, 0, 0, 0, 0, 0, w1, 0, 0, 0, 0, 0],
            [0, -w2, 0, 0, 0, -w3, 0, w2, 0, 0, 0, -w3],
            [0, 0, -w6, 0, w7, 0, 0, 0, w6, 0, w7, 0],
            [0, 0, 0, -w10, 0, 0, 0, 0, 0, w10, 0, 0],
            [0, 0, -w7, 0, w9, 0, 0, 0, w7, 0, w8, 0],
            [0, w3, 0, 0, 0, w5, 0, -w3, 0, 0, 0, w4]
        ])
        if DEBUG: print("kprime", kprime)
        if x1 == x2 and y1 == y2:
            if z2 > z1:
                lam = np.array([
                    [0, 0, 1],
                    [0, 1, 0],
                    [-1, 0, 0]
                ])
            else:
                lam = np.array([
                    [0, 0, -1],
                    [0, 1, 0],
                    [1, 0, 0]
                ])
        else:
            CXx = (x2 - x1)/L
            CYx = (y2 - y1)/L
            CZx = (z2 - z1)/L
            D = math.sqrt(CXx**2 + CYx**2)
            CXy = -CYx / D
            CYy = CXx / D
            CZy = 0
            CXz = -CXx * CZx / D
            CYz = -CYx * CZx / D
            CZz = D
            lam = np.array([
                [CXx, CYx, CZx],
                [CXy, CYy, CZy],
                [CXz, CYz, CZz]
            ])

        zero3 = np.zeros((3, 3))

        row1 = np.concatenate((lam, zero3, zero3, zero3))
        row2 = np.concatenate((zero3, lam, zero3, zero3))
        row3 = np.concatenate((zero3, zero3, lam, zero3))
        row4 = np.concatenate((zero3, zero3, zero3, lam))

        R = np.concatenate((row1, row2, row3, row4), axis=1)

        Rp = np.transpose(R)
        if DEBUG: print("R:", R, "shape:", R.shape)
        out = np.dot(Rp, kprime)
        out = np.dot(out, R)
        if DEBUG: print(out)
        return out

    def SpaceTrussAssemble(self, K,k,i,j):
        # puts together stiffness matrix
            #need to look up if there  is a better way of doing this
        
        for val1 in range(12):
            for val2 in range(12):
                if val1 <= 5:
                    index1 = 6 * i + val1
                else:
                    index1 = 6 * j + (val1 - 6)

                if val2 <= 5:
                    index2 = 6 * i + val2
                else:
                    index2 = 6 * j + (val2 - 6)

                K[index1][index2] += k[val1][val2]
        if DEBUG: print(K)
        
        return K

    def eval(self):
        if TIME: start = timer()
        
        # get held values if they exist
        if self.buffer == True and not self.disp == "":
            load = json.loads(self.disp)
            U = np.array(load)
            return U

        # set input sockets
        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]
        input_socket_3 = self.inputs[2]
        input_socket_4 = self.inputs[3]
        input_socket_5 = self.inputs[4]
        input_socket_6 = self.inputs[5]
        input_socket_7 = self.inputs[6]
        input_socket_8 = self.inputs[7]
        input_socket_9 = self.inputs[8]
        input_socket_10 = self.inputs[9]

        input_socket_1.set_value(self.E)
        input_socket_2.set_value(self.A)
        input_socket_3.set_value(self.G)
        input_socket_4.set_value(self.Iy)
        input_socket_5.set_value(self.Iz)
        input_socket_6.set_value(self.J)

        # get inputs from previous nodes
        print("!!!")
        print(self.material_input)
        if self.material_input == "VALUE":
            E = self.get_value(input_socket_1)
            G = self.get_value(input_socket_3)
            Iy = self.get_value(input_socket_4)
            Iz = self.get_value(input_socket_5)
            J = self.get_value(input_socket_6)
        elif self.material_input == "NODE":
            print("NODE")
            mat_vect = self.get_value(input_socket_7)
            print(mat_vect)
            E = mat_vect[0]
            G = mat_vect[1]
            Iy = mat_vect[2]
            Iz = mat_vect[3]
            J = mat_vect[4]

        A = self.get_value(input_socket_2)        
        self.object = self.get_value(input_socket_8)
        object = self.object
        ob = object.data        

        # create bmesh environment
        bm = bmesh.new()
        bm.from_mesh(self.object.data)

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
        edge_matrix = [0,0,0]
        properties = [0,0,0,0,0,0]
        new_row_1 = edge_matrix
        new_row_2 = properties
        i = 0
        for edge in bm.edges:
            new_row_1[0] = edge.index
            new_row_1[1] = edge.verts[0].index
            new_row_1[2] = edge.verts[1].index
            new_row_2[0] = E
            new_row_2[1] = A
            new_row_2[2] = G
            new_row_2[3] = Iy
            new_row_2[4] = Iz
            new_row_2[5] = J
            if DEBUG: print(new_row_1,new_row_2)
            edge_matrix = np.vstack([edge_matrix, new_row_1])
            properties = np.vstack([properties, new_row_2])
            i += 1
        edge_matrix = np.delete(edge_matrix, 0, 0)
        properties = np.delete(properties, 0, 0)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        max = (edge_matrix[:,1:].max() + 1) * 6
        if DEBUG: print(max)

        # create stiffness matrix
        k = np.zeros((len(edge_matrix),12,12))
        bm.edges.ensure_lookup_table()
        for i in range(len(edge_matrix)):
            k[i, :, :]=self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3],properties[i, 4],properties[i, 5], bm.edges[i].verts[0].co, bm.edges[i].verts[1].co)
        if DEBUG: print(k.shape)

        # create global stiffness matrix
        if TIME: assem_start = timer()
        K=np.zeros((max,max))
        for i in range(len(edge_matrix)):        
            K=self.SpaceTrussAssemble(K,k[i, :, :],edge_matrix[i,1],edge_matrix[i,2])
        if TIME: assem_end = timer()
        print("space truss assemble", assem_end - assem_start)
        # print("shape:", K.shape)
        # print("K", K)

        bool = ((self.get_value(input_socket_9)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_10)
        # print("Force:", F)
        bool = np.ravel(bool)
        # print("bool after", bool)
        # print(bool.shape)
        boolv,boolh = np.ix_(bool, bool)
        # print(boolv)

        # print(K.shape)

        # apply boundary conditions
        Ksolve = K[boolv,boolh]
        F = F[boolv]
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        print('applying boundary conditions')
        # print(Ksolve)
        # print(F)

        # solve for displacement
        Ksolve_csr = sparse.csr_matrix(Ksolve)
        F_csr = sparse.csr_matrix(F)
        print('solving')
        u = scipy.sparse.linalg.spsolve(Ksolve_csr, F_csr)
        print('solving done')
        if DEBUG: print(u)
        bound=np.array([0])
        U=np.zeros((max,1))
        j = 0
        for i in range(len(U)):
            if bool[i] == 1:
                U[i] = u[j]
                j = j + 1

        if DEBUG: print(U)
        if TIME: end = timer()
        print("time:", end - start)
        
        
        array = U.tolist()
        self.disp = json.dumps(array)

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
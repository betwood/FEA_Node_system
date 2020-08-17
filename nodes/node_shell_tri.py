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

class NodeShellTri(Node, NodeSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'TriShellNode'
    # Label for nice name display
    bl_label = "Shell Tri node"

    E: bpy.props.FloatProperty(default=21000000)
    v: bpy.props.FloatProperty(default=.3)
    t: bpy.props.FloatProperty(default=.1)
    object: bpy.props.PointerProperty(type=Object)
    solver_type: bpy.props.StringProperty(default="1DFRAME")
    disp: bpy.props.StringProperty(default="")

    

    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Output")
        self.inputs.new('SocketTypeFloat', "E").init("E")
        self.inputs.new('SocketTypeFloat', "v").init("v")
        self.inputs.new('SocketTypeMatrix', "Material")
        self.inputs.new('SocketTypeFloat', "t").init("t")
        self.inputs.new('SocketTypeMatrix', "t_mat")
        self.inputs.new('SocketTypeObject', "Object").init("object")
        self.inputs.new('SocketTypeMatrix', "Boundary conditions")
        self.inputs.new('SocketTypeMatrix', "Forces")
        self.set_inputs()


    def draw_buttons(self, context, layout):
        pass

    def set_inputs(self):
        if self.material_input == "VALUE":
            self.inputs[0].hide = False
            self.inputs[1].hide = False
            self.inputs[2].hide = True
        else:
            self.inputs[0].hide = True
            self.inputs[1].hide = True
            self.inputs[2].hide = False
        if self.size_input == "VALUE":
            self.inputs[3].hide = False
            self.inputs[4].hide = True
        else:
            self.inputs[3].hide = True
            self.inputs[4].hide = False
    
    def update_value(self, context):
        print("updated")
        return None

    def CSTElement(self, properties, coorloc, e):
        # print(properties)
        # print(properties.shape)
        t = properties[0, 3]
        Eprop = properties[0, 0]
        v = properties[0, 2]
        # print(v ** 2)
        E = (Eprop / (1 - v ** 2)) * np.array([[1, v, 0], [v, 1, 0], [0, 0, (1 - v) / 2]])
        # print("E", E)
        #not right need local space values here also based on element not edge
        # print(coorloc)
        # bm = bmesh.new()
        # bm.from_mesh(self.object.data)
        # bm.edges.ensure_lookup_table()
        x1 = coorloc[0, 0]
        x2 = coorloc[1, 0]
        x3 = coorloc[2, 0]
        y1 = coorloc[0, 1]
        y2 = coorloc[1, 1]
        y3 = coorloc[2, 1]

        A = (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2

        B = np.array(
            [[y2 - y3, 0, y3 - y1, 0, y1 - y2, 0],
            [0, x3 - x2, 0, x1 - x3, 0, x2 - x1],
            [x3 - x2, y2 - y3, x1 - x3, y3 - y1, x2 - x1, y1 - y2]]
        )
        
        k = t * A * np.dot(np.dot(np.transpose(B), E), B)
        return k

    def DKTElement(self, properties, coorloc):
        x23 = coorloc[1, 0] - coorloc[2, 0]
        x31 = coorloc[2, 0] - coorloc[0, 0]
        x12 = coorloc[0, 0] - coorloc[1, 0]
        y23 = coorloc[1, 1] - coorloc[2, 1]
        y31 = coorloc[2, 1] - coorloc[0, 1]
        y12 = coorloc[0, 1] - coorloc[1, 1]
        # print("0, 1", coorloc[0, 1])
        E = properties[0, 0]
        v = properties[0, 2]
        t = properties[0, 3]
        Db = E * t ** 3 / (12 * 1 - v ** 2) * np.array([[1, v, 0], [v, 1, 0], [0, 0, (1 - v) / 2]])
        A = (1 / 2) * (x31 * y12 - x12 * y31)
        z = np.array([1/2, 1/2, 0])
        n = np.array([0, 1/2, 1/2])
        w = np.array([1/3, 1/3, 1/3])
        k = np.zeros((9, 9))
        for j in range(3):
            for i in range(3):
                B = self.B(properties, coorloc, z[i], n[j])
                # print("B", B)
                B = 1 / (2 * A) * B
                k = w[j] * w[i] * np.dot(np.dot(np.transpose(B), Db), B)
                # print("before", k)
        
        return 2 * A * k

    def B(self, properties, coorloc, z, n):
        x23 = coorloc[1, 0] - coorloc[2, 0]
        x31 = coorloc[2, 0] - coorloc[0, 0]
        x12 = coorloc[0, 0] - coorloc[1, 0]
        y23 = coorloc[1, 1] - coorloc[2, 1]
        y31 = coorloc[2, 1] - coorloc[0, 1]
        y12 = coorloc[0, 1] - coorloc[1, 1]
        l23 = (x23 ** 2 + y23 ** 2) ** (1/2)
        l31 = (x31 ** 2 + y31 ** 2) ** (1/2)
        l12 = (x12 ** 2 + y12 ** 2) ** (1/2)
        # print("locations", coorloc)
        # print("distance", l23, l31, l12)
        P4 = - 6 * x23 / (l23 ** 2)
        P5 = - 6 * x31 / (l31 ** 2) 
        P6 = - 6 * x12 / (l12 ** 2)
        q4 = 3 * x23 * y23 / (l23 ** 2)
        q5 = 3 * x31 * y31 / (l31 ** 2) 
        q6 = 3 * x12 * y12 / (l12 ** 2)
        r4 = 3 * y23 ** 2 / (l23 ** 2)
        r5 = 3 * y31 ** 2 / (l31 ** 2) 
        r6 = 3 * y12 ** 2 / (l12 ** 2)
        t4 = -6 * y23 / (l23 ** 2)
        t5 = -6 * y31 / (l31 ** 2) 
        t6 = -6 * y12 / (l12 ** 2)       

        Hxz = np.array([
            [P6 * (1 - 2 * z) + (P5 - P6) * n],
            [q6 * (1 - 2 * z) - (q5 + q6) * n],
            [-4 + 6 * (z + n) + r6 * (1 - 2 * z) - n * (r5 + r6)],
            [-P6 * (1 - 2 * z) + n * (P4 + P6)],
            [q6 * (1 - 2 * z) - n * (q6 - q4)],
            [-2 + 6 * z + r6 * (1 - 2 * z) + n * (r4 - r6)],
            [-n * (P5 + P4)],
            [n * (q4 - q5)],
            [-n * (r5 - r4)]
        ])
        

        Hyz = np.array([
            [t6 * (1 - 2 * z) + (t5 - t6) * n],
            [1+ r6 * (1 - 2 * z) - (r5 + r6) * n],
            [-q6 * (1 - 2 * z) - n * (q5 + q6)],
            [-t6 * (1 - 2 * z) + n * (t4 + t6)],
            [-1 + r6 * (1 - 2 * z) - n * (r4 - r6)],
            [q6 * (1 - 2 * z)+ n * (q4 - q6)],
            [-n * (t4 + t5)],
            [n * (r4 - r5)],
            [-n * (q4 - q5)]
        ])

        Hxn = np.array([
            [P5 * (1 - 2 * n) + (P6 - P5) * z],
            [q5 * (1 - 2 * n) - (q5 + q6) * z],
            [-4 + 6 * (z + n) + r5 * (1 - 2 * n) - z * (r5 + r6)],
            [z * (P4 + P6)],
            [z * (q4 - q6)],
            [-z * (r6 - r4)],
            [P5 * ( 1 - 2 * n) - z * (P4 + P5)],
            [q5 * (1 - 2 * n) + z * (q4 - q5)],
            [-2 + 6 * n + r5 * (1- 2 * n) + z * (r4 - r5)]
        ])

        Hyn = np.array([
            [t5 * (1 - 2 * n) + (t6 - t5) * z],
            [1+ r5 * (1 - 2 * n) - (r5 + r6) * z],
            [-q5 * (1 - 2 * n) - z * (q5 + q6)],
            [z * (t4 + t6)],
            [z * (r4 - r6)],
            [-z * (q4 - q6)],
            [t5 * ( 1 - 2 * n) - z * (t4 + t5)],
            [1 - r5 * (1 - 2 * n) + z * (r4 - r5)],
            [-q5 * (1- 2 * n) + z * (q4 - q5)]
        ])
        
        # print("Hxz", Hxz)
        # print("B row 1", y31 * np.transpose(Hxz) + y12 * np.transpose(Hxn))

        B = np.vstack([
            y31 * np.transpose(Hxz) + y12 * np.transpose(Hxn),
            -x31 * np.transpose(Hyz) - x12 * np.transpose(Hyn),
            -x31 * np.transpose(Hxz) - x12 * np.transpose(Hxn) + y31 * np.transpose(Hyz) + y12 * np.transpose(Hyn)
        ])
        # print("shape", B.shape)
        return B

    def normalize(self, vector):
        length = (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** (1/2)
        return vector / length

    def ElementStiffnessMatrix(self, properties, coormat, edge_matrix, e):
        # convert coordinates from global to local
        # print(coormat)
        xj = (coormat[3] + coormat[6])/2
        xk = (coormat[0] + coormat[6])/2
        xi = (coormat[0] + coormat[3])/2
        yj = (coormat[4] + coormat[7])/2
        yk = (coormat[1] + coormat[7])/2
        yi = (coormat[1] + coormat[4])/2
        zj = (coormat[5] + coormat[8])/2
        zk = (coormat[2] + coormat[8])/2
        zi = (coormat[2] + coormat[5])/2
        Vx = np.array([xj - xk, yj - yk, zj - zk])
        Vr = np.array([coormat[6] - xi, coormat[7] - yi, coormat[8] - zi])
        # print(Vx.shape)
        # print(Vr.shape)
        Vz = np.cross(Vx, Vr)
        Vy = np.cross(Vz, Vx)
        lx = self.normalize(Vx)
        ly = self.normalize(Vy)
        lz = self.normalize(Vz)
        transform = np.array(
            [lx, ly, lz]
        )
        # print("transform", transform)
        coorloc = np.zeros((3,3))
        # print(coorloc)
        transposet = np.transpose(transform)

        x = np.arange(4).reshape((2,2))
        xt = np.transpose(x)
        # print("x", x.transpose)
        # print("shapes", transposet.shape, coormat[0:3].shape)
        # print(np.dot(transposet, coormat[0:3]))
        coorloc[0, :] = np.dot(transposet, coormat[0:3])
        coorloc[1, :] = np.dot(transposet, coormat[3:6])
        coorloc[2, :] = np.dot(transposet, coormat[6:9])
        kcst = self.CSTElement(properties, coorloc, e)
        kdkt = self.DKTElement(properties, coorloc)
        # print("!!!!!")
        # print(kcst)
        # print(kdkt)
        k = np.zeros((18, 18))
        for i in range(3):
            for j in range(3):
                k[i * 6, j * 6] += kcst[i * 2, j * 2]
                k[i * 6 + 1, j * 6] += kcst[i * 2 + 1, j * 2]
                k[i * 6, j * 6 + 1] += kcst[i * 2, j * 2 + 1]
                k[i * 6 + 1, j * 6 + 1] += kcst[i * 2 + 1, j * 2 + 1]
                k[i * 6 + 2, j * 6 + 2] += kdkt[i * 2, j * 2]
                k[i * 6 + 3, j * 6 + 2] += kdkt[i * 2 + 1, j * 2]
                k[i * 6 + 4, j * 6 + 2] += kdkt[i * 2 + 2, j * 2]
                k[i * 6 + 2, j * 6 + 3] += kdkt[i * 2, j * 2 + 1]
                k[i * 6 + 3, j * 6 + 3] += kdkt[i * 2 + 1, j * 2 + 1]
                k[i * 6 + 4, j * 6 + 3] += kdkt[i * 2 + 2, j * 2 + 1]
                k[i * 6 + 2, j * 6 + 4] += kdkt[i * 2, j * 2 + 2]
                k[i * 6 + 3, j * 6 + 4] += kdkt[i * 2 + 1, j * 2 + 2]
                k[i * 6 + 4, j * 6 + 4] += kdkt[i * 2 + 2, j * 2 + 2]
                k[i * 6 + 5, j * 6 + 5] += max(np.diagonal(kcst)) / 1000
        # k1 = np.concatenate((kcst, np.zeros((6,12)), axis=0)
        # k2 = np.concatenate((np.zeros((9,6)), kdkt, np.zeros((9,3))), axis=0)
        # kmax = max(np.diagnal(k1)) / 1000
        # kdrill = np.array([
        #     [kmax, 0, 0],
        #     [0, kmax, 0],
        #     [0, 0, kmax]
        # ])
        # k3 = np.concatenate((np.zeros((3,15)), kdrill), axis=0)
        # k = np.concatenate((k1, k2, k3), axis=1)
        z = np.zeros((3,3))
        T1 = np.concatenate((transform, z, z, z, z, z), axis=0)
        T2 = np.concatenate((z, transform, z, z, z, z), axis=0)
        T3 = np.concatenate((z, z, transform, z, z, z), axis=0)
        T4 = np.concatenate((z, z, z, transform, z, z), axis=0)
        T5 = np.concatenate((z, z, z, z, transform, z), axis=0)
        T6 = np.concatenate((z, z, z, z, z, transform), axis=0)
        T = np.concatenate((T1, T2, T3, T4, T5, T6), axis = 1)
        K = np.dot(np.dot(np.transpose(T), k), T)

        return K


    def SpaceTrussAssemble(self, K,k,i,j):
        # puts together stiffness matrix
            #need to look up if there  is a better way of doing this
        
        for val1 in range(18):
            for val2 in range(18):
                if val1 <= 5:
                    index1 = 6 * i + val1
                elif val1 <= 11:
                    index1 = 6 * j + (val1 - 6)
                else:
                    index1 = 6 * j + (val1 - 12)

                if val2 <= 5:
                    index2 = 6 * i + val2
                elif val2 <= 11:
                    index2 = 6 * j + (val2 - 6)
                else:
                    index2 = 6 * j + (val1 - 12)

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

        input_socket_1.set_value(self.E)
        input_socket_2.set_value(self.v)

        # get inputs from previous nodes
        # print("!!!")
        # print(self.material_input)
        if self.material_input == "VALUE":
            E = self.get_value(input_socket_1)
            v = self.get_value(input_socket_2)
            
        elif self.material_input == "NODE":
            # print("NODE")
            mat_vect = self.get_value(input_socket_3)
            # print(mat_vect)
            E = mat_vect[0]
            v = mat_vect[2]

        if self.size_input == "VALUE":
            input_socket_4.set_value(self.t)
            t = self.get_value(input_socket_4)
        else:
            t = self.get_value(input_socket_5)


                
        self.object = self.get_value(input_socket_6)
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
        edge_matrix = np.zeros((18), dtype=int)
        properties = [0,0,0,0,0,0]
        coormat = np.zeros((9))
        new_row_1 = edge_matrix
        new_row_2 = properties
        i = 0
        G = 0
        for face in bm.faces:
            j = 0
            coorelement = np.array([])
            for loop in face.loops:
                vert = loop.vert
                coordinates = np.array([vert.co[0], vert.co[1], vert.co[2]])
                coorelement = np.hstack([coorelement, coordinates])

                for i in range(6):
                    new_row_1[j] = vert.index
                    j = j + 1

            new_row_2[0] = E
            new_row_2[1] = G
            new_row_2[2] = v
            new_row_2[3] = t
            if DEBUG: print(new_row_1,new_row_2)
            edge_matrix = np.vstack([edge_matrix, new_row_1])
            properties = np.vstack([properties, new_row_2])
            # print(coorelement)
            # print(coormat)
            coormat = np.vstack([coormat, coorelement])
            i += 1
        edge_matrix = np.delete(edge_matrix, 0, 0) # find better way to initialize (redundant)
        properties = np.delete(properties, 0, 0)
        coormat = np.delete(coormat, 0, 0)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        # print(edge_matrix.shape)
        max = (edge_matrix[:,1:].max() + 1) * 6
        if DEBUG: print(max)


        bm.edges.ensure_lookup_table()
        # k = np.zeros((12,12))
        K=np.zeros((max,max))
        for e in range(len(edge_matrix)):
            # print(e)
            # print("coormat", coormat)
            # print("1", coormat[e, 0])
            k = self.ElementStiffnessMatrix(properties, coormat[e, :], edge_matrix, e)
            # print(k)

            K= self.SpaceTrussAssemble(K, k, edge_matrix[e,0], edge_matrix[e,6])

            # for i in range(12):
            #     for j in range(12):
            #         pass
            # print("k", k)

        # print("global:")
        # print(K)

        # print(edge_matrix)

        # # create stiffness matrix
        # k = np.zeros((len(edge_matrix),12,12))
        # bm.edges.ensure_lookup_table()
        # for i in range(len(edge_matrix)):
        #     k[i, :, :]=self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2],properties[i, 3],properties[i, 4],properties[i, 5], bm.edges[i].verts[0].co, bm.edges[i].verts[1].co)
        # if DEBUG: print(k.shape)

        # # create global stiffness matrix
        # if TIME: assem_start = timer()
        # K=np.zeros((max,max))
        # for i in range(len(edge_matrix)):        
        #     K=self.SpaceTrussAssemble(K,k[i, :, :],edge_matrix[i,1],edge_matrix[i,2])
        # if TIME: assem_end = timer()
        # print("space truss assemble", assem_end - assem_start)
        # # print("shape:", K.shape)
        # # print("K", K)

        bool = ((self.get_value(input_socket_7)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_8)
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
        # print(Ksolve)
        # print(F)
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

        # print("U")
        # print(U)

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
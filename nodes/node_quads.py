from telnetlib import DET
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
from scipy import integrate
from scipy.stats import uniform
from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base_solver_dynamic import NodeDynamicSolverBase
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False
DEBUG2 = False
TIME = True
CONTACT = False

class NodeQuad(Node, NodeDynamicSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'QuadNode'
    # Label for nice name display
    bl_label = "Quad node Testing"

    E: bpy.props.FloatProperty(default=21000000)
    v: bpy.props.FloatProperty(default=.3)
    t: bpy.props.FloatProperty(default=.1)
    object: bpy.props.PointerProperty(type=Object)
    solver_type: bpy.props.StringProperty(default="2DTruss")
    disp: bpy.props.StringProperty(default="")

    object1: bpy.props.PointerProperty(type=Object)
    object2: bpy.props.PointerProperty(type=Object)
    contact: bpy.props.StringProperty(name="Vertex Group")
    target: bpy.props.StringProperty(name="Vertex Group")

    DET = 0.0

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
        # self.inputs.new('SocketTypeObject', "Object").init("object1")
        # self.inputs.new('SocketTypeObject', "Object").init("object2")
        self.set_inputs()


    def draw_buttons(self, context, layout):
        super().draw_buttons(context, layout)
        # col = layout.column()
        # if self.object:
        #     col.prop_search(self, "contact", self.object, "vertex_groups")
        # if self.object:
        #     col.prop_search(self, "target", self.object, "vertex_groups")

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

    def update_object(self):
        # self.object1 = self.get_value(self.inputs[8])
        # self.object2 = self.get_value(self.inputs[9])
        return super().update_object()

    def ApplyBoundaryConditions(self, K):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.ravel(bool)

        N = K.shape[1]

        chi_interior = np.ones(N)
        chi_interior[bool] = 0.0
        I_int = sparse.spdiags(chi_interior, [0], N, N).tocsr()

        chi_boundary = np.zeros(N)
        chi_boundary[bool] = 1.0
        I_bound = sparse.spdiags(chi_boundary, [0], N, N).tocsr()

        Kp = I_int * K * I_int + I_bound

        return Kp

    def ApplyBoundaryConditionsForce(self, F):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.invert(bool)
        
        bool = np.ravel(bool)

        for i in range(len(bool)):
            if bool[i] == False:
                F[i] = 0

        return F

    def ApplyBoundaryConditionsDisplacement(self, u):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.invert(bool)
        
        # F = self.get_value(input_socket_5)
        bool = np.ravel(bool)
        # boolv,boolh = np.ix_(bool, bool)

        # apply boundary conditions
        u = u[bool]

        return u

    def Quads(self, NEL, ITYPE, NINT, THIC, YM, PR, XX, S, IOUT):
        
        # Dimension arrays
        B = np.zeros((8,4))
        S = np.zeros((8,8))
        DB = np.zeros(4)


        # TODO: add precision
        # create guass sampling points and weights
        XG = np.array([
            [0, 0, 0, 0],
            [-.577, .577, 0, 0],
            [-.775, 0, -.775, 0],
            [-.861, -.340, .340, .861]
        ])

        WGT = np.array([
            [2,0,0,0],
            [1,1,0,0],
            [.556,.889, .556, 0],
            [.348, .652, .652, .348]
        ])

        F = YM / (1 + PR)
        G = F * PR / (1 - 2 * PR)
        H = F + G

        D = np.array([
            [H, G, 0],
            [G, H, 0],
            [0, 0, F/2]
        ])

        Eprop = YM
        v = PR

        D = (Eprop / (1 - v ** 2)) * np.array([[1, v, 0], [v, 1, 0], [0, 0, (1 - v) / 2]])
        

        # THIC = 1

        for i in range(8):
            for j in range (8):
                S[i, j] = 0
                
        IST = 3
        

        for lx in range(NINT):
            RI = XG[lx, NINT - 1]
            test = XG[0,1]
            for ly in range(NINT):
                SI = XG[ly, NINT - 1]
                XBAR = THIC
                B = self.STDM(XX, B, self.DET, RI, SI, XBAR, NEL, ITYPE, IOUT)
                WT = WGT[NINT - 1, lx] * WGT[NINT - 1, ly] * XBAR * self.DET

                for j in range(8):
                    for k in range(IST):
                        DB[k] = 0.0
                        for l in range(IST):
                            DB[k] += D[k, l] * B[l, j]
                    for i in range(j, 8):
                        STIFF = 0.0
                        for l in range(IST):
                            STIFF += B[l, i] * DB[l]
                            S[i, j] += STIFF * WT

        for j in range(8):
            for i in range(j, 8):
                S[j, i] = S[i, j]


        return S

    def STDM (self, XX, B, det, R, S, XBAR, NEL, ITYPE, IOUT):
        RP = 1 + R
        SP = 1 + S
        RM = 1 - R
        SM = 1 - S

        H = np.array([
            [.25 * RP * SP],
            [.25 * RM * SP],
            [.25 * RM * SM], 
            [.25 * RP * SM]
        ])

        P = np.array([
            [.25 * SP, -.25 * SP, -.25 * SM, .25 * SM],
            [.25 * RP, .25 * RM, -.25 * RM, -.25 * RP]
        ])

        J = np.array([
            [0.0, 0.0], 
            [0.0, 0.0]
        ])
        DUM = -.125
        J[0,0] = DUM
        print(J)


        for i in range(2):
            for j in range(2):
                DUM = 0.0
                for k in range(4):
                    DUM += P[i, k] * XX[j, k]
                print(DUM)
                J[i,j] = DUM
                print(J)

        DetJ = J[0,0] * J[1,1] - J[1, 0] * J[0, 1]

        #TODO add error message for negative determinant

        DUM = 1/DetJ

        XJI = np.array([
            [J[1,1] * DUM, -J[0,1] * DUM],
            [-J[1, 0] * DUM, J[1,1] * DUM]
        ])

        B = np.zeros([4,8])

        K2 = 0
        for k in range(4):
            K2 += 2
            for i in range(2):
                B[0, K2 - 2] += XJI[0, i] * P[i, k]
                B[1, K2 - 1] += XJI[1, i] * P[i, k]
            B[2, K2 - 1] = B[0, K2 - 2]
            B[2, K2 - 2] = B[1, K2 - 1]

        self.DET = DetJ

        return B 
                

    def CSTElement(self, properties, coorloc):
        # assign properties
        t = properties[0, 3] # thickness
        Eprop = properties[0, 0] # modulus of elasticity
        v = properties[0, 2] # poissons ratio

        # plane stress matrix
        E = (Eprop / (1 - v ** 2)) * np.array([[1, v, 0], [v, 1, 0], [0, 0, (1 - v) / 2]])

        # get coordinates
        x1 = coorloc[0]
        x2 = coorloc[3]
        x3 = coorloc[6]
        x4 = coorloc[9]
        y1 = coorloc[1]
        y2 = coorloc[4]
        y3 = coorloc[7]
        y4 = coorloc[10]


        roots = np.array([-1/(3 ** .5), 1/(3 ** .5)])
        wj = 1
        wi = 1
        k = np.zeros((8,8))
        for j in range(2):
            for i in range(2):
                z = roots[i]
                n = roots[j]
                dn1dz = - (1 - n)/4
                dn1dn = - (1 - z)/4
                dn2dz = (1 - n)/4
                dn2dn = - (1 + z)/4
                dn3dz = (1 + n)/4
                dn3dn = (1 + z)/4
                dn4dz = -(1 + n)/4
                dn4dn = (1 - z)/4
                J11 = dn1dz * x1 + dn2dz * x2 + dn3dz * x3 + dn4dz * x4
                J12 = dn1dz * y1 + dn2dz * y2 + dn3dz * y3 + dn4dz * y4
                J21 = dn1dn * x1 + dn2dn * x2 + dn3dn * x3 + dn4dn * x4
                J22 = dn1dn * y1 + dn2dn * y2 + dn3dn * y3 + dn4dn * y4
                detJ = J11 * J22 - J12 * J21
                A = (1/detJ) * np.array([
                    [J22, -J12, 0, 0],
                    [0, 0, -J21, J11],
                    [-J21, J11, J22, -J12]
                ])
                G = np.array([
                    [dn1dz, 0, dn2dz, 0, dn3dz, 0, dn4dz, 0],
                    [dn1dn, 0, dn2dn, 0, dn3dn, 0, dn4dn, 0],
                    [0, dn1dz, 0, dn2dz, 0, dn3dz, 0, dn4dz],
                    [0, dn1dn, 0, dn2dn, 0, dn3dn, 0, dn4dn]
                ])

                B = np.dot(A, G)
                Btest = np.array([
                    [1 + z, 0, -(1+z), 0, -(1 - z), 0, (1 - z), 0],
                    [0, 1 + n, 0, 1 - n, 0, -(1 - n), 0, -(1 + n)],
                    [(1 + n), (1 - z), (1 - n),- (1 + z), -(1 - n), -(1 - z), -(1 + n), (1 - z)]
                ])

                k += wi * wj * np.dot(np.dot(np.transpose(B), E), B) * detJ
        return t * k

        

    def cstassembletest(self, K, k, v_num):
        for val1 in range(8):
            for val2 in range(8):
                # index1 = 6 * v_num[i] + val1
                # index2 = 6 * v_num[j] + val2
                if val1 <= 1:
                    index1 = 2 * v_num[0] + val1
                elif val1 <= 3:
                    index1 = 2 * v_num[1] + (val1 - 2)
                elif val1 <= 5:
                    index1 = 2 * v_num[2] + (val1 - 4)
                else:
                    index1 = 2 * v_num[3] + (val1 - 6)

                if val2 <= 1:
                    index2 = 2 * v_num[0] + val2
                elif val2 <= 3:
                    index2 = 2 * v_num[1] + (val2 - 2)
                elif val2 <= 5:
                    index2 = 2 * v_num[2] + (val2 - 4)
                else:
                    index2 = 2 * v_num[3] + (val2 - 6)

                K[index1][index2] += k[val1][val2]
        # print(K)
        return K

    def Contact2D(self):
        # gi = self.object1.vertex_groups[self.contact].index

        max = len(self.object1.data.vertices)
        numVertex = len(self.object1.data.vertices) + len(self.object2.data.vertices)
        x = np.zeros(numVertex)
        y = np.zeros(numVertex)
        vs = np.zeros((max,1))
        vs = ((vs == 1))
        i = 0
        
        # gi = self.object.vertex_groups[self.contact].index
        bm = bmesh.new()
        bm.from_mesh(self.object1.data)

        for v in bm.verts: #this will need to take already known data to speed up and for multiple iterations
            # print(v.groups)
            vertex = self.object1.matrix_world @ v.co
            x[i] = vertex.x
            y[i] = vertex.y
            # for g in v.groups:
            #     if g.group == gi:
            i += 1
        
        i = len(self.object1.data.vertices)

        bm = bmesh.new()
        bm.from_mesh(self.object2.data)

        for v in bm.verts: #this will need to take already known data to speed up and for multiple iterations
            # print(v.groups)
            vertex = self.object2.matrix_world @ v.co
            x[i] = vertex.x
            y[i] = vertex.y
            # for g in v.groups:
            #     if g.group == gi:
            i += 1

        # calculate preliminary values
        Bs = .5 # solve for this base on smallest master surface dimension

        xMax = np.max(x)
        xMin = np.min(x)
        yMax = np.max(y)
        yMin = np.min(y)

        Sx = (int)((xMax - xMin) / Bs + 1)
        Sy = (int)((yMax - yMin) / Bs + 1)

        # step 1: zero nbox
        nb = Sx * Sy # calculate number of buckets
        nbox = np.zeros(nb, dtype=int) #length is number of contact nodes
        lbox = np.zeros(numVertex,dtype=int)

        # step 2: find bucket id for each node
        # need to loop over all verticies
        for i in range(len(x)):
            Sxi = (int)((x[i]- xMin) / Bs + 1)
            Syi = (int)((y[i]- yMin) / Bs + 1)


            Bi = (Syi - 1) * Sx + (Sxi - 1)
            # step 3: store bucket id for node i in lbox
            lbox[i] = Bi

            # step 4: increment counter for bucket Bi
            nbox[Bi] += 1

        # step 5: Calculate pointer for each bucket j into sorted list of nodes
        npoint = np.zeros(nb, dtype=int)
        npoint[0] = 1

        # loop over all buckets
        for j in range(nb):

            npoint[j] = npoint[j - 1] + nbox[j - 1]

        # step 6: zero vector nbox
        nbox = np.zeros(nb, dtype=int)

        # step 7: sort nodes according to their bucket
        ndsort = np.zeros(len(x), dtype=int)
        for i in range(len(x)):
            index = nbox[lbox[i]] + npoint[lbox[i]] - 1
            ndsort[index] = i
            nbox[lbox[i]] = nbox[lbox[i]] + 1

        
        return 

    def ContactSolve(self):
        

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

        # get list of faces
        faces = [f for f in bm.faces]

        # get number of elements
        numElements = len(faces)

        elementMatrixLength = len(bm.faces) * 4

        elementMatrix = np.zeros((7,elementMatrixLength), dtype=int)
        index = 0
        indexface = 0
        connectivity = np.zeros(2, dtype=int)
        
        # create matrix for elements
        for face in bm.faces:
            indexedge = 0
            for edge in face.edges:
                elementMatrix[index, 0] = indexface
                elementMatrix[index, 1] = indexedge
                indexvert = 0
                for vert in edge.verts:
                    elementMatrix[index, 2 + indexvert] = edge.vert
                    connectivity[indexvert] = edge.vert
                    indexvert += 1
                elementMatrix[index, 4] = connectivity.min
                elementMatrix[index, 5] = connectivity.max
                elementMatrix[index, 6] = connectivity.max + connectivity.min * 4
                
                index += 1
                indexedge += 1
            indexface += 1

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
        # print(coormat)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        # print(edge_matrix.shape)
        max = (edge_matrix[:,1:].max() + 1) * 2

        bm.edges.ensure_lookup_table()
        K=np.zeros((max,max))
        for e in range(len(edge_matrix)):
            k = self.CSTElement(properties, coormat[e,:])
            store = np.zeros((8,8))
            quadtest = self.Quads(1, 1, 2, 1, properties[0], properties[1], coormat[e,:], store, 1)
            K = self.cstassembletest(K, k, edge_matrix[e,:])


        bool = ((self.get_value(input_socket_7)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_8)
        bool = np.ravel(bool)
        boolv,boolh = np.ix_(bool, bool)

        # apply boundary conditions
        Ksolve = K[boolv,boolh]
        F = F[boolv]
        # print(Ksolve)
        # print(F)
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        print('applying boundary conditions')
        if DEBUG2: print(Ksolve)
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

        self.Contact2D()

        return U

        #create connectivity matrix




    def eval(self):
        
        if CONTACT:
            self.ContactSolve()

        

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

        elementMatrixLength = len(bm.faces) * 4

        elementMatrix = np.zeros((elementMatrixLength, 7), dtype=int)
        index = 0
        indexface = 0
        connectivity = np.zeros(2, dtype=int)
        
        # create matrix for elements
        for face in bm.faces:
            indexedge = 0
            for edge in face.edges:
                elementMatrix[index, 0] = indexface
                elementMatrix[index, 1] = indexedge
                indexvert = 0
                for vert in edge.verts:
                    elementMatrix[index, 2 + indexvert] = vert.co.x
                    connectivity[indexvert] = vert.co.y
                    indexvert += 1
                elementMatrix[index, 4] = min(connectivity)
                elementMatrix[index, 5] = np.max(connectivity)
                elementMatrix[index, 6] = np.max(connectivity) + min(connectivity) * 4
                
                index += 1
                indexedge += 1
            indexface += 1

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
        # print(coormat)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        # print(edge_matrix.shape)
        max = (edge_matrix[:,1:].max() + 1) * 2

        bm.edges.ensure_lookup_table()
        K=np.zeros((max,max))
        for e in range(len(edge_matrix)):
            k = self.CSTElement(properties, coormat[e,:])
            store = np.zeros((8,8))
            
            xx = np.vstack((coormat[e, 0:11:3], coormat[e, 1:11:3]))
            quadtest = self.Quads(1, 1, 2, properties[0,3], properties[0,0], properties[0,2], xx, store, 1)
            K = self.cstassembletest(K, k, edge_matrix[e,:])


        bool = ((self.get_value(input_socket_7)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_8)
        bool = np.ravel(bool)
        boolv,boolh = np.ix_(bool, bool)

        # apply boundary conditions
        Ksolve = K[boolv,boolh]
        F = F[boolv]
        # print(Ksolve)
        # print(F)
        if DEBUG: print(F.shape)
        F= np.reshape(F, (-1,1))
        # F=F[1:6,:]
        print('applying boundary conditions')
        if DEBUG2: print(Ksolve)
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
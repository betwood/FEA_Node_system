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
from .node_base_solver import NodeSolverBase
from .node_base import NodeBase
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False
DEBUG2 = False
TIME = True

class NodeBilinearQuad(Node, NodeSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'QuadBilinearNode'
    # Label for nice name display
    bl_label = "Bilinear Quad node"

    E: bpy.props.FloatProperty(default=21000000)
    v: bpy.props.FloatProperty(default=.3)
    t: bpy.props.FloatProperty(default=.1)
    object: bpy.props.PointerProperty(type=Object)
    solver_type: bpy.props.StringProperty(default="2DTruss")
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
        super().draw_buttons(context, layout)

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
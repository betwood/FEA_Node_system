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

class NodeShellQuad(Node, NodeSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'QuadShellNode'
    # Label for nice name display
    bl_label = "Shell Quad node"

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

    def test(self):
        coor = np.array([
            [0,0,0],
            [0.25,0,0],
            [.25,.25,0],
            [0,.25,0]
        ])
        property = np.array([[210000000, 0, .3, .025]])
        cst = self.CSTElement(property, coor)
        print(cst)
        
        # coor1 = np.array([
        #     [-4, -4, 0],
        #     [4, -4, 0],
        #     [-4, 4, 0]
        # ])
        # coor2 = np.array([
        #     [-4, 4, 0],
        #     [4, -4, 0],
        #     [4, 4, 0]
        # ])
        # v_num1 = np.array([0,1,2])
        # v_num2 = np.array([2,1,3])
        # property = np.array([[10000, 0, .3, 1]])
        # K = np.zeros((12,12))
        # kdkt1 = self.DKTElement(property, coor1)
        # bool = [False, False, False, False, False, False, True, True, True]
        # boolv,boolh = np.ix_(bool, bool)
        # d1 = kdkt1[boolv, boolh]
        # solve = kdkt1[boolv, boolh]
        # kdkt2 = self.DKTElement(property, coor2)
        # bool = [True, True, True, False, False, False, False, False, False]
        # boolv,boolh = np.ix_(bool, bool)
        # solve += kdkt2[boolv, boolh]
        # K = self.DKTAssemble(K,kdkt1, v_num1)
        # K = self.DKTAssemble(K,kdkt2, v_num2)
        # bool = [False, False, False, False, False, False, True, True, True, False, False, False]
        # boolv,boolh = np.ix_(bool, bool)
        # Ksolve = K[boolv, boolh]
        # F = np.array([[5], [0], [0]])
        # Ksolve_csr = sparse.csr_matrix(Ksolve)
        # F_csr = sparse.csr_matrix(F)
        # print('solving')
        # u = scipy.sparse.linalg.spsolve(Ksolve_csr, F_csr)
        # print('solving done')


    def DKTAssemble(self, K, k, v_num):
        for val1 in range(9):
            for val2 in range(9):
                # index1 = 6 * v_num[i] + val1
                # index2 = 6 * v_num[j] + val2
                if val1 <= 2:
                    index1 = 3 * v_num[0] + val1
                elif val1 <= 5:
                    index1 = 3 * v_num[1] + (val1 - 3)
                else:
                    index1 = 3 * v_num[2] + (val1 - 6)

                if val2 <= 2:
                    index2 = 3 * v_num[0] + val2
                elif val2 <= 5:
                    index2 = 3 * v_num[1] + (val2 - 3)
                else:
                    index2 = 3 * v_num[2] + (val2 - 6)

                K[index1][index2] += k[val1][val2]
        # print(K)
        return K



    def test2(self):
        coor = np.array([
            [0, 0, 0],
            [0.5, 0, 0],
            [0, 0.25, 0]
        ])
        v_num = np.array([0,2,1])
        property = np.array([[210000000, 0, .3, .025]])
        K = np.zeros((18,18))
        kcst = self.CSTElement(property, coor)
        kdkt = self.DKTElement(property, coor)
        k = np.zeros((18, 18))
        for i in range(3):
            for j in range(3):
                k[i * 6, j * 6] += kcst[i * 2, j * 2]
                k[i * 6 + 1, j * 6] += kcst[i * 2 + 1, j * 2]
                k[i * 6, j * 6 + 1] += kcst[i * 2, j * 2 + 1]
                k[i * 6 + 1, j * 6 + 1] += kcst[i * 2 + 1, j * 2 + 1]
                k[i * 6 + 2, j * 6 + 2] += kdkt[i * 3, j * 3]
                k[i * 6 + 3, j * 6 + 2] += kdkt[i * 3 + 1, j * 3]
                k[i * 6 + 4, j * 6 + 2] += kdkt[i * 3 + 2, j * 3]
                k[i * 6 + 2, j * 6 + 3] += kdkt[i * 3, j * 3 + 1]
                k[i * 6 + 3, j * 6 + 3] += kdkt[i * 3 + 1, j * 3 + 1]
                k[i * 6 + 4, j * 6 + 3] += kdkt[i * 3 + 2, j * 3 + 1]
                k[i * 6 + 2, j * 6 + 4] += kdkt[i * 3, j * 3 + 2]
                k[i * 6 + 3, j * 6 + 4] += kdkt[i * 3 + 1, j * 3 + 2]
                k[i * 6 + 4, j * 6 + 4] += kdkt[i * 3 + 2, j * 3 + 2]
                
        drill = max(np.diagonal(k)) / 1000
        for i in range(3):
            for j in range(3):
                k[i * 6 + 5, j * 6 + 5] += drill
        K = self.SpaceTrussAssemble(K,k, v_num)

        i = [2, 3, 4, 5, 0, 1]
        test = self.reorder(kcst, i)
        print(K)
        return K

        
        # print(kcst)
        # ob = bpy.context.active_object
        # me = ob.data
        # bm = bmesh.from_edit_mesh(me)

        # coordinates = np.zeros((3,3))
        # coorelement = []
        # for f in bm.faces:
        #     print(f.index)
        #     coorelement = []
        #     for vert in f.verts:
        # #           v = loop.vert
        #         # print(v.index, v.co[0], v.co[1], v.co[2])
        #         coordinates = np.array([vert.co[0], vert.co[1], vert.co[2]])
        #         coorelement = np.hstack([coorelement, coordinates])
        #         print(vert.index, coorelement
    def detJ(self, z, n, coorloc):
        x = np.array([coorloc[0,0], coorloc[1,0], coorloc[2,0], coorloc[3,0]])
        y = np.array([
            [coorloc[0,1]],
            [coorloc[1,1]],
            [coorloc[2,1]],
            [coorloc[3,1]],
        ])
        J = np.array([
            [0, 1 - n, n - z, z - 1],
            [n - 1, 0, z + 1, -z - n],
            [z - n, -z - 1, 0, n + 1],
            [1 - z, z + n, -n - 1, 0]
        ])
        detJ = (1/8) * np.dot(np.dot(x, J), y)
        return detJ

    def CSTElement(self, properties, coorloc):
        t = properties[0, 3]
        Eprop = properties[0, 0]
        v = properties[0, 2]

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
        x4 = coorloc[3, 0]
        y1 = coorloc[0, 1]
        y2 = coorloc[1, 1]
        y3 = coorloc[2, 1]
        y4 = coorloc[3, 1]

        # A = (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2
        # print(A)

        # B = (1 / (2 * A)) * np.array(
        #     [[y2 - y3, 0, y3 - y1, 0, y1 - y2, 0],
        #     [0, x3 - x2, 0, x1 - x3, 0, x2 - x1],
        #     [x3 - x2, y2 - y3, x1 - x3, y3 - y1, x2 - x1, y1 - y2]]
        # )
        # if DEBUG2: print(B)
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

    def Geometry(self, coorloc):
        x23 = coorloc[1, 0] - coorloc[2, 0]
        x31 = coorloc[2, 0] - coorloc[0, 0]
        x12 = coorloc[0, 0] - coorloc[1, 0]
        y23 = coorloc[1, 1] - coorloc[2, 1]
        y31 = coorloc[2, 1] - coorloc[0, 1]
        y12 = coorloc[0, 1] - coorloc[1, 1]
        l23 = (x23 ** 2 + y23 ** 2) ** (1/2)
        l31 = (x31 ** 2 + y31 ** 2) ** (1/2)
        l12 = (x12 ** 2 + y12 ** 2) ** (1/2)
        X = np.array([x23, x31, x12])
        Y = np.array([y23, y31, y12])
        L = np.array([l23, l31, l12])
        return X, Y, L

    def DKTElement(self, properties, coorloc):
        # [X, Y, L] = self.Geometry(coorloc)
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
        Db = E * t ** 3 / (12 * (1 - v ** 2)) * np.array([[1, v, 0], [v, 1, 0], [0, 0, (1 - v) / 2]])
        A = (1 / 2) * (x31 * y12 - x12 * y31)
        z = np.array([1/2, 1/2, 0])
        n = np.array([0, 1/2, 1/2])
        w = np.array([1/6, 1/6, 1/6])
        k1 = np.zeros((9, 9))
        # for j in range(3):
        for i in range(3):
            B = self.B(properties, coorloc, z[i], n[i])
            # print("B", B)
            k1 += w[i] * np.dot(np.dot(np.transpose(B), Db), B)
            # print("before", k)
        
        n = 10
        dz = 1/n
        k = np.zeros((9, 9))
        z = 0
        nu = 0
        for i in range(n):
            for j in range(n):
                z = dz/2 + i * dz
                dnu = (1 - z)/n
                nu = dnu/2 + j * dnu
                B = self.B(properties, coorloc, z, nu)
                k += dz * dnu * np.dot(np.dot(np.transpose(B), Db), B)

            

        

        # f = lambda z, n: np.dot(np.dot(np.transpose(self.B(properties, coorloc, z, n)), Db), self.B(properties, coorloc, z, n))
        # k1 = integrate.dblquad(f, 0, 1, lambda n: 0, lambda n: 1 - n)
        return 2 * A * k

    def B(self, properties, coorloc, z, n):
        x1 = coorloc[0, 0]
        x2 = coorloc[1, 0]
        x3 = coorloc[2, 0]
        y1 = coorloc[0, 1]
        y2 = coorloc[1, 1]
        y3 = coorloc[2, 1]
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
            [1 + r6 * (1 - 2 * z) - (r5 + r6) * n],
            [-q6 * (1 - 2 * z) - n * (q5 + q6)],
            [-t6 * (1 - 2 * z) + n * (t4 + t6)],
            [-1 + r6 * (1 - 2 * z) - n * (r4 - r6)],
            [q6 * (1 - 2 * z) + n * (q4 - q6)],
            [-n * (t4 + t5)],
            [n * (r4 - r5)],
            [-n * (q4 - q5)]
        ])

        Hxn = np.array([
            [-P5 * (1 - 2 * n) - z * (P6 - P5)],
            [q5 * (1 - 2 * n) - z * (q5 + q6)],
            [-4 + 6 * (z + n) + r5 * (1 - 2 * n) - z * (r5 + r6)],
            [z * (P4 + P6)],
            [z * (q4 - q6)],
            [-z * (r6 - r4)],
            [P5 * (1 - 2 * n) - z * (P4 + P5)],
            [q5 * (1 - 2 * n) + z * (q4 - q5)],
            [-2 + 6 * n + r5 * (1 - 2 * n) + z * (r4 - r5)]
        ])

        Hyn = np.array([
            [-t5 * (1 - 2 * n) - z * (t6 - t5)],
            [1 + r5 * (1 - 2 * n) - z * (r5 + r6)],
            [-q5 * (1 - 2 * n) + z * (q5 + q6)],
            [z * (t4 + t6)],
            [z * (r4 - r6)],
            [-z * (q4 - q6)],
            [t5 * ( 1 - 2 * n) - z * (t4 + t5)],
            [1 - r5 * (1 - 2 * n) + z * (r4 - r5)],
            [-q5 * (1 - 2 * n) + z * (q4 - q5)]
        ])
        
        # print("Hxz", Hxz)
        # print("B row 1", y31 * np.transpose(Hxz) + y12 * np.transpose(Hxn))

        B = np.vstack([
            y31 * np.transpose(Hxz) + y12 * np.transpose(Hxn),
            -x31 * np.transpose(Hyz) - x12 * np.transpose(Hyn),
            -x31 * np.transpose(Hxz) - x12 * np.transpose(Hxn) + y31 * np.transpose(Hyz) + y12 * np.transpose(Hyn)
        ])
        # print("shape", B.shape)
        A = x31 * y12 - x12 * y31 #this is 2A but used as 2A in next line
        return (1 / A) * B

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
        Vx = np.array([[xj - xk], [yj - yk], [zj - zk]])
        Vr = np.array([[coormat[6] - xi], [coormat[7] - yi], [coormat[8] - zi]])
        # print(Vx.shape)
        # print(Vr.shape)
        Vz = np.cross(Vx, Vr, axis=0)
        Vy = np.cross(Vz, Vx, axis=0)
        lx = self.normalize(Vx)
        ly = self.normalize(Vy)
        lz = self.normalize(Vz)
        transform = np.hstack([lx, ly, lz])
        # transform = transform.reshape(3,3)
        # print("transform", transform)
        coorloc = np.zeros((3,3))
        # print(coorloc)
        transposet = np.transpose(transform)

        x = np.arange(4).reshape((2,2))
        xt = np.transpose(x)
        # print("x", x.transpose)
        print("shapes", transposet.shape, coormat[0:3].shape)
        # print(np.dot(transposet, coormat[0:3]))
        coorloc[0, :] = np.dot(transposet, coormat[0:3])
        coorloc[1, :] = np.dot(transposet, coormat[3:6])
        coorloc[2, :] = np.dot(transposet, coormat[6:9])

        # create non  local coordinates for testing
        # coorloc[0, :] = coormat[0:3]
        # coorloc[1, :] = coormat[3:6]
        # coorloc[2, :] = coormat[6:9]
        # print("coorloc", coorloc)

        kcst = self.CSTElement(properties, coorloc)
        kdkt = self.DKTElement(properties, coorloc)
        # print("!!!!!")
        # print(kcst)
        # print(kdkt)
        k = np.zeros((18, 18))
        # look into 3 X 3 nature
        for i in range(3):
            for j in range(3):
                k[i * 6, j * 6] += kcst[i * 2, j * 2]
                k[i * 6 + 1, j * 6] += kcst[i * 2 + 1, j * 2]
                k[i * 6, j * 6 + 1] += kcst[i * 2, j * 2 + 1]
                k[i * 6 + 1, j * 6 + 1] += kcst[i * 2 + 1, j * 2 + 1]
                k[i * 6 + 2, j * 6 + 2] += kdkt[i * 3, j * 3]
                k[i * 6 + 3, j * 6 + 2] += kdkt[i * 3 + 1, j * 3]
                k[i * 6 + 4, j * 6 + 2] += kdkt[i * 3 + 2, j * 3]
                k[i * 6 + 2, j * 6 + 3] += kdkt[i * 3, j * 3 + 1]
                k[i * 6 + 3, j * 6 + 3] += kdkt[i * 3 + 1, j * 3 + 1]
                k[i * 6 + 4, j * 6 + 3] += kdkt[i * 3 + 2, j * 3 + 1]
                k[i * 6 + 2, j * 6 + 4] += kdkt[i * 3, j * 3 + 2]
                k[i * 6 + 3, j * 6 + 4] += kdkt[i * 3 + 1, j * 3 + 2]
                k[i * 6 + 4, j * 6 + 4] += kdkt[i * 3 + 2, j * 3 + 2]
        
        drill = max(np.diagonal(k)) / 1000
        for i in range(3):
            for j in range(3):
                k[i * 6 + 5, j * 6 + 5] += drill
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
        T1 = np.concatenate((transposet, z, z, z, z, z), axis=0)
        T2 = np.concatenate((z, transposet, z, z, z, z), axis=0)
        T3 = np.concatenate((z, z, transposet, z, z, z), axis=0)
        T4 = np.concatenate((z, z, z, transposet, z, z), axis=0)
        T5 = np.concatenate((z, z, z, z, transposet, z), axis=0)
        T6 = np.concatenate((z, z, z, z, z, transposet), axis=0)
        T = np.concatenate((T1, T2, T3, T4, T5, T6), axis = 1)
        K = np.dot(np.dot(np.transpose(T), k), T)
        
        # print(K)
        bool = [True, True, False, False, False, False,True, True, False, False, False, False,True, True, False, False, False, False]
        boolv,boolh = np.ix_(bool, bool)
        cstdebug = K[boolv, boolh]
        # print(cstdebug)
        i = np.array([
            [2, 3, 4, 5, 0, 1],
            [2, 3, 4, 5, 0, 1],
            [2, 3, 4, 5, 0, 1],
            [0, 1, 2, 3, 4, 5],
            [2, 3, 4, 5, 0, 1],
            [4, 5, 0, 1, 2, 3],
            [4, 5, 0, 1, 2, 3],
            [0, 1, 2, 3, 4, 5],
            [0, 1, 2, 3, 4, 5],
            [0, 1, 2, 3, 4, 5],
            [2, 3, 4, 5, 0, 1],
            [2, 3, 4, 5, 0, 1],
            ])
        # self.reorder(cstdebug, i[e, :])
        return K

    def reorder(self, cst, i):
        temp = cst[:, i]
        ele_new = temp[i, :]
        # print("ele", ele_new)
        

    def cstassembletest(self, K, k, v_num):
        bool = [True, True, False, False, False, False,True, True, False, False, False, False,True, True, False, False, False, False]
        boolv,boolh = np.ix_(bool, bool)
        kcst = k[boolv, boolh]
        for val1 in range(6):
            for val2 in range(6):
                # index1 = 6 * v_num[i] + val1
                # index2 = 6 * v_num[j] + val2
                if val1 <= 1:
                    index1 = 2 * v_num[0] + val1
                elif val1 <= 3:
                    index1 = 2 * v_num[1] + (val1 - 2)
                else:
                    index1 = 2 * v_num[2] + (val1 - 4)

                if val2 <= 1:
                    index2 = 2 * v_num[0] + val2
                elif val2 <= 3:
                    index2 = 2 * v_num[1] + (val2 - 2)
                else:
                    index2 = 2 * v_num[2] + (val2 - 4)

                K[index1][index2] += kcst[val1][val2]
        # print(K)
        return K

    def SpaceTrussAssemble(self, K,k, v_num):
        # puts together stiffness matrix
            #need to look up if there  is a better way of doing this
        
        # for i in range(3):
        #     for j in range(3):
        for val1 in range(18):
            for val2 in range(18):
                # index1 = 6 * v_num[i] + val1
                # index2 = 6 * v_num[j] + val2
                if val1 <= 5:
                    index1 = 6 * v_num[0] + val1
                elif val1 <= 11:
                    index1 = 6 * v_num[1] + (val1 - 6)
                else:
                    index1 = 6 * v_num[2] + (val1 - 12)

                if val2 <= 5:
                    index2 = 6 * v_num[0] + val2
                elif val2 <= 11:
                    index2 = 6 * v_num[1] + (val2 - 6)
                else:
                    index2 = 6 * v_num[2] + (val2 - 12)

                K[index1][index2] += k[val1][val2]
        print(K)

        
        # bool = [True, True, False, False, False, False,True, True, False, False, False, False,True, True, False, False, False, False,True, True, False, False, False, False]
        # boolv,boolh = np.ix_(bool, bool)
        # cstdebug = K[boolv, boolh]
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
        edge_matrix = np.zeros((3), dtype=int)
        properties = [0,0,0,0,0,0]
        coormat = np.zeros((9))
        new_row_1 = np.zeros((3), dtype=int)
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
        print(coormat)
        if DEBUG: print('edge_matrix',edge_matrix)
        if DEBUG: print('properties',properties)
        # print(edge_matrix.shape)
        max = (edge_matrix[:,1:].max() + 1) * 6
        if DEBUG: print(max)


        bm.edges.ensure_lookup_table()
        # k = np.zeros((12,12))
        K=np.zeros((max,max))
        Kcst=np.zeros((22, 22))
        for e in range(len(edge_matrix)):
            # print(e)
            # print("coormat", coormat)
            # print("1", coormat[e, 0])
            # k = self.ElementStiffnessMatrix(properties, coormat[e, :], edge_matrix, e)
            # print(k)
            k = self.CSTElement(properties, coormat[e,:])

            # K = self.SpaceTrussAssemble(K, k, edge_matrix[e, :])
            K = self.cstassembletest(K, k, edge_matrix[e,:])
            # for i in range(18):
            #     print("K:", K[i,:])
            #     print("K2:", K2[i,:])

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
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

class NodeBilinearQuadContact(Node, NodeDynamicSolverBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'BilinearQuadContact'
    # Label for nice name display
    bl_label = "Bilinear Quad Contact"

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
    v: bpy.props.FloatProperty(default=.3)
    t: bpy.props.FloatProperty(default=.1)
    rho: bpy.props.FloatProperty(default=.1)
    num_t: bpy.props.IntProperty(default=20)
    dt: bpy.props.FloatProperty(default=.1)
    d0: bpy.props.FloatProperty(default=0)
    v0: bpy.props.FloatProperty(default=0)
    b: bpy.props.FloatProperty(default=1/6, min=0, max=1/2)
    g: bpy.props.FloatProperty(default=1/2)
    object: bpy.props.PointerProperty(type=Object)
    disp: bpy.props.StringProperty(default="")
    
    def init(self, context):
        self.outputs.new('SocketTypeMatrix', "Output")
        self.inputs.new('SocketTypeFloat', "E").init("E")
        self.inputs.new('SocketTypeFloat', "v").init("v")
        # self.inputs.new('SocketTypeMatrix', "Material")
        self.inputs.new('SocketTypeFloat', "t").init("t")
        # self.inputs.new('SocketTypeMatrix', "t_mat")
        self.inputs.new('SocketTypeObject', "Object").init("object")
        self.inputs.new('SocketTypeMatrix', "Boundary conditions")
        self.inputs.new('SocketTypeMatrix', "Forces")
        # self.set_inputs()

    def draw_buttons(self, context, layout):
        pass

    def draw_buttons_ext(self, context, layout):
        super().draw_buttons_ext(context, layout)


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

        # 2 X 2 guass point integration
        roots = np.array([-1/(3 ** .5), 1/(3 ** .5)])
        wj = 1
        wi = 1
        k = np.zeros((8,8))
        m = np.zeros((8,8))
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

                N1 = (1 - n) * (1 - z) / 4
                N2 = (1 + n) * (1 - z) / 4
                N3 = (1 + n) * (1 + z) / 4
                N4 = (1 - n) * (1 + z) / 4

                N = np.array([
                    [N1, 0, N2, 0, N3, 0, N4, 0],
                    [0, N1, 0, N2, 0, N3, 0, N4]
                ])

                k += wi * wj * np.dot(np.dot(np.transpose(B), E), B) * detJ
                m += np.dot(np.transpose(N), N) * detJ


        return t * k, t * self.rho * m

    def SpaceTrussAssemble(self, K,k, v_num, M, m):
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
                M[index1][index2] += m[val1][val2]
        return M, K

    def TangentStiffnessMatrix(self):
        # calculate tangent stiffness matrix
        #initialize Kt
        Kt = np.zeros((8,8))


        return Kt

    def NewtonRaphsonIteration(self):
        # initialize values (moved elsewhere later)
        tol = 1.0 * 10 ** -5
        it = 0
        maxiter = 20

        while conv < tol & it < maxiter:
            # start load increment
            # Use displacement and load from prior increment

            # find tangent stiffness
            Kt = TangentStiffnessMatrix()
            # predict new displacement
            # Find secant stiffness
            # Find new load error (residual)
            # Check error magnitude

    def Elastic(self):
        max = (edge_matrix[:,1:].max() + 1) * 2

        # create stiffness matrix
        k = np.zeros((4,4))
        # m = np.zeros((4,4))
        K = np.zeros((max, max))
        # M = np.zeros((max, max))
        for i in range(len(edge_matrix)):
            [k, m] = self.CSTElement(properties,coormat[i,:])
            
            [M, K] = self.SpaceTrussAssemble(K, k, edge_matrix[i,:], M, m)


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

        # 2 X 2 guass point integration
        roots = np.array([-1/(3 ** .5), 1/(3 ** .5)])
        wj = 1
        wi = 1
        k = np.zeros((8,8))
        m = np.zeros((8,8))
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

                N1 = (1 - n) * (1 - z) / 4
                N2 = (1 + n) * (1 - z) / 4
                N3 = (1 + n) * (1 + z) / 4
                N4 = (1 - n) * (1 + z) / 4

                N = np.array([
                    [N1, 0, N2, 0, N3, 0, N4, 0],
                    [0, N1, 0, N2, 0, N3, 0, N4]
                ])

                k += wi * wj * np.dot(np.dot(np.transpose(B), E), B) * detJ
                m += np.dot(np.transpose(N), N) * detJ


        return t * k, t * self.rho * m

    def NLFEA(self):
        dummy = 0

    
    def eval(self):
        ld = False
        if ld == True:
            U = self.largedispeval()
            return U
        
        # set input sockets
        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]
        input_socket_3 = self.inputs[2]
        input_socket_4 = self.inputs[3]
        input_socket_5 = self.inputs[4]
        input_socket_6 = self.inputs[5]

        input_socket_1.set_value(self.E)
        input_socket_2.set_value(self.A)
        input_socket_3.set_value(self.t)

        # get inputs from previous nodes
        E = self.get_value(input_socket_1)
        v = self.get_value(input_socket_2)
        t = self.get_value(input_socket_3)
        self.object = self.get_value(input_socket_4, "object")
        object = self.object
        ob = object.data
        
        # create bmesh
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

        # create stiffness matrix
        k = np.zeros((4,4))
        m = np.zeros((4,4))
        K = np.zeros((max, max))
        M = np.zeros((max, max))
        for i in range(len(edge_matrix)):
            [k, m] = self.CSTElement(properties,coormat[i,:])
            
            [M, K] = self.SpaceTrussAssemble(K, k, edge_matrix[i,:], M, m)
        

        bool = ((self.get_value(input_socket_5)))
        bool = np.invert(bool)
        
        F = self.get_value(input_socket_6)
        bool = np.ravel(bool)
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

            # add displacement values to coormat
            coormat2 = coormat

            D=np.zeros((max, self.num_t))
            j = 0
            for z in range(len(D)):
                if bool[z] == 1:
                    D[z, :] = d[j, :]
                    j = j + 1

            for x in range(edge_matrix.shape[0]):
                for y in range(edge_matrix.shape[1]):
                    coormat2[x, 3 * y] += D[edge_matrix[x, y] * 2, i + 1]
                    coormat2[x, 3 * y + 1] += D[edge_matrix[x, y] * 2 + 1, i + 1]
            
            K = np.zeros((max, max))
            M = np.zeros((max, max))
            for e in range(len(edge_matrix)):
                [k, m] = self.CSTElement(properties,coormat2[e,:])
                [M, K] = self.SpaceTrussAssemble(K, k, edge_matrix[e,:], M, m)
            K = K[boolv,boolh]
            M = M[boolv, boolh]

            
        
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
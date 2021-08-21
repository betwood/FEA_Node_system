import bpy
import numpy as np
import bmesh
import scipy
import math
import mathutils
from numpy.linalg import inv

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base_solver_dynamic import NodeDynamicSolverBase
from scipy import sparse
from scipy import linalg
from .node_base import NodeBase
from timeit import default_timer as timer
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False
TIME = True

class NodePlaneTrussDynamic(Node, NodeDynamicSolverBase):

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
    tol: bpy.props.FloatProperty(default=10e-6)
    maxiter: bpy.props.IntProperty(default=100)
    forceinc: bpy.props.IntProperty(default=20)
    isDynamic: bpy.props.BoolProperty(default=False)
    damp1: bpy.props.FloatProperty(default = .1)
    damp2: bpy.props.FloatProperty(default = .1)

    stiffAssemTime = 0
    solveTime = 0
    boundaryConditionAssign = 0
    forceSolveTime = 0
    
    
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
        layout.prop(self,"isDynamic", text="Dynamic Simulation")
        layout.label(text="Nonlinear solver properties:")
        layout.prop(self, "tol", text="tolerance")
        layout.prop(self, "maxiter", text="max number of iterations")
        layout.prop(self, "forceinc", text="number of force increments")
        layout.prop(self, "damp1", text="damping factor related to mass")
        layout.prop(self, "damp2", text="damping factor related to stiffness")


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

    def SpaceTrussElementStiffness(self, L,E,A, C, S, N):
        # print(N)

        # solve for stiffness matrix
        kl = ((E * A) / L) * np.array([
            [C ** 2, C * S, -C ** 2, - C * S],
            [C * S, S ** 2, -C * S, -S ** 2],
            [-C ** 2, -C * S, C ** 2, C * S],
            [-C * S, -S ** 2, C * S, S ** 2],
        ])

        knl = (N / L) * np.array([
            [1, 0, -1, 0],
            [0, 1, 0, -1],
            [-1, 0, 1, 0],
            [0, -1, 0, 1]
        ])

        k = kl + knl
        return k

    def SpaceTrussMass(self, L,E,A, C, S, N):

        # # solve for mass matrix
        mhat = ((self.rho * A * L)/6) * np.array([
            [2, 0, 1, 0],
            [0, 2, 0, 1],
            [1, 0, 2, 0],
            [0, 1, 0, 2]
        ])

        # # transformation matrix
        T = np.array([
            [C, S, 0, 0],
            [-S, C, 0, 0],
            [0, 0, C, S],
            [0, 0, -S, C]
        ])


        m = np.dot(np.dot(np.transpose(T), mhat), T) 
        return m

    def SpaceTrussAssemble(self, K,k,i,j):
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
                # M[index1][index2] += m[val1][val2]
        return K

    def Ksolve(self, E, A, bool, D):
        boolv,boolh = np.ix_(bool, bool)
        d = np.zeros((len(bool), 1))
        j = 0
        for i in range(len(bool)):
            if bool[i] == 1:
                d[i] = D[j]
                j = j + 1
        
        bm = bmesh.new()
        bm.from_mesh(self.object.data)
        edge_matrix = [0,0,0]
        properties = [0,0,0,0]
        new_row_1 = edge_matrix
        new_row_2 = properties
        i = 0
        for edge in bm.edges:
            new_row_1[0] = edge.index
            new_row_1[1] = edge.verts[0].index
            new_row_1[2] = edge.verts[1].index
            x1 = edge.verts[0].co[0] + d[edge.verts[0].index * 2]
            x2 = edge.verts[1].co[0] + d[edge.verts[1].index * 2]
            y1 = edge.verts[0].co[1] + d[edge.verts[0].index * 2 + 1]
            y2 = edge.verts[1].co[1] + d[edge.verts[1].index * 2 + 1]
            new_row_2[0] = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** .5
            new_row_2[1] = E
            new_row_2[2] = A
            # if edge.verts[1].co[0]-edge.verts[0].co[0] == 0:
            #     new_row_2[3] = 0
            # else:
            new_row_2[3] = np.arctan2((y2 - y1), (x2 - x1))

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

        K = K[boolv,boolh]
        M = M[boolv, boolh]
        return K, M, max   

    def TrussLength(self, X, Y, u, edge):
        # calculate truss length
        L = np.zeros(len(edge))
        for truss in edge:
            i1 = truss[1]
            i2 = truss[2]
            L[truss[0]] = math.sqrt(((X[i2] + u[i2 * 2]) - (X[i1] + u[i1 * 2])) ** 2 + ((Y[i2] + u[i2 * 2 + 1]) - (Y[i1] + u[i1 * 2 + 1])) ** 2)

        return L

    def CreateStiffnessMatrix(self, max, properties, edge_matrix, cos, sin, N, row, col):
        # create the stiffness matrix
        k = np.zeros((4,4))
        m = np.zeros((4,4))
        K = np.zeros((max, max))
        M = np.zeros((max, max))
        # print(cos)
        # print(N)
        
        value = np.zeros(len(edge_matrix) * 4 ** 2)
        for i in range(len(edge_matrix)):
            k = self.SpaceTrussElementStiffness(properties[i, 0],properties[i, 1],properties[i, 2], cos[i], sin[i], N[i])
            
            
            # K = self.SpaceTrussAssemble(K, k, edge_matrix[i,1], edge_matrix[i,2])

            # will need to try and find efficient way of doing this (might be able to initialize as long as elements not changing)
            value[16 * i : 16*i + 16] = k.ravel()

        K = sparse.coo_matrix((value,(row,col)),shape=(max,max)).tocsr()


        # I = np.array([0,0,1,3,1,0,0])
        # J = np.array([0,2,1,3,1,0,0])
        # V = np.array([1,1,1,1,1,1,1])
        # B = sparse.coo_matrix((V,(I,J)),shape=(4,4))
        # Bdense = B.toarray()
        # Kdense = Ktest.toarray()

        return K

    def CreateMassMatrix(self, max, properties, edge_matrix, cos, sin, N, row, col):
        # create the stiffness matrix
        m = np.zeros((4,4))
        M = np.zeros((max, max))
        # print(cos)
        # print(N)
        
        value = np.zeros(len(edge_matrix) * 4 ** 2)
        for i in range(len(edge_matrix)):
            m = self.SpaceTrussMass(properties[i, 0],properties[i, 1],properties[i, 2], cos[i], sin[i], N[i])
            
            
            # K = self.SpaceTrussAssemble(K, k, edge_matrix[i,1], edge_matrix[i,2])

            # will need to try and find efficient way of doing this (might be able to initialize as long as elements not changing)
            value[16 * i : 16*i + 16] = m.ravel()

        M = sparse.coo_matrix((value,(row,col)),shape=(max,max)).tocsr()


        # I = np.array([0,0,1,3,1,0,0])
        # J = np.array([0,2,1,3,1,0,0])
        # V = np.array([1,1,1,1,1,1,1])
        # B = sparse.coo_matrix((V,(I,J)),shape=(4,4))
        # Bdense = B.toarray()
        # Kdense = Ktest.toarray()

        return M

    def ApplyBoundaryConditions(self, K):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        # bool = np.invert(bool)
        
        # F = self.get_value(input_socket_5)
        bool = np.ravel(bool)
        # boolv,boolh = np.ix_(bool, bool)

        # apply boundary conditions
        # K = K[boolv,boolh]
        # M = M[boolv, boolh]

        N = K.shape[1]

        chi_interior = np.ones(N)
        chi_interior[bool] = 0.0
        I_int = sparse.spdiags(chi_interior, [0], N, N).tocsr()

        chi_boundary = np.zeros(N)
        chi_boundary[bool] = 1.0
        I_bound = sparse.spdiags(chi_boundary, [0], N, N).tocsr()

        Kp = I_int * K * I_int + I_bound

        #TODO: optimize
        # for i in range(len(bool)):
        #     if bool[i] == False:
        #         K[i, :] = 0
        #         K[:, i] = 0
        #         K[i, i] = 1
        # Ii = I_int.todense()
        # Ib = I_bound.todense()
        # Kpd = Kp.todense()
        # Kd = K.todense()


        return Kp

    def ApplyBoundaryConditionsForce(self, F):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.invert(bool)
        
        # F = self.get_value(input_socket_5)
        bool = np.ravel(bool)
        # boolv,boolh = np.ix_(bool, bool)

        # apply boundary conditions
        # F = F[boolv]
        # F = np.reshape(F, (-1,1))

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

    def SolveCosSin(self, X, Y, u, L, edge):
        cos = np.zeros(len(edge))
        sin = np.zeros(len(edge))
        for truss in edge:
            i1 = truss[1]
            i2 = truss[2]
            cos[truss[0]] = ((X[i2] + u[i2 * 2]) - (X[i1] + u[i1 * 2])) / L[truss[0]]
            sin[truss[0]] = ((Y[i2] + u[i2 * 2 + 1]) - (Y[i1] + u[i1 * 2 + 1])) / L[truss[0]]

        return cos, sin

    def SolveAxialForces(self, A, E, L0, L, edge, sin, cos, max):
        # print("A:", A)
        

        N = np.zeros(len(edge))
        F = np.zeros((max, 1))
        for truss in edge:
            # C = cos[truss[0]]
            # S = sin[truss[0]]
            # T = np.array([
            # [C, S, 0, 0],
            # [-S, C, 0, 0],
            # [0, 0, C, S],
            # [0, 0, -S, C]])

            i1 = truss[1]
            i2 = truss[2]
            N[truss[0]] = (A * E / L0[truss[0]]) * (L[truss[0]] - L0[truss[0]])
            # calculate F (should be able to do this in one line?)
            Ftemp = N[truss[0]] * np.array([-cos[truss[0]], -sin[truss[0]], cos[truss[0]], sin[truss[0]]]) #TODO: find out why this worked
            # NArray =  np.array([[-N[truss[0]]], [0], [N[truss[0]]], [0]])
            # Ftest = np.dot(np.transpose(T), NArray)

            F[i1 * 2] += Ftemp[0]
            F[i1 * 2 + 1] += Ftemp[1]
            F[i2 * 2] += Ftemp[2]
            F[i2 * 2 + 1] += Ftemp[3]
        return N, F

    def LoadIncrement(self, max, properties, edge_matrix, cos, sin, F, X, Y, N, u, L0, Fnp1, row, col):
        numInc = self.forceinc # TODO: create input force node for node increments

        for i in range(numInc):
            print(i)
            loadInc = 1 / numInc

            dF = F * loadInc

            # create initial tangent stiffness matrix
            K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, N, row, col)

            #apply boundary conditions
            K = self.ApplyBoundaryConditions(K) # don't want F in this (might want to split function)


            # solve for incremental displacement
            Fcsr = sparse.csr_matrix(dF)
            du = scipy.sparse.linalg.spsolve(K, Fcsr)
            # du = np.linalg.solve(K, dF).ravel()
            # print(du)

            
            # update variables
            Fnp1 += dF
            u += du.ravel()
            # current truss lengths
            L = self.TrussLength(X, Y, u, edge_matrix) # will need to check this calculation and possibly make global

            # solve for cos and sin
            [cos, sin] = self.SolveCosSin(X, Y, u, L, edge_matrix)

            # Update N
            [Nnp1, Fint] = self.SolveAxialForces(properties[0, 2], properties[0, 1], L0, L, edge_matrix, sin, cos, max)

            

            # check residual
            R = Fint - Fnp1
            R = self.ApplyBoundaryConditionsForce(R)

            Rnorm = np.linalg.norm(R)

            [Nnp1, u] = self.NewtonRaphsonIteration(max, properties, edge_matrix, cos, sin, Fnp1, X, Y, du, Rnorm, u, Nnp1, R, L0, row, col)

        return u

    def ReconstructU(self, u, max):
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.invert(bool)
        
        # F = self.get_value(input_socket_5)
        bool = np.ravel(bool)
        U=np.zeros((max,1))
        j = 0
        for i in range(len(U)):
            if bool[i] == 1:
                U[i] = u[j]
                j = j + 1
        
        return U

    def NewtonRaphsonIteration(self, max, properties, edge_matrix, cos, sin, Fnp1, X, Y, du, Rnorm, unp1, Nnp1, R, L0, row, col):
        
        duk = np.zeros(len(du))
        # set iteration variables
        k = 0
        tol = self.tol
        maxiter = self.maxiter

        # print(R)
        # store n in temp variable
        

        while (Rnorm > tol) and (k < maxiter) or (k < 1):
            
            Ntemp = Nnp1 # seems like this should be outside loop
            # Calculate K
            # create initial tangent stiffness matrix
            if TIME: start = timer()
            K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, Ntemp, row, col)
            if TIME: end = timer()
            if TIME: self.stiffAssemTime += end - start

            if TIME: start = timer()
            #apply boundary conditions
            K = self.ApplyBoundaryConditions(K) # don't want F in this (might want to split function)
            if TIME: end = timer()
            if TIME: self.boundaryConditionAssign += end - start

            #print("K", K)
            #print("R", R)
            #print(np.linalg.solve(K, R).ravel())
            if TIME: start  = timer()
            Rcsr = sparse.csr_matrix(R)
            dukp1 = duk.ravel() - scipy.sparse.linalg.spsolve(K, Rcsr)
            # dukp1 = duk.ravel() - np.linalg.solve(K, R).ravel() 
            if TIME: end = timer()
            if TIME: self.solveTime += end - start        

            if TIME: start = timer()
            # current truss lengths
            L = self.TrussLength(X, Y, unp1.ravel() + dukp1.ravel(), edge_matrix) # will need to check this calculation and possibly make global

            # solve for cos and sin
            [cos, sin] = self.SolveCosSin(X, Y, unp1.ravel() + dukp1.ravel(), L, edge_matrix)

            # solve for axial forces and forces
            [Ntemp, Fint] = self.SolveAxialForces(properties[0, 2], properties[0, 1], L0, L, edge_matrix, sin, cos, max)

            
            # calculate residual
            R = Fint - Fnp1
            

            R = self.ApplyBoundaryConditionsForce(R)

            Rnorm = np.linalg.norm(R)
            if TIME: end = timer()
            if TIME: self.forceSolveTime += end - start

            # increase iteration count
            duk = dukp1
            k += 1

        du = unp1.ravel() + dukp1.ravel()
        print("number iterations", k)
        print("residual:", Rnorm)
        
        return Ntemp, du

    def Initialize(self):
        # set input sockets
        input_socket_1 = self.inputs[0]
        input_socket_2 = self.inputs[1]
        input_socket_3 = self.inputs[2]
        
        input_socket_5 = self.inputs[4]

        input_socket_1.set_value(self.E)
        input_socket_2.set_value(self.A)

        

        # get inputs from previous nodes
        E = self.get_value(input_socket_1)
        A = self.get_value(input_socket_2)
        self.object = self.get_value(input_socket_3, "object")
        object = self.object
        ob = object.data
        
        # create bmesh
        bm = bmesh.new()
        bm.from_mesh(self.object.data)

        # get force vector (will need to update for boundary conditions)
        F = self.get_value(input_socket_5)
        Fnp1 = np.zeros((len(F), 1))


        # create a matrix of values for edges
        #new row 1
            # column 0 = element number
            # column 1 = node 1
            # column 2 = node 2
        # new row 2
            # column 0 = initial length
            # column 1 = modulus of elasticity
            # column 2 = area
            # column 3 = theta
        # axial force vector (start as all zeros)
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

        # create ID matrix for active degrees of freedom
        ID = np.zeros((6,edge_matrix[:,1:].max() + 1))

        # rows that are not used
        ID[2:, :] = 1

        # apply boundary conditions
        input_socket_4 = self.inputs[3]
        bool = ((self.get_value(input_socket_4))) #might want to store somewhere
        bool = np.reshape(bool,(2, -1), order = 'F')


        # create equation ID based off of this matrix
        k = 1
        for j in range(len(ID[0])):
            for i in range(len(ID)):
                if ID[i,j] == 1:
                    ID[i,j] = 0
                elif bool[i,j] == True:
                    ID[i,j] = 0
                else:
                    ID[i,j] = k
                    k += 1

        # create connectivity row and col for creating sparse matrix
        row = np.zeros(len(edge_matrix) * 4 ** 2)
        col = np.zeros(len(edge_matrix) * 4 ** 2)
        for edge in edge_matrix:
            index = edge[0] * 16
            e1 = 2 * edge[1]
            e2 = 2 * edge[2]
            rowtemp = np.array([e1, e1 + 1, e2, e2 + 1,e1, e1 + 1, e2, e2 + 1,e1, e1 + 1, e2, e2 + 1,e1, e1 + 1, e2, e2 + 1])
            coltemp = np.array([e1, e1, e1, e1, e1 + 1, e1 + 1, e1 + 1, e1 + 1, e2, e2, e2, e2, e2 + 1, e2 + 1, e2 + 1, e2 + 1])
            row[index:index+16] = rowtemp
            col[index:index+16] = coltemp

        # initialize axial force matrix
        N = np.zeros(len(edge_matrix))
        
        # global displacements
        u = np.zeros(max)

        # create vectors for initial x and y
        X = np.zeros(len(bm.verts))
        Y = np.zeros(len(bm.verts))

        for vert in bm.verts:
            X[vert.index] = vert.co.x
            Y[vert.index] = vert.co.y

        # current truss lengths
        L0 = self.TrussLength(X, Y, u, edge_matrix) # will need to check this calculation and possibly make global

        # solve for cos and sin
        [cos, sin] = self.SolveCosSin(X, Y, u, L0, edge_matrix)
        
        # create initial tangent stiffness matrix
        K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, N, row, col)

        #apply boundary conditions
        K = self.ApplyBoundaryConditions(K)

        # u = self.ApplyBoundaryConditionsDisplacement(u)
        if(self.isDynamic == True):
            u = self.SolveDynamic(max, properties, edge_matrix, cos, sin, F, X, Y, N, u, L0, Fnp1, row, col)
        else:
            u = self.LoadIncrement(max, properties, edge_matrix, cos, sin, F, X, Y, N, u, L0, Fnp1, row, col)


        if DEBUG: print(max)
        # u = self.ReconstructU(u, max)
        return u

    def SolveDynamic(self, max, properties, edge_matrix, cos, sin, F, X, Y, N, u, L0, Fnp1, row, col):
        # algorithm for dynamic solution based on newmarks equations
        # max = 2
        # K = sparse.csr_matrix([[6, -2], [-2, 4]])
        # M = sparse.csr_matrix([[2, 0], [0, 1]])

        # initialize displacement, velocity, and acceleration
        d = np.zeros((max, self.num_t))
        v = np.zeros((max, self.num_t))
        a = np.zeros((max, self.num_t))
        f = np.zeros((max, self.num_t))

        # initial conditions
        d0 = np.zeros((max, 1))
        d[:, 0] = d0[:, 0]
        v0 = np.zeros((max, 1))
        v[:, 0] = v0[:, 0]
        
        # will need to redo this
        # F = np.array([[0], [10]])
        F = self.get_force(F, f)
        
        # Calculate K and M
        if TIME: start = timer()
        K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, N, row, col)
                

        # create mass matrix
        M = self.CreateMassMatrix(max, properties, edge_matrix, cos, sin, N, row, col)
        C = self.damp1 * M + self.damp2 * K
        #apply boundary conditions
        K = self.ApplyBoundaryConditions(K) 
        M = self.ApplyBoundaryConditions(M)
        C = self.ApplyBoundaryConditions(C)

        # current truss lengths
        # L = self.TrussLength(X, Y, d[:, i], edge_matrix) # will need to check this calculation and possibly make global

        # # solve for cos and sin
        # [cos, sin] = self.SolveCosSin(X, Y, d[:, i], L, edge_matrix)

        # solve for axial forces and forces
        [Ntemp, Fs] = self.SolveAxialForces(properties[0, 2], properties[0, 1], L0, L0, edge_matrix, sin, cos, max)
            

        Fprev = np.zeros(max)
        
        t = 0

        # initial conditions
        a[:, 0:1] = scipy.sparse.linalg.inv(M).dot(F[:, 0:1] - K.dot(d[:, 0:1])) #unsure how to calculate current force here
        a0 = 1/(self.b * self.dt **2)
        a1 = self.g/(self.b * self.dt)
        a2 = 1/(self.b * self.dt)
        a3 = 1/(2 * self.b) - 1
        a4 = self.g/self.b - 1
        a5 = self.dt/2 * (self.g/self.b - 2)
        a6 = self.dt * ( 1 - self.g)
        a7 = self.g * self.dt
        Khat = K + a0 * M + a1 * C
        # triangulize khat possibly
        for i in range(self.num_t - 1):
            
            R_tpdt = F[:, i + 1]
            Rhat_tpdt = R_tpdt + M * (a0 * d[:, i] + a2 * v[:, i] + a3 * a[:, i]) + C * (a1 * d[:, i] + a4 * v[:, i] + a5 * a[:, i]) # need to solve for p_ip1

            d_i = d[:, i].copy()
            # # current truss lengths
            # L = self.TrussLength(X, Y, d_i, edge_matrix) # will need to check this calculation and possibly make global

            # # solve for cos and sin
            # [cos, sin] = self.SolveCosSin(X, Y, d_i, L, edge_matrix)
            # K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, Ntemp, row, col)
            # K = self.ApplyBoundaryConditions(K)
            # [Ntemp, Fs] = self.SolveAxialForces(properties[0, 2], properties[0, 1], L0, L, edge_matrix, sin, cos, max)

            
            [dd, K, Fs] = self.NewtonRhapsonDynamic(a0,a1, M, d_i, Rhat_tpdt, Fs, K, X, Y,  edge_matrix, Ntemp, row, col, properties, L0, max, i, C)            

            d[:, i + 1] = dd

            a[:, i + 1] = a0 * (d[:, i+1] - d[:, i]) - a2 * v[:, i] - a3 * a[:, i]
            v[:, i + 1] = v[:, i] + a6 * a[:, i] + a7 * a[:, i + 1]

        return d

    def ModifiedNewtonRhapsonDynamic(self, Khat):
        for j in range(self.maxiter):
            duj = scipy.sparse.linalg.spsolve(Khat, Res)


    def NewtonRhapsonDynamic(self, a0,a1, M, d_i, Rhat_tpdt, Fs, K, X, Y, edge_matrix, Ntemp, row, col, properties, L0, max, i, C):
        d_ip1_j = d_i
        for j in range(self.maxiter):

            Res = Rhat_tpdt - Fs.ravel() - a0 * M * d_ip1_j - a1 * C * d_ip1_j
            Res = self.ApplyBoundaryConditionsForce(Res)


            # check convergence
            Rnorm = np.linalg.norm(Res)
            if Rnorm < self.tol:
                print("number iterations", j)
                print("residual:", Rnorm)
                break

            # calculate change in u
            Khat = K + a0 * M + a1 * C
            # solve for displacement
            duj = scipy.sparse.linalg.spsolve(Khat, Res)
            d_ip1_j += duj
            # recalculate K and F
            # current truss lengths
            L = self.TrussLength(X, Y, d_ip1_j, edge_matrix) # will need to check this calculation and possibly make global

            # solve for cos and sin
            [cos, sin] = self.SolveCosSin(X, Y, d_ip1_j, L, edge_matrix)
            K = self.CreateStiffnessMatrix(max, properties, edge_matrix, cos, sin, Ntemp, row, col)
            K = self.ApplyBoundaryConditions(K)
            [Ntemp, Fs] = self.SolveAxialForces(properties[0, 2], properties[0, 1], L0, L, edge_matrix, sin, cos, max)
        
        return d_ip1_j, K, Fs

    def eval(self):
        ld = True
        if ld == True:
            if TIME: start = timer()
            
            U = self.Initialize()
            if TIME: end = timer()
            print("Total Time:", end - start)
            print("Stiffness assemble time:", self.stiffAssemTime)
            print("Boundary condition time:", self.boundaryConditionAssign)
            print("Solve time:", self.solveTime)
            print("Force time:", self.forceSolveTime)
            return U

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

        # create bmesh
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
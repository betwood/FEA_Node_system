import bpy
import numpy as np
import bmesh
import math
import mathutils

from bpy.types import NodeTree, Node, NodeSocket, Object, Operator
from .node_base import NodeBase
from .node_output import UpdateObjectNodeOperator
# from ..sockets.socket_object import SocketObject
# from ..sockets.socket_matrix import SocketMatrix

DEBUG = False

class NodeGenericOutput(Node, NodeBase):

    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'OutputGeneric'
    # Label for nice name display
    bl_label = "Generic Solver Output"


    sk_index: bpy.props.IntProperty(min=1, max=10)
    vc_index: bpy.props.IntProperty(min=1, max=6)
    material: bpy.props.PointerProperty(type=bpy.types.Material)

    drop_down_items = (
        ('DISP', "Displacement", "Add the 2 values together"),
        ('STRAIN', "Strain", "Subtract the second value from the first"),
        ('STRESS', "Stress", "Subtract the second value from the first"),
    )

    output_type: bpy.props.EnumProperty(
        name = "Operation type",
        description = "The type of operation that will be used",
        items = drop_down_items,
        default = 'DISP',
    )


    
    def init(self, context):
        self.inputs.new('SocketTypeMatrix', "output")


    def draw_buttons(self, context, layout):
        if(bpy.context.active_node == self):
            layout.operator("node.evaluate", text="evaluate")
        layout.prop(self, "output_type", text="")
        layout.prop(self, "sk_index", text="shape key index")
        layout.prop(self, "vc_index", text="vertex color index")
        layout.prop(self, "material")

        # self.eval()
        # 
        # col.prop_search(self, "object", context.scene, "objects")
        pass

    def draw_buttons_ext(self, context, layout):
        layout.operator("node.evaluate", text="evaluate")
        layout.operator("node.add_material", text="materials")
        layout.label(text=str(self.object))
        pass

    def get_object(self):
        print(self.object)
        return self.object

    def update_value(self, context):
        print("updated")

    def add_material(self):
        mat_name = self.material.name
        print(mat_name)

        material = bpy.data.materials.get(mat_name)
        material.use_nodes = True
        nodes = material.node_tree.nodes

        # add colorramp node
        color_ramp = nodes.new("ShaderNodeValToRGB")
        color_ramp.color_ramp.elements[0].color = (0, 0, 1, 1)
        color_ramp.color_ramp.elements.new(.5)
        color_ramp.color_ramp.elements[1].color = (0, 1, 0, 1)
        color_ramp.color_ramp.elements[2].color = (1, 0, 0, 1)
        color_ramp.location = (-500, 300)

        mix = nodes.new("ShaderNodeMixRGB")
        mix.location = (-700, 300)
        material.node_tree.links.new(color_ramp.inputs[0], mix.outputs[0])



        vc_1 = nodes.new("ShaderNodeAttribute")
        # basis vertex color group (might make selectible later)
        vc_1.attribute_name = "col"
        vc_1.location = (-900, 400)
        material.node_tree.links.new(mix.inputs[1], vc_1.outputs[0])

        self.object.data.vertex_colors.active_index = self.vc_index
        vc = self.object.data.vertex_colors.active
        vc_2 = nodes.new("ShaderNodeAttribute")
        vc_2.attribute_name = vc.name
        vc_2.location = (-900, 200)
        material.node_tree.links.new(mix.inputs[2], vc_2.outputs[0])


        path = 'nodes["' + mix.name + '"].inputs[0].default_value'
        driver = material.node_tree.driver_add(path).driver
        driver.type = 'SCRIPTED'

        var = driver.variables.new()
        var.targets[0].id_type = 'KEY'
        var.targets[0].id = bpy.data.shape_keys["Key"]
        self.object.active_shape_key_index = self.sk_index
        sk = self.object.active_shape_key
        key = sk.name
        var.targets[0].data_path = 'key_blocks["' + key + '"].value'

        driver.expression = "var"

    def absmax(self, a, axis=None):
        amax = a.max(axis)
        amin = a.min(axis)
        return np.where(-amin > amax, amin, amax)

    def eval(self):
        # set sockets
        input_socket_1 = self.inputs[0]

        # get inputs from previous nodes
        U = self.get_value(input_socket_1)
        # print(U)

        # get basis shape key
        self.object.active_shape_key_index = 0
        sk_basis = self.object.active_shape_key

        # make active shape key the index value and select it
        self.object.active_shape_key_index = self.sk_index
        sk = self.object.active_shape_key


        #apply to mesh
        for i in range(len(self.object.data.vertices)):
            if self.solve_type == "1DFRAME":
                sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[6*i], U[6*i+1], U[6*i+2]))
            else:
                sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[3*i], U[3*i+1], U[3*i+2]))
        
        # make base vertex color
        self.object.data.vertex_colors.active_index = 0
        vc_base = self.object.data.vertex_colors.active

        # make active shape key the index value and select it
        self.object.data.vertex_colors.active_index = self.vc_index
        vc = self.object.data.vertex_colors.active

        if self.solve_type == "1DFRAME":
            disp = np.zeros(int(len(U) / 6))
            rot = np.zeros(int(len(U) / 6))
            for i in range(len(disp)):
                disp[i] = math.sqrt(U[6 * i] ** 2 + U[6 * i + 1] ** 2 + U[6 * i + 2] ** 2)
                rot[i] = math.sqrt(U[6 * i + 3] ** 2 + U[6 * i + 4] ** 2 + U[6 * i + 5] ** 2)
        else:
            disp = np.zeros(int(len(U) / 3))
            for i in range(len(disp)):
                disp[i] = math.sqrt(U[3 * i] ** 2 + U[3 * i + 1] ** 2 + U[3 * i + 2] ** 2)


        bm = bmesh.new()
        bm.from_mesh(self.object.data)

        strain = np.zeros(int(len(disp)))
        node = np.zeros(int(len(disp)))
        connections = np.zeros(int(len(disp)))
        i=0
        for edge in bm.edges:
            node1 = edge.verts[0].index
            node2 = edge.verts[1].index
            node[node1] += (disp[node1] - disp[node2]) / edge.calc_length()
            node[node2] += (disp[node2] - disp[node1]) / edge.calc_length()
            connections[node1] += 1
            connections[node2] += 1
            i += 1
        strain = node / connections

        i = 0
        #apply to mesh
        for poly in self.object.data.polygons:
            for loop_index in poly.loop_indices:
                # sk.data[i].co = sk_basis.data[i].co + mathutils.Vector((U[3*i], U[3*i+1], U[3*i+2]))
                loop = self.object.data.loops[loop_index]
                v = loop.vertex_index

                if self.output_type == "DISP":
                    color_value = disp[v] / self.absmax(disp)
                elif self.output_type == "STRESS":
                    color_value = disp[v] / self.absmax(disp)
                elif self.output_type == "STRAIN":
                    color_value = strain[v] / self.absmax(strain)
                    
                color = (color_value, color_value, color_value, 1.0)
                vc.data[loop_index].color = color
                
                vc_base.data[loop_index].color = (0.0, 0.0, 0.0, 1.0)
        return

class UpdateMaterialNodeOperator(Operator):
    """Add a simple box mesh"""
    bl_idname = "node.add_material"
    bl_label = "add nodes to material"

    # node_group_name = StringProperty()

    node = None

    def execute(self, context):
        # node = [i for i in bpy.data.node_groups[bpy.context.space_data.node_tree.name].nodes if i.bl_idname == "Outputnode"][0]
        node = context.active_node
        node.add_material()
        return{'FINISHED'}
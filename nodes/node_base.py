import bpy
from bpy.props import *

class NodeBase:

    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == 'FEATreeType'

    # === Basics ===
    # Description string
    '''Base Node'''
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'BaseNodeType'
    # Label for nice name display
    bl_label = "Base Node"
    # Icon identifier
    bl_icon = 'SOUND'

        # === Custom Properties ===
    # These work just like custom properties in ID data blocks
    # Extensive information can be found under
    # http://wiki.blender.org/index.php/Doc:2.6/Manual/Extensions/Python/Properties
    object: bpy.props.PointerProperty(type=bpy.types.Object)

    # === Optional Functions ===
    # Initialization function, called when a new node is created.
    # This is the most common place to create the sockets for a node, as shown below.
    # NOTE: this is not the same as the standard __init__ function in Python, which is
    #       a purely internal Python method and unknown to the node system!
    def init(self, context):
        # self.inputs.new('CustomSocketType', "Hello")
        # self.inputs.new('NodeSocketFloat', "World")
        # self.inputs.new('NodeSocketVector', "!")

        # self.outputs.new('NodeSocketColor', "How")
        # self.outputs.new('NodeSocketColor', "are")
        # self.outputs.new('NodeSocketFloat', "you")
        pass

    # Copy function to initialize a copied node from an existing one.
    def copy(self, node):
        print("Copying from node ", node)

    # Free function to clean up on removal.
    def free(self):
        print("Removing node ", self, ", Goodbye!")

    # Additional buttons displayed on the node.
    def draw_buttons(self, context, layout):
        # layout.label(text="Node settings")
        # layout.prop(self, "my_float_prop")
        pass

    # Detail buttons in the sidebar.
    # If this function is not defined, the draw_buttons function is used instead
    def draw_buttons_ext(self, context, layout):
        # layout.prop(self, "my_float_prop")
        # # my_string_prop button will only be visible in the sidebar
        # layout.prop(self, "my_string_prop")
        layout.label(text=str(self.object))
        pass

    def get_value(self, socket, type="default"):
        if not socket.is_linked:
            val = socket.default_value
        else:
            val = socket.links[0].from_node.eval()
            # if not type == "matrix":
            #     socket.links[0].from_node.eval()
            #     val = socket.links[0].from_socket.default_value
            # else:
                
        return val

    def set_object(self, socket, object, type):
        if type == "in":
            from_node = socket.links[0].from_node
            from_node.object = object
            for input in self.inputs:
                print(input)
                if input.is_linked:
                    from_node.set_object(input, self.object, "in")
        if type == "out":
            to_node = socket.links[0].to_node
            to_node.object = object
            for output in self.outputs:
                if output.is_linked:
                    to_node.set_object(output, self.object, "out")
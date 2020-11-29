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
    object: bpy.props.PointerProperty(type=bpy.types.Object) # object operation is being run on
    solve_type: bpy.props.StringProperty(default="test") # used for 

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
        layout.label(text=self.solve_type)
        pass

    def get_value(self, socket, type="default"):
        
        # gets value from socket if linked gets it from the node otherwise gets the socket default value
        if not socket.is_linked:
            val = socket.default_value
        else:
            val = socket.links[0].from_node.eval()
                
        return val

    def set_object(self, socket, object, type, solve_type):
        # takes the input nodes and sets object and solve type on them
        if type == "in":
            
            # get object and solve type from link
            from_node = socket.links[0].from_node
            from_node.object = object
            from_node.solve_type = solve_type
            
            # take inputs and send object and solve type to these nodes
            for input in self.inputs:
                if input.is_linked: # check if input socket is connected to another node
                    from_node.set_object(input, self.object, "in", self.solve_type)
        
        # take the output nodes and set objects and solve type
        if type == "out":
            for link in socket.links:
                
                # get object and solve type from node
                to_node = link.to_node
                to_node.object = object
                to_node.solve_type = solve_type

                # send object and solve type to output nodes
                for output in self.outputs:
                    if output.is_linked:
                        to_node.set_object(output, self.object, "out", self.solve_type)

    def update_value(self, context):
        self.set_inputs()
        return None

    def set_inputs(self):
        # print("1")
        pass
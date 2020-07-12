import bpy
from bpy.types import NodeTree, Node, NodeSocket

# Derived from the NodeTree base type, similar to Menu, Operator, Panel, etc.
class FEATree(NodeTree):
    # Description string
    '''A custom node tree type that will show up in the editor type list'''
    # Optional identifier string. If not explicitly defined, the python class name is used.
    bl_idname = 'FEATreeType'
    # Label for nice name display
    bl_label = "FEA nodes"
    # Icon identifier
    bl_icon = 'OUTLINER_OB_LATTICE'
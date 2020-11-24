# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# Sources used:
# http://what-when-how.com/the-finite-element-method/
# https://en.wikipedia.org/wiki/List_of_second_moments_of_area
# A first course in the finite element method by Daryl L. Logan (fourth edition)
# MATLAB guide to finite elements an interactive approach by Peter Kattan (second edition)
# Development of Membrane, Plate and Flat Shell Elements in Java by Kaushalkumar Kansara
# Schuster Engineering youtube channel FEA course playlist


bl_info = {
    "name" : "FEA_nodes",
    "author" : "Ben Wood",
    "description" : "",
    "blender" : (2, 80, 0),
    "version" : (1, 0, 0),
    "location" : "",
    "warning" : "",
    "category" : "Generic"
}


from . import auto_load
import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem

auto_load.init()

import bpy
import subprocess

py_exec = bpy.app.binary_path_python
# ensure pip is installed
subprocess.call([
    str(py_exec),
    "-m",
    "ensurepip",
    "--user"
])
# update pip
subprocess.call([
    str(py_exec),
    "-m",
    "pip",
    "install",
    "--upgrade",
    "pip"
])
# install packages
subprocess.call([
    str(py_exec),
    "-m",
    "pip",
    "install",
    "scipy",
    "-t .",
])

class MyNodeCategory(NodeCategory):
    @classmethod
    def poll(cls, context):
        return context.space_data.tree_type == 'FEATreeType'


# all categories in a list
node_categories = [
    # identifier, label, items list
    MyNodeCategory('BASICNODES', "Basic Nodes", items=[
        # our basic node
        
        
        NodeItem("MathNode"),
        
        NodeItem("VectorMathNode"),
        
        NodeItem("CombineBCSNode"),
        NodeItem("CombineForcesNode"),
    ]),
    # static nodes
    MyNodeCategory('INPUTS', "inputs", items=[
        NodeItem("BCNode"),
        NodeItem("ForceNode"),
        NodeItem("InputMaterialNode"),
        NodeItem("InputObjectNode"),
        NodeItem("VectorInputNode"),
        NodeItem("InputNode"),
        NodeItem("AreaInputNode"),
    ]),
    # outputs
    MyNodeCategory('OUTPUTS', "outputs", items=[
        NodeItem("OutputShapekeyNode"),
        NodeItem("OutputVertexColorNode"),
        NodeItem("OutputGeneric"),
        NodeItem("OutputNode"),
    ]),
    # solvers
    MyNodeCategory('SOLVERS', "solvers", items=[
        NodeItem("SpaceTrussNode"),
        NodeItem("SpaceFrameNode"),
        NodeItem("PlaneTrussNodeDynamic"),
        NodeItem("SpaceTrussNodeDynamic"),
        NodeItem("SpaceFrameNodeDynamic"),
    ]),
]


def register():
    auto_load.register()
    nodeitems_utils.register_node_categories('FEA_NODES_categories', node_categories)

def unregister():
    auto_load.unregister()
    nodeitems_utils.unregister_node_categories('FEA_NODES_categories')

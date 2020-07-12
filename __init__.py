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

bl_info = {
    "name" : "FEA_nodes",
    "author" : "Ben Wood",
    "description" : "",
    "blender" : (2, 80, 0),
    "version" : (0, 0, 1),
    "location" : "",
    "warning" : "",
    "category" : "Generic"
}


from . import auto_load
import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem

auto_load.init()

class MyNodeCategory(NodeCategory):
    @classmethod
    def poll(cls, context):
        return context.space_data.tree_type == 'FEATreeType'


# all categories in a list
node_categories = [
    # identifier, label, items list
    MyNodeCategory('BASICNODES', "Basic Nodes", items=[
        # our basic node
        NodeItem("InputNode"),
        NodeItem("OutputNode"),
        NodeItem("OperationNode"),
        NodeItem("VectorInputnode"),
        NodeItem("VectorOperationNode"),
    ]),
    # static nodes
    MyNodeCategory('STATICSTRUCTURAL', "static structural", items=[
        NodeItem("InputObjectnode"),
        NodeItem("BCNode"),
        NodeItem("ForceNode"),
        NodeItem("CombineBCSNode"),
        NodeItem("CombineForcesNode"),
        NodeItem("OutputShapekeynode"),
    ]),
    # solvers
    MyNodeCategory('SOLVERS', "solvers", items=[
        NodeItem("SpaceFrameNode"),
        NodeItem("SpaceTrussNode"),
    ]),
]


def register():
    auto_load.register()
    nodeitems_utils.register_node_categories('FEA_NODES', node_categories)

def unregister():
    auto_load.unregister()
    nodeitems_utils.unregister_node_categories('FEA_NODES')

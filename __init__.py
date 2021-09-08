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
# An optimized method for calculating the linear and nonlinear response of SDOF system subjected to an arbitrary base excitation by Hossein Kabir and Mojtaba Sadeghi
# Dynamics of structures by Chopra
# MIT open courseware nonlinear analysis
# Finite element procedures by Bathe


bl_info = {
    "name" : "FEA_nodes",
    "author" : "Ben Wood",
    "description" : "",
    "blender" : (2, 93, 0),
    "version" : (1, 0, 0),
    "location" : "",
    "warning" : "Requires installation of dependencies (scipy)",
    "category" : "Generic"
}


from . import auto_load
import os
import sys
import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem
import bpy
import subprocess
import importlib
from collections import namedtuple

Dependency = namedtuple("Dependency", ["module", "package", "name"])

# Declare all modules that this add-on depends on, that may need to be installed. The package and (global) name can be
# set to None, if they are equal to the module name. See import_module and ensure_and_import_module for the explanation
# of the arguments. DO NOT use this to import other parts of your Python add-on, import them as usual with an
# "import" statement.
dependencies = (Dependency(module="scipy", package=None, name=None),)

dependencies_installed = False


def import_module(module_name, global_name=None, reload=True):
    """
    Import a module.
    :param module_name: Module to import.
    :param global_name: (Optional) Name under which the module is imported. If None the module_name will be used.
       This allows to import under a different name with the same effect as e.g. "import numpy as np" where "np" is
       the global_name under which the module can be accessed.
    :raises: ImportError and ModuleNotFoundError
    """
    if global_name is None:
        global_name = module_name

    if global_name in globals():
        importlib.reload(globals()[global_name])
    else:
        # Attempt to import the module and assign it to globals dictionary. This allow to access the module under
        # the given name, just like the regular import would.
        globals()[global_name] = importlib.import_module(module_name)


def install_pip():
    """
    Installs pip if not already present. Please note that ensurepip.bootstrap() also calls pip, which adds the
    environment variable PIP_REQ_TRACKER. After ensurepip.bootstrap() finishes execution, the directory doesn't exist
    anymore. However, when subprocess is used to call pip, in order to install a package, the environment variables
    still contain PIP_REQ_TRACKER with the now nonexistent path. This is a problem since pip checks if PIP_REQ_TRACKER
    is set and if it is, attempts to use it as temp directory. This would result in an error because the
    directory can't be found. Therefore, PIP_REQ_TRACKER needs to be removed from environment variables.
    :return:
    """

    try:
        # Check if pip is already installed
        subprocess.run([sys.executable, "-m", "pip", "--version"], check=True)
    except subprocess.CalledProcessError:
        import ensurepip

        ensurepip.bootstrap()
        os.environ.pop("PIP_REQ_TRACKER", None)


def install_and_import_module(module_name, package_name=None, global_name=None):
    """
    Installs the package through pip and attempts to import the installed module.
    :param module_name: Module to import.
    :param package_name: (Optional) Name of the package that needs to be installed. If None it is assumed to be equal
       to the module_name.
    :param global_name: (Optional) Name under which the module is imported. If None the module_name will be used.
       This allows to import under a different name with the same effect as e.g. "import numpy as np" where "np" is
       the global_name under which the module can be accessed.
    :raises: subprocess.CalledProcessError and ImportError
    """
    if package_name is None:
        package_name = module_name

    if global_name is None:
        global_name = module_name

    # Blender disables the loading of user site-packages by default. However, pip will still check them to determine
    # if a dependency is already installed. This can cause problems if the packages is installed in the user
    # site-packages and pip deems the requirement satisfied, but Blender cannot import the package from the user
    # site-packages. Hence, the environment variable PYTHONNOUSERSITE is set to disallow pip from checking the user
    # site-packages. If the package is not already installed for Blender's Python interpreter, it will then try to.
    # The paths used by pip can be checked with `subprocess.run([bpy.app.binary_path_python, "-m", "site"], check=True)`

    # Create a copy of the environment variables and modify them for the subprocess call
    environ_copy = dict(os.environ)
    environ_copy["PYTHONNOUSERSITE"] = "1"

    subprocess.run([sys.executable, "-m", "pip", "install", package_name], check=True, env=environ_copy)

    # The installation succeeded, attempt to import the module again
    import_module(module_name, global_name)


class FEA_PT_Panel(bpy.types.Panel):
    bl_idname = "FEA_PT_Panel"
    bl_label = "External python packages"
    bl_category = "FEA_nodes"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout

        for dependency in dependencies:
            if dependency.name is None and hasattr(globals()[dependency.module], "__version__"):
                layout.label(text=f"{dependency.module} {globals()[dependency.module].__version__}")
            elif hasattr(globals()[dependency.name], "__version__"):
                layout.label(text=f"{dependency.module} {globals()[dependency.name].__version__}")
            else:
                layout.label(text=f"{dependency.module}")


class FEA_PT_warning_panel(bpy.types.Panel):
    bl_idname = "FEA_PT_warning_panel"
    bl_label = "External python packages"
    bl_category = "FEA_nodes"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    @classmethod
    def poll(self, context):
        return not dependencies_installed

    def draw(self, context):
        layout = self.layout

        lines = [f"Please install the missing dependencies for the \"{bl_info.get('name')}\" add-on.",
                 f"1. Open the preferences (Edit > Preferences > Add-ons).",
                 f"2. Search for the \"{bl_info.get('name')}\" add-on.",
                 f"3. Open the details section of the add-on.",
                 f"4. Click on the \"{FEA_OT_install_dependencies.bl_label}\" button.",
                 f"   This will download and install the missing Python packages, if Blender has the required",
                 f"   permissions."]

        for line in lines:
            layout.label(text=line)


class FEA_OT_install_dependencies(bpy.types.Operator):
    bl_idname = "example.install_dependencies"
    bl_label = "Install dependencies"
    bl_description = ("Downloads and installs the required python packages for this add-on. "
                      "Internet connection is required. Blender may have to be started with "
                      "elevated permissions in order to install the package")
    bl_options = {"REGISTER", "INTERNAL"}

    @classmethod
    def poll(self, context):
        # Deactivate when dependencies have been installed
        return not dependencies_installed

    def execute(self, context):
        try:
            install_pip()
            for dependency in dependencies:
                install_and_import_module(module_name=dependency.module,
                                          package_name=dependency.package,
                                          global_name=dependency.name)
        except (subprocess.CalledProcessError, ImportError) as err:
            self.report({"ERROR"}, str(err))
            return {"CANCELLED"}

        global dependencies_installed
        dependencies_installed = True

        # Register the panels, operators, etc. since dependencies are installed
        for cls in classes:
            bpy.utils.register_class(cls)

        return {"FINISHED"}


class FEA_preferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    def draw(self, context):
        layout = self.layout
        layout.operator(FEA_OT_install_dependencies.bl_idname, icon="CONSOLE")


preference_classes = (FEA_PT_warning_panel,
                      FEA_OT_install_dependencies,
                      FEA_preferences)




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
        # NodeItem("OutputVertexColorNode"),
        NodeItem("OutputGeneric"),
        #NodeItem("OutputNode"),
        # NodeItem("OutputValueNode")
    ]),
    # solvers
    MyNodeCategory('SOLVERS', "solvers", items=[
        NodeItem("SpaceTrussNode"),
        NodeItem("SpaceFrameNode"),
        NodeItem("PlaneTrussNodeDynamic"),
        NodeItem("SpaceTrussNodeDynamic"),
        NodeItem("SpaceFrameNodeDynamic"),
        # NodeItem("TriShellNode"),
        # NodeItem("QuadShellNode"),
        NodeItem("QuadBilinearNode"),
        # NodeItem("BilinearQuadContact")
    ]),
]


def register():
    
    global dependencies_installed
    dependencies_installed = False

    for cls in preference_classes:
        bpy.utils.register_class(cls)

    print("preferences installed")
    try:
        for dependency in dependencies:
            import_module(module_name=dependency.module, global_name=dependency.name)
        
        dependencies_installed = True
    except ModuleNotFoundError:
        # Don't register other panels, operators etc.
        return

    bpy.utils.register_class(FEA_PT_Panel)

    auto_load.init()
    auto_load.register()
    try:
        nodeitems_utils.register_node_categories('FEA_NODES_categories', node_categories)
    except:
        nodeitems_utils.unregister_node_categories('FEA_NODES_categories')
        nodeitems_utils.register_node_categories('FEA_NODES_categories', node_categories)


def unregister():
    for cls in preference_classes:
        bpy.utils.unregister_class(cls)

    bpy.utils.unregister_class(FEA_PT_Panel)

    if dependencies_installed:
        auto_load.unregister()
        nodeitems_utils.unregister_node_categories('FEA_NODES_categories')
    

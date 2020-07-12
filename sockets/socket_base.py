import bpy
from bpy.props import *

class SocketBase:
    default_prop: StringProperty()

    def init(self, default_prop=""):
        self.default_prop = default_prop

    def draw_color(self, context, node):
        return self.color

    def draw_layout(self, context, layout, node, text):
        layout.prop(node, self.default_prop, text=text)

    def draw(self, context, layout, node, text):
        layout.label(text=text)
        # if self.is_output:
        #     layout.label(text=text)
        # else:
        #     if self.is_linked:
        #         layout.label(text=text)
        #     else:
        #         draw_layout(context, layout, node, text)
        

    def set_value(self, value):
        self.default_value = value
import sys, os
from argparse import ArgumentParser
import numpy as np

from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectGui import *
from direct.task import Task

from data_parser import *
from utils import ravel_to_grid

class SimplifiedRig():
    def __init__(self, rig_origin, head_node, left_hand_node, right_hand_node, left_aim_node, right_aim_node):
        self._rig_origin = rig_origin
        self._head_node = head_node
        self._left_hand_node = left_hand_node
        self._right_hand_node = right_hand_node
        self._left_aim_node = left_aim_node
        self._right_aim_node = right_aim_node

    def get_rig_pose(self):
        return self._rig_origin.getPos(), self._rig_origin.getQuat()

    def get_head_pose(self):
        return self._head_node.getPos(), self._head_node.getQuat()
    
    def get_left_hand_pose(self):
        return self._left_hand_node.getPos(), self._left_hand_node.getQuat()
    
    def get_right_hand_pose(self):
        return self._right_hand_node.getPos(), self._right_hand_node.getQuat()
    
    def get_left_aim_pose(self):
        return self._left_aim_node.getPos(), self._left_aim_node.getQuat()
    
    def get_right_aim_pose(self):
        return self._right_aim_node.getPos(), self._right_aim_node.getQuat()

class VarianceVis(ShowBase):

    def __init__(self, data_path, use_openxr=False):

        # Imports here based on flag, since they both modify panda3d config file.
        # TODO: Find a better way to handle config file, independent of imports
        if use_openxr:
            from p3dopenxr.p3dopenxr import P3DOpenXR
        else:
            from davis import DaVis

        ShowBase.__init__(self)

        self.vis_scale = 1.0
        self.initial_zoom_level = 5.0

        if use_openxr:
            # Create VR backend
            self.openxr = P3DOpenXR()
            self.openxr.init()

            # Setup rig
            self.rig = SimplifiedRig(
                rig_origin = self.openxr.tracking_space_anchor,
                head_node = self.openxr.hmd_anchor,
                left_hand_node = self.openxr.left_hand_anchor,
                right_hand_node = self.openxr.right_hand_anchor,
                left_aim_node = self.openxr.left_aim_anchor,
                right_aim_node = self.openxr.right_aim_anchor)
            
            # TODO: Create input handlers
            self.openxr.action_set.add_callback("/user/hand/left/input/trigger", self.on_left_trigger)
            self.openxr.action_set.add_callback("/user/hand/right/input/trigger", self.on_right_trigger)
            self.openxr.action_set.add_callback("/user/hand/left/input/thumbstick", self.on_left_analog)
            self.openxr.action_set.add_callback("/user/hand/right/input/thumbstick", self.on_right_analog)

        else:
            # Create VR backend
            self.davis = DaVis()

            # Setup rig
            self.rig = SimplifiedRig(
                rig_origin = self.davis.rig,
                head_node = self.davis.head,
                left_hand_node = self.davis.flystick8,
                right_hand_node = self.davis.flystick9,
                left_aim_node=self.davis.flystick8,
                right_aim_node=self.davis.flystick9)
            
            # TODO: Create input handlers
            self.davis.add_input_callback("left_trigger", self.on_left_trigger)
            self.davis.add_input_callback("right_trigger", self.on_right_trigger)
            self.davis.add_input_callback("left_analog", self.on_left_analog)
            self.davis.add_input_callback("right_analog", self.on_right_analog)

        # Clear color
        base.win.setClearColorActive(True)
        base.win.setClearColor(VBase4(0.0, 0.0, 0.0, 1.0))

        # Create pointer and ray casting setup
        self.left_pointer = self.create_pointer(self.rig._left_aim_node, use_openxr)
        self.pointer_collision_node = self.left_pointer.attachNewNode(CollisionNode('left_pointer_collision_node'))
        self.pointer_collision_node.node().add_solid(CollisionRay())
        self.pointer_collision_mask = BitMask32.bit(1)
        self.pointer_collision_node.node().setFromCollideMask(self.pointer_collision_mask)
        self.pointer_collision_handler = CollisionHandlerQueue()

        # Initialize slice data format
        self.init_slice_data()

        # Parse input data
        h, res, objs = parse_data(data_path)
        self.grid_res = res
        self.grid_spacing = h
        self.scene_objects = objs

        # Create slice geometry
        self.create_slices()

        # Create GUI
        # self.create_gui()
        self.create_3D_gui()

        # Load reconstructed model
        reconstructed_model_path = os.path.join("..", os.path.join(os.path.dirname(data_path), "reconstructed.obj"))
        self.reconstructed_model = loader.loadModel(reconstructed_model_path)
        self.reconstructed_model.setName("reconstructed_model")
        self.reconstructed_model.reparentTo(self.vis_node)

        # Scale and position reconstructed model
        max_size = res[0]
        grid_size = res[0] * h[0]
        model_scale = grid_size / max_size
        self.reconstructed_model.setScale(model_scale)
        # TODO: Figure out why the model is offset like this in z axis
        self.reconstructed_model.setPos(-grid_size / 2.0, grid_size / 2.0, -grid_size / 2.0 + h[2] * 30.0)
        quat = LQuaternionf()
        quat.setFromAxisAngle(-90.0, Vec3(0.0, 0.0, 1.0))
        self.reconstructed_model.setQuat(quat)
        self.reconstructed_model.hide()
        self.reconstructed_model.setColor(0.6, 0.6, 0.6, 1.0)

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.2, 0.2, 1))
        alnp = render.attachNewNode(alight)
        render.setLight(alnp)

        dlight = DirectionalLight('dlight')
        dlight.setColor((0.8, 0.8, 0.5, 1))
        dlnp = render.attachNewNode(dlight)
        dlnp.setHpr(0, 60, 0)
        render.setLight(dlnp)

        # Navigationa and interaction variables
        self.transformation_started = False
        self.hand_transformation_anchor = Vec3()
        self.vis_transformation_anchor = Vec3()
        self.hand_orientation_anchor = LQuaternion()
        self.vis_orientation_anchor = LQuaternion()

        self.rotation_speed = 200.0  # degrees per meter of hand movement
        self.zoom_speed = 3.0
        
        self.slice_offset = 0.0
        self.slice_speed = 1.0

        self.last_update_time = 0.0
        taskMgr.add(self.update_task, "update_task")

    """ Update task for handling navigation and interaction """
    def update_task(self, task):
        delta_time = task.time - self.last_update_time
        self.last_update_time = task.time

        if self.transformation_started:
            current_pos = self.rig.get_right_hand_pose()[0]
            delta = current_pos - self.hand_transformation_anchor

            new_pos = self.vis_transformation_anchor + Vec3(0.0, delta.getY() * self.vis_scale * self.zoom_speed, 0.0)
            self.vis_node.setPos(new_pos)

            # Rotations
            new_orientation = LQuaternion(self.vis_orientation_anchor)

            # Yaw/Pitch
            if delta.getX() != 0.0 or delta.getZ() != 0.0:
                rot_axis = Vec3(delta.getZ(), 0.0, -delta.getX())
                rot_strength = rot_axis.length()
                rot_axis.normalize()

                delta_rotation = LQuaternion()
                delta_rotation.setFromAxisAngle(rot_strength * self.rotation_speed, rot_axis)

                new_orientation *= delta_rotation

            # Measure roll
            delta_orientation = LQuaternion()
            delta_orientation = self.rig.get_right_hand_pose()[1] * self.hand_orientation_anchor.conjugate()
            right_xformed = delta_orientation.getRight()
            roll_angle = np.arctan2(right_xformed.getZ(), right_xformed.getX())  # roll around Y axis

            if roll_angle != 0.0:
                roll_rotation = LQuaternion()
                roll_rotation.setFromAxisAngle(-np.degrees(roll_angle), Vec3(0.0, 1.0, 0.0))
                new_orientation *= roll_rotation

            self.vis_node.setQuat(new_orientation)

        return Task.cont

    def init_slice_data(self):
        array = GeomVertexArrayFormat()
        array.addColumn("vertex", 3, Geom.NTFloat32, Geom.CPoint)
        array.addColumn("texcoord", 2, Geom.NTFloat32, Geom.CTexcoord)

        format = GeomVertexFormat()
        format.addArray(array)
        self.slice_format = GeomVertexFormat.registerFormat(format)

        # Load slice shader
        self.slice_shader = Shader.load(Shader.SL_GLSL,
            vertex="slice.vert",
            fragment="slice.frag")

    # vertices - (N, 3) array of vertex positions of the slice
    # colors - (N, 4) array of RGBA colors for each vertex
    # Res - resolution of grid (res, res, res)
    def create_slice_geometry(self, name, vertices, colors, res):
        slice_data = GeomVertexData('slice', self.slice_format, Geom.UHStatic)

        num_vertices = vertices.shape[0]
        slice_data.setNumRows(num_vertices)

        vertex_writer = GeomVertexWriter(slice_data, 'vertex')
        uv_writer = GeomVertexWriter(slice_data, 'texcoord')

        for i in range(num_vertices):
            vertex_writer.addData3(*vertices[i])
            u = (i % res) / (res - 1)
            v = (i // res) / (res - 1)
            uv_writer.addData2(u, v)

        prim = GeomTriangles(Geom.UHStatic)
        for j in range(res - 1):
            for i in range(res - 1):
                idx = j * res + i
                prim.addVertices(idx, idx + res, idx + 1)
                prim.addVertices(idx + 1, idx + res, idx + res + 1)
        prim.closePrimitive()

        slice_geom = Geom(slice_data)
        slice_geom.addPrimitive(prim)

        slice_node = GeomNode(name)
        slice_node.addGeom(slice_geom)

        slice_node.setIntoCollideMask(self.pointer_collision_mask)

        return slice_node

    def create_slices(self):
        self.vis_node = self.render.attachNewNode("visualization_root")
        self.vis_node.setScale(self.vis_scale, self.vis_scale, self.vis_scale)

        self.vis_node.setPos(0.0, self.initial_zoom_level, 0.0)

        for (name, vertices, colors) in self.scene_objects:
            # Master node
            slices_node = self.vis_node.attachNewNode(name)

            # Create uniform buffer with color data
            colors = colors.reshape(*list(self.grid_res), 3).transpose(0, 2, 1, 3).reshape(-1, 3)

            padded_colors = np.pad(colors, (0, 1), 'constant', constant_values=1.0)
            color_buffer = ShaderBuffer("color_buffer", padded_colors, GeomEnums.UH_static)

            # Slices
            size = self.grid_res[0]
            slice_const = np.zeros((size * size), dtype=np.int32)
            slice_plane_u = np.expand_dims(np.arange(start=0, stop=size), 1)
            slice_plane_u = slice_plane_u.repeat(size, axis=1).flatten().astype(np.int32)
            slice_plane_v = np.expand_dims(np.arange(start=0, stop=size), 0).repeat(size, axis=0).flatten().astype(np.int32)

            # Slice X (YZ plane)
            x_slice_indices = np.stack((slice_const, slice_plane_u, slice_plane_v), axis=-1)
            x_slice_flat_indices = ravel_to_grid(x_slice_indices, (size, size, size))
            x_slice_node = self.create_slice_geometry(name + "_slice_x", vertices[x_slice_flat_indices], colors, size)
            x_slice_np = self.render.attachNewNode(x_slice_node)
            x_slice_np.setTwoSided(True)
            x_slice_np.set_light_off(1)
            x_slice_np.reparentTo(slices_node)
            x_slice_np.hide()

            x_slice_np.setShader(self.slice_shader)
            x_slice_np.setShaderInput("color_buffer", color_buffer)
            x_slice_np.setShaderInput("slice_offset", 0.0)
            x_slice_np.setShaderInput("grid_res", int(self.grid_res[0]))
            x_slice_np.setShaderInput("axis", 0)

            # Slice Y (XZ plane)
            y_slice_indices = np.stack((slice_plane_u, slice_const, slice_plane_v), axis=-1)
            y_slice_flat_indices = ravel_to_grid(y_slice_indices, (size, size, size))
            y_slice_node = self.create_slice_geometry(name + "_slice_y", vertices[y_slice_flat_indices], colors, size)
            y_slice_np = self.render.attachNewNode(y_slice_node)
            y_slice_np.setTwoSided(True)
            y_slice_np.set_light_off(1)
            y_slice_np.reparentTo(slices_node)
            y_slice_np.hide()

            y_slice_np.setShader(self.slice_shader)
            y_slice_np.setShaderInput("color_buffer", color_buffer)
            y_slice_np.setShaderInput("slice_offset", 0.0)
            y_slice_np.setShaderInput("grid_res", int(self.grid_res[1]))
            y_slice_np.setShaderInput("axis", 1)

            # Slice Z (XY plane)
            z_slice_indices = np.stack((slice_plane_u, slice_plane_v, slice_const), axis=-1)
            z_slice_flat_indices = ravel_to_grid(z_slice_indices, (size, size, size))
            z_slice_node = self.create_slice_geometry(name + "_slice_z", vertices[z_slice_flat_indices], colors, size)
            z_slice_np = self.render.attachNewNode(z_slice_node)
            z_slice_np.setTwoSided(True)
            z_slice_np.set_light_off(1)
            z_slice_np.reparentTo(slices_node)
            z_slice_np.hide()

            z_slice_np.setShader(self.slice_shader)
            z_slice_np.setShaderInput("color_buffer", color_buffer)
            z_slice_np.setShaderInput("slice_offset", 0.0)
            z_slice_np.setShaderInput("grid_res", int(self.grid_res[2]))
            z_slice_np.setShaderInput("axis", 2)

    def hit_test_framesize(self, widget, gui_x, gui_y):
        center = widget.getPos()
        width = widget.getWidth() * widget.getScale()[0]
        height = widget.getHeight() * widget.getScale()[2]

        return (center[0] - width / 2.0 <= gui_x <= center[0] + width / 2.0) and \
               (center[2] - height / 2.0 <= gui_y <= center[2] + height / 2.0)

    class Widget3D():
        def __init__(self, name, root_node, texture, buffer, card):
            self.name = name
            self.root_node = root_node
            self.texture = texture
            self.buffer = buffer
            self.card = card

    def create_3d_widget(self, name):
        # Create render texture
        ui_tex_size = (1024, 1024)
        ui_tex = Texture()
        ui_tex.setup_2d_texture(
            ui_tex_size[0], ui_tex_size[1], Texture.T_unsigned_byte, Texture.F_rgba8)
        
        fb_props = FrameBufferProperties()
        fb_props.set_rgba_bits(8, 8, 8, 8)
        fb_props.set_depth_bits(1)

        win_props = WindowProperties.size(*ui_tex_size)

        ui_buffer = self.graphicsEngine.make_output(
            self.pipe, name, -2,
            fb_props, win_props,
            GraphicsPipe.BF_refuse_window,
            self.win.get_gsg(), self.win
        )

        ui_buffer.add_render_texture(
            ui_tex,
            GraphicsOutput.RTMBindOrCopy,
            GraphicsOutput.RTPColor
        )

        # Create scene
        ui_root = NodePath(name)
        ui_cam_np = self.makeCamera(ui_buffer)
        ui_cam_np.reparent_to(ui_root)
        # ui_cam_np.node().set_scene(ui_root)

        ui_root.set_depth_test(True)
        ui_root.set_depth_write(True)
        ui_root.set_two_sided(True)

        lens = OrthographicLens()
        lens.set_film_size(2, 2)         # DirectGUI uses -1..1 space by default
        lens.set_near_far(-100, 100)
        ui_cam_np.node().set_lens(lens)

        ui_buffer.set_clear_color((0, 0, 0, 0))
        ui_buffer.set_clear_depth(1.0)
        ui_buffer.set_clear_color_active(True)
        ui_buffer.set_clear_depth_active(True)
        ui_buffer.get_display_region(0).set_camera(ui_cam_np)

        ui_card = self.creat_3D_quad(name, ui_tex)

        return self.Widget3D(name, ui_root, ui_tex, ui_buffer, ui_card)

    def creat_3D_quad(self, name, texture):
        cm = CardMaker(name + "_card")
        cm.set_frame(-0.5, 0.5, -0.5, 0.5)
        ui_card = self.render.attach_new_node(cm.generate())
        ui_card.set_texture(texture, 1)
        ui_card.set_transparency(True)
        ui_card.set_light_off(1)
        return ui_card

    """ Create 3D GUI for slice selection and control """
    def create_3D_gui(self):
        self.slice_selection_widget = self.create_3d_widget("slice_selection_widget")
        self.slice_selection_widget.card.set_pos(3.0, 4.0, 0.0)
        self.slice_selection_widget.card.set_scale(2.0, 1.0, 2.0)
        quat = LQuaternionf()
        quat.setFromAxisAngle(-30.0, Vec3(0.0, 0.0, 1.0))
        self.slice_selection_widget.card.set_quat(quat)

        self.slice_selection_widget.card.node().setIntoCollideMask(self.pointer_collision_mask)

        self.all_buttons = []

        ui_frame = DirectFrame(
            parent=self.slice_selection_widget.root_node,
            frameColor=(0.1, 0.1, 0.1, 0.5),
            frameSize=(-0.9, 0.9, -0.9, 0.9),
            sortOrder=-1
        )
        ui_frame.set_depth_write(False)

        # === Slice visibility toggles ===
        def select_slice(status, name):
            slice_node = self.render.find("**/" + name)
            if not slice_node.isEmpty():
                if status:
                    slice_node.show()
                else:
                    slice_node.hide()

        index = 0
        selection_buttons = []
        for (name, vertices, colors) in self.scene_objects:
            selection_buttons.append(DirectCheckButton(sortOrder=1, text = name + "_slice_x", pos=(0.0, 1.0, 0.5 - index * 0.2), scale=.1, parent=ui_frame, command=select_slice, extraArgs=[name + "_slice_x"]))
            selection_buttons.append(DirectCheckButton(sortOrder=1, text = name + "_slice_y", pos=(0.0, 1.0, 0.0 - index * 0.2), scale=.1, parent=ui_frame, command=select_slice, extraArgs=[name + "_slice_y"]))
            selection_buttons.append(DirectCheckButton(sortOrder=1, text = name + "_slice_z", pos=(0.0, 1.0, -0.5 - index * 0.2), scale=.1, parent=ui_frame, command=select_slice, extraArgs=[name + "_slice_z"]))
            index += 1

        self.all_buttons.extend(selection_buttons)

        # === Active slice radio buttons ===
        # Update the slice slider
        def set_active_slice():
            pass

        # Add button
        active_buttons = []
        self.geom_selection = [""]

        index = 0
        for (name, vertices, colors) in self.scene_objects:
            name_x = name + "_slice_x"
            button_x = DirectRadioButton(pos=(0.0, 1.0, 0.53 - index * 0.2), scale=.1, parent=ui_frame, command=set_active_slice, variable=self.geom_selection, value=[name_x])

            offset_horizontal = button_x.getWidth() * 0.65
            button_x.setX(-offset_horizontal)
            active_buttons.append(button_x)
            
            name_y = name + "_slice_y"
            active_buttons.append(DirectRadioButton(pos=(-offset_horizontal, 1.0, 0.03 - index * 0.2), scale=0.1, parent=ui_frame, command=set_active_slice, variable=self.geom_selection, value=[name_y]))
            name_z = name + "_slice_z"
            active_buttons.append(DirectRadioButton(pos=(-offset_horizontal, 1.0, -0.47 - index * 0.2), scale=0.1, parent=ui_frame, command=set_active_slice, variable=self.geom_selection, value=[name_z]))
            index += 1

        self.all_buttons.extend(active_buttons)

        for button in active_buttons:
            button.setOthers(active_buttons)
            button.uncheck()

        # Create toggle for reconstructed model
        def toggle_reconstruction(status):
            model_node = self.render.find("**/reconstructed_model")
            if model_node.isEmpty():
                return

            if status:
                model_node.show()
            else:
                model_node.hide()

        model_button = DirectCheckButton(text="reconstruction", pos=(0.0, 1.0, 0.8), scale=0.1, parent=ui_frame, command=toggle_reconstruction)
        
        self.all_buttons.append(model_button)

    """ Create 2D GUI for slice selection and control """
    def create_gui(self):    
        # === Slice slider ===
        def update_slice():
            slice_node = self.render.find("**/" + self.geom_selection[0])
            if slice_node.isEmpty():
                return

            # Update shader uniform
            slice_float = self.slice_slider['value']
            slice_node.setShaderInput("slice_offset", slice_float)

            # Move geometry
            if "_slice_x" in self.geom_selection[0]:
                slice_node.setX(slice_float * self.grid_spacing[0])
            elif "_slice_y" in self.geom_selection[0]:
                slice_node.setY(slice_float * self.grid_spacing[1])
            elif "_slice_z" in self.geom_selection[0]:
                slice_node.setZ(slice_float * self.grid_spacing[2])

        self.slice_slider = DirectSlider(geom=self.slice_selection_gui_node, pos = (0.0, 0.0, 0.2), scale=0.8, parent=self.a2dpBottomCenter, command=update_slice, range=(0, self.grid_res[0]-1), value=0, pageSize=1)

        # === Slice visibility toggles ===
        def select_slice(status, name):
            slice_node = self.render.find("**/" + name)
            if not slice_node.isEmpty():
                if status:
                    slice_node.show()
                else:
                    slice_node.hide()

        index = 0
        selection_buttons = []
        for (name, vertices, colors) in self.scene_objects:
            selection_buttons.append(DirectCheckButton(geom=self.slice_selection_gui_node, text = name + "_slice_x", pos=(-0.5, 1.0, -0.2 - index * 0.2), scale=.05, parent=self.a2dpTopRight, command=select_slice, extraArgs=[name + "_slice_x"]))
            selection_buttons.append(DirectCheckButton(geom=self.slice_selection_gui_node, text = name + "_slice_y", pos=(-0.5, 1.0, -0.25 - index * 0.2), scale=.05, parent=self.a2dpTopRight, command=select_slice, extraArgs=[name + "_slice_y"]))
            selection_buttons.append(DirectCheckButton(geom=self.slice_selection_gui_node, text = name + "_slice_z", pos=(-0.5, 1.0, -0.3 - index * 0.2), scale=.05, parent=self.a2dpTopRight, command=select_slice, extraArgs=[name + "_slice_z"]))
            index += 1

        # === Active slice radio buttons ===
        # Update the slice slider
        def set_active_slice():
            pass

        # Add button
        active_buttons = []
        self.geom_selection = [""]

        index = 0
        for (name, vertices, colors) in self.scene_objects:
            name_x = name + "_slice_x"
            parent_x = selection_buttons[index * 3]
            button_x = DirectRadioButton(geom=self.slice_selection_gui_node, pos=(0.0, 0.0, 0.0), scale=1.0, parent=parent_x, command=set_active_slice, variable=self.geom_selection, value=[name_x])
            
            offset_horizontal = parent_x.getWidth() * 0.5 + button_x.getWidth() * 0.85
            offset_vertical = button_x.getHeight() * 0.25
            button_x.setPos(-offset_horizontal, 0.0, offset_vertical)
            active_buttons.append(button_x)
            
            name_y = name + "_slice_y"
            parent_y = selection_buttons[index * 3 + 1]
            active_buttons.append(DirectRadioButton(geom=self.slice_selection_gui_node, pos=(-offset_horizontal, 0.0, offset_vertical), scale=1.0, parent=parent_y, command=set_active_slice, variable=self.geom_selection, value=[name_y]))
            name_z = name + "_slice_z"
            parent_z = selection_buttons[index * 3 + 2]
            active_buttons.append(DirectRadioButton(geom=self.slice_selection_gui_node, pos=(-offset_horizontal, 0.0, offset_vertical), scale=1.0, parent=parent_z, command=set_active_slice, variable=self.geom_selection, value=[name_z]))
            index += 1

        for button in active_buttons:
            button.setOthers(active_buttons)
            button.uncheck()

    def on_left_trigger(self, active: bool, changed: bool):
        if changed:
            if active:
                # Perform raycast
                ray = CollisionRay()
                # ray.setOrigin(self.rig.get_left_aim_pose()[0])
                # ray.setDirection(self.rig.get_left_aim_pose()[1].getForward())
                ray.setOrigin(0.0, 0.0, 0.0)
                ray.setDirection(0.0, 1.0, 0.0)
                self.pointer_collision_node.node().setSolid(0, ray)

                pointer_traverser = CollisionTraverser()
                pointer_traverser.addCollider(self.pointer_collision_node, self.pointer_collision_handler)
                pointer_traverser.traverse(self.render)
                pointer_traverser.showCollisions(render)

                if self.pointer_collision_handler.getNumEntries() == 0:
                    return
                
                self.pointer_collision_handler.sort_entries()

                found_entry = None
                for entry in self.pointer_collision_handler.getEntries():
                    hit_node_path = entry.getIntoNodePath()
                    if hit_node_path.isHidden():
                        continue
                    found_entry = entry
                    break

                if found_entry is None:
                    return

                if "slice_selection_widget_card" in found_entry.getIntoNodePath().getName():
                    # Handle UI
                    hit_pos = found_entry.getSurfacePoint(self.slice_selection_widget.card)
                    
                     # Convert local hit point -> UV on card frame (-0.5..0.5 => 0..1)
                    u = hit_pos.get_x() + 0.5
                    v = hit_pos.get_z() + 0.5  # because card is in XZ plane by default

                    if 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0:
                        # Convert UV -> DirectGUI mouse coords (-1..1)
                        gui_x = (u * 2.0) - 1.0
                        gui_y = (v * 2.0) - 1.0

                        for button in self.all_buttons:
                            if self.hit_test_framesize(button, gui_x, gui_y):
                                if isinstance(button, DirectCheckButton):
                                    button["indicatorValue"] = not button["indicatorValue"]
                                    button.setIndicatorValue()
                                    cmd = button["command"]
                                    if cmd:
                                        extra = button["extraArgs"]
                                        if extra is None:
                                            extra = []
                                        cmd(button["indicatorValue"], *extra)
                                elif isinstance(button, DirectRadioButton):
                                    button["indicatorValue"] = not button["indicatorValue"]
                                    button.setIndicatorValue()

                                    self.geom_selection = button["value"]

                                    for other in button["others"]:
                                        if other != button:
                                            other["indicatorValue"] = False
                                            other.setIndicatorValue()
                else:
                    # Handle slices
                    pass

    def on_right_trigger(self, active: bool, changed: bool):
        if changed:
            self.transformation_started = active
            if active:
                self.hand_transformation_anchor = self.rig.get_right_hand_pose()[0]
                self.vis_transformation_anchor = self.vis_node.getPos()
                self.hand_orientation_anchor = self.rig.get_right_hand_pose()[1]
                self.vis_orientation_anchor = self.vis_node.getQuat()
            else:
                self.hand_transformation_anchor = Vec3()
                self.vis_transformation_anchor = Vec3()
                self.hand_orientation_anchor = LQuaternion()
                self.vis_transformation_anchor = LQuaternion()

    def update_slice(self):
        slice_node = self.render.find("**/" + self.geom_selection[0])
        if slice_node.isEmpty():
            return

        # Update shader uniform
        slice_node.setShaderInput("slice_offset", self.slice_offset)

        # Move geometry
        if "_slice_x" in self.geom_selection[0]:
            slice_node.setX(self.slice_offset * self.grid_spacing[0])
        elif "_slice_y" in self.geom_selection[0]:
            slice_node.setY(self.slice_offset * self.grid_spacing[1])
        elif "_slice_z" in self.geom_selection[0]:
            slice_node.setZ(self.slice_offset * self.grid_spacing[2])

    def on_left_analog(self, x, y):
        if abs(x) > 1e-4:
            delta = x * self.slice_speed
            self.slice_offset += delta
            self.slice_offset = np.clip(self.slice_offset, 0.0, self.grid_res[0] - 1)
            self.update_slice()

    def on_right_analog(self, x, y):
        if abs(x) > 0.1 or abs(y) > 0.1:
            pass

    def create_pointer(self, parent, use_openxr, length=10.0, radius=0.02):
        pointer = parent.attachNewNode("pointer")
        beam = loader.loadModel("models/box")
        beam.reparentTo(pointer)

        if use_openxr:
            beam.setScale(radius, length, radius)
        else:
            beam.setScale(radius, radius, -length)
            
        # beam.setScale(0.02, 0.02, 0.02)
        
        beam.setTwoSided(True)

        beam.setLightOff()
        beam.clearTexture()
        beam.setColor(0.2, 0.4, 1.0, 1.0)
        beam.setAttrib(ColorAttrib.makeFlat((0.2, 0.4, 1.0, 1.0)))

        return pointer

def main(args):
    parser = ArgumentParser(description="Visualize Covariance and underlying Point Cloud")
    parser.add_argument("data_path", type=str, help="path to data")
    parser.add_argument("--use-openxr", action="store_true", help="use OpenXR VR support")
    args = parser.parse_args(args)

    app = VarianceVis(args.data_path, use_openxr=args.use_openxr)
    app.run()

if __name__ == "__main__":
    main(sys.argv[1:])
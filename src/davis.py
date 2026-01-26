from panda3d.core import *
from panda3d.vrpn import VrpnClient
from direct.task import Task

# loadPrcFileData("", "fullscreen 1")
loadPrcFileData("", "win-size 1920 1080")
loadPrcFileData("", "framebuffer-stereo 1")
# loadPrcFileData("", "side-by-side-stereo 1")

class DaVis():

    IOD_METERS = 0.064
    LEDWALL_WIDTH_METERS = 4.6
    LEDWALL_HEIGHT_METERS = 2.6
    Z_CALIBRATION_METERS = 3.0
    TRIGGER_LEFT_ID = 0
    TRIGGER_RIGHT_ID = 8
    TRANSFORMATION_SPEED = 2
    NEAR_PLANE = Z_CALIBRATION_METERS
    FAR_PLANE = 1000

    def __init__(self):
        self.setup_vrpn()
        self.setup_stereo()

        self.input_callbacks = {}

        self.left_trigger_prev_state = False
        self.right_trigger_prev_state = False

        # Setup navigation variables
        # self.transformation_started = False
        # self.reference_pos = LVecBase3(0, 0, 0)
        # self.reference_quat = Quat()

    def add_input_callback(self, name, callback):
        self.input_callbacks[name] = callback

    def setup_vrpn(self):
        # connect to vrpn server
        self.vrpnclient = VrpnClient("localhost")
        
        # register update method used to poll new data from the VRPN server via the VrpnClient
        taskMgr.add(self.poll_vrpn_server, "poll_vrpn_server")
        
        # CAVE parent rig, moves according to tracking and covise-style scene navigation
        self.rig = base.render.attachNewNode("rig")

        # VRPN trackers:
        
        # In Panda3D, input is handled via the data graph, a tree that lives separately to the scene graph.
        # Therefore, attaching a tracker node input (data graph) to a scene graph node will not work.
        # Instead we need to create a node in the data graph for the input, a node in the scene graph for the actual scene, and convert
        # from the data graph node to the scene graph node using a Transform2SG object ("Transform to Scene Graph").

        # head glasses tracker (sensor #1)
        self.head_tracker = TrackerNode(self.vrpnclient, "DTrack:2")        # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        base.dataRoot.node().addChild(self.head_tracker)                    # add tracker to DATA graph
        self.head = base.render.attachNewNode("head")                            # create a corresponding node for the SCENE graph
        d2s_head = Transform2SG("d2s_head")                                 # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.head_tracker.addChild(d2s_head)                                # add the transform object as a child to the DATA graph node
        d2s_head.setNode(self.head.node())                                  # set the output node to the SCENE graph node
        self.head.reparentTo(self.rig)

        # attach left and righ eye trackers (only used for projection matrix calculation)
        self.left = self.head.attachNewNode("left")
        self.left.setPos(-DaVis.IOD_METERS / 2.0, 0.0, 0.0)
        self.right = self.head.attachNewNode("right")
        self.right.setPos(+DaVis.IOD_METERS / 2.0, 0.0, 0.0)

        # Flystick 8 tracker (sensor #8)
        self.flystick8_tracker = TrackerNode(self.vrpnclient, "DTrack:8")   # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        base.dataRoot.node().addChild(self.flystick8_tracker)               # add tracker to DATA graph
        self.flystick8 = base.render.attachNewNode("flystick8")                  # create a corresponding node for the SCENE graph
        d2s_flystick8 = Transform2SG("d2s_flystick8")                       # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.flystick8_tracker.addChild(d2s_flystick8)                      # add the transform object as a child to the DATA graph node
        d2s_flystick8.setNode(self.flystick8.node())                        # set the output node to the SCENE graph node
        self.flystick8.reparentTo(self.rig)

        # Flystick 9 tracker (sensor #9)
        self.flystick9_tracker = TrackerNode(self.vrpnclient, "DTrack:9")   # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        base.dataRoot.node().addChild(self.flystick9_tracker)               # add tracker to DATA graph
        self.flystick9 = base.render.attachNewNode("flystick9")                  # create a corresponding node for the SCENE graph
        d2s_flystick9 = Transform2SG("d2s_flystick9")                       # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.flystick9_tracker.addChild(d2s_flystick9)                      # add the transform object as a child to the DATA graph node
        d2s_flystick9.setNode(self.flystick9.node())                        # set the output node to the SCENE graph node
        self.flystick9.reparentTo(self.rig)
        
        # VRPN buttons:
        self.buttons = ButtonNode(self.vrpnclient, "DTrack")
        taskMgr.add(self.handle_vrpn_buttons, "handle_vrpn_buttons")
        
        # VRPN analogs:
        self.analogs = AnalogNode(self.vrpnclient, "DTrack")
        taskMgr.add(self.handle_vrpn_analogs, "handle_vrpn_analogs")

    def poll_vrpn_server(self, t):
        self.vrpnclient.poll()
        self.head.setQuat(Quat()) # discard tracked orientation (cannot face away from wall)
        return Task.cont
    
    # DTrack exposes a single button device for all Flystick buttons, that means:
    # Buttons 0-8 are for Flystick 9 and buttons 8-16 are for Flystick 8.
    def handle_vrpn_buttons(self, t):
        left_trigger_pressed = self.buttons.getButtonState(DaVis.TRIGGER_LEFT_ID)
        callback = self.input_callbacks.get("left_trigger", None)
        if callback is not None:
            callback(left_trigger_pressed, self.left_trigger_prev_state != left_trigger_pressed)
        self.left_trigger_prev_state = left_trigger_pressed

        right_trigger_pressed = self.buttons.getButtonState(DaVis.TRIGGER_RIGHT_ID)
        callback = self.input_callbacks.get("right_trigger", None)
        if callback is not None:
            callback(right_trigger_pressed, self.right_trigger_prev_state != right_trigger_pressed)
        self.right_trigger_prev_state = right_trigger_pressed

        # if not self.transformation_started and tbtn_pressed:
        #     self.transformation_started = True

        # if not tbtn_pressed:
        #     self.transformation_started = False
        #     self.reference_pos = LVecBase3(self.flystick9.getPos())
        #     self.reference_quat = Quat(self.flystick9.getQuat())       

        # if self.transformation_started:
        #     pass
        #     dt = globalClock.getDt()
            
        #     # translation
        #     self.rig.setPos(self.rig, (self.flystick9.getPos() - self.reference_pos) * DaVis.TRANSFORMATION_SPEED * dt)

        #     # rotation
        #     q_old_inv = Quat(self.reference_quat)
        #     q_old_inv.invertInPlace()
        #     q_diff = q_old_inv * self.flystick9.getQuat()
        #     h, _, _ = q_diff.getHpr()
        #     q_yaw_only = Quat()
        #     q_yaw_only.setHpr((h * DaVis.TRANSFORMATION_SPEED * dt, 0, 0))
        #     self.rig.setQuat(self.rig.getQuat() * q_yaw_only)
        return Task.cont
    
    # DTrack exposes a single analog device for all Flystick joysticks, that means:
    # Flystick 9 x-Axis: 0
    # Flystick 9 y-Axis: 1
    # Flystick 8 x-Axis: 2
    # Flystick 8 y-Axis: 3
    def handle_vrpn_analogs(self, t):
        x8 = self.analogs.getControlState(2)
        y8 = self.analogs.getControlState(3)
        callback = self.input_callbacks.get("left_analog", None)
        if callback is not None:
            callback(x8, y8)
        
        x9 = self.analogs.getControlState(0)
        y9 = self.analogs.getControlState(1)
        callback = self.input_callbacks.get("right_analog", None)
        if callback is not None:
            callback(x9, y9)
        
        return Task.cont

    def setup_stereo(self):

        # disable default cam
        base.camNode.setActive(False)
        base.cam.node().setActive(False)

        # make_display_region automatically returns a StereoDisplayRegion if framebuffer-stereo is set to 1 in config
        # both left and right display region share the default camera initially
        self.stereo_display_region = base.win.make_display_region()
        left_display_region = self.stereo_display_region.getLeftEye()
        right_display_region = self.stereo_display_region.getRightEye()
        
        # create and configure left and right eye camera:

        self.left_cam = self.left.attachNewNode(Camera("left_cam"))
        self.right_cam = self.right.attachNewNode(Camera("right_cam"))

        screen_z = DaVis.Z_CALIBRATION_METERS
        screen_left = -DaVis.LEDWALL_WIDTH_METERS / 2.0
        screen_right = DaVis.LEDWALL_WIDTH_METERS / 2.0
        screen_top = DaVis.LEDWALL_HEIGHT_METERS / 2.0
        screen_bottom = -DaVis.LEDWALL_HEIGHT_METERS / 2.0

        self.screen_ul = LVecBase3(screen_left, screen_z, screen_top)
        self.screen_ur = LVecBase3(screen_right, screen_z, screen_top)
        self.screen_ll = LVecBase3(screen_left, screen_z, screen_bottom)
        self.screen_lr = LVecBase3(screen_right, screen_z, screen_bottom)

        left_lens = PerspectiveLens()
        left_lens.setNear(DaVis.NEAR_PLANE)
        left_lens.setFar(DaVis.FAR_PLANE)

        right_lens = PerspectiveLens()
        right_lens.setNear(DaVis.NEAR_PLANE)
        right_lens.setFar(DaVis.FAR_PLANE)

        self.left_cam.node().setLens(left_lens)
        self.right_cam.node().setLens(right_lens)

        left_display_region.setCamera(self.left_cam)
        right_display_region.setCamera(self.right_cam)

        taskMgr.add(self.update_projection_matrix, "update_projection_matrix")

    def update_projection_matrix(self, t):
        
        head_pos = self.head.getPos()
        print("Head pos:", head_pos)
        
        left_lens = PerspectiveLens()
        left_lens.setNear(DaVis.Z_CALIBRATION_METERS - head_pos.getY())
        left_lens.setFar(DaVis.FAR_PLANE)

        right_lens = PerspectiveLens()
        right_lens.setNear(DaVis.Z_CALIBRATION_METERS - head_pos.getY())
        right_lens.setFar(DaVis.FAR_PLANE)

        #self.left_cam.node().setLens(left_lens)
        #self.right_cam.node().setLens(right_lens)

        ul_l = self.left.getRelativePoint(self.rig, self.screen_ul)
        ur_l = self.left.getRelativePoint(self.rig, self.screen_ur)
        ll_l = self.left.getRelativePoint(self.rig, self.screen_ll)
        lr_l = self.left.getRelativePoint(self.rig, self.screen_lr)

        ul_r = self.right.getRelativePoint(self.rig, self.screen_ul)
        ur_r = self.right.getRelativePoint(self.rig, self.screen_ur)
        ll_r = self.right.getRelativePoint(self.rig, self.screen_ll)
        lr_r = self.right.getRelativePoint(self.rig, self.screen_lr)

        flags = Lens.FC_off_axis

        self.left_cam.node().getLens().setFrustumFromCorners(
            ul_l, ur_l, ll_l, lr_l, flags
        )

        self.right_cam.node().getLens().setFrustumFromCorners(
            ul_r, ur_r, ll_r, lr_r, flags
        )

        return Task.cont

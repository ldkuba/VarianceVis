# Variance Visualization VR

### Requirements

All packages can be found in `requirements.txt` and installed with `pip install -r requirements.txt`.

### Panda3d-Openxr
The local version of the panda3d openxr integration can be found under `external/panda3d-openxr` (**as a gitsubmodule**)it  and includes a few modifications:

- Fix for Windows - passing correct OpenGL format enums to swapchain initialization. Based on https://discourse.panda3d.org/t/openxr-support-for-panda3d/30126/5

- Fix crash when closing session.

- Add additional input handling in default action set, specifically for oculus touch controllers 

**IMPORTANT**: the input binding suggestions and interaction profile are hardcoded for oculus touch controllers. See `actionset.py`, and https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-interaction-profiles for details.

### Input data
This demo uses `data/sdpsr_scalar_mean.pt`, which is a pickle created using `numpy.save()`, which contains 3D volume data, along with some metadata about the dimensions of the volume. The format is pretty straightforward and the reader can be found in `src/data_parser.py`. The writer is not included in this project.

### Demo UI/Presentation
The demo shows slices of volume data along with a 3D model of the object reconstructed from the "mean" volume.
Each slice(X, Y, Z) can be enabled or disabled independently in the UI widget, by clicking on it's name.
Additionally, you can move one selected slice along it's axis. The "active" slice which will be moved, can be selected by the radio buttons, 
found to the **left** of the respective activate/deactivate buttons.

### Controls
#### Right controller - Navigation
Hold the trigger button on the right controller, to start navigation, based on the controllers pose, relative to the pose when the trigger was pressed.
- Left-right: rotate around Z (vertical axis)
- Up-down: rotate around X (horizontal axis)
- Forward-backward: Move the visualization forward or backward (a.k.a. zoom)

#### Left controller - Selection
The left controller is used to interact with the UI elements. Aim the pointer at the desired UI element and click with the trigger button.

When a slice is slected as "active" using the radio buttons, it can be moved along it's axis using the left joystick X axis (left-right).
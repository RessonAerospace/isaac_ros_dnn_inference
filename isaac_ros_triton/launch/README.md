# Using `isaac_ros_triton_with_image.py` launch file

The following should be run from the root of the repo running inside the `isaac_ros_commons` container.

## Setting up model

```bash
# Create and navigate to model dir
mkdir -p models/peoplesemsegnet/1
cd models/peoplesemsegnet/1

# Download the model
wget https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplesemsegnet/versions/deployable_v1.0/files/peoplesemsegnet.etlt

# Convert the model to a TensorRT plan
/opt/nvidia/tao/tao-converter -k tlt_encode -d 3,544,960 -p input_1,1x3x544x960,1x3x544x960,1x3x544x960 -e model.plan -o softmax_1 peoplesemsegnet.etlt
```

## Building and running nodes

```bash
# Build nodes
colcon build && . install/setup.bash

# Run from launch file
ros2 launch isaac_ros_triton isaac_ros_triton_with_image.py
```
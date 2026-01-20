# Simple Vehicle Gazebo Model

This is a simple wheeled vehicle model for Gazebo simulation.

## Model Structure

The model consists of:
- **Base Platform**: 2.0m x 1.2m x 0.3m rectangular base
- **Four Wheels**: Cylindrical wheels (radius: 0.25m, width: 0.15m)
- **Main Body**: 0.8m x 0.8m x 0.7m box on top of the platform
- **Turret/Cargo Box**: 0.6m x 0.3m x 0.25m box on the front

## How to Use

### Option 1: Add to Gazebo Model Path

1. Set the `GAZEBO_MODEL_PATH` environment variable:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/udbhaw/Kavach/models
```

2. Add this line to your `~/.bashrc` to make it permanent:
```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/udbhaw/Kavach/models' >> ~/.bashrc
source ~/.bashrc
```

### Option 2: Copy to Default Gazebo Models Directory

```bash
mkdir -p ~/.gazebo/models
cp -r /home/udbhaw/Kavach/models/simple_vehicle ~/.gazebo/models/
```

## Using in a World File

Add the model to your Gazebo world file (`.world`):

```xml
<model name="my_vehicle">
  <include>
    <uri>model://simple_vehicle</uri>
  </include>
  <pose>0 0 0 0 0 0</pose>
</model>
```

## Testing the Model

### Launch Gazebo with the model:

```bash
cd /home/udbhaw/Kavach/models
gazebo --verbose
```

Then insert the model through Gazebo's GUI:
1. Click on the "Insert" tab in the left panel
2. Look for "Simple Vehicle" in the model list
3. Click and place it in the world

### Or create a test world file:

```bash
# See test_vehicle.world in this directory
gazebo test_vehicle.world
```

## Customization

You can modify the model by editing `model.sdf`:
- Change dimensions in the `<size>` tags
- Adjust positions with `<pose>` tags
- Modify colors in the `<material>` sections
- Add physics properties by setting `<static>false</static>` and adding inertial properties
- Add sensors, plugins, or additional links as needed

## Model Properties

- **Type**: Static (non-moving) by default
- **Materials**: Gray color scheme with varying shades for different parts
- **Collision**: Enabled for all parts
- **Physics**: Static (change to dynamic if you want to add wheels/motors)

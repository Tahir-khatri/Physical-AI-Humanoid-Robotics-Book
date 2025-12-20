# This is a conceptual Omniverse Replicator script.
# It is intended to be run within the NVIDIA Isaac Sim script editor.

import omni.replicator.core as rep
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.objects import VisualCuboid
import numpy as np

# --- Simulation Setup ---
# It is assumed that you have an Isaac Sim stage open.
# For a real use case, you would load your specific humanoid robot USD file.
# Here, we create a simple cube to represent our robot.
robot_prim = VisualCuboid(
    prim_path="/World/MyRobot",
    name="humanoid_robot",
    position=(0, 0, 1.0),
    scale=(0.5, 0.5, 2.0),
    color=np.array([0.2, 0.4, 0.8])
)
# We also create a simple ground plane.
ground_prim = VisualCuboid(
    prim_path="/World/GroundPlane",
    name="ground",
    position=(0, 0, -0.01),
    scale=(100, 100, 0.02),
    color=np.array([0.5, 0.5, 0.5])
)

# --- Replicator Graph ---
with rep.new_layer():

    # 1. Define the camera that will capture the images.
    camera = rep.create.camera(position=(10, 0, 5), look_at=robot_prim.prim_path)

    # 2. Define the randomization that will occur on every frame.
    # This is the core of creating a diverse dataset.
    with rep.trigger.on_frame():
        # Randomize the camera's position
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((-5, -5, 3), (5, 5, 8)),
                look_at=robot_prim.prim_path
            )
        
        # Randomize the color of the robot
        with rep.get.prims(prim_paths=[robot_prim.prim_path]):
            rep.modify.attribute("color", rep.distribution.uniform(
                (0.1, 0.1, 0.1), 
                (0.9, 0.9, 0.9)
            ))
            
        # Randomize the lighting
        # First, get a handle to the default distant light in the scene
        light_prim = rep.get.prims(path_pattern="/World/defaultLight")
        with light_prim:
            rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
            rep.modify.pose(rotation=rep.distribution.uniform((-90, -90, -180), (90, 90, 180)))


    # 3. Define the Render Product, which specifies the output resolution.
    render_product = rep.create.render_product(camera, (1280, 720))

    # 4. Initialize and attach the writers that will save the data to disk.

    # This writer will save the RGB images.
    rgb_writer = rep.WriterRegistry.get("BasicWriter")
    rgb_writer.initialize(
        output_dir="_output/rgb",
        rgb=True
    )
    rgb_writer.attach([render_product])

    # This writer will save the 2D bounding box data in the standard KITTI format.
    # We define a "semantic mapping" to tell the writer what to label.
    # The key 'robot' is the class name, and the value is a list of prim paths
    # belonging to that class.
    bbox_kitti_writer = rep.WriterRegistry.get("KittiWriter")
    bbox_kitti_writer.initialize(
        output_dir="_output/bounding_box_2d_kitti",
        semantic_types={'robot': [robot_prim.prim_path]}
    )
    bbox_kitti_writer.attach([render_product])
    
    print("Replicator graph configured. Starting data generation for 100 frames...")

    # 5. Run the orchestrator to execute the graph and generate the data.
    # This will simulate 100 frames, applying the randomizations and writing
    # the output from the attached writers at each frame.
    rep.orchestrator.run(num_frames=100)

    print("Data generation complete. Check the '_output' directory.")

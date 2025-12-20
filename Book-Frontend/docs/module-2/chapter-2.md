---
id: chapter-2
title: 'Chapter 2: Immersive Rendering & HRI with Unity'
sidebar_label: 'Immersive Rendering & HRI'
---

# Chapter 2: Immersive Rendering & HRI with Unity

While Gazebo provides the raw, functional physics simulation, the next frontier in creating a true digital twin lies in achieving photorealistic visuals and designing complex, intuitive human-robot interactions (HRI). This is where a professional game engine like **Unity** excels. By integrating our ROS 2-enabled robot into Unity, we can create environments that are not only physically accurate but also visually indistinguishable from reality, which is critical for training vision-based AI and developing safe, interactive behaviors.

This chapter is a comprehensive guide to leveraging Unity's **High Definition Render Pipeline (HDRP)** for state-of-the-art rendering. We will then dive into the practicalities of HRI, scripting interactions in C# that bridge user input with the robot's ROS 2 action servers, and even publishing data from Unity back into the ROS ecosystem.

## Why Unity for Robotics?

Unity, at its core, is a real-time 3D development platform. While its fame comes from creating video games, its capabilities are a perfect match for advanced robotics simulation:

1.  **Photorealistic Graphics**: Unity's HDRP is engineered to produce stunning, high-fidelity visuals, simulating real-world lighting, materials, and camera effects. An AI trained on near-photorealistic images from Unity is far more likely to succeed in the real world.
2.  **Rich Development Environment**: The Unity Editor is a mature, user-friendly tool for building worlds, managing assets, and scripting complex logic.
3.  **C# Scripting**: Unity uses C# for scripting, a powerful, modern, object-oriented language that is excellent for designing complex HRI scenarios and simulation logic.
4.  **ROS 2 Integration**: Through the official **Unity Robotics Hub**, connecting a Unity simulation to a ROS 2 network has become incredibly streamlined. This package provides the tools to send and receive ROS messages and call services directly from C# scripts.
5.  **Cross-Platform**: Unity's ability to deploy to a vast range of platforms (Windows, macOS, Linux, VR/AR headsets) makes it an incredibly flexible tool for HRI research and development.

## Setting the Stage: The High Definition Render Pipeline (HDRP)

HDRP is a specialized, high-fidelity render pipeline built by Unity for creating graphically demanding applications. It is not the default pipeline, so setting it up is a deliberate choice.

### Initializing a Unity Project with HDRP

1.  Open the Unity Hub and create a new project.
2.  From the list of templates, you **must** select **"3D (HDRP)"**. This template pre-configures the project with all the necessary packages, settings, and a sample scene to get you started. This process can take several minutes as Unity imports and configures numerous assets.

### Core Concepts of HDRP

When you open an HDRP project, you'll notice a few key differences and new features.

-   **Volumes**: In HDRP, post-processing effects (like bloom, color grading, and fog) and environment settings are controlled by a system called **Volumes**. You create a "Volume" object in your scene (e.g., a Global Volume that affects the whole scene) and add "Volume Overrides" to it. For example, to add a bloom effect, you would add a `Bloom` override to your Volume and tweak its `Threshold` and `Intensity` parameters. This component-based approach allows for powerful, localized effects; you could have one volume for an indoor area and another for an outdoor area, with a seamless blend between them.
-   **Physically Based Lighting**: HDRP uses physically based units for lighting. A light's intensity is measured in Lumens or Lux, just like in the real world. This allows for more predictable and realistic lighting results. When you create a light (e.g., a Point Light or Spot Light), you will see these physically based parameters in the Inspector. This is a significant departure from older rendering models and is key to achieving photorealism.
-   **Advanced Materials**: HDRP introduces a new set of "Lit" shaders that allow for incredibly detailed material properties. You can create materials that accurately simulate metals, plastics, glass, and fabrics by controlling parameters like `Metallic`, `Smoothness`, `Coat Mask`, and `Subsurface Scattering` (for translucent materials like skin or jade). Understanding the **Mask Map** and **Detail Map** is crucial. The Mask Map is a single texture that packs different data into its RGBA channels: Metallic (R), Ambient Occlusion (G), Detail Map Mask (B), and Smoothness (A). This is an efficient way to control multiple material properties at once.
-   **Real-Time Ray Tracing**: For supported hardware (NVIDIA RTX series), HDRP can enable real-time ray tracing for shadows, reflections, and global illumination, pushing visual fidelity to its absolute limit.

### A Deeper Dive into Lighting

Creating a believable scene is all about light. HDRP provides several light types:
- **Directional Light**: Simulates a very distant light source, like the sun. Its rays are parallel, and it affects the entire scene.
- **Point Light**: Emits light in all directions from a single point, like a bare light bulb.
- **Spot Light**: Emits light in a cone shape. Perfect for flashlights or stage lighting.
- **Area Light**: Emits light from a surface, either a rectangle or a disc. This is excellent for simulating soft, diffuse light from windows or fluorescent light panels.

Beyond direct lighting, **Global Illumination (GI)** is what makes scenes look real. GI simulates indirect lighting—light that bounces off one surface and illuminates another. HDRP supports both real-time and baked GI solutions. For robotics simulation where the environment is often static, "baking" the lighting into lightmaps is an incredibly effective way to get beautiful, realistic bounced light with very high performance.

## Scripting Human-Robot Interaction (HRI)

Now that we have a beautiful scene, we need to make it interactive. Our goal is to create a C# script that can detect user input and translate it into a ROS 2 service call. This requires the Unity Robotics Hub package.

### Setting up the ROS 2 Connection

1.  **Install Unity Robotics Hub**: Go to the Unity Asset Store, search for "Unity Robotics Hub", and import it into your project.
2.  **Configure ROS Settings**: After import, a new menu item "Robotics" will appear. Go to `Robotics -> ROS Settings`. Here you can specify the `ROS_IP` address of the machine running your ROS 2 nodes and the `ROS_MASTER_URI`. For a local setup, these can often be left as default.
3.  **Add a ROS Connection**: In your scene, create an empty GameObject and name it `ROSConnection`. Add the `ROSConnection` component to it from the Robotics Hub. This component establishes and maintains the connection to the ROS 2 network.

### Example 1: A "Look At" Command via Raycast

Let's create a script that allows a user to click anywhere on the floor, and have the robot turn to look at that point. This is a classic example of gaze interaction.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.MyRobotInterfaces; // Import our custom service message type

public class HRIController : MonoBehaviour
{
    private ROSConnection ros;
    private string serviceName = "/head/set_look_at_target";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<SetLookAtTargetRequest, SetLookAtTargetResponse>(serviceName);
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit) && hit.collider.CompareTag("Floor"))
            {
                Vector3 targetPoint = hit.point;
                Debug.Log($"User clicked on point: {targetPoint}");

                SetLookAtTargetRequest request = new SetLookAtTargetRequest
                {
                    target = new RosMessageTypes.Geometry.PointMsg
                    {
                        x = targetPoint.x,
                        y = targetPoint.y,
                        z = targetPoint.z
                    }
                };

                ros.SendServiceMessage<SetLookAtTargetResponse>(serviceName, request, LookAtCallback);
            }
        }
    }

    void LookAtCallback(SetLookAtTargetResponse response)
    {
        if (response.success)
        {
            Debug.Log("ROS service call successful: Robot is looking at the target.");
        }
        else
        {
            Debug.LogWarning("ROS service call failed.");
        }
    }
}
```
**To make this work:** Save this script as `HRIController.cs`, attach it to a GameObject, ensure your floor has the "Floor" tag, and have a corresponding ROS 2 service server running. This script demonstrates a complete HRI loop: capturing user intent and translating it into a concrete, actionable command.

### Example 2: Two-Way Communication and UI

A robust digital twin requires two-way communication. Let's create a scenario where Unity both sends and receives data. We will create a UI button that tells the robot to "wave", and we will subscribe to a ROS topic that reports the robot's battery level.

First, create a simple UI Canvas (`GameObject -> UI -> Canvas`) and add a button (`GameObject -> UI -> Button`).

Now, create the script:
```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // For standard message types like String and Float32

public class AdvancedHRIController : MonoBehaviour
{
    public Button waveButton;
    public Text batteryText;

    private ROSConnection ros;
    private string waveServiceName = "/robot/trigger_animation";
    private string batteryTopicName = "/robot/battery_level";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register the service we will call
        ros.RegisterRosService<TriggerAnimationRequest, TriggerAnimationResponse>(waveServiceName);
        
        // Subscribe to the battery topic
        ros.Subscribe<Float32Msg>(batteryTopicName, BatteryCallback);

        // Add a listener to our UI button
        waveButton.onClick.AddListener(OnWaveButtonClick);
    }

    void OnWaveButtonClick()
    {
        Debug.Log("Wave button clicked!");
        TriggerAnimationRequest request = new TriggerAnimationRequest { animation_name = "wave" };
        ros.SendServiceMessage<TriggerAnimationResponse>(waveServiceName, request, WaveCallback);
    }

    void WaveCallback(TriggerAnimationResponse response)
    {
        if (response.success)
        {
            Debug.Log("Robot wave animation triggered successfully.");
        }
        else
        {
            Debug.LogWarning("Failed to trigger robot wave animation.");
        }
    }

    void BatteryCallback(Float32Msg batteryMsg)
    {
        // Update the UI text with the received battery level
        // The value is typically between 0.0 and 1.0, so we format it as a percentage
        float batteryPercentage = batteryMsg.data * 100.0f;
        batteryText.text = $"Battery: {batteryPercentage:F1}%";
    }
}
```
**To make this work:**
1. Save the script as `AdvancedHRIController.cs` and attach it to a GameObject.
2. In the Inspector, drag your UI Button to the `waveButton` field and a UI Text element to the `batteryText` field.
3. On the ROS 2 side, you would need:
    - A service server at `/robot/trigger_animation` that can play a pre-defined "wave" animation.
    - A node that continuously publishes the robot's battery level (as a float between 0.0 and 1.0) to the `/robot/battery_level` topic.

This more advanced example shows the true power of a Unity-based digital twin: a rich, interactive visual front-end that is deeply integrated with the robot's underlying ROS 2-based cognitive and control architecture. This powerful combination is the cornerstone of modern digital twin development, enabling rapid prototyping and testing of complex AI behaviors in a safe, repeatable, and visually realistic environment.
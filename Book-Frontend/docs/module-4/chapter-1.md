---
id: chapter-1
title: 'Chapter 1: Voice-to-Action with OpenAI Whisper'
sidebar_label: 'Voice-to-Action with Whisper'
---

# Chapter 1: Voice-to-Action with OpenAI Whisper

Welcome to the final module, where we give our robot its voice and a higher level of cognitive function. The ultimate goal of robotics is to create machines that can interact with humans and the world in a natural, intuitive way. The most natural form of human communication is speech. This chapter provides an exhaustive technical guide to building the first component of this system: a **Voice-to-Action pipeline**.

We will bridge the gap between human speech and robotic action by integrating **OpenAI's Whisper**, a state-of-the-art speech recognition model, into our ROS 2 ecosystem. We will create a robust pipeline that can listen to a microphone, transcribe spoken words into text, parse that text for commands, and trigger ROS 2 services as a result.

## Why Whisper for Robotics?

In a robotics context, a speech recognition system needs to be more than just accurate; it needs to be robust to a variety of real-world conditions.
- **Noise Robustness**: Robots are not operated in soundproof booths. They have noisy fans, whirring motors, and operate in environments with background chatter. Whisper has been trained on a massive and diverse dataset from the web, making it exceptionally good at understanding speech even in the presence of significant background noise.
- **Accuracy**: Whisper provides near-human-level accuracy on a wide range of accents, languages, and technical vocabularies. For a robot, the difference between "go to the *chair*" and "go to the *stair*" is critical. Whisper's accuracy minimizes these potentially dangerous misinterpretations.
- **Open Source and Local Execution**: While a cloud API is available, Whisper's models are open source. This means you can download and run them locally on your own hardware (especially on NVIDIA GPUs for best performance). For robotics, local execution is a huge advantage as it reduces latency (no internet round-trip) and enhances privacy (sensitive audio data doesn't leave the robot).

## The Voice-to-Action Pipeline Architecture

Our goal is to build a modular pipeline composed of distinct ROS 2 nodes, each with a single responsibility.

1.  **Audio Capture Node**: A simple node that uses a library like `sounddevice` in Python to capture raw audio data from a microphone and publish it to a ROS 2 topic.
2.  **Whisper Bridge Node (The Core of this Chapter)**: This is the central component. It subscribes to the raw audio topic, accumulates audio data, sends it to the Whisper model for transcription, and publishes the resulting text to another topic.
3.  **Command Parser Node**: This node subscribes to the text transcription topic. It contains simple logic (e.g., regular expressions or keyword spotting) to parse the text and determine if it contains a valid command for the robot.
4.  **Action Client Node(s)**: Once the Command Parser identifies a valid command, it doesn't execute the action itself. Instead, it calls a ROS 2 service or action server on the appropriate node (e.g., the `nav2_bringup` action server) to execute the high-level behavior.

This decoupled architecture is robust and extensible. We can swap out the Whisper node for a different STT engine or create a more advanced NLP-based command parser without changing the other components of the system.

## Building the Whisper Bridge Node

Let's focus on the `whisper_ros_bridge.py` script. This node will perform several key functions.

### 1. Audio Data Handling
The node will subscribe to a topic publishing `audio_common_msgs/msg/AudioData`. It will need to buffer this incoming audio data. It's inefficient to run a Whisper transcription on every tiny chunk of audio. Instead, we'll accumulate audio for a few seconds or use a Voice Activity Detection (VAD) library (like `webrtcvad`) to detect when a user has started and stopped speaking. For this guide, we'll use a simpler "push-to-talk" approach where we trigger transcription manually.

### 2. Whisper Integration
We'll use the official `openai-whisper` Python library. You can install it via pip: `pip install openai-whisper`. You will also need `ffmpeg` installed on your system.

The core of the integration involves loading the model and then calling the `transcribe()` function on our accumulated audio data.

```python
import whisper

# Load the model. 'base' is a good starting point. 
# For higher accuracy on a GPU, you might use 'medium' or 'large'.
model = whisper.load_model("base")

# result is a dictionary containing the transcribed text and other info
result = model.transcribe("path/to/my/audio_file.wav")
transcribed_text = result["text"]
print(transcribed_text)
```

### Conceptual `whisper_ros_bridge.py`

Here is a conceptual script for our node. It will expose a service to start/stop recording and trigger transcription.

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import whisper
from scipy.io.wavfile import write

class WhisperBridge(Node):
    def __init__(self):
        super().__init__('whisper_bridge')
        
        # ROS 2 Publishers and Services
        self.transcription_publisher = self.create_publisher(String, '/voice_transcription', 10)
        self.srv = self.create_service(Trigger, 'trigger_transcription', self.trigger_transcription_callback)
        
        # Whisper Model
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("base.en") # Using the English-only base model
        self.get_logger().info("Whisper model loaded.")
        
        # Audio recording parameters
        self.samplerate = 16000  # 16kHz is standard for Whisper
        self.channels = 1
        self.audio_buffer = []
        self.is_recording = False

    def trigger_transcription_callback(self, request, response):
        if not self.is_recording:
            # Start recording
            self.is_recording = True
            self.audio_buffer = [] # Clear buffer
            self.get_logger().info('Recording started...')
            
            # This is a simplified approach. A real implementation would use a separate thread.
            # Here we just record for a fixed duration for simplicity.
            duration = 5  # seconds
            myrecording = sd.rec(int(duration * self.samplerate), samplerate=self.samplerate, channels=self.channels)
            sd.wait()  # Wait until recording is finished
            
            self.get_logger().info('Recording finished.')
            self.is_recording = False

            # Save to a temporary file to pass to Whisper
            temp_file = "/tmp/temp_audio.wav"
            write(temp_file, self.samplerate, myrecording)

            # Transcribe
            self.get_logger().info('Transcribing...')
            result = self.model.transcribe(temp_file)
            transcribed_text = result["text"]
            self.get_logger().info(f'Transcription: "{transcribed_text}"')
            
            # Publish the transcription
            msg = String()
            msg.data = transcribed_text
            self.transcription_publisher.publish(msg)
            
            response.success = True
            response.message = transcribed_text
        else:
            response.success = False
            response.message = "Already recording."
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WhisperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Use this Node

1.  **Run the Node**: `ros2 run my_robot_vla whisper_ros_bridge.py`
2.  **Trigger a Recording**: From another terminal, call the service:
    ```bash
    ros2 service call /trigger_transcription std_srvs/srv/Trigger
    ```
3.  **Speak**: Speak a command into your microphone within the 5-second window.
4.  **Observe the Output**: The node will log the transcription and publish it to the `/voice_transcription` topic. You can listen to the topic with:
    ```bash
    ros2 topic echo /voice_transcription
    ```

## From Text to Action: The Command Parser

The next step in the pipeline is a simple command parser. This node subscribes to `/voice_transcription` and acts upon the text. For our MVP, we can use simple keyword spotting.

**Conceptual `command_parser.py`:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')
        self.subscription = self.create_subscription(
            String,
            '/voice_transcription',
            self.listener_callback,
            10)
        # Action client to call Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: "{command}"')

        # Simple keyword-based parsing
        if "go to the kitchen" in command:
            self.send_nav_goal("kitchen")
        elif "go to the table" in command:
            self.send_nav_goal("table")

    def send_nav_goal(self, location_name):
        self.get_logger().info(f"Sending navigation goal for '{location_name}'...")
        goal_msg = NavigateToPose.Goal()
        # In a real system, you would look up the coordinates for 'location_name'
        # from a database or parameter file.
        if location_name == "kitchen":
            goal_msg.pose.pose.position.x = 5.0
            goal_msg.pose.pose.position.y = -2.0
            goal_msg.pose.pose.orientation.w = 1.0
        else: # table
            goal_msg.pose.pose.position.x = -1.5
            goal_msg.pose.pose.position.y = 3.0
            goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # Here you could add feedback callbacks, etc.
        self.get_logger().info("Goal sent to Nav2.")

# ... main function ...
```
This simple pipeline demonstrates the power of this architecture. We have successfully created a bridge from human speech all the way to a high-level navigation command, with each step being a modular, inspectable ROS 2 component. In the next chapter, we will replace this simple keyword parser with a much more powerful and flexible LLM-based planner.

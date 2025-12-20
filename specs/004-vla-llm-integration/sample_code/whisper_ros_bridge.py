# This is a conceptual Python script for a ROS 2 node that uses Whisper.
# It demonstrates the core logic of capturing audio and transcribing it.
# A real implementation would require more robust error handling, threading,
# and likely a more sophisticated audio capture mechanism (e.g., VAD).

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

import sounddevice as sd
import numpy as np
import whisper
from scipy.io.wavfile import write
import tempfile
import os

class WhisperBridge(Node):
    """
    A ROS 2 node that provides a service to trigger audio recording and transcription.
    It records audio for a fixed duration, saves it to a temporary file,
    transcribes it using OpenAI's Whisper, and publishes the result.
    """
    def __init__(self):
        super().__init__('whisper_bridge')
        
        # --- Parameters ---
        self.declare_parameter('model_size', 'base.en')
        self.declare_parameter('record_duration', 5.0)
        self.declare_parameter('samplerate', 16000)
        
        model_size = self.get_parameter('model_size').get_parameter_value().string_value
        self.record_duration = self.get_parameter('record_duration').get_parameter_value().double_value
        self.samplerate = self.get_parameter('samplerate').get_parameter_value().integer_value
        self.channels = 1

        # --- ROS 2 Interfaces ---
        self.transcription_publisher = self.create_publisher(String, '/voice_transcription', 10)
        self.srv = self.create_service(Trigger, 'trigger_transcription', self.trigger_transcription_callback)
        
        # --- Whisper Model Loading ---
        self.get_logger().info(f"Loading Whisper model '{model_size}'...")
        try:
            self.model = whisper.load_model(model_size)
            self.get_logger().info("Whisper model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            self.get_logger().error("Please ensure you have the model files and dependencies (like ffmpeg) installed correctly.")
            # Quit if model fails to load
            rclpy.shutdown()
            return
        
        self.is_recording = False
        self.get_logger().info(f"Whisper Bridge is ready. Call the '{self.srv.srv_name}' service to start a {self.record_duration}s recording.")


    def trigger_transcription_callback(self, request, response):
        """
        Service callback to start recording, transcribe, and publish.
        """
        if self.is_recording:
            response.success = False
            response.message = "Already recording."
            self.get_logger().warn("Transcription trigger called while already recording.")
            return response

        self.is_recording = True
        self.get_logger().info(f"Recording for {self.record_duration} seconds...")

        try:
            # Record audio from the default microphone
            audio_data = sd.rec(int(self.record_duration * self.samplerate), samplerate=self.samplerate, channels=self.channels, dtype='float32')
            sd.wait()  # Wait until recording is finished
            self.get_logger().info("Recording finished.")

            # Use a temporary file to store the audio
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
                temp_filename = tmpfile.name
                # The audio data from sounddevice is float32, Whisper expects float32, so this is fine.
                write(temp_filename, self.samplerate, audio_data)

            # Transcribe the audio file
            self.get_logger().info(f"Transcribing audio from {temp_filename}...")
            result = self.model.transcribe(temp_filename, fp16=False) # fp16=False for CPU
            transcribed_text = result["text"].strip()
            
            os.remove(temp_filename) # Clean up the temporary file

            self.get_logger().info(f'Transcription: "{transcribed_text}"')
            
            # Publish the transcription if it's not empty
            if transcribed_text:
                msg = String()
                msg.data = transcribed_text
                self.transcription_publisher.publish(msg)
                response.success = True
                response.message = transcribed_text
            else:
                response.success = False
                response.message = "No speech detected."

        except Exception as e:
            self.get_logger().error(f"An error occurred during transcription: {e}")
            response.success = False
            response.message = f"Error: {e}"
        finally:
            self.is_recording = False
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WhisperBridge()
    if not rclpy.ok():
        return
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from TTS.api import TTS
import sounddevice as sd

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_engine')
        self.get_logger().info("Initializing Coqui TTS node...")

        # Load TTS model
        self.tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=False, gpu=False)
        self.sample_rate = self.tts.synthesizer.output_sample_rate

        # Subscribe to a topic
        self.subscription = self.create_subscription(
            String,
            '/speak_text',
            self.speak_callback,
            10
        )
        self.get_logger().info("Subscribed to /speak_text")

    def speak_callback(self, msg: String):
        text = msg.data
        self.get_logger().info(f"Received text: {text}")

        try:
            # Convert text to speech (audio waveform)
            audio = self.tts.tts(text)
            # Play audio
            sd.play(audio, samplerate=self.sample_rate)
            sd.wait()
            self.get_logger().info("Done speaking.")
        except Exception as e:
            self.get_logger().error(f"Error during TTS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

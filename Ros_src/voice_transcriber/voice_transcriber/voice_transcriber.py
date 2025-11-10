import argparse
import os
import numpy as np
import speech_recognition as sr
import whisper
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
from colorama import Fore

from datetime import datetime, timedelta
from queue import Queue
from time import sleep
from sys import platform
from hri_msgs.msg import IdsList, LiveSpeech
from std_srvs.srv import SetBool
import threading




class voice_transcriber(Node):
    def __init__(self):
        super().__init__('voice_transcriber')

        # self.publisher_ = self.create_publisher(String, 'voice_data', 10)
        # Publisher for /humans/voices/tracked
        self.ids_pub = self.create_publisher(IdsList, '/humans/voices/tracked', 10)
        
        # Publisher for /humans/voices/shayan/speech
        self.speech_pub = None

        self.args = self.argsparsing()
        
        self.recording = False
        self.srv = self.create_service(SetBool, 'recording', self.recording_callback)



        # self.running()
        gui_thread = threading.Thread(target=self.running)
        gui_thread.start()


    def recording_callback(self, request, response):
        self.get_logger().info(f"Received request: data = {request.data}")
        
        if self.recording:
            if request.data:
                response.success = True
                response.message = 'Recording is already activated.'
                self.get_logger().info(Fore.GREEN +response.message)
            else:
                self.recording = False
                response.success = False
                response.message = 'Recording Stopped.'
                self.get_logger().info(Fore.RED +response.message)
        else:
            if request.data:
                self.recording = True
                response.success = True
                response.message = 'Recording is on, now.'
                self.get_logger().info(Fore.GREEN +response.message)
            else:
                response.success = False
                response.message = 'There was no recording happening.'
                self.get_logger().info(Fore.RED +response.message)
        return response


    def argsparsing(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--model", default="medium", help="Model to use",
                            choices=["tiny", "base", "small", "medium", "large"])
        parser.add_argument("--non_english", action='store_true',
                            help="Don't use the english model.")
        parser.add_argument("--energy_threshold", default=1000,
                            help="Energy level for mic to detect.", type=int)
        parser.add_argument("--record_timeout", default=2,
                            help="How real time the recording is in seconds.", type=float)
        parser.add_argument("--phrase_timeout", default=3,
                            help="How much empty space between recordings before we "
                                 "consider it a new line in the transcription.", type=float)
        if 'linux' in platform:
            parser.add_argument("--default_microphone", default='pulse',
                                help="Default microphone name for SpeechRecognition. "
                                     "Run this with 'list' to view available Microphones.", type=str)
        args = parser.parse_args()

        return args
    
    ##############################################################

    def running(self):
        # Create and publish IdsList message
        ids_msg = IdsList()
        ids_msg.ids = ['user']
        self.ids_pub.publish(ids_msg)
        self.get_logger().info(f'Published to /humans/voices/tracked: {ids_msg}')
        self.speech_pub = self.create_publisher(LiveSpeech, '/humans/voices/user/speech', 10)

        # The last time a recording was retrieved from the queue.
        phrase_time = None
        # Thread safe Queue for passing data from the threaded recording callback.
        data_queue = Queue()
        # Bytes object which holds audio data for the current phrase
        phrase_bytes = bytes()
        # We use SpeechRecognizer to record our audio because it has a nice feature where it can detect when speech ends.
        recorder = sr.Recognizer()
        recorder.energy_threshold = self.args.energy_threshold
        # Definitely do this, dynamic energy compensation lowers the energy threshold dramatically to a point where the SpeechRecognizer never stops recording.
        recorder.dynamic_energy_threshold = False

        # Important for linux users.
        # Prevents permanent application hang and crash by using the wrong Microphone
        if 'linux' in platform:
            mic_name = self.args.default_microphone
            if not mic_name or mic_name == 'list':
                print("Available microphone devices are: ")
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    print(f"Microphone with name \"{name}\" found")
                return
            else:
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    if mic_name in name:
                        source = sr.Microphone(sample_rate=16000, device_index=index)
                        break
        else:
            source = sr.Microphone(sample_rate=16000)

        # Load / Download model
        model = self.args.model
        if self.args.model != "large" and not self.args.non_english:
            model = model + ".en"
        audio_model = whisper.load_model(model)

        record_timeout = self.args.record_timeout
        phrase_timeout = self.args.phrase_timeout

        transcription = ['']

        with source:
            recorder.adjust_for_ambient_noise(source)

        def record_callback(_, audio:sr.AudioData) -> None:
            """
            Threaded callback function to receive audio data when recordings finish.
            audio: An AudioData containing the recorded bytes.
            """
            # Grab the raw bytes and push it into the thread safe queue.
            data = audio.get_raw_data()
            data_queue.put(data)

        # Create a background thread that will pass us raw audio bytes.
        # We could do this manually but SpeechRecognizer provides a nice helper.
        recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)

        publish_list = []
        published_number_ = 0
        # Cue the user that we're ready to go.
        print("Model loaded.\n")

        while True:
            try:
                while self.recording:
                    now = datetime.utcnow()
                    # Pull raw recorded audio from the queue.
                    if not data_queue.empty():
                        phrase_complete = False
                        # If enough time has passed between recordings, consider the phrase complete.
                        # Clear the current working audio buffer to start over with the new data.
                        if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                            phrase_bytes = bytes()
                            phrase_complete = True
                        # This is the last time we received new audio data from the queue.
                        phrase_time = now

                        # Combine audio data from queue
                        audio_data = b''.join(data_queue.queue)
                        data_queue.queue.clear()

                        # Add the new audio data to the accumulated data for this phrase
                        phrase_bytes += audio_data

                        # Convert in-ram buffer to something the model can use directly without needing a temp file.
                        # Convert data from 16 bit wide integers to floating point with a width of 32 bits.
                        # Clamp the audio stream frequency to a PCM wavelength compatible default of 32768hz max.
                        audio_np = np.frombuffer(phrase_bytes, dtype=np.int16).astype(np.float32) / 32768.0

                        # Read the transcription.
                        result = audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
                        text = result['text'].strip()

                        # If we detected a pause between recordings, add a new item to our transcription.
                        # Otherwise edit the existing one.
                        if phrase_complete:
                            transcription.append(text)
                            publish_list.append(text)
                            # msg = String()
                            # msg.data = text
                            # self.publisher_.publish(msg=msg)
                            # self.get_logger().info('the following message had been published {0}'.format(text))
                        else:
                            transcription[-1] = text
                            if len(publish_list) > 0:
                                publish_list[-1] = text

                        # Clear the console to reprint the updated transcription.
                        # os.system('cls' if os.name=='nt' else 'clear')

                        # print(Fore.GREEN + 'the published commands are : \n\n')
                        # for i in range(published_number_):
                        #     print(Fore.YELLOW + '{0} - published command: {1}'.format(i, publish_list[i]))                        

                        # print(Fore.GREEN + '\nThe recieved command:\n\n')
                        # for line in transcription:
                        #     print(Fore.BLUE + line)

                        # print(Fore.GREEN + '\nWaiting for new command ....\n')
                        # # Flush stdout.
                        # print('', end='', flush=True)

                    else:
                        # Infinite loops are bad for processors, must sleep.
                        sleep(0.25)
                        if len(publish_list) > published_number_:
                            # msg = String()
                            # msg.data = publish_list[-1]
                            # self.publisher_.publish(msg=msg)
                            # Create and publish LiveSpeech message
                            speech_msg = LiveSpeech()
                            speech_msg.final = publish_list[-1]
                            self.speech_pub.publish(speech_msg)
                            self.get_logger().info(Fore.CYAN +'******************\n')
                            self.get_logger().info(Fore.RED +'Message had been published: ' + Fore.YELLOW + str(speech_msg.final))
                            published_number_ += 1
                        
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    node = voice_transcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
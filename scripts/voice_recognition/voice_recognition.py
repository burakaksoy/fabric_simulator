import speech_recognition as sr
from PyQt5.QtCore import QThread, pyqtSignal

import rospy

"""
Installation: 
pip3 install SpeechRecognition 
sudo apt-get install python3-pyaudio (or pip3 install pyaudio)
pip3 install pocketsphinx (for offline speech recognition)
pip3 install pyttsx3 (for text-to-speech - OFFLINE)
pip3 install gtts (for text-to-speech - Google's ONLINE)
sudo apt install ffmpeg (for playing the audio from online gtts using ffplay command)
"""

class VoiceCommandThread(QThread):
    command_recognized = pyqtSignal(str)  # (important) Signal to emit recognized text

    def __init__(self, parent=None, offline_mode=False):
        super().__init__(parent)
        self._running = True  # A flag to allow clean shutdown if needed
        self.offline_mode = offline_mode

    def run(self):
        r = sr.Recognizer()

        # Adjust for your own microphone index if needed
        with sr.Microphone() as source:
            # Optionally adjust for ambient noise:
            r.adjust_for_ambient_noise(source, duration=1)  
            while self._running:
                try:
                    # print("Listening for commands...")
                    rospy.loginfo_throttle(30, "Listening for commands...")
                    audio_data = r.listen(source, timeout=5, phrase_time_limit=4)

                    if self.offline_mode:
                        # For offline use, you can switch to pocketsphinx:
                        recognized_text = r.recognize_sphinx(audio_data)
                    else:
                        # For Online Use Google approach
                        recognized_text = r.recognize_google(audio_data)
                    
                    recognized_text = recognized_text.lower().strip()

                    # print(f"Recognized: {recognized_text}")
                    rospy.loginfo(f"Recognized: {recognized_text}")

                    # (important) Emit the recognized text to the GUI
                    self.command_recognized.emit(recognized_text)

                except sr.WaitTimeoutError:
                    # No speech was heard within timeout
                    pass
                except sr.UnknownValueError:
                    # Could not understand the audio
                    pass
                except sr.RequestError as e:
                    # print(f"Could not request results from service; {e}")
                    rospy.logerr(f"Could not request results from service; {e}")
    
    def stop(self):
        self._running = False

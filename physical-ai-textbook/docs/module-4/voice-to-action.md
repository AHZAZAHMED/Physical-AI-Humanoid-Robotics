---
title: Voice to Action
---

# Voice to Action

Voice to Action (VTA) systems enable humanoid robots to understand spoken commands and convert them into executable robotic actions. This module covers implementing voice recognition, natural language processing, and action execution for humanoid robots.

## Introduction to Voice to Action Systems

Voice to Action systems for humanoid robots involve:
- **Speech Recognition**: Converting speech to text
- **Natural Language Understanding**: Interpreting user intent
- **Action Mapping**: Converting commands to robotic actions
- **Speech Synthesis**: Providing verbal feedback
- **Context Awareness**: Understanding commands in context

## Architecture of VTA Systems

### Processing Pipeline
1. **Audio Capture**: Microphone array processing
2. **Speech Recognition**: ASR (Automatic Speech Recognition)
3. **Intent Classification**: Understanding user intent
4. **Entity Extraction**: Identifying relevant parameters
5. **Action Planning**: Converting to executable actions
6. **Action Execution**: Performing the requested actions
7. **Feedback Generation**: Verbal or non-verbal feedback

### System Components
- **Audio Processing**: Noise reduction and beamforming
- **Language Model**: Understanding natural language
- **Action Database**: Mapping commands to actions
- **Context Manager**: Maintaining conversation state

## Speech Recognition for Robotics

### Real-time Speech Recognition
Robots require real-time speech recognition with low latency:

```python
import speech_recognition as sr
import threading

class RobotSpeechRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_listening = False

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def start_listening(self, callback):
        self.is_listening = True
        threading.Thread(target=self._listen_loop, args=(callback,)).start()

    def _listen_loop(self, callback):
        with self.microphone as source:
            while self.is_listening:
                try:
                    audio = self.recognizer.listen(source, timeout=1)
                    text = self.recognizer.recognize_google(audio)
                    callback(text)
                except sr.WaitTimeoutError:
                    continue
                except sr.UnknownValueError:
                    continue
```

### Noise Reduction Techniques
- **Beamforming**: Focus on speaker's voice
- **Echo Cancellation**: Remove robot's own speech
- **Noise Suppression**: Filter background noise
- **Voice Activity Detection**: Detect speech presence

## Natural Language Understanding

### Intent Recognition
Identifying the user's intent from spoken commands:

```python
import spacy
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.naive_bayes import MultinomialNB

class IntentClassifier:
    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")
        self.vectorizer = TfidfVectorizer()
        self.classifier = MultinomialNB()
        self.intents = {}

    def extract_intent(self, text):
        # Preprocess text
        doc = self.nlp(text.lower())
        tokens = [token.lemma_ for token in doc if not token.is_stop and not token.is_punct]

        # Extract features
        features = self.vectorizer.transform([' '.join(tokens)])

        # Predict intent
        intent = self.classifier.predict(features)[0]
        confidence = max(self.classifier.predict_proba(features)[0])

        return intent, confidence
```

### Common Robot Commands
- **Navigation**: "Go to the kitchen", "Move forward"
- **Manipulation**: "Pick up the red ball", "Open the door"
- **Interaction**: "What's your name?", "Tell me a joke"
- **Control**: "Stop", "Wait", "Follow me"

## Action Mapping and Execution

### Command to Action Mapping
Mapping natural language commands to robotic actions:

```python
class ActionMapper:
    def __init__(self):
        self.action_map = {
            'move_to': ['go to', 'move to', 'navigate to', 'walk to'],
            'pick_up': ['pick up', 'grasp', 'take', 'get'],
            'place': ['place', 'put', 'set down'],
            'follow': ['follow', 'come with me', 'accompany'],
            'stop': ['stop', 'halt', 'freeze'],
            'answer': ['what', 'how', 'why', 'tell me', 'explain']
        }

    def map_command(self, command):
        command_lower = command.lower()

        for action, keywords in self.action_map.items():
            for keyword in keywords:
                if keyword in command_lower:
                    # Extract parameters using NLP
                    params = self.extract_parameters(command_lower, action)
                    return action, params

        return 'unknown', {}

    def extract_parameters(self, command, action):
        # Extract entities based on action type
        if action == 'move_to':
            # Look for location entities
            locations = ['kitchen', 'living room', 'bedroom', 'office', 'bathroom']
            for loc in locations:
                if loc in command:
                    return {'location': loc}
        elif action == 'pick_up':
            # Look for object entities
            objects = ['ball', 'cup', 'book', 'phone', 'keys']
            for obj in objects:
                if obj in command:
                    color = self.extract_color(command)
                    return {'object': obj, 'color': color}

        return {}

    def extract_color(self, command):
        colors = ['red', 'blue', 'green', 'yellow', 'black', 'white']
        for color in colors:
            if color in command:
                return color
        return None
```

## Integration with ROS 2

### Voice Command Node
Creating a ROS 2 node for voice command processing:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers
        self.nav_goal_pub = self.create_publisher(Pose, 'navigation_goal', 10)
        self.action_pub = self.create_publisher(String, 'robot_action', 10)

        # Subscriber
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)

        # Action mapper
        self.action_mapper = ActionMapper()

        self.get_logger().info('Voice Command Node initialized')

    def voice_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Map command to action
        action, params = self.action_mapper.map_command(command)

        if action != 'unknown':
            # Execute action
            self.execute_action(action, params)
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def execute_action(self, action, params):
        if action == 'move_to':
            self.navigate_to_location(params['location'])
        elif action == 'pick_up':
            self.pick_up_object(params)
        elif action == 'stop':
            self.stop_robot()

    def navigate_to_location(self, location):
        # Publish navigation goal
        goal = Pose()
        # Set goal based on location map
        self.nav_goal_pub.publish(goal)
```

## OpenAI Whisper Integration

### Using Whisper for Speech Recognition
OpenAI Whisper provides robust speech recognition capabilities:

```python
import whisper
import torch
import pyaudio
import wave
import numpy as np

class WhisperVoiceProcessor:
    def __init__(self, model_size="base"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024

    def record_audio(self, duration=5):
        p = pyaudio.PyAudio()

        stream = p.open(format=self.audio_format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)

        self.get_logger().info("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        self.get_logger().info("Recording finished")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to WAV file
        filename = "temp_recording.wav"
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.audio_format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return filename

    def transcribe_audio(self, audio_file):
        # Load audio file
        audio = whisper.load_audio(audio_file)
        audio = whisper.pad_or_trim(audio)

        # Convert to log-Mel spectrogram
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        # Decode
        options = whisper.DecodingOptions()
        result = whisper.decode(self.model, mel, options)

        return result.text
```

## Context and State Management

### Maintaining Conversation Context
Handling multi-turn conversations and context:

```python
class ConversationContext:
    def __init__(self):
        self.context = {}
        self.history = []
        self.current_task = None
        self.entities = {}

    def update_context(self, user_input, robot_response):
        self.history.append({
            'user': user_input,
            'robot': robot_response,
            'timestamp': time.time()
        })

        # Update entities mentioned
        self.extract_entities(user_input)

    def extract_entities(self, text):
        # Extract and store entities from text
        doc = self.nlp(text)
        for ent in doc.ents:
            self.entities[ent.text] = ent.label_

    def get_context_for_intent(self, intent):
        # Provide context relevant to the current intent
        if intent == 'follow':
            return {'previous_location': self.context.get('last_location')}
        elif intent == 'pick_up':
            return {'available_objects': self.entities}
        return self.context
```

## Speech Synthesis and Feedback

### Text-to-Speech for Robot Responses
Providing verbal feedback to users:

```python
import pyttsx3
import threading

class RobotSpeechSynthesizer:
    def __init__(self):
        self.engine = pyttsx3.init()

        # Configure voice properties
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # Choose voice
        self.engine.setProperty('rate', 150)  # Speed of speech
        self.engine.setProperty('volume', 0.9)  # Volume level

    def speak(self, text, blocking=False):
        if blocking:
            self.engine.say(text)
            self.engine.runAndWait()
        else:
            # Speak in a separate thread
            thread = threading.Thread(target=self._speak_thread, args=(text,))
            thread.start()

    def _speak_thread(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def set_voice_properties(self, rate=None, volume=None, voice=None):
        if rate is not None:
            self.engine.setProperty('rate', rate)
        if volume is not None:
            self.engine.setProperty('volume', volume)
        if voice is not None:
            self.engine.setProperty('voice', voice)
```

## Multi-modal Integration

### Combining Voice with Visual Information
Enhancing voice commands with visual context:

```python
class MultiModalVTA:
    def __init__(self):
        self.voice_processor = WhisperVoiceProcessor()
        self.vision_processor = VisionProcessor()  # Custom vision module
        self.action_mapper = ActionMapper()
        self.context_manager = ConversationContext()

    def process_multimodal_command(self, audio_file, visual_context=None):
        # Transcribe audio
        text = self.voice_processor.transcribe_audio(audio_file)

        # Enhance with visual context if available
        if visual_context:
            text = self.enhance_with_visual_context(text, visual_context)

        # Map to action
        action, params = self.action_mapper.map_command(text)

        # Execute with context
        self.execute_contextual_action(action, params, visual_context)

    def enhance_with_visual_context(self, text, visual_context):
        # Add visual information to the command
        # Example: "Pick up the red ball" -> "Pick up the red ball on the table"
        if 'the' in text and visual_context.get('objects'):
            # Add spatial information based on visual detection
            enhanced_text = self.add_spatial_context(text, visual_context)
            return enhanced_text
        return text
```

## Performance and Robustness

### Handling Noisy Environments
Techniques for robust operation in challenging acoustic conditions:

1. **Multiple Microphones**: Array processing for better signal
2. **Adaptive Filtering**: Adjust to changing noise conditions
3. **Confidence Scoring**: Verify recognition accuracy
4. **Fallback Mechanisms**: Alternative input methods

### Error Handling and Recovery
```python
def handle_recognition_error(self, error_type):
    if error_type == "no_speech":
        self.speak("I didn't hear anything. Could you repeat that?")
    elif error_type == "unclear":
        self.speak("I didn't understand. Could you rephrase that?")
    elif error_type == "unknown_command":
        self.speak("I don't know how to do that yet.")
```

## Best Practices

1. **Privacy**: Secure handling of voice data
2. **Latency**: Optimize for real-time response
3. **Accuracy**: Validate recognition results
4. **Feedback**: Provide clear status to users
5. **Fallback**: Have alternative interaction methods

## Learning Objectives

After completing this module, you will be able to:
- Implement speech recognition for humanoid robots
- Design natural language understanding systems
- Map voice commands to robotic actions
- Integrate with ROS 2 and OpenAI Whisper
- Handle multi-modal voice and visual commands
- Optimize VTA systems for real-world deployment
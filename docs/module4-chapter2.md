---
id: module4-chapter2
title: Language-Driven Robot Control
sidebar_label: Chapter 2 - Language Control
---

# Language-Driven Robot Control

## Natural Language Interfaces

Enable robots to understand and execute natural language commands in real-time.

### Command Parser

```python
import spacy
from enum import Enum

class Action(Enum):
    PICK = "pick"
    PLACE = "place"
    MOVE = "move"
    OPEN = "open"
    CLOSE = "close"

class LanguageParser:
    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")

    def parse_command(self, text):
        doc = self.nlp(text)

        # Extract action verb
        action = None
        for token in doc:
            if token.pos_ == "VERB":
                if token.lemma_ in [a.value for a in Action]:
                    action = Action(token.lemma_)
                    break

        # Extract object
        target_object = None
        for chunk in doc.noun_chunks:
            if chunk.root.dep_ == "dobj":
                target_object = chunk.text
                break

        # Extract location
        location = None
        for token in doc:
            if token.dep_ == "prep":
                location = token.head.text

        return {
            "action": action,
            "object": target_object,
            "location": location
        }

# Usage
parser = LanguageParser()
result = parser.parse_command("pick up the red cup from the table")
print(result)
# {'action': Action.PICK, 'object': 'the red cup', 'location': 'table'}
```

### Grounded Language Understanding

```python
class GroundedCommandExecutor:
    def __init__(self, robot, vision_system):
        self.robot = robot
        self.vision = vision_system
        self.parser = LanguageParser()

    def execute(self, command):
        # Parse command
        parsed = self.parser.parse_command(command)

        # Ground objects in scene
        detected_objects = self.vision.detect_objects()
        target = self.match_object(parsed['object'], detected_objects)

        if target is None:
            return f"Cannot find {parsed['object']}"

        # Execute action
        if parsed['action'] == Action.PICK:
            self.robot.pick(target.pose)
            return f"Picked up {target.name}"

        elif parsed['action'] == Action.PLACE:
            location_obj = self.match_object(parsed['location'], detected_objects)
            self.robot.place(location_obj.pose)
            return f"Placed {target.name} on {location_obj.name}"

    def match_object(self, description, objects):
        # Use CLIP for semantic matching
        from transformers import CLIPProcessor, CLIPModel

        model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

        # Compare description with object images
        images = [obj.image for obj in objects]
        inputs = processor(
            text=[description],
            images=images,
            return_tensors="pt",
            padding=True
        )

        outputs = model(**inputs)
        probs = outputs.logits_per_image.softmax(dim=1)

        best_match_idx = probs.argmax().item()
        return objects[best_match_idx]
```

## Dialogue Systems

### Conversational Robot Agent

```python
from openai import OpenAI

class RobotDialogueAgent:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        self.conversation_history = []
        self.system_prompt = """
        You are a helpful robot assistant. You can:
        - Navigate to locations
        - Pick and place objects
        - Open and close containers
        - Answer questions about the environment

        When asked to do something, confirm understanding and ask for clarification if needed.
        Keep responses concise and action-oriented.
        """

    def process_utterance(self, user_input, scene_description):
        # Add to history
        self.conversation_history.append({
            "role": "user",
            "content": f"Scene: {scene_description}\nUser: {user_input}"
        })

        # Get response
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                *self.conversation_history
            ]
        )

        assistant_msg = response.choices[0].message.content
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_msg
        })

        return assistant_msg

# Usage
agent = RobotDialogueAgent(api_key="your-key")

scene = "I see a table with a red cup and blue box on it."

response1 = agent.process_utterance("Can you get me the cup?", scene)
print(response1)  # "Sure! I'll pick up the red cup from the table."

response2 = agent.process_utterance("Actually, get the box instead", scene)
print(response2)  # "No problem, I'll get the blue box instead."
```

### Intent Recognition

```python
from transformers import pipeline

class IntentClassifier:
    def __init__(self):
        self.classifier = pipeline(
            "zero-shot-classification",
            model="facebook/bart-large-mnli"
        )

        self.candidate_labels = [
            "pick_and_place",
            "navigation",
            "question_answering",
            "open_container",
            "describe_scene"
        ]

    def classify(self, utterance):
        result = self.classifier(
            utterance,
            self.candidate_labels
        )

        return {
            "intent": result['labels'][0],
            "confidence": result['scores'][0]
        }

# Usage
classifier = IntentClassifier()
intent = classifier.classify("Where is the nearest exit?")
print(intent)  # {'intent': 'navigation', 'confidence': 0.92}
```

## Interactive Learning

### Learning from Demonstrations

```python
class InteractiveLearner:
    def __init__(self, model):
        self.model = model
        self.demonstrations = []

    def collect_demo(self, instruction, trajectory):
        """Collect human demonstration"""
        self.demonstrations.append({
            'instruction': instruction,
            'states': trajectory.states,
            'actions': trajectory.actions
        })

    def learn_from_feedback(self, instruction, predicted_action, feedback):
        """Update policy based on human feedback"""
        if feedback == "correct":
            # Positive reinforcement
            self.model.update(instruction, predicted_action, reward=1.0)
        elif feedback == "incorrect":
            # Request correction
            print("What should I have done?")
            correct_action = get_user_input()
            self.model.update(instruction, correct_action, reward=1.0)
            self.model.update(instruction, predicted_action, reward=-1.0)

# Usage
learner = InteractiveLearner(model=my_vla_model)

# Collect demonstration
trajectory = robot.execute_with_recording("pick up the cup")
learner.collect_demo("pick up the cup", trajectory)

# Interactive correction
action = model.predict("pick up the ball")
learner.learn_from_feedback("pick up the ball", action, feedback="incorrect")
```

### Active Learning

```python
class ActiveLearner:
    def __init__(self, model):
        self.model = model
        self.uncertainty_threshold = 0.3

    def should_ask_for_help(self, instruction, image):
        # Predict with uncertainty
        action, uncertainty = self.model.predict_with_uncertainty(
            instruction,
            image
        )

        if uncertainty > self.uncertainty_threshold:
            # Ask for demonstration
            print(f"I'm not sure how to '{instruction}'. Can you show me?")
            return True

        return False

    def execute_with_active_learning(self, instruction, image):
        if self.should_ask_for_help(instruction, image):
            demo = collect_human_demonstration()
            self.model.learn_from_demo(demo)

        action = self.model.predict(instruction, image)
        return action
```

## Multimodal Interaction

### Gesture + Language

```python
class MultimodalInterface:
    def __init__(self):
        self.gesture_detector = MediaPipeGestureDetector()
        self.speech_recognizer = SpeechRecognizer()

    def process_interaction(self, video_frame, audio):
        # Detect pointing gesture
        gesture = self.gesture_detector.detect(video_frame)

        # Recognize speech
        speech_text = self.speech_recognizer.transcribe(audio)

        # Combine modalities
        if gesture.type == "pointing":
            pointed_object = self.get_object_at_point(gesture.direction)
            command = f"{speech_text} {pointed_object}"
        else:
            command = speech_text

        return command

# Usage
interface = MultimodalInterface()

# User points at cup and says "pick that up"
video = capture_video_frame()
audio = record_audio()

command = interface.process_interaction(video, audio)
print(command)  # "pick that up cup_3"
```

## Safety and Grounding

### Safety Filters

```python
class SafetyFilter:
    def __init__(self):
        self.forbidden_actions = [
            "harm", "attack", "destroy", "damage"
        ]

        self.forbidden_areas = [
            "stairs", "edge", "cliff"
        ]

    def is_safe(self, command, current_state):
        # Check for forbidden actions
        for forbidden in self.forbidden_actions:
            if forbidden in command.lower():
                return False, f"Cannot perform '{forbidden}' action"

        # Check physical safety
        if self.would_cause_damage(command, current_state):
            return False, "Action may cause damage"

        # Check workspace limits
        if not self.within_workspace(command, current_state):
            return False, "Target is outside workspace"

        return True, "Safe to execute"

    def would_cause_damage(self, command, state):
        # Check if action would cause collision, fall, etc.
        simulated_result = self.simulate_action(command, state)
        return simulated_result.has_collision

# Usage
safety_filter = SafetyFilter()
is_safe, message = safety_filter.is_safe("move forward 5 meters", robot_state)

if not is_safe:
    print(f"Unsafe action: {message}")
```

### Physical Grounding

```python
class PhysicalGrounding:
    def __init__(self, robot):
        self.robot = robot

    def verify_preconditions(self, action):
        """Verify action is physically possible"""
        if action.type == "pick":
            # Check if object is reachable
            if not self.robot.can_reach(action.target):
                return False, "Object out of reach"

            # Check gripper capacity
            if action.target.weight > self.robot.max_payload:
                return False, "Object too heavy"

        return True, "Preconditions satisfied"

    def adapt_to_reality(self, planned_action, current_state):
        """Adapt plan based on current state"""
        if current_state != expected_state:
            # Replan
            new_action = self.replan(planned_action, current_state)
            return new_action

        return planned_action
```

## Key Takeaways

- **Natural language** enables intuitive robot control
- **Dialogue systems** support interactive collaboration
- **Interactive learning** improves from human feedback
- **Multimodal fusion** combines speech, gesture, vision
- **Safety filters** prevent unsafe actions

## Practical Exercises

1. Build command parser for 10 common commands
2. Implement dialogue agent with conversation memory
3. Create interactive learning system
4. Add gesture recognition to language interface

## Next Steps

In the final chapter:
- End-to-end VLA systems
- Training and deployment
- Real-world case studies
- Future directions

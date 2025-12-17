---
id: module4-chapter1
title: Vision-Language Models for Robotics
sidebar_label: Chapter 1 - Vision-Language Models
---

# Vision-Language Models for Robotics

## The VLA Revolution

Vision-Language-Action (VLA) models represent a paradigm shift: robots that understand natural language commands and directly map visual inputs to actions.

### Traditional vs VLA Approach

**Traditional Pipeline:**
```
Image → Object Detector → Task Planner → Motion Planner → Action
         (trained)         (hand-coded)    (hand-coded)
```

**VLA Approach:**
```
Image + Language → VLA Model → Action
                  (end-to-end learned)
```

### Why VLAs Matter

1. **Natural Interaction**: "Pick up the red cup" instead of programming
2. **Generalization**: Transfer to new objects and scenes
3. **Few-Shot Learning**: Adapt with minimal examples
4. **End-to-End**: No hand-engineered pipelines

## Foundation Models for Robotics

### Vision-Language Models

**Popular Models:**
- **CLIP** (OpenAI): Image-text alignment
- **BLIP-2** (Salesforce): Image captioning and VQA
- **LLaVA**: Large Language and Vision Assistant
- **GPT-4V**: Multimodal GPT

### CLIP for Robotics

```python
import torch
import clip
from PIL import Image

# Load CLIP model
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Prepare inputs
image = preprocess(Image.open("robot_view.jpg")).unsqueeze(0).to(device)
text_options = clip.tokenize([
    "a red cup",
    "a blue box",
    "a green ball"
]).to(device)

# Get similarities
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text_options)

    # Normalize
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True)

    # Compute similarity
    similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)

print("Probabilities:", similarity)
# Output: [0.85, 0.10, 0.05] → Red cup detected!
```

### Object Detection with Language

```python
from transformers import OwlViTProcessor, OwlViTForObjectDetection
import torch

# Load OWL-ViT (Open-Vocabulary Object Detector)
processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")

# Detect objects with natural language
image = Image.open("scene.jpg")
texts = [["a red apple", "a coffee mug", "a laptop"]]

inputs = processor(text=texts, images=image, return_tensors="pt")
outputs = model(**inputs)

# Process detections
target_sizes = torch.Tensor([image.size[::-1]])
results = processor.post_process_object_detection(
    outputs=outputs,
    target_sizes=target_sizes,
    threshold=0.1
)

# Print detected objects
for box, score, label in zip(results[0]["boxes"], results[0]["scores"], results[0]["labels"]):
    box = [round(i, 2) for i in box.tolist()]
    print(f"Detected {texts[0][label]} with confidence {round(score.item(), 3)} at {box}")
```

## Large Language Models for Planning

### LLMs as Task Planners

```python
from openai import OpenAI

client = OpenAI()

def plan_task(instruction):
    prompt = f"""
    You are a robot task planner. Break down the following instruction into atomic actions:

    Instruction: {instruction}

    Available actions:
    - move_to(location)
    - pick(object)
    - place(object, location)
    - open(container)
    - close(container)

    Output a Python list of action tuples.
    """

    response = client.chat.completions.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

    return response.choices[0].message.content

# Example
task = "Get me a soda from the fridge"
plan = plan_task(task)
print(plan)
# Output:
# [
#   ("move_to", "fridge"),
#   ("open", "fridge"),
#   ("pick", "soda"),
#   ("close", "fridge"),
#   ("move_to", "user"),
#   ("place", "soda", "table")
# ]
```

### Code as Policies

```python
class LLMPolicy:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def generate_code(self, task, context):
        prompt = f"""
        Write Python code to accomplish this task:
        Task: {task}

        Available functions:
        - robot.move_to(x, y, z)
        - robot.grasp()
        - robot.release()
        - robot.get_object_pose(name)

        Current scene: {context}

        Write only the code, no explanations.
        """

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        return response.choices[0].message.content

    def execute(self, task, robot, scene):
        # Generate code
        code = self.generate_code(task, scene.describe())

        # Execute safely
        namespace = {'robot': robot}
        exec(code, namespace)

# Usage
policy = LLMPolicy(api_key="your-key")
policy.execute("stack the blue block on the red block", robot, scene)
```

## Vision-Language-Action Models

### RT-1 (Robotics Transformer 1)

Google's RT-1 directly maps images and language to robot actions.

**Architecture:**
```
Image (300x300) ──┐
                  ├──> Transformer ──> Action (7-DOF)
Language Tokens ──┘
```

**Implementation Concept:**

```python
import torch
import torch.nn as nn

class RT1(nn.Module):
    def __init__(self, vocab_size, action_dim=7):
        super().__init__()

        # Vision encoder (EfficientNet)
        self.vision_encoder = efficientnet_b3(pretrained=True)
        self.vision_fc = nn.Linear(1536, 512)

        # Language encoder
        self.language_embedding = nn.Embedding(vocab_size, 512)

        # Transformer
        self.transformer = nn.Transformer(
            d_model=512,
            nhead=8,
            num_encoder_layers=6,
            num_decoder_layers=6
        )

        # Action head
        self.action_head = nn.Linear(512, action_dim)

    def forward(self, image, language_tokens):
        # Encode vision
        vision_features = self.vision_encoder(image)
        vision_features = self.vision_fc(vision_features)

        # Encode language
        lang_features = self.language_embedding(language_tokens)

        # Combine with transformer
        combined = self.transformer(lang_features, vision_features)

        # Predict action
        action = self.action_head(combined[:, -1, :])

        return action

# Usage
model = RT1(vocab_size=10000)
image = torch.randn(1, 3, 300, 300)
instruction = torch.randint(0, 10000, (1, 20))  # "pick up the cup"

action = model(image, instruction)
print(action)  # [x, y, z, roll, pitch, yaw, gripper]
```

### OpenVLA

Open-source vision-language-action model.

```python
from transformers import AutoModel, AutoTokenizer
import torch

# Load OpenVLA
model = AutoModel.from_pretrained("openvla/openvla-7b")
tokenizer = AutoTokenizer.from_pretrained("openvla/openvla-7b")

def predict_action(image, instruction):
    # Tokenize instruction
    inputs = tokenizer(instruction, return_tensors="pt")

    # Add image
    inputs['pixel_values'] = image

    # Predict action
    with torch.no_grad():
        outputs = model(**inputs)
        action = outputs.action

    return action

# Example
from PIL import Image
image = Image.open("robot_camera.jpg")
action = predict_action(image, "pick up the red block")
```

## Multimodal Prompting

### Visual Prompting

```python
def visual_few_shot_learning(model, demos, query_image, instruction):
    """
    demos: list of (image, instruction, action) tuples
    """

    # Create multimodal prompt
    prompt = "Learn from these examples:\n\n"

    for i, (demo_img, demo_inst, demo_action) in enumerate(demos):
        prompt += f"Example {i+1}:\n"
        prompt += f"Image: [Image {i+1}]\n"
        prompt += f"Instruction: {demo_inst}\n"
        prompt += f"Action: {demo_action}\n\n"

    prompt += f"Now, given:\n"
    prompt += f"Image: [Query Image]\n"
    prompt += f"Instruction: {instruction}\n"
    prompt += f"What action should I take?\n"

    # Process with VLA model
    action = model(prompt, images=[*[d[0] for d in demos], query_image])

    return action
```

## Key Takeaways

- **Foundation models** enable natural language robot control
- **CLIP/OWL-ViT** provide open-vocabulary object detection
- **LLMs** can generate task plans and code
- **VLA models** map vision + language directly to actions
- **Few-shot learning** enables rapid adaptation

## Practical Exercises

1. Implement CLIP-based object selector
2. Use GPT-4 for multi-step task planning
3. Fine-tune small VLA on custom task
4. Build visual prompting system

## Next Steps

In the next chapter:
- Language-driven robot control
- Dialogue systems for robots
- Interactive learning
- Safety and grounding

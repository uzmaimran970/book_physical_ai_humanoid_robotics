---
sidebar_position: 7
title: "باب 6: روبوٹکس کے لیے مشین لرنینگ"
description: "ری انفورسمنٹ لرنینگ، تقلیدی تعلیم، اور فاؤنڈیشن ماڈلز"
keywords: [machine learning, reinforcement learning, robotics ai, foundation models]
reading_time: 6
difficulty: advanced
word_count: 4500
---

# باب 6: روبوٹکس کے لیے مشین لرنینگ

**مشکل کی سطح:** اعلیٰ درجے کی
**پڑھنے کا وقت:** 6 منٹ
**الفاظ کی تعداد:** تقریباً 4,500 الفاظ

## خلاصہ

مشین لرنینگ نے روبوٹس کو تجربے سے سیکھنے کے قابل بنا کر روبوٹکس میں انقلاب برپا کر دیا ہے۔ یہ باب کنٹرول کے لیے ری انفورسمنٹ لرنینگ، تقلیدی تعلیم، اور روبوٹکس کے لیے فاؤنڈیشن ماڈلز پر مشتمل ہے۔

## روبوٹکس کے لیے مشین لرنینگ کیوں؟

### روایتی نقطہ نظر کی حدود

**دستی طریقے سے بنائے گئے کنٹرولرز:**
- ماہرانہ علم کی ضرورت
- ماحولیاتی تبدیلیوں کے لیے نازک
- محدود عمومیت
- ڈیزائن میں وقت طلب

### ML کے فوائد

- ڈیٹا سے سیکھیں
- نئی صورتحال کے مطابق ڈھلیں
- اعلیٰ جہتی ان پٹس کو سنبھالیں
- بہترین رویوں کو دریافت کریں

## ری انفورسمنٹ لرنینگ (RL)

### بنیادی تصورات

**ایجنٹ-ماحول کی تعامل:**
```
ایجنٹ حالت (s_t) کا مشاہدہ کرتا ہے
    ↓
ایجنٹ عمل (a_t) انجام دیتا ہے
    ↓
ماحول نئی حالت (s_t+1) میں منتقل ہوتا ہے
    ↓
ایجنٹ کو انعام (r_t) ملتا ہے
    ↓
مقصد: مجموعی انعام کو زیادہ سے زیادہ کریں
```

**کلیدی اجزاء:**
- **State space (S):** تمام ممکنہ روبوٹ configurations
- **Action space (A):** تمام ممکنہ اعمال
- **Reward function (R):** Scalar feedback signal
- **Policy (π):** حالات سے اعمال تک کی mapping
- **Value function (V):** متوقع مجموعی انعام

### Policy Gradient طریقے

**REINFORCE:**
- براہ راست policy کو بہتر بنائیں
- زیادہ تغیر
- سادہ لیکن غیر موثر

**Actor-Critic:**
- Actor: Policy network
- Critic: Value network
- تغیر کو کم کرتا ہے
- مثالیں: A3C, A2C

**Proximal Policy Optimization (PPO):**
- مستحکم policy updates
- Clipped objective
- روبوٹکس میں وسیع پیمانے پر استعمال
- کارکردگی اور استحکام کا اچھا توازن

### Value-Based طریقے

**Q-Learning:**
- Action-value function Q(s, a) سیکھیں
- بہترین policy: argmax Q منتخب کریں
- Off-policy learning

**Deep Q-Networks (DQN):**
- Q-function کے لیے neural network
- Experience replay
- استحکام کے لیے target network
- مجرد اعمال میں کامیاب

### Model-Based RL

**ماحول کا ماڈل سیکھیں:**
- عمل دیئے جانے پر اگلی حالت کی پیش گوئی
- سیکھے ہوئے ماڈل کا استعمال کرتے ہوئے منصوبہ بندی
- زیادہ sample efficient

**طریقے:**
- World models
- سیکھی ہوئی dynamics کے ساتھ Model Predictive Control
- Dyna architecture

## تقلیدی تعلیم (Imitation Learning)

### Behavioral Cloning

**تصور:** ماہر مظاہروں سے supervised learning

**عمل:**
1. ماہر مظاہرے جمع کریں (state, action pairs)
2. ماہر کی نقل کرنے کے لیے policy کی تربیت کریں
3. روبوٹ پر policy کو تعینات کریں

**چیلنجز:**
- Distribution shift
- ماہر ڈیٹا جمع کرنے کی لاگت
- مجموعی غلطیاں

### Inverse Reinforcement Learning (IRL)

**مقصد:** مظاہروں سے reward function سیکھیں

**بدیہی:** ماہر کسی نامعلوم reward کو زیادہ سے زیادہ کرتا ہے

**اطلاقات:**
- انسان-روبوٹ تعاون
- ترجیحات سیکھیں
- حفاظتی رکاوٹیں

### Dataset Aggregation (DAgger)

**Distribution shift کا حل:**
1. ماہر ڈیٹا پر policy کی تربیت کریں
2. Policy چلائیں، حالات جمع کریں
3. نئی حالتوں پر اعمال کے لیے ماہر سے پوچھیں
4. Dataset میں جمع کریں
5. Policy کو دوبارہ تربیت دیں
6. دہرائیں

## ادراک کے لیے Deep Learning

### Convolutional Neural Networks (CNNs)

**بصری پروسیسنگ:**
- Object detection (YOLO, Faster R-CNN)
- Semantic segmentation
- گہرائی کا تخمینہ
- End-to-end visuomotor control

### Recurrent Neural Networks (RNNs)

**وقتی استدلال:**
- تسلسل کی پیش گوئی
- جزوی طور پر قابل مشاہدہ حالات
- LSTMs, GRUs
- ماضی کے مشاہدات کی یادداشت

### Transformers

**Attention mechanism:**
- طویل فاصلے کی dependencies
- Multi-modal fusion (vision + language)
- بہت سے کاموں میں جدید ترین
- مثالیں: ViT, CLIP, RT-1/RT-2

## Sim-to-Real Transfer

### The Reality Gap

**مسئلہ:** simulation میں تربیت یافتہ policies حقیقی روبوٹس پر ناکام ہو جاتی ہیں

**وجوہات:**
- سادہ شدہ physics
- کامل sensors (کوئی شور نہیں)
- کوئی تاخیر نہیں
- مثالی contact dynamics

### Domain Randomization

**نقطہ نظر:** متنوع simulated ماحول پر تربیت دیں

**بے ترتیب بنائیں:**
- Physics parameters (friction, mass)
- بصری ظاہری شکل (textures, lighting)
- Sensor noise
- کیمرے کی پوزیشن

**نتیجہ:** تغیرات کے لیے مضبوط policy، حقیقی دنیا میں عام ہو جاتی ہے

### Domain Adaptation

**تکنیک:** Sim اور real distributions کے درمیان پل بنائیں

**طریقے:**
- GAN-based image translation
- Feature-level adaptation
- تربیت کے دوران privileged information

### System Identification

**حقیقی روبوٹ parameters سیکھیں:**
- حقیقی dynamics کی پیمائش کریں
- Simulator کو fine-tune کریں
- تکراری: sim → real → sim کو update کریں

## انسانی Feedback سے سیکھنا

### Teleoperation

**انسان روبوٹ کو دور سے کنٹرول کرتا ہے:**
- متنوع ڈیٹا جمع کریں
- اعلیٰ معیار کے مظاہرے
- مہنگا لیکن موثر

**Interfaces:**
- VR controllers
- Motion capture
- Haptic devices

### Preference-Based Learning

**درجہ بندی پر مبنی feedback:**
- دو trajectories دکھائیں
- انسان بہتر کو منتخب کرتا ہے
- ترجیحات سے reward سیکھیں
- سکورنگ سے زیادہ قدرتی

### Reinforcement Learning from Human Feedback (RLHF)

**عمل:**
1. مظاہرے جمع کریں
2. ابتدائی policy کی تربیت کریں
3. Trajectories بنائیں
4. انسان trajectories کو درجہ دیتا/اسکور کرتا ہے
5. Reward model کی تربیت کریں
6. سیکھے ہوئے reward کے ساتھ policy کو بہتر بنائیں
7. دہرائیں

## Multi-Task Learning

### مقصد

متعدد کاموں کے لیے واحد policy

### طریقے

**Task Conditioning:**
- Input: (state, task_id)
- ایک network، متعدد outputs
- کاموں میں مشترکہ features

**Meta-Learning:**
- جلدی سیکھنا سیکھیں
- Few-shot adaptation
- MAML، Reptile algorithms

**Hierarchical RL:**
- اعلیٰ سطح کی policy: Subgoal منتخب کریں
- نچلی سطح کی policy: Subgoal حاصل کریں
- Options framework

## روبوٹکس کے لیے Foundation Models

### Vision-Language Models (VLMs)

**مثالیں:** CLIP، DALL-E، GPT-4V

**اطلاقات:**
- زبان کی شرط پر manipulation
- Zero-shot object recognition
- زبان سے task planning

### Robotic Foundation Models

**RT-1 (Robotics Transformer 1):**
- روبوٹک کنٹرول کے لیے Transformer
- 130K روبوٹ trajectories پر تربیت یافتہ
- نئی اشیاء اور ماحول میں عام ہوتا ہے

**RT-2:**
- Vision-language model (PaLi-X) پر بنایا گیا
- قدرتی زبان کے commands
- بہتر عمومیت

**PaLM-E:**
- Embodied multimodal model
- Vision + language + robot state
- 562B parameters

### Large Language Models (LLMs) برائے Planning

**LLMs کو اعلیٰ سطح کے planners کے طور پر استعمال کریں:**
- Input: Task کی تفصیل
- Output: Subgoals کی ترتیب
- مثال: "کافی بنائیں" → اقدامات

**چیلنجز:**
- فزیکل دنیا میں grounding
- حفاظت اور وشوسنییتا
- Hallucination کی روک تھام

## Offline Reinforcement Learning

### محرک

**مقررہ dataset سے سیکھیں:**
- کوئی براہ راست تعامل نہیں (زیادہ محفوظ)
- logged ڈیٹا استعمال کریں
- موجودہ datasets سے فائدہ اٹھائیں

### چیلنجز

**Distributional shift:**
- نئے اعمال کی تلاش نہیں کر سکتے
- محتاط ہونا ضروری ہے

### Algorithms

**Conservative Q-Learning (CQL):**
- Out-of-distribution اعمال کو سزا دیں
- قابل ثابت محفوظ policy بہتری

**Implicit Q-Learning (IQL):**
- واضح maximization سے بچیں
- زیادہ مستحکم

## حفاظت اور مضبوطی

### محفوظ تلاش

**Constrained RL:**
- بہتری میں حفاظتی رکاوٹیں
- غیر محفوظ رویے پر ضمانت شدہ حدود

**Shielding:**
- حفاظتی layer غیر محفوظ اعمال کو override کرتی ہے
- Formal verification طریقے

### Adversarial Robustness

**Input perturbations کے لیے مضبوطی:**
- مشاہدات میں چھوٹی تبدیلیاں
- Adversarial training
- Certified defenses

### Out-of-Distribution Detection

**بے ضابطگیوں کا پتہ لگائیں:**
- تربیتی ڈیٹا سے دور inputs
- عدم یقینی کی مقدار کا تعین
- محفوظ رویے پر واپسی

## عملی تحفظات

### Sample Efficiency

**چیلنج:** حقیقی روبوٹ ڈیٹا مہنگا ہے

**حل:**
- Model-based RL
- Transfer learning
- Sim-to-real
- Offline RL

### Reward Engineering

**Reward shaping:**
- Sparse rewards سیکھنا مشکل ہے
- گھنے درمیانی rewards
- استحصال بمقابلہ تلاش میں توازن

**Intrinsic motivation:**
- تجسس پر مبنی تلاش
- نیا پن کے bonuses

### Hyperparameter Tuning

**بہت سے hyperparameters:**
- Learning rate
- Network architecture
- تلاش کے parameters

**طریقے:**
- Grid search
- Random search
- Bayesian optimization
- Population-based training

## حقیقی دنیا کی کامیابی کی کہانیاں

### DeepMimic (2018)

- اکروبیٹک حرکات سیکھیں
- Motion capture ڈیٹا کی نقل کریں
- Backflips، spins، rolls
- Quadruped میں sim-to-real transfer

### Learning Dexterity (OpenAI، 2019)

- Rubik's cube manipulation
- Simulation میں خالص RL
- Domain randomization
- Shadow hand (24 DOF)

### Mobile ALOHA (2024)

- Bimanual mobile manipulation
- Imitation learning
- کھانا پکانا، صفائی کے کام
- کم لاگت والا hardware

## مستقبل کی سمتیں

### Foundation Models

- بڑے datasets پر پہلے سے تربیت یافتہ
- مخصوص کاموں کے لیے fine-tune
- Few-shot learning
- Multi-modal reasoning

### World Models

- ماحولیاتی dynamics سیکھیں
- Latent space میں منصوبہ بندی کریں
- تخیل پر مبنی منصوبہ بندی

### Lifelong Learning

- بھولے بغیر مسلسل سیکھنا
- آن لائن موافقت
- مجموعی skill کا حصول

### Embodied AI

- LLMs اور VLMs کے ساتھ انضمام
- فزیکل دنیا کے بارے میں استدلال
- روبوٹکس میں عقل عام

## خلاصہ

مشین لرنینگ کے ذریعے قابل بناتی ہے:
- تجربے سے سیکھنا (RL)
- مظاہروں سے سیکھنا (imitation)
- مضبوط sim-to-real transfer
- Multi-task عمومیت
- روبوٹکس کے لیے foundation models

کلیدی تبادلے:
- Sample efficiency بمقابلہ حتمی کارکردگی
- Model-free بمقابلہ model-based
- Simulation بمقابلہ حقیقی دنیا کی تربیت

## کلیدی نکات

✓ RL آزمائش اور غلطی کے ذریعے رویے کو بہتر بناتا ہے
✓ Imitation learning ماہرانہ علم سے فائدہ اٹھاتا ہے
✓ Sim-to-real transfer کو domain randomization کی ضرورت ہے
✓ Foundation models زبان کی شرط پر کنٹرول کو قابل بناتے ہیں
✓ تعیناتی کے لیے حفاظت اور مضبوطی اہم ہیں

---

**پچھلا:** [باب 5: Manipulation اور Grasping](./chapter-5.md)
**اگلا:** [باب 7: انسان-روبوٹ تعامل](./chapter-7.md)

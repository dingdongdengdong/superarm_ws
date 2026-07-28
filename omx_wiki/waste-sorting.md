# Waste Sorting Pipeline

## Recommended design

Use a modular pipeline rather than one large end-to-end VLA:

```text
RGB-D image
   ↓
instance segmentation
   ↓
class and mask
   ↓
class-to-bin rule
   ↓
selected target and destination
   ↓
ACT or motion planner
   ↓
pick and place
```

## Perception

Instance segmentation is preferred over bounding-box-only detection because irregular or overlapping waste requires a more precise object boundary.

Recommended outputs per object:

```text
class_id
confidence
instance_mask
3D centroid or grasp point
estimated orientation
```

Use RGB-D and camera calibration to project the selected mask into robot coordinates.

## Sorting rule

For a fixed service definition, use deterministic routing:

```python
WASTE_TO_BIN = {
    "aluminum_can": "recycle",
    "pet_bottle": "recycle",
    "paper": "recycle",
    "plastic_bag": "general_or_policy_defined",
    "tissue": "general",
    "unknown": "reject",
}
```

Keep local recycling rules configurable. Do not hard-code uncertain material policies inside the learned motion model.

## Target selection

When several objects are visible, score candidates using factors such as:

- detection confidence;
- reachable workspace;
- distance from the end effector;
- occlusion;
- estimated grasp quality;
- operational priority.

The first implementation may simply choose the nearest high-confidence reachable object.

## Grasp abstraction

Do not expose every AmazingHand joint initially.

Start with:

```text
arm: five commands
hand: one grasp scalar
```

Later extend to:

```text
grasp_mode = pinch | wrap | wide
closure = 0.0 … 1.0
```

The hand controller maps these low-dimensional commands to finger-joint targets.

## Teacher behavior by object type

The teacher may use object-specific grasp templates:

| Waste type | Initial grasp strategy |
|---|---|
| Can | Side wrap or top grasp |
| PET bottle | Side wrap |
| Paper | Top pinch or edge pinch |
| Plastic bag | Corner/edge pinch |
| Cup | Side or rim grasp |

Begin with rigid objects before deformable waste.

## Is language required?

No, not for a fixed sorting service.

Use structured conditions:

```text
target_class = aluminum_can
destination_bin = recycle
```

Language becomes useful only when instructions vary, for example:

- “Collect cans first.”
- “Do not touch transparent bottles.”
- “Move contaminated plastic to inspection.”

In that case, use a VLM or small VLA as a high-level selector and keep ACT or the planner as the low-level manipulation policy.

## Dataset requirements

Every episode should record:

- visible object set;
- selected target;
- selected destination;
- camera images;
- robot state;
- executed action;
- success and failure reason;
- object class and pose;
- source domain: simulation or real.

## 한국어 요약

쓰레기 분류는 YOLO segmentation, 분리배출 rule, target selector, ACT manipulation으로 나누는 것이 좋습니다. 고정 작업에는 언어가 필요하지 않으며 `target_class`와 `destination_bin` 조건만으로 충분합니다.

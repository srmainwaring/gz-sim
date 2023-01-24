# Readme

Notes

1. Use scene->DestroyVisual(visual, true) to remove visuals, otherwise
   we are left with memory leaks, and the number of visual nodes
   increases each time the filter size is reset.
 2. The Arrow and Axis visuals seem to have loops in the scene tree, as
    when they are removed using DestroyVisual with recursion set,
    a warning is issues by BaseScene L297
 3. Issue 2 is not seen when using a simple primitive such as Sphere.


Pose
- Topic: topic string
  - Depth: depth of incoming message queue
  - History Policy: System Default, Keep Last, Keep All.
  - Reliability Policy: System Default, Reliable, Best Effort.
  - Durability Policy: System Default, Transient Local, Volatile.
  - Filter size: 10
- Shape: Arrow, Axes
  - Arrow:
    - Color: 255, 25, 0
    - Alpha: 1
    - Shaft Length: 1
    - Shaft Radius: 0.05
    - Head Length: 0.3
    - Head Radius: 0.1
  - Axes:
    - Axes Length: 1
    - Axes Radius: 0.1


## Integration Tests

When running integration tests from the command line specify both the 
render engine and backend:

```bash
GZ_ENGINE_TO_TEST="ogre2" GZ_ENGINE_BACKEND="metal" ./build/gz-rendering7/bin/INTEGRATION_scene --gtest_filter="*AddRemove*"
```


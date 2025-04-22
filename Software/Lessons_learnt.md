## Lessons Learnt

### Nav2 Parameters Test

We should have conducted more testing and fine-tuning of the Nav2 stack, particularly in terms of how the costmap updates in real time. During our run, the costmap update rate was too slow, which hindered the robot’s ability to quickly react to obstacles and changes in the environment.

**Potential Improvements:**

- Increase the frequency of costmap updates
- Expand the `laser_scan_max_range` parameter
- Better tune the inflation radius and cost scaling factors

These changes could have made the robot more responsive in navigating complex mazes, while creating a more accurate buffer zone around the robot to reduce the risk of collisions.

---

### Heat Sensor Calibration

During the actual run, the heat sensing logic had issues where it fired too early or could not detect the heat source. This revealed the need to fine-tune heat detection thresholds and the conditions for triggering flare firing.

**Suggested Action:**

- Further calibration in a controlled environment
- Test with various distances and temperatures

This would help achieve more reliable detections and minimize false triggers.

---

### Servo Sweep

Initially, the heat sensor was mounted on a servo to perform a sweeping motion (180°) to expand detection coverage. While conceptually viable, it introduced several technical issues:

- The servo frequently fell off, affecting recorded values
- Constant turning likely increased noise in the readings

---

### Navigation Strategy

Our initial strategy relied on frontier-based exploration. However, this proved inefficient in the maze we were given, causing the robot to spend excessive time in central regions and miss heat sources placed near corners.

**Future Strategy Ideas:**

- Adopt a wall-following strategy for better edge coverage
- Consider a hybrid strategy: prioritize frontiers adjacent to walls
- Use LiDAR distance readings to explore open areas quickly

While navigating toward larger open spaces can help collect map data efficiently, it also risks missing smaller yet important sections. A balance must be struck based on environment constraints and task priorities.


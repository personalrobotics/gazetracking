# gazetracking
Eye gaze tracking using Pupil Labs head-mounted tracker.

## Publishing gaze positions with ROS
Start Pupil Capture. Ensure that the Pupil Remote plugin is running (available in Pupil Capture under "General" -> "Open plugin").

Ensure that the Pupil Remote `IP` and `port` are set to match the values in `pupil_listener.py` (usually `tcp://127.0.0.1:50020`).

Run the pupil listener:
```bash
rosrun gazetracking pupil_listener.py
```

This should publish information on two ROS topics: `pupil_info` and `gaze_info`. The `gaze_info` topic contains a subset of the information from `pupil_info` that is most commonly used, specifically the position of the gaze (`norm_pos`) and the confidence value (`confidence`).

For more information about the data contained in `pupil_info`, see the Pupil Capture website here: [https://github.com/pupil-labs/pupil/wiki/Data-Format](https://github.com/pupil-labs/pupil/wiki/Data-Format).



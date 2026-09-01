# brov_msgs

Topic names specific to the BlueROV2 Heavy (Mr. Pinchy).

See [`msg/Topics.msg`](./msg/Topics.msg). Use these constants when creating
subs/pubs rather than re-typing the strings:

```python
from brov_msgs.msg import Topics as BrovTopics

self.create_subscription(OverrideRCIn, BrovTopics.RC_OVERRIDE_TOPIC, cb, 10)
```

Topics that really are meant to be changed at launch time stay parameters, with
their default value taken from here — see `bluerov_sitl`'s `imu_topic`.

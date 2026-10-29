"""Per-topic timestamp checks independent of DDS cross-topic delivery order."""


class TopicStamps:
    def __init__(self):
        self.latest = {}

    def record(self, topic, stamp):
        assert stamp >= 0, (topic, stamp)
        assert stamp > self.latest.get(topic, -1), (topic, "Stamp did not advance", stamp)
        self.latest[topic] = stamp

    def coherent(self):
        clock = self.latest.get("/clock")
        return clock is not None and all(
            stamp <= clock + 100_000_000 for stamp in self.latest.values()
        )

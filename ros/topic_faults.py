"""Private transport controls, independent of ROS message types and simulation code."""

import heapq
import random

import msgpack
import zmq


class TopicFaults:
    def __init__(self, context, private_dir):
        self.socket = context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(f"ipc://{private_dir / 'bridge.sock'}")
        self.waiting = False
        self.plan = None
        self.rules = {}
        self.audit = {}
        self.pending = []
        self.pending_bytes = 0
        self.sequence = 0
        self.now_ns = 0

    def poll(self):
        if self.waiting and self.socket.poll(0):
            plan = msgpack.unpackb(self.socket.recv(), raw=False)
            self.waiting = False
            if plan != self.plan:
                self.plan = plan
                self.rules = {}
                for fault in plan["faults"]:
                    rule = self.rules.setdefault(fault["topic"], {})
                    rule.update(fault)
                self.audit = {}
        if not self.waiting:
            self.socket.send(msgpack.packb({
                "op": "poll", "audit": self.audit,
                "episode": self.plan["episode"] if self.plan else None,
            }, use_bin_type=True))
            self.waiting = True

    def submit(self, topic, publish, message, source_ns):
        rule = self.rules.get(topic)
        if rule is None:
            publish(message)
            return
        audit = self.audit.setdefault(topic, {"received": 0, "dropped": 0, "delivered": 0})
        audit["received"] += 1
        draw = random.Random(f"{self.plan['seed']}:{topic}:{source_ns}").random()
        if draw < rule.get("rate", 0):
            audit["dropped"] += 1
            return
        due = self.now_ns + round(rule.get("ms", 0) * 1e6)
        size = len(getattr(message, "data", b""))
        if len(self.pending) >= 4096 or self.pending_bytes + size > 64 * 1024 * 1024:
            audit["overflow"] = audit.get("overflow", 0) + 1
            return
        self.sequence += 1
        self.pending_bytes += size
        heapq.heappush(
            self.pending, (due, self.sequence, publish, message, self.now_ns, size, audit),
        )
        self.advance(self.now_ns)

    def advance(self, now_ns):
        self.now_ns = max(self.now_ns, now_ns)
        while self.pending and self.pending[0][0] <= self.now_ns:
            _, _, publish, message, queued_ns, size, audit = heapq.heappop(self.pending)
            self.pending_bytes -= size
            publish(message)
            audit["delivered"] += 1
            delay = self.now_ns - queued_ns
            audit["min_delay_ns"] = min(audit.get("min_delay_ns", delay), delay)
            audit["max_delay_ns"] = max(audit.get("max_delay_ns", delay), delay)

    def close(self):
        self.socket.close()

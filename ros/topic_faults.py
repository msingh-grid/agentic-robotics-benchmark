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
                    rule["rng"] = random.Random(f"{plan['seed']}:{fault['topic']}")
                self.audit = {}
        if not self.waiting:
            self.socket.send(msgpack.packb({"op": "poll", "audit": self.audit}, use_bin_type=True))
            self.waiting = True

    def submit(self, topic, publish, message, source_ns):
        rule = self.rules.get(topic)
        if rule is None:
            publish(message)
            return
        audit = self.audit.setdefault(topic, {"received": 0, "dropped": 0, "delivered": 0})
        audit["received"] += 1
        if rule["rng"].random() < rule.get("rate", 0):
            audit["dropped"] += 1
            return
        due = self.now_ns + round(rule.get("ms", 0) * 1e6)
        if len(self.pending) >= 4096:
            audit["overflow"] = audit.get("overflow", 0) + 1
            return
        self.sequence += 1
        heapq.heappush(
            self.pending, (due, self.sequence, topic, publish, message, source_ns),
        )
        self.advance(self.now_ns)

    def advance(self, now_ns):
        self.now_ns = max(self.now_ns, now_ns)
        while self.pending and self.pending[0][0] <= self.now_ns:
            _, _, topic, publish, message, source_ns = heapq.heappop(self.pending)
            publish(message)
            audit = self.audit[topic]
            audit["delivered"] += 1
            delay = self.now_ns - source_ns
            audit["min_delay_ns"] = min(audit.get("min_delay_ns", delay), delay)
            audit["max_delay_ns"] = max(audit.get("max_delay_ns", delay), delay)

    def close(self):
        self.socket.close()

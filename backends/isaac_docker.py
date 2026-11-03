"""Private capability placeholders for the future real sandbox."""


class Lifecycle:
    def __init__(self):
        # TODO: connect the Isaac sandbox's lifecycle and recording controls.
        raise NotImplementedError("Isaac lifecycle is not available")


class GroundTruth:
    def __init__(self, private_dir):
        self.private_dir = private_dir

    def snapshot(self):
        # TODO: connect the Isaac sandbox's grader-only measurement channel.
        raise NotImplementedError("Isaac ground truth is not available")


class Faults:
    def __init__(self, private_dir):
        self.private_dir = private_dir

    def configure(self, scenario, seed):
        # TODO: connect the Isaac sandbox's harness-only fault channel.
        raise NotImplementedError("Isaac faults are not available")

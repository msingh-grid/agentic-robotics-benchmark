"""Require finalized MCAP files with message counts matching rosbag metadata."""

import argparse
from collections import Counter
from pathlib import Path

import rosbag2_py

from public_contract import ACTION_TOPICS, TOPICS

MAGIC = b"\x89MCAP0\r\n"


def validate(path):
    metadata = rosbag2_py.Info().read_metadata(str(path), "mcap")
    if metadata.storage_identifier != "mcap" or metadata.message_count <= 0:
        raise ValueError("No finalized MCAP messages")
    if not metadata.relative_file_paths:
        raise ValueError("MCAP file list is empty")
    for filename in metadata.relative_file_paths:
        with (path / filename).open("rb") as source:
            if source.read(8) != MAGIC:
                raise ValueError("Invalid MCAP header")
            source.seek(-8, 2)
            if source.read(8) != MAGIC:
                raise ValueError("MCAP footer is missing")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    counts = Counter()
    while reader.has_next():
        topic, _, _ = reader.read_next()
        if topic not in (*TOPICS, *ACTION_TOPICS):
            raise ValueError("Unexpected topic in episode recording")
        counts[topic] += 1
    expected = {
        item.topic_metadata.name: item.message_count
        for item in metadata.topics_with_message_count
    }
    if sum(counts.values()) != metadata.message_count or any(
        counts[topic] != count for topic, count in expected.items()
    ) or set(counts) - set(expected):
        raise ValueError("MCAP messages do not match finalized metadata")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    validate(args.bag)
    print("MCAP_FINALIZED_OK", flush=True)


if __name__ == "__main__":
    main()

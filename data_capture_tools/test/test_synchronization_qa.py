"""Unit tests for synchronization QA timing calculations."""

from data_capture_tools.synchronization_qa import (
    TopicSamples,
    _event_coverage,
    _nearest_offsets_ms,
    _topic_metrics,
)


def test_topic_metrics_use_header_and_signed_bag_offset() -> None:
    samples = TopicSamples(
        name="/sensor",
        type_name="example/msg/Stamped",
        bag_ns=[1_010_000_000, 1_110_000_000, 1_210_000_000],
        header_ns=[1_000_000_000, 1_100_000_000, 1_200_000_000],
    )

    metrics = _topic_metrics(samples)

    assert metrics["header_available"] is True
    assert metrics["median_hz"] == 10.0
    assert metrics["bag_minus_header_median_ms"] == 10.0
    assert metrics["backward_jumps"] == 0


def test_headerless_topic_uses_bag_timeline() -> None:
    samples = TopicSamples(
        name="/legacy",
        type_name="std_msgs/msg/Float32",
        bag_ns=[1_000_000_000, 1_050_000_000, 1_100_000_000],
        header_ns=[None, None, None],
    )

    metrics = _topic_metrics(samples)

    assert metrics["header_available"] is False
    assert metrics["timestamp_basis"] == "bag_receive"
    assert metrics["median_hz"] == 20.0
    assert metrics["bag_minus_header_median_ms"] is None


def test_nearest_offsets_exclude_nonoverlapping_edges() -> None:
    result = _nearest_offsets_ms(
        [0, 1_001_000_000, 1_999_000_000, 4_000_000_000],
        [1_000_000_000, 2_000_000_000],
    )

    assert result["pairs"] == 2
    assert result["median_ms"] == 0.0


def test_event_coverage_computes_action_defined_hold() -> None:
    action = TopicSamples(name="/robot_action_event", type_name="test")
    required = (
        "trial_run_received",
        "phase1_goal_sent",
        "phase1_action_succeeded",
        "phase2_close_goal_sent",
        "phase2_close_action_succeeded",
        "hold_start",
        "hold_end",
        "phase3_open_goal_sent",
        "phase3_open_action_succeeded",
        "sequence_finished",
    )
    action.events = [
        {"event": name, "stamp_ns": index * 1_000_000_000}
        for index, name in enumerate(required)
    ]

    coverage = _event_coverage({"/robot_action_event": action})

    assert coverage["complete"] is True
    assert coverage["hold_duration_sec"] == [1.0]

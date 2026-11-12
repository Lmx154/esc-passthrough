//! Host tests for `esc_passthrough::trigger`.
//!
//! The one request recognised is `24 4D 3C 00 01 01`: `$`, `M`, `<`, length 0, command 1
//! (`MSP_API_VERSION`), checksum 0 ^ 1 = 1. It is what the configurator sends first and again after
//! every restart (`_plan/ref/am32-configurator/components/SerialDevice.vue:618`, four requests of
//! the same shape follow it) and what the decision record names for the trigger
//! (`_plan/decision-esc-link-20261112.md`, second amendment, point 1). The expected byte count of a
//! firing is worked out here by a naive search over the whole stream, which shares no code with the
//! crate.
//!
//! The streams that carry the request in the properties below are random bytes, and random
//! well-formed frames of another format: `0xFD`, a length byte, that many bytes. The recogniser
//! knows neither.

use esc_passthrough::trigger::{MspTrigger, REQUEST};
use proptest::prelude::*;

const BYTES: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];

/// Offsets (one past the last byte) of every occurrence of `BYTES` in `stream`, by comparing every
/// window.
fn naive_ends(stream: &[u8]) -> Vec<usize> {
    let mut ends = Vec::new();
    let mut start = 0;
    while start + BYTES.len() <= stream.len() {
        if stream[start..start + BYTES.len()] == BYTES {
            ends.push(start + BYTES.len());
        }
        start += 1;
    }
    ends
}

/// Feeds `stream` in the given chunks, restarting on the rest of a chunk after each firing, and
/// returns the offsets (in the whole stream) of every firing's last byte plus one.
fn fire_ends(stream: &[u8], cuts: &[usize]) -> Vec<usize> {
    let mut points: Vec<usize> = cuts.iter().map(|c| c % (stream.len() + 1)).collect();
    points.push(0);
    points.push(stream.len());
    points.sort_unstable();
    let mut trigger = MspTrigger::new();
    let mut ends = Vec::new();
    for pair in points.windows(2) {
        let base = pair[0];
        let mut rest = &stream[pair[0]..pair[1]];
        let mut used_total = 0;
        while let Some(n) = trigger.feed(rest) {
            assert!(
                n >= 1 && n <= rest.len(),
                "n = {n} for {} bytes",
                rest.len()
            );
            used_total += n;
            ends.push(base + used_total);
            rest = &rest[n..];
        }
    }
    ends
}

// ------------------------------------------------------------------ pub items

#[test]
fn request_constant_is_the_six_bytes() {
    assert_eq!(REQUEST, BYTES);
    // The checksum is the XOR of length and command.
    assert_eq!(REQUEST[3] ^ REQUEST[4], REQUEST[5]);
}

#[test]
fn new_has_nothing_matched() {
    assert_eq!(MspTrigger::new(), MspTrigger::default());
    // Fresh: the request in one piece fires at its sixth byte.
    assert_eq!(MspTrigger::new().feed(&BYTES), Some(6));
    // And the first five bytes alone do not.
    assert_eq!(MspTrigger::new().feed(&BYTES[..5]), None);
}

#[test]
fn feed_reports_the_bytes_used_including_the_last_one() {
    let mut trigger = MspTrigger::new();
    assert_eq!(trigger.feed(&BYTES), Some(6));

    let mut stream = vec![0x00, 0xFF, 0x24];
    stream.extend_from_slice(&BYTES);
    stream.extend_from_slice(&[0x55, 0x66]);
    let mut trigger = MspTrigger::new();
    // 3 bytes of noise, 6 of request: 9 used; the two after it are left alone.
    assert_eq!(trigger.feed(&stream), Some(9));
    assert_eq!(trigger.feed(&stream[9..]), None);
}

#[test]
fn feed_returns_none_for_other_traffic_and_never_for_a_prefix_alone() {
    let mut trigger = MspTrigger::new();
    assert_eq!(trigger.feed(&[]), None);
    assert_eq!(
        trigger.feed(&[0x00, 0x24, 0x4D, 0x3C, 0x00, 0x02, 0x02]),
        None
    );
    assert_eq!(trigger.feed(&BYTES[..5]), None);
}

#[test]
fn feed_keeps_a_partial_match_for_the_next_call() {
    for split in 0..=BYTES.len() {
        let mut trigger = MspTrigger::new();
        let first = trigger.feed(&BYTES[..split]);
        if split < BYTES.len() {
            assert_eq!(first, None, "split {split}");
            assert_eq!(
                trigger.feed(&BYTES[split..]),
                Some(BYTES.len() - split),
                "split {split}"
            );
        } else {
            assert_eq!(first, Some(6));
        }
    }
}

#[test]
fn feed_fires_once_per_occurrence_and_starts_again_after_firing() {
    let mut stream = BYTES.to_vec();
    stream.extend_from_slice(&BYTES);
    let mut trigger = MspTrigger::new();
    assert_eq!(trigger.feed(&stream), Some(6));
    assert_eq!(trigger.feed(&stream[6..]), Some(6));
    assert_eq!(trigger.feed(&stream[12..]), None);
    // A third copy cut in half across calls after a firing.
    assert_eq!(trigger.feed(&BYTES), Some(6));
    assert_eq!(trigger.feed(&BYTES[..3]), None);
    assert_eq!(trigger.feed(&BYTES[3..]), Some(3));
}

#[test]
fn feed_restarts_a_match_on_a_dollar_inside_a_failed_one() {
    // `$` `M` `$` `M` `<` ...: the second `$` is a new start, not noise.
    let stream = [0x24, 0x4D, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
    assert_eq!(MspTrigger::new().feed(&stream), Some(8));
    // `$` `$` `M` ...
    let stream = [0x24, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
    assert_eq!(MspTrigger::new().feed(&stream), Some(7));
    // The partial match is `$M<`, `$M<` again.
    let stream = [0x24, 0x4D, 0x3C, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
    assert_eq!(MspTrigger::new().feed(&stream), Some(9));
}

#[test]
fn feed_ignores_every_single_byte_corruption() {
    for position in 0..BYTES.len() {
        for value in 0..=255u8 {
            if value == BYTES[position] {
                continue;
            }
            let mut corrupted = BYTES;
            corrupted[position] = value;
            for split in 0..=corrupted.len() {
                let mut trigger = MspTrigger::new();
                assert_eq!(trigger.feed(&corrupted[..split]), None);
                assert_eq!(
                    trigger.feed(&corrupted[split..]),
                    None,
                    "byte {position} = {value:#04x}"
                );
            }
        }
    }
}

#[test]
fn feed_recovers_after_a_corrupted_request() {
    for position in 0..BYTES.len() {
        let mut stream = BYTES.to_vec();
        stream.extend_from_slice(&{
            let mut corrupted = BYTES;
            corrupted[position] ^= 0x10;
            corrupted
        });
        stream.extend_from_slice(&BYTES);
        assert_eq!(
            fire_ends(&stream, &[]),
            vec![6, 18],
            "corrupted byte {position}"
        );
    }
}

#[test]
fn feed_ignores_any_other_length_or_command_with_a_valid_checksum() {
    for length in 0..=255u8 {
        for command in 0..=255u8 {
            let mut frame = vec![0x24, 0x4D, 0x3C, length, command];
            frame.extend(std::iter::repeat_n(0u8, usize::from(length)));
            frame.push(length ^ command);
            let mut trigger = MspTrigger::new();
            let result = trigger.feed(&frame);
            if length == 0 && command == 1 {
                assert_eq!(result, Some(6));
            } else {
                assert_eq!(result, None, "length {length} command {command}");
            }
        }
    }
}

#[test]
fn feed_ignores_the_request_with_the_other_directions() {
    for direction in [0x3E, 0x21, 0x00, 0xFF] {
        let stream = [0x24, 0x4D, direction, 0x00, 0x01, 0x01];
        assert_eq!(
            MspTrigger::new().feed(&stream),
            None,
            "direction {direction:#04x}"
        );
    }
    // Version 2 framing.
    assert_eq!(
        MspTrigger::new().feed(&[0x24, 0x58, 0x3C, 0x00, 0x01, 0x01]),
        None
    );
}

// ---------------------------------------------------------------- properties

fn noisy_byte() -> impl Strategy<Value = u8> {
    // Heavy on the bytes of the request, so that partial matches are common.
    prop_oneof![
        3 => any::<u8>(),
        1 => Just(0x24u8),
        1 => Just(0x4Du8),
        1 => Just(0x3Cu8),
        1 => Just(0x00u8),
        1 => Just(0x01u8),
    ]
}

fn random_bytes() -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(noisy_byte(), 0..300)
}

/// Well-formed frames of another format: 0xFD, a length, that many bytes.
fn framed_bytes() -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(prop::collection::vec(noisy_byte(), 0..60), 0..10).prop_map(|payloads| {
        let mut stream = Vec::new();
        for payload in payloads {
            stream.push(0xFD);
            stream.push(payload.len() as u8);
            stream.extend_from_slice(&payload);
        }
        stream
    })
}

/// `base` with the request spliced in at each of `positions` (taken modulo the base length + 1,
/// in the base's own coordinates, so that no insertion lands inside another).
fn insert_requests(base: &[u8], positions: &[usize]) -> Vec<u8> {
    let mut points: Vec<usize> = positions.iter().map(|p| p % (base.len() + 1)).collect();
    points.sort_unstable();
    let mut stream = Vec::new();
    let mut from = 0;
    for point in points {
        stream.extend_from_slice(&base[from..point]);
        stream.extend_from_slice(&BYTES);
        from = point;
    }
    stream.extend_from_slice(&base[from..]);
    stream
}

proptest! {
    // Invariant 5.5: the 6 bytes inserted anywhere in random bytes give exactly one firing each,
    // at every split of the stream.
    #[test]
    fn insertions_in_random_bytes_fire_exactly_once_each(
        base in random_bytes(),
        positions in prop::collection::vec(any::<usize>(), 0..5),
        cuts in prop::collection::vec(any::<usize>(), 0..12),
    ) {
        prop_assume!(naive_ends(&base).is_empty());
        let stream = insert_requests(&base, &positions);
        let ends = fire_ends(&stream, &cuts);
        prop_assert_eq!(ends.len(), positions.len());
        prop_assert_eq!(&ends, &naive_ends(&stream));
        // And byte by byte.
        let singles: Vec<usize> = (0..stream.len()).collect();
        prop_assert_eq!(fire_ends(&stream, &singles), ends);
    }

    #[test]
    fn insertions_in_framed_streams_fire_exactly_once_each(
        base in framed_bytes(),
        positions in prop::collection::vec(any::<usize>(), 0..5),
        cuts in prop::collection::vec(any::<usize>(), 0..12),
    ) {
        prop_assume!(naive_ends(&base).is_empty());
        let stream = insert_requests(&base, &positions);
        let ends = fire_ends(&stream, &cuts);
        prop_assert_eq!(ends.len(), positions.len());
        prop_assert_eq!(&ends, &naive_ends(&stream));
    }

    // With no insertion the count is whatever a naive search finds in the random bytes.
    #[test]
    fn streams_without_an_insertion_fire_as_a_naive_search_does(
        stream in prop_oneof![random_bytes(), framed_bytes()],
        cuts in prop::collection::vec(any::<usize>(), 0..12),
    ) {
        prop_assert_eq!(fire_ends(&stream, &cuts), naive_ends(&stream));
    }

    // Bytes that never contain a `$` cannot fire at all.
    #[test]
    fn streams_without_a_dollar_never_fire(
        stream in prop::collection::vec(any::<u8>().prop_filter("not $", |b| *b != 0x24), 0..300),
        cuts in prop::collection::vec(any::<usize>(), 0..12),
    ) {
        prop_assert!(fire_ends(&stream, &cuts).is_empty());
    }

    // Arbitrary bytes never panic, and a firing always reports a plausible count.
    #[test]
    fn arbitrary_bytes_never_panic(
        chunks in prop::collection::vec(prop::collection::vec(any::<u8>(), 0..40), 0..20),
    ) {
        let mut trigger = MspTrigger::new();
        for chunk in &chunks {
            let mut rest = chunk.as_slice();
            while let Some(n) = trigger.feed(rest) {
                prop_assert!(n >= 1 && n <= rest.len());
                rest = &rest[n..];
            }
        }
    }
}

# Serial request deadline and connection diagnosis

This update is for the repository's pinned PyModbus 3.11.3 / pySerial 3.5.
It retains ASCII, 9600 baud, 8N1, zero retries, the register map, and the existing
motion interlocks. A port that opens is not proof that either motor has replied.

## Why the change is needed

PyModbus's synchronous receive loop repeatedly calls `recv()` until a valid
frame or an empty read. The individual serial receive wait has a timeout, but
continued nonmatching data can keep the overall request running. Its receive
wait also uses wall-clock time. A blocked startup leaves the GUI waiting for
drive acknowledgement and feedback, so Enable remains unavailable.

`BoundedSerialClient` supplies a single 0.300 s monotonic budget for each request,
including serial write and all receive fragments. It retains the PyModbus
ASCII/LRC/address parser. Reads consume available bytes instead of waiting for
the incoming buffer to become quiet. Writes have a finite timeout. No retry or
automatic reconnection is introduced. A failed request returns to the existing
fault handler, which attempts to disable both selected drives.

This bounds the application receive loop, not arbitrary kernel stalls or OS
scheduling delays. It cannot guarantee a motor stop after communication loss.
The separate 0.650 s encoder-read limit and 2 s freshness check are unchanged.

## Run the read-only check

Close Studio and other programs using the adapter. From the repository root:

```bash
timeout --signal=TERM --kill-after=2s 15s uv run python -m spirob_cable.diagnose
```

The outer Linux timeout covers the whole diagnostic process, including an OS
open failure or stall. If it terminates the command, report that too.

The diagnostic reads `config.json` without modifying it, opens each distinct
port once, and tests the configured active IDs sequentially. It reads holding
register 10 (LPR) and 22/20/22 (encoder). It NEVER calls disable, enable, target,
speed, or configuration writes. If one ID fails, it still checks the next ID.

Each result includes the last request's transmitted bytes, received byte count,
first 64 received bytes, duration, and discarded pre-request backlog count:

- `PASS`: valid register responses with expected LPR and encoder timing. Motor
  movement and write acknowledgements have not been tested.
- `RX bytes=0`: no response bytes were received within the request budget.
- Nonzero RX with deadline failure: bytes arrived but no valid matching response
  was completed. Inspect the bytes before attributing this to noise, echo,
  another device, baud/framing mismatch, or wiring.
- `CHECK`: a response exists, but LPR differs from 334 or encoder acquisition
  exceeds the current application limit. The diagnostic changes neither value.

The GUI now says `Connecting… Cancel` while startup checks run, exposes fault
diagnostics, preserves the original fault after worker shutdown, and explains
disabled Enable controls in their tooltips. A normal reconnect clears the
previous fault display; it does not automatically enable the motors.

## Verification

59 automated tests pass, including the real PyModbus parser with fragmented
valid frames, invalid checksums, wrong IDs, continuous noise, silent devices,
write acknowledgements and startup failure display/recovery. On a Linux pseudo
terminal injecting continuous junk, the original client exceeded 0.800 s and
needed the test watchdog; the updated client failed at approximately 0.300 s.
A second pseudo-terminal test verified that a noisy ID 5 does not prevent the
diagnostic from checking a valid ID 7, and that every transmitted request was a
holding-register read. No physical motor hardware was connected during these
checks. The user's actual received bytes still need to be examined.

## Commit after applying the patch

```bash
git add spirob_cable/transport.py spirob_cable/diagnose.py spirob_cable/drive.py spirob_cable/ui.py tests/test_transport.py tests/test_rig.py tests/test_ui.py docs/SERIAL_DIAGNOSTICS.md
git diff --cached --stat
git commit -m "Bound serial transactions and retain connection fault diagnostics"
git push
```

Review the staged file list before committing. Use the existing SSH remote and
branch; do not reinitialize the repository.

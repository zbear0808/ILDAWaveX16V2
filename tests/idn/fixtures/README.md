# IDN fixtures

Literal, spec-derived IDN-Hello / IDN-RT / IDN-Stream datagrams and expected
outcomes. They are the test oracle for the Python sender and response parsers
(`tests/idn/test_sender.py`). The native tests under `tests/idn/native/` embed
their own spec-derived bytes next to each case.

## Provenance and independence

- Bytes and expected interpretations were laid out by hand from the field
  tables of **IDN-Stream Revision 002 (July 2025)** and the **IDN-Hello draft
  2022-03-27**, not produced by `tools/idn_sender.py` or the firmware. Every
  file names both documents under `sources`; every segment and fixture cites
  section and page (`"Stream 2.2 p10"`, `"Hello 6.3.3 p25"`).
- The PDFs are not in the repository (redistribution not confirmed). Nothing
  here reads them at test time.
- Broken responses are single-feature variations: `variation_of` names the
  valid fixture and `varied` says what changed.
- `"interpretation": "normative"` means the outcome follows directly from a
  SHALL/SHOULD in the cited text. `"project"` marks this project's reading of
  something the drafts leave open; its `note` says why. Do not treat project
  outcomes as the standard.
- Passing these fixtures is not a conformance claim.
- A fixture belongs here only if a test reads it.

## Files

| File | Contents |
|---|---|
| `hello-management.json` | ping, scan, service map requests and responses (valid and broken) |
| `rt-ack.json` | IDN-RT acknowledgements (success, 0xEB, event flags, broken) |
| `rt-stream-valid.json` | routing-only Void, first data message with the 8-octet XYRGBI dictionary (service ID 0 and 1), follow-on data, channel close, connection close, abort |
| `sender.json` | what the sender must emit for given parameters and input points |
| `batching.json` | tables for batch planning, per-packet capacity and cumulative timing |

## Schema (`idn-fixtures/1`)

Each file: `{schema, family, description, sources, fixtures: [...]}`. Each fixture
has `id` (unique across files), `kind`, `summary`, `refs`, `interpretation`, and
(except tables) `expect`.

- `kind: "datagram"` — one UDP payload. `direction` is `client-to-server` or
  `server-to-client`. `length` is the octet count; `segments` is the payload in
  order, one entry per field:
  `{"offset": 4, "field": "msg.total_size", "hex": "00c0", "value": "192 octets", "ref": "Stream 2.1 p7-8"}`.
  The datagram is the concatenation of all `hex` strings.
- `kind: "sender-sequence"` — `sender` holds the parameters and input
  (`input_points` as `[x, y, r16, g16, b16, i16]`, or `input_blank_count`);
  `packets` lists the expected datagrams in order, either `{"ref": <datagram id>}`
  or literal header `segments` with the full `datagram_length` (sample octets
  covered elsewhere). `send_at_s` is the earliest legal send time from the
  stream origin.
- `kind: "table"` — `cases` of plain inputs and expected values.

`expect.outcome` vocabulary: `accept` (a valid client datagram), `respond`,
`parse_ok` / `parse_error` (client-side response parsing, with `reason`),
`match` / `match-prefix` (sender output). Decoded sample expectations use
signed 16-bit `x`/`y` and raw 8-bit color octets unless the field name says
otherwise.

`tools/test_idn.py` checks all of this before any test runs: schema keys, hex
syntax, contiguous offsets, literal lengths, unique ids, references, and a
`note` on every project interpretation.

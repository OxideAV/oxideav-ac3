# oxideav-ac3

Pure-Rust **AC-3 (Dolby Digital)** audio decoder — elementary streams per
ATSC A/52:2018 (= ETSI TS 102 366). Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Status

Early WIP. Implementation follows the A/52 spec incrementally:

- [x] Sync frame + BSI parse (§5.3)
- [ ] Audio-block parse (§5.4)
- [ ] Exponent decode (§7.1) + bit allocation (§7.2)
- [ ] Mantissa decode (§7.3)
- [ ] IMDCT synthesis (§7.9)
- [ ] Channel coupling + downmix (§7.4–§7.8)

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-ac3 = "0.0"
```

## Codec ID

- Codec: `"ac3"`; output sample format `S16` interleaved.

## License

MIT — see [LICENSE](LICENSE).

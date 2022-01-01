# Contributing

## README Generation

The README file is generated with [cargo-readme], run this if you change the docstring in `src/lib.rs`.

```bash
cargo install cargo-readme
cargo readme > README.md
```

[cargo-readme]: https://github.com/livioribeiro/cargo-readme

## Unit tests

Unit tests are required for all new features.

To test this crate run with the `--all-features` flag:

```bash
cargo test --all-features
```

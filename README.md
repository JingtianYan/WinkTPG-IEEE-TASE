# WinkTPG: An Execution Framework for Multi-Agent Path Finding Using Temporal Reasoning

Official implementation of **WinkTPG: An Execution Framework for Multi-Agent Path Finding Using Temporal Reasoning** by Jingtian Yan, Stephen F. Smith, and Jiaoyang Li, published in *IEEE Transactions on Automation Science and Engineering*. WinkTPG incrementally refines MAPF plans into kinodynamically feasible speed profiles while accounting for execution-time uncertainty. See the [journal paper](https://doi.org/10.1109/TASE.2026.3688563) for details.

## Build

Requires C++20, CMake, and Boost (`system`, `program_options`, and `filesystem`).

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/tpg -p example_path.txt -m 1 -o output.csv
```

Use `-m 1` for WinkTPG or `-m 0` for the TPG baseline. Run `./build/tpg --help` for all options.

## Citation

```bibtex
@article{yan2026winktpg,
  title   = {WinkTPG: An Execution Framework for Multi-Agent Path Finding Using Temporal Reasoning},
  author  = {Yan, Jingtian and Smith, Stephen F. and Li, Jiaoyang},
  journal = {IEEE Transactions on Automation Science and Engineering},
  volume  = {23},
  pages   = {9162--9175},
  year    = {2026},
  doi     = {10.1109/TASE.2026.3688563}
}
```

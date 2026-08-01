# FDTSP-2SM

Python implementation of the FDTSP-2SM (Flexible Drones Traveling Salesman Problem - 2 Stage Matheuristic): solves joint truck-drone routing to minimize delivery time. Includes Simulated Annealing and Genetic Algorithm heuristics (1-5 drones), plus an exact Gurobi (MILP) model.

## Repository structure

```
repo_fdtsp_2sm/
├── fdtsp_2sm/
│   ├── funciones_c.cp38-win_amd64.pyd     # Compiled feasibility/arrival-time module (Python 3.8, Windows amd64 only)
│   ├── lkh_FDTSP.py                       # Wrapper around the LKH-3 solver for the initial truck route
│   ├── Matheuristica_with_route_plot.py   # Main 2-stage matheuristic (VNS + Gurobi refinement) + route plotting
│   └── modelo_Gurobi.py                   # Exact MILP model (full and reduced formulations) in Gurobi
├── fdtsp_ga/
│   ├── Soluciones/                        # Intermediate CSV files written/removed during each run
│   ├── 1d-GA.py ... 5d-GA.py              # Genetic Algorithm heuristic, for 1 to 5 drones
│   └── testeo_factibilidad.py             # Feasibility-check module shared by the GA scripts
├── fdtsp_sa/
│   ├── Soluciones/                        # Intermediate CSV files written/removed during each run
│   ├── 1d-SA.py ... 5d-SA.py              # Simulated Annealing heuristic, for 1 to 5 drones
│   └── testeo_factibilidad.py             # Feasibility-check module shared by the SA scripts
├── instancias/                            # Instance CSV files (X, Y columns) used by the matheuristic and SA/GA scripts
├── instancias_modelo/                     # Instance CSV files used by the standalone exact model
├── resultados/                            # Pre-computed benchmark results and figures (xlsx summaries, png plots)
├── instancias.txt                         # List of instance filenames (one per line), for -n / -a flags
├── instancias_modelo.txt                  # List of instance filenames for the exact-model runs
├── LICENSE.md
└── README.md
```

> **Note:** the LKH-3 solver executable is not included in this repository (its license does not permit redistribution) — see [Requirements](#requirements) below to obtain it.

## Requirements

- **Python 3.9** on **Windows (amd64)** — required specifically because `fdtsp_2sm/funciones_c.cp38-win_amd64.pyd` is a compiled extension built for that exact platform/Python combination. It will not import under a different OS or Python version.
- Python packages: `numpy`, `pandas`, `matplotlib`, `openpyxl`
- [Gurobi](https://www.gurobi.com/) with an active license (academic or commercial) — required by `modelo_Gurobi.py` and by the Gurobi-refinement step in `Matheuristica_with_route_plot.py`
- [LKH-3.0.7](http://webhotel4.ruc.dk/~keld/research/LKH-3/) — download and compile separately, then place the executable where `lkh_FDTSP.py` expects it (`LKH-3.0.7/LKH` relative to the working directory)

Install the Python dependencies with:

```bash
pip install numpy pandas matplotlib openpyxl
```

## Instance files

Each instance is a `.csv` file with `X` and `Y` columns, named with the pattern:

```
<grid>_<n_nodes>_<endurance>_<drone_speed>.csv
```

`instancias.txt` / `instancias_modelo.txt` list the instance filenames to run, one per line.

## Usage

### Matheuristic (main method, with route plotting)

Run from the repository root:

```bash
python fdtsp_2sm/Matheuristica_with_route_plot.py -a <iterMax_VNS> -b <K_VNS> -c <mod1> -d <mod2> \
    -e <p_VNS> -f <q_VNS> -g <alfa> -h <temp> -i <finalTemp> -j <iterMax_SA> -k <n_SA> \
    -l <penalizacion> -m <cytn> -n instancias.txt
```

All 14 flags are required. Route plots (`.png` and `.pdf`) are saved to a `route_plots/` folder created automatically inside the working directory.

### Simulated Annealing / Genetic Algorithm baselines

Run from inside `fdtsp_sa/` or `fdtsp_ga/`, so the relative path to `../instancias` resolves correctly:

```bash
cd fdtsp_sa
python 1d-SA.py -a ../instancias.txt   # 1 drone, Simulated Annealing

cd ../fdtsp_ga
python 5d-GA.py -a ../instancias.txt   # 5 drones, Genetic Algorithm
```

## Results

`resultados/` contains pre-computed outputs from running the full benchmark (SA, GA, and matheuristic across all instances and seeds): aggregated `.xlsx` summaries by instance size, number of drones, and delivery area, plus the corresponding comparison plots. These are provided for reference/reproducibility of the reported results rather than generated directly by the scripts above.

## Notes

- Running the matheuristic requires a valid Gurobi license on the machine — without one, the Gurobi-refinement step and the standalone exact model will fail.
- `route_plots/` and `Soluciones/` are populated automatically at runtime; `Soluciones/` should normally end up empty after a full run (intermediate files are deleted once no longer needed).

## Authors

Developed by Carlos Contreras-Bolton and Benjamin Brandt.

This project builds on the Flexible Drones Traveling Salesman Problem (FDTSP) introduced by:

> Shih-Hao Lu, R.J. Kuo, Yi-Ting Ho, Anh-Tu Nguyen, "Improving the efficiency of last-mile delivery with the flexible drones traveling salesman problem," *Expert Systems with Applications*, Volume 209, 2022, 118351. https://doi.org/10.1016/j.eswa.2022.118351

The original authors' code is available at https://codeocean.com/capsule/8582408/tree/v2. See [LICENSE.md](LICENSE.md) for full citation details and license terms.

## License

This project is licensed under the MIT License — see [LICENSE.md](LICENSE.md) for details.

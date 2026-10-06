<h3 align="center"><i><b>modular-kf</i></b></h3>

<div align="center">

![GitHub Repo stars](https://img.shields.io/github/stars/walterlivingston/modular-kf)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> A modular tool for creating kalman filters.
    <br> 
</p>

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Supported Languages ](#supported-languages-)
- [Usage](#usage)
- [Tests](#tests)

## Supported Languages <a name = "supported-languages"></a>

<a href="https://github.com/walterlivingston/modular-kf/tree/matlab"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/matlab/matlab-original.svg?sanitize=true" title="MATLAB" alt="MATLAB" width="40" height="40"/></a>&nbsp;

>Click image for that language's library

## Usage <a name = "usage"></a>

A filter is built from three blocks. `kf` runs the Kalman filter equations; the blocks supply the problem-specific models. All blocks are handle objects, so give each filter its own instances.

| Block | Subclass of | Implements |
| --- | --- | --- |
| State | `stateblock` | `updateStateTransitionMatrix(x)` → continuous Jacobian `F`<br>`updateState(x, dt)` → propagated state<br>`calcProcessCovarianceMatrix(dt)` → `[Qc, Bw]`<br>`propagate(x, dt, relinearize[, u])` → `[x, F]` |
| Measurement | `measurementblock` | `updateObservationMatrix(x, y)` → `H`<br>`calcMeasurementCovarianceMatrix()` → `R`<br>`update(x, y, relinearize)` → `[yhat, H]` |
| Covariance | `covarianceblock` | `calcInnovationCovarianceMatrix(filter, H, R)` → `[S, reject]` |

`basicCovBlock` is a ready-made covariance block that never rejects. Write your own covariance block to gate measurements (e.g. on normalized innovation squared).

The filter discretizes `F`, `Qc` and `Bw` with Van Loan's method (`utils/vl_discretize.m`), so state blocks work in continuous time. `state_sigmas` holds whatever `calcProcessCovarianceMatrix` needs to build `Qc`, sized to the columns of `Bw`.

```matlab
filter = kf(myStateBlock(...), myMeasBlock(...), basicCovBlock(), ...
            x_i  = x0, ...          % initial state
            P_i  = P0, ...          % initial covariance
            mode = "extended");     % "linear", "extended" or "error"

for k = 2:N
    filter.process(dt);                     % time update (or process(dt, u = u(k)))
    [~, rejected] = filter.update(y(:,k));  % measurement update
end
```

- **Modes:** `"linear"` linearizes once about `x_lin` (default zeros, must be an equilibrium). `"extended"` relinearizes every step. `"error"` runs an error-state filter and requires the blocks to implement `applyError`.
- **One-call overrides:** `process(dt, stateBlock, covBlock)` and `update(y, measBlock, covBlock)` use the given blocks for that call only. Pass `[]` to skip one.
- **Rejections:** `filter.rejected` (also returned by `update`) is true when the covariance block rejected the last measurement; `x` and `P` are left unchanged.

See `examples/pendulum_example.m` for a complete linear vs. extended comparison.

## Tests <a name = "tests"></a>

From the repository root:

```matlab
runtests("tests")
```

The tests also run on every push and pull request via GitHub Actions (`.github/workflows/tests.yml`).

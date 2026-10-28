"""Offline Particle Swarm Optimization (PSO) for controller parameter tuning.

Fulfills Plan.md §10.3 & §19 (Milestone M6):
- Strictly offline parameter tuning (controller gains, weight matrices, scheduling).
- Never used in real-time control loops.
- Deterministically reproducible via explicit random seeds.
- Automatically exports optimized parameter sets with objective & metadata to YAML.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable
import numpy as np
import yaml


@dataclass
class PSOResult:
    """Complete results and diagnostic history of a PSO optimization run."""

    best_params: dict[str, float]
    best_cost: float
    history_best_cost: list[float]
    iterations: int
    n_particles: int
    convergence_time_s: float
    reproducibility_seed: int

    def export_to_yaml(
        self,
        output_file: str | Path,
        objective_name: str = "custom_kpi_objective",
        extra_metadata: dict | None = None,
    ) -> Path:
        """Export optimized parameters and traceability metadata to YAML (Plan.md §10.3)."""
        p = Path(output_file).expanduser().resolve()
        p.parent.mkdir(parents=True, exist_ok=True)

        doc = {
            "pso_optimization": {
                "objective": objective_name,
                "best_cost": float(self.best_cost),
                "iterations_completed": self.iterations,
                "swarm_size": self.n_particles,
                "execution_time_s": round(self.convergence_time_s, 3),
                "reproducibility_seed": self.reproducibility_seed,
                "generated_at": datetime.now().isoformat(),
                "metadata": extra_metadata or {},
            },
            "optimized_parameters": {k: float(v) for k, v in self.best_params.items()},
        }

        with open(p, "w", encoding="utf-8") as f:
            yaml.dump(doc, f, sort_keys=False, indent=2)

        return p


class PSOOptimizer:
    """Offline Particle Swarm Optimizer with linearly decaying inertia weight."""

    def __init__(
        self,
        cost_func: Callable[[np.ndarray], float],
        bounds: list[tuple[float, float]],
        param_names: list[str],
        n_particles: int = 20,
        n_iterations: int = 25,
        c1: float = 1.5,
        c2: float = 1.5,
        w_start: float = 0.9,
        w_end: float = 0.4,
        seed: int = 42,
    ):
        if len(bounds) != len(param_names):
            raise ValueError("bounds and param_names must have the same length")

        self.cost_func = cost_func
        self.bounds = np.asarray(bounds, dtype=float)
        self.param_names = list(param_names)
        self.n_particles = n_particles
        self.n_iterations = n_iterations
        self.c1 = c1
        self.c2 = c2
        self.w_start = w_start
        self.w_end = w_end
        self.seed = seed
        self.rng = np.random.RandomState(seed)

    def optimize(self, verbose: bool = False) -> PSOResult:
        """Execute the swarm search across parameter space."""
        import time
        t0 = time.perf_counter()

        dim = len(self.param_names)
        lb = self.bounds[:, 0]
        ub = self.bounds[:, 1]
        v_max = 0.25 * (ub - lb)

        # Initialize particles uniformly within bounds
        X = self.rng.uniform(lb, ub, size=(self.n_particles, dim))
        V = self.rng.uniform(-v_max, v_max, size=(self.n_particles, dim))

        p_best = X.copy()
        p_best_cost = np.full(self.n_particles, np.inf)

        for i in range(self.n_particles):
            cost_val = float(self.cost_func(X[i]))
            p_best_cost[i] = cost_val

        g_best_idx = int(np.argmin(p_best_cost))
        g_best = p_best[g_best_idx].copy()
        g_best_cost = float(p_best_cost[g_best_idx])

        cost_history: list[float] = [g_best_cost]

        # Iteration loop
        for it in range(self.n_iterations):
            w = self.w_start - (self.w_start - self.w_end) * (it / max(1, self.n_iterations - 1))

            r1 = self.rng.rand(self.n_particles, dim)
            r2 = self.rng.rand(self.n_particles, dim)

            # Velocity update
            V = (
                w * V
                + self.c1 * r1 * (p_best - X)
                + self.c2 * r2 * (g_best - X)
            )
            # Clip velocity to prevent divergence
            V = np.clip(V, -v_max, v_max)

            # Position update
            X = X + V
            # Clip position to parameter bounds
            X = np.clip(X, lb, ub)

            # Evaluate fitness
            for i in range(self.n_particles):
                val = float(self.cost_func(X[i]))
                if val < p_best_cost[i]:
                    p_best_cost[i] = val
                    p_best[i] = X[i].copy()
                    if val < g_best_cost:
                        g_best_cost = val
                        g_best = X[i].copy()

            cost_history.append(g_best_cost)
            if verbose:
                print(f"[PSO Iter {it+1:02d}/{self.n_iterations:02d}] Best Cost: {g_best_cost:.5f}")

        elapsed = time.perf_counter() - t0
        best_dict = {name: float(g_best[j]) for j, name in enumerate(self.param_names)}

        return PSOResult(
            best_params=best_dict,
            best_cost=g_best_cost,
            history_best_cost=cost_history,
            iterations=self.n_iterations,
            n_particles=self.n_particles,
            convergence_time_s=elapsed,
            reproducibility_seed=self.seed,
        )


__all__ = ("PSOResult", "PSOOptimizer")

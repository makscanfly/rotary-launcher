from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

import numpy as np
from scipy.optimize import minimize_scalar, root_scalar


@dataclass(frozen=True)
class PlanResult:
    fi_rad: float
    fi_deg: float
    omega: float
    omega2: float

    frequency_hz: float   # f = omega/(2*pi)
    period_s: float       # T = 1/f
    fi_time_s: float      # t_fi = fi/omega

    method: str
    diagnostics: Dict[str, float]


class TrajectoryPlanning:

    def __init__(self, *, h: float, d: float, R: float, g: float = 9.8):
        self.h = float(h)
        self.d = float(d)
        self.R = float(R)
        self.g = float(g)

        self.xf: Optional[float] = None
        self.yf: Optional[float] = None

    def set_target(self, xf: float, yf: float) -> None:
        self.xf = float(xf)
        self.yf = float(yf)


    @staticmethod
    def _frequency_hz(omega: float) -> float:
        return omega / (2.0 * math.pi)

    @staticmethod
    def _period_s_from_frequency(frequency_hz: float) -> float:
        return float("inf") if frequency_hz <= 0 else 1.0 / frequency_hz

    @staticmethod
    def _fi_time_s(fi_rad: float, omega: float) -> float:
        return fi_rad / omega

    # minimalizacja omega

    def solve_by_optimization(
        self,
        *,
        fi_min_deg: float = 1.0,
        fi_max_deg: float = 89.0,
        xatol: float = 1e-10,
        maxiter: int = 800,
    ) -> PlanResult:
        self._require_target()

        fi_min = math.radians(fi_min_deg)
        fi_max = math.radians(fi_max_deg)

        res = minimize_scalar(
            lambda fi: self._objective_omega2(fi),
            method="bounded",
            bounds=(fi_min, fi_max),
            options={"xatol": xatol, "maxiter": int(maxiter)},
        )

        fi_out = float(res.x)
        ok2 = self._omega2_from_hit(fi_out)

        if (not res.success) or (not math.isfinite(ok2)):
            raise RuntimeError(
                "Optimization did not find a valid solution in the given fi range. "
                "Try adjusting bounds or verify feasibility of the target."
            )

        omega_out = math.sqrt(ok2)
        freq = self._frequency_hz(omega_out)
        period = self._period_s_from_frequency(freq)
        fi_time = self._fi_time_s(fi_out, omega_out)

        y_check = self.y_at_x(self.xf, fi_out, omega_out)

        # obliczenie theta dla znalezionego rozwiązania
        delta_at_target = self._delta_from_fi_omega(fi_out, omega_out, x=self.xf)
        theta_at_target = abs(delta_at_target)

        diag = {
            "scipy_success": 1.0 if res.success else 0.0,
            "scipy_nfev": float(res.nfev),
            "y_at_xf": float(y_check) if math.isfinite(y_check) else float("nan"),
            "y_error": float(y_check - self.yf) if math.isfinite(y_check) else float("nan"),
            "objective_omega2": float(res.fun),

            # daignostyka
            "delta_at_target_deg_internal": float(math.degrees(delta_at_target)) if math.isfinite(delta_at_target) else float("nan"),
            "theta_at_target_deg": float(math.degrees(theta_at_target)) if math.isfinite(theta_at_target) else float("nan"),
        }

        return PlanResult(
            fi_rad=fi_out,
            fi_deg=math.degrees(fi_out),
            omega=omega_out,
            omega2=ok2,
            frequency_hz=freq,
            period_s=period,
            fi_time_s=fi_time,
            method="optimization_min_omega",
            diagnostics=diag,
        )

    # zadawanie kąta trafienia theta

    def solve_by_impact_angle(
        self,
        *,
        theta_deg: Optional[float] = None,
        theta_rad: Optional[float] = None,
        fi_min_deg: float = 1.0,
        fi_max_deg: float = 89.0,
        scan_n: int = 8000,
        xtol: float = 1e-12,
        rtol: float = 1e-12,
        maxiter: int = 200,
    ) -> PlanResult:
        self._require_target()
        theta = self._parse_angle(theta_deg=theta_deg, theta_rad=theta_rad, name="theta")

        # zamiana dla łatwiejszych obliczeń
        delta = -theta

        bracket = self._find_bracket_for_F(delta, fi_min_deg, fi_max_deg, n=scan_n)
        if bracket is None:
            raise RuntimeError(
                "No sign-change bracket found for the equation in the given fi range "
                "(or omega is infeasible throughout). Adjust fi bounds or verify feasibility."
            )

        sol = root_scalar(
            lambda fi: self._F_eq9(fi, delta),
            bracket=bracket,
            method="brentq",
            xtol=xtol,
            rtol=rtol,
            maxiter=int(maxiter),
        )
        if not sol.converged:
            raise RuntimeError("Root solve for the equation did not converge.")

        fi_out = float(sol.root)
        omega_out = self._omega_from_angle_constraint(fi_out, delta)
        if not math.isfinite(omega_out):
            raise RuntimeError("Solved fi, but omega computed from angle constraint is invalid (NaN/inf/<=0).")

        freq = self._frequency_hz(omega_out)
        period = self._period_s_from_frequency(freq)
        fi_time = self._fi_time_s(fi_out, omega_out)

        ok2 = omega_out * omega_out
        y_check = self.y_at_x(self.xf, fi_out, omega_out)

        # obliczenie theta dla znalezionego rozwiązania
        delta_at_target = self._delta_from_fi_omega(fi_out, omega_out, x=self.xf)
        theta_at_target = abs(delta_at_target)

        diag = {
            "bracket_a_rad": float(bracket[0]),
            "bracket_b_rad": float(bracket[1]),
            "F_at_root": float(self._F_eq9(fi_out, delta)),
            "y_at_xf": float(y_check) if math.isfinite(y_check) else float("nan"),
            "y_error": float(y_check - self.yf) if math.isfinite(y_check) else float("nan"),
            "theta_deg": float(math.degrees(theta)),
            "delta_deg_internal": float(math.degrees(delta)),

            # diagnostyka
            "delta_at_target_deg_internal": float(math.degrees(delta_at_target)) if math.isfinite(delta_at_target) else float("nan"),
            "theta_at_target_deg": float(math.degrees(theta_at_target)) if math.isfinite(theta_at_target) else float("nan"),
        }

        return PlanResult(
            fi_rad=fi_out,
            fi_deg=math.degrees(fi_out),
            omega=omega_out,
            omega2=ok2,
            frequency_hz=freq,
            period_s=period,
            fi_time_s=fi_time,
            method="impact_angle_constraint_theta",
            diagnostics=diag,
        )

    # zakres dopuszczalnego theta dla danego (xf,yf)

    def theta_range_for_target(
        self,
        *,
        xf: float,
        yf: float,
        f_max_hz: float = 5.0,
        fi_min_deg: float = 1.0,
        fi_max_deg: float = 89.0,
        scan_n: int = 12000,
    ) -> Dict[str, float]:

        self.set_target(xf, yf)

        # 1) zakładam że minimalne theta to omega pochodzące z minimalizacji
        sol_min_omega = self.solve_by_optimization(fi_min_deg=fi_min_deg, fi_max_deg=fi_max_deg)
        delta_min = self._delta_from_fi_omega(sol_min_omega.fi_rad, sol_min_omega.omega, x=self.xf)
        theta_min = abs(delta_min)

        # 2) maksymalna omega to ta odpowiadająca maksymalnej częstotliwości
        omega_max = 2.0 * math.pi * float(f_max_hz)

        # 3) zanlezienie kąta wystrzału fi dla maksymalnej omegi 
        fi_for_omega_max = self._solve_fi_for_fixed_omega(
            omega=omega_max,
            fi_min_deg=fi_min_deg,
            fi_max_deg=fi_max_deg,
            scan_n=scan_n,
        )

        # 4) znalezienie theta_max z omega_max i fi_for_omega_max
        delta_max = self._delta_from_fi_omega(fi_for_omega_max, omega_max, x=self.xf)
        theta_max = abs(delta_max)

        tmin, tmax = (theta_min, theta_max) if theta_min <= theta_max else (theta_max, theta_min)

        return {
            "theta_min_rad": float(tmin),
            "theta_max_rad": float(tmax),
            "theta_min_deg": float(math.degrees(tmin)),
            "theta_max_deg": float(math.degrees(tmax)),
            "delta_at_min_omega_deg_internal": float(math.degrees(delta_min)),
            "fi_at_min_omega_deg": float(sol_min_omega.fi_deg),
            "omega_min": float(sol_min_omega.omega),
            "delta_at_omega_max_deg_internal": float(math.degrees(delta_max)),
            "fi_at_omega_max_deg": float(math.degrees(fi_for_omega_max)),
            "omega_max": float(omega_max),
            "f_max_hz": float(f_max_hz),
        }

    # fizyka trajektorii i ligika pomocnicza

    def y_at_x(self, x: float, fi: float, omega: float) -> float:
        s = math.sin(fi)
        c = math.cos(fi)
        if abs(s) < 1e-12 or omega <= 0:
            return float("nan")

        t = (x + self.d + self.R * c) / (omega * self.R * s)

        return (
            self.h
            + self.R * s
            + (x + self.d + self.R * c) * c / s
            - 0.5 * self.g * t * t
        )

    def _omega2_from_hit(self, fi: float) -> float:
        s = math.sin(fi)
        c = math.cos(fi)
        if abs(s) < 1e-12:
            return float("nan")

        X = self.xf + self.d + self.R * c
        A = X / (self.R * s)

        denom = (self.h + self.R * s + X * (c / s) - self.yf)
        if abs(denom) < 1e-12:
            return float("nan")

        val = 0.5 * self.g * (A * A) / denom
        if (not math.isfinite(val)) or val <= 0.0:
            return float("nan")
        return val

    def _objective_omega2(self, fi: float) -> float:
        ok2 = self._omega2_from_hit(fi)
        return ok2 if math.isfinite(ok2) else float("inf")

    def _F_eq9(self, fi: float, delta: float) -> float:
        s = math.sin(fi)
        c = math.cos(fi)
        if abs(s) < 1e-12:
            return float("nan")

        X = self.xf + self.d + self.R * c
        cot = c / s

        lhs = (
            self.h
            + self.R * s
            + X * cot
            - 0.5 * X * (cot - math.tan(delta))
        )
        return lhs - self.yf

    def _omega_from_angle_constraint(self, fi: float, delta: float) -> float:
        s = math.sin(fi)
        if abs(s) < 1e-12:
            return float("nan")

        denom_factor = (1.0 / math.tan(fi) - math.tan(delta))  # cot(fi) - tan(delta)
        if abs(denom_factor) < 1e-12:
            return float("nan")

        X = self.xf + self.d + self.R * math.cos(fi)
        omega2 = (self.g * X) / (denom_factor * (self.R ** 2) * (s ** 2))

        if (not math.isfinite(omega2)) or omega2 <= 0.0:
            return float("nan")
        return math.sqrt(omega2)


    def _delta_from_fi_omega(self, fi: float, omega: float, *, x: float) -> float:
        s = math.sin(fi)
        c = math.cos(fi)

        if abs(s) < 1e-12 or omega <= 0:
            return float("nan")

        cot = c / s
        X = x + self.d + self.R * c

        term = self.g * X / ((omega * omega) * (self.R * self.R) * (s * s))
        m = cot - term  # dy/dx

        if not math.isfinite(m):
            return float("nan")

        return math.atan(m)


    def _solve_fi_for_fixed_omega(
        self,
        *,
        omega: float,
        fi_min_deg: float,
        fi_max_deg: float,
        scan_n: int,
    ) -> float:
        self._require_target()

        a = math.radians(fi_min_deg)
        b = math.radians(fi_max_deg)

        def residual(fi: float) -> float:
            yv = self.y_at_x(self.xf, fi, omega)
            return yv - self.yf

        fis = np.linspace(a, b, int(scan_n))
        prev_fi = None
        prev_val = None

        bracket = None
        for fi in fis:
            val = residual(float(fi))
            if not math.isfinite(val):
                prev_fi, prev_val = float(fi), None
                continue

            if prev_val is not None and val * prev_val < 0.0:
                bracket = (float(prev_fi), float(fi))
                break

            prev_fi, prev_val = float(fi), float(val)

        if bracket is None:
            raise RuntimeError(
                "Nie znaleziono przedziału dla fi (brak zmiany znaku y(xf)-yf) przy zadanym omega. "
                "Punkt (xf,yf) nie jest osiągalny dla tego omega i zakresu fi."
            )

        sol = root_scalar(residual, bracket=bracket, method="brentq", xtol=1e-12, rtol=1e-12, maxiter=200)
        if not sol.converged:
            raise RuntimeError("Rozwiązywanie fi dla stałego omega nie zbiegło.")

        return float(sol.root)

    def _find_bracket_for_F(
        self,
        delta: float,
        fi_min_deg: float,
        fi_max_deg: float,
        *,
        n: int,
    ) -> Optional[Tuple[float, float]]:
        a = math.radians(fi_min_deg)
        b = math.radians(fi_max_deg)
        fis = np.linspace(a, b, int(n))

        prev_fi = None
        prev_val = None

        for fi in fis:
            val = self._F_eq9(fi, delta)

            if (not math.isfinite(val)) or (not math.isfinite(self._omega_from_angle_constraint(fi, delta))):
                prev_fi, prev_val = fi, None
                continue

            if prev_val is not None and (val * prev_val) < 0.0:
                return (float(prev_fi), float(fi))

            prev_fi, prev_val = fi, val

        return None

    def _require_target(self) -> None:
        if self.xf is None or self.yf is None:
            raise ValueError("Target not set. Call set_target(xf, yf) first.")

    @staticmethod
    def _parse_angle(*, theta_deg: Optional[float], theta_rad: Optional[float], name: str) -> float:
        if (theta_deg is None) == (theta_rad is None):
            raise ValueError(f"Provide exactly one of {name}_deg or {name}_rad.")
        return math.radians(float(theta_deg)) if theta_deg is not None else float(theta_rad)


#TrajecoryPlanning = TrajectoryPlanning


# Przykład użycia
if __name__ == "__main__":
    planner = TrajectoryPlanning(h=0.4, d=0.3, R=0.3, g=9.8)
    planner.set_target(xf=1.6, yf=0.8)

    sol = planner.solve_by_optimization()
    print('Paramety trafienia minimalizacja omega:\n', sol)

    print('\n')

    tr = planner.theta_range_for_target(xf=1.6, yf=0.8, f_max_hz=5.0)
    print('Dopuszczalny zakres kąta trafienia:\n', tr)

    print('\n')

    sol2 = planner.solve_by_impact_angle(theta_deg=67.0)
    print('Parametry dla trafienia pod ostrym kątem theta = 67:\n', sol2)

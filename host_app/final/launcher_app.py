"""
Wyrzutnia obrotowa — aplikacja terminalowa (CLI)

Założenia:
- BLE: komunikacja z ESP o nazwie "LauncherESP"
- Planner: TrajectoryPlanning (wyznaczanie fi i okresu)
- Wejście użytkownika przez await asyncio.to_thread(input, ...)
- Parametr f_max_hz jest stały (wynika z mechaniki)
"""

from __future__ import annotations

import asyncio
import math
from typing import Optional

# Importy zgodne z ostatnimi zmianami
from BLEconnection import (  # <- zmień nazwę modułu na właściwą, jeśli masz inną
    LauncherESPBLE,
    EVT_EMERGENCY_STOP,
    EVT_PARAMS_READ,
    EVT_SHOT_DONE,
    LauncherEvent,
    BleError,
    DeviceNotFoundError,
    ConnectionFailedError,
    GattMismatchError,
    DisconnectedError,
    EmergencyStopError,
    ParamsAckTimeoutError,
    ShotDoneTimeoutError,
)

from TrajectoryPlanning import TrajectoryPlanning  # <- zmień nazwę modułu na właściwą, jeśli masz inną


# ---------------- Konfiguracja aplikacji ----------------

APP_NAME = "Wyrzutnia obrotowa"

# Parametry mechaniki / geometrii (ustaw docelowe wartości)
H_M = 0.364
D_M = 0.2
R_M = 0.311
G = 9.8105 

# Limit mechaniczny (stały, bez edycji przez użytkownika)
F_MAX_HZ = 4.2

# Timeouty
ACK_TIMEOUT_S = 5.0

# fi bounds (zgodne z plannerem)
FI_MIN_DEG = 1.0
FI_MAX_DEG = 89.0


# ---------------- Helpers: I/O ----------------

async def ainput(prompt: str) -> str:
    """Asynchroniczny input (nie blokuje event loop)."""
    return await asyncio.to_thread(input, prompt)


def parse_float(s: str) -> float:
    """Akceptuje kropkę i przecinek."""
    s = s.strip().replace(",", ".")
    return float(s)


def format_deg(x: float, nd: int = 2) -> str:
    if not math.isfinite(x):
        return "NaN"
    return f"{x:.{nd}f}°"


def format_hz(x: float, nd: int = 3) -> str:
    if not math.isfinite(x):
        return "NaN"
    return f"{x:.{nd}f} Hz"


def format_us(x: int) -> str:
    return f"{x} µs"


def within_allowed_area(xf: float, yf: float) -> bool:
    """
    TODO: Zastąp docelową walidacją obszaru dopuszczalnego.
    Na ten moment przepuszcza wszystko.
    """
    return True


def period_us_from_period_s(period_s: float) -> int:
    """
    Konwersja do uint32 (ESP oczekuje uint32 microseconds).
    """
    if not math.isfinite(period_s) or period_s <= 0:
        raise ValueError("Nieprawidłowy okres (period_s).")
    us = int(round(period_s * 1_000_000.0))
    if not (0 <= us <= 0xFFFFFFFF):
        raise ValueError("Okres po konwersji nie mieści się w uint32.")
    return us


# ---------------- App flow ----------------

async def connect_with_retry(ble: LauncherESPBLE) -> None:
    while True:
        try:
            await ble.connect()
            print("Uzyskano połączenie z wyrzutnią.")
            return
        except (DeviceNotFoundError, ConnectionFailedError, GattMismatchError, BleError):
            ans = (await ainput("Nie udało się połączyć z wyrzutnią. Naciśnij Enter, aby spróbować ponownie, lub wpisz 'q', aby zakończyć: ")).strip().lower()
            if ans == "q":
                raise SystemExit(0)


async def read_target_xy() -> tuple[float, float]:
    while True:
        try:
            x_str = await ainput("Podaj odległość celu x [m]: ")
            xf = parse_float(x_str)

            y_str = await ainput("Podaj wysokość celu y [m]: ")
            yf = parse_float(y_str)

            if not within_allowed_area(xf, yf):
                print("Cel poza dopuszczalnym obszarem.")
                continue

            return xf, yf

        except ValueError:
            print("Nieprawidłowa wartość. Wpisz liczbę (np. 1.6 lub 1,6).")


async def choose_method() -> str:
    while True:
        s = (await ainput("Wybierz metodę wyznaczania trajektorii:\n  m - minimalizacja prędkości kątowej\n  k - zadaj kąt trafienia w cel\nWybór: ")).strip().lower()
        if s in ("m", "k"):
            return s
        print("Nieprawidłowy wybór. Wpisz 'm' lub 'k'.")


async def choose_theta_deg(theta_min: float, theta_max: float) -> float:
    while True:
        s = (await ainput("Podaj kąt trafienia θ [deg]: ")).strip()
        try:
            theta = parse_float(s)
        except ValueError:
            print("Nieprawidłowa wartość. Wpisz liczbę (np. 67 lub 67.0).")
            continue

        if theta < theta_min or theta > theta_max:
            print(f"Kąt poza zakresem. Dopuszczalny zakres: {theta_min:.2f}° – {theta_max:.2f}°.")
            continue

        return theta


async def confirm_plan_summary(
    *,
    xf: float,
    yf: float,
    method: str,
    theta_input_deg: Optional[float],
    period_us: int,
    fi_deg: float,
    freq_hz: float,
    theta_at_target_deg: float,
) -> bool:
    print("\nPodsumowanie:")
    print(f"  Cel: x = {xf:.3f} m, y = {yf:.3f} m")
    if method == "m":
        print("  Metoda: minimalizacja prędkości kątowej")
    else:
        print(f"  Metoda: zadany kąt trafienia θ = {theta_input_deg:.2f}°")

    print(f"  Częstotliwość: {format_hz(freq_hz)}")
    print(f"  Okres: {format_us(period_us)}")
    print(f"  Kąt wystrzału fi: {format_deg(fi_deg)}")
    print(f"  Kąt trafienia θ (wyliczony): {format_deg(theta_at_target_deg)}")

    ans = (await ainput("Zatwierdzić powyższe dane? Enter = tak, 'n' = nie: ")).strip().lower()
    return ans != "n"


async def send_to_launcher(ble: LauncherESPBLE, *, period_us: int, fi_deg: float) -> None:
    ans = (await ainput("Wyślij parametry do wyrzutni? Enter = wyślij, 'n' = anuluj: ")).strip().lower()
    if ans == "n":
        raise RuntimeError("Anulowano wysyłkę parametrów.")

    await ble.send_params_and_wait_ack(period_us, fi_deg, ack_timeout_s=ACK_TIMEOUT_S)
    print("Parametry przyjęte przez wyrzutnię.")


async def wait_for_completion(ble: LauncherESPBLE) -> None:
    print("Oczekiwanie na zakończenie wystrzału...")
    await ble.wait_for_shot_done(timeout_s=None)
    print("Wystrzał zakończony.")


async def main() -> None:
    print(APP_NAME)
    print("-" * len(APP_NAME))

    # Planner
    planner = TrajectoryPlanning(h=H_M, d=D_M, R=R_M, g=G)

    # BLE (logowanie tylko przez aplikację; klasa jest cicha)
    def on_event(evt: LauncherEvent) -> None:
        # Opcjonalne: w CLI zwykle nie musisz wypisywać wszystkich eventów,
        # bo i tak czekasz na nie metodami await.
        # Jeżeli chcesz, możesz odkomentować poniższe:
        #
        # if evt.code == EVT_PARAMS_READ:
        #     print("ESP: EVT_PARAMS_READ")
        # elif evt.code == EVT_SHOT_DONE:
        #     print("ESP: EVT_SHOT_DONE")
        # elif evt.code == EVT_EMERGENCY_STOP:
        #     print("ESP: EVT_EMERGENCY_STOP")
        #
        pass

    ble = LauncherESPBLE(on_event=on_event)

    try:
        # 1) Połączenie
        await connect_with_retry(ble)

        # 2) Wprowadzanie danych i planowanie
        while True:
            xf, yf = await read_target_xy()
            planner.set_target(xf=xf, yf=yf)

            method = await choose_method()

            theta_input_deg: Optional[float] = None

            try:
                if method == "m":
                    sol = planner.solve_by_optimization(fi_min_deg=FI_MIN_DEG, fi_max_deg=FI_MAX_DEG)
                else:
                    tr = planner.theta_range_for_target(
                        xf=xf,
                        yf=yf,
                        f_max_hz=F_MAX_HZ,
                        fi_min_deg=FI_MIN_DEG,
                        fi_max_deg=FI_MAX_DEG,
                    )
                    theta_min = float(tr["theta_min_deg"])
                    theta_max = float(tr["theta_max_deg"])
                    print(f"Dostępny zakres kątów trafienia θ: {theta_min:.2f}° – {theta_max:.2f}°")

                    theta_input_deg = await choose_theta_deg(theta_min, theta_max)
                    sol = planner.solve_by_impact_angle(theta_deg=theta_input_deg, fi_min_deg=FI_MIN_DEG, fi_max_deg=FI_MAX_DEG)

                # Konwersje do tego, co wysyła BLE
                period_us = period_us_from_period_s(sol.period_s)
                fi_deg = float(sol.fi_deg)
                freq_hz = float(sol.frequency_hz)

                theta_at_target_deg = float(sol.diagnostics.get("theta_at_target_deg", float("nan")))

                ok = await confirm_plan_summary(
                    xf=xf,
                    yf=yf,
                    method=method,
                    theta_input_deg=theta_input_deg,
                    period_us=period_us,
                    fi_deg=fi_deg,
                    freq_hz=freq_hz,
                    theta_at_target_deg=theta_at_target_deg,
                )
                if not ok:
                    print("Anulowano. Wprowadź dane ponownie.\n")
                    continue

                # 3) Wysyłka i oczekiwanie
                try:
                    await send_to_launcher(ble, period_us=period_us, fi_deg=fi_deg)
                except RuntimeError as e:
                    # np. anulowanie wysyłki
                    print(str(e))
                    print("Wprowadź dane ponownie.\n")
                    continue

                await wait_for_completion(ble)
                return  # zakończ program po SHOT_DONE

            except EmergencyStopError:
                print("Emergency Stop. Zakończono działanie programu.")
                return
            except ParamsAckTimeoutError:
                print("Brak potwierdzenia z wyrzutni. Sprawdź połączenie.")
                return
            except DisconnectedError:
                print("Utracono połączenie z wyrzutnią. Zakończono działanie programu.")
                return
            except ShotDoneTimeoutError:
                print("Nie odebrano informacji o zakończeniu wystrzału w zadanym czasie.")
                return
            except ValueError as e:
                # np. period_us poza zakresem, dane wejściowe
                print(f"Błąd danych: {e}")
                print("Wprowadź dane ponownie.\n")
                continue
            except RuntimeError as e:
                # np. solver nie znalazł rozwiązania / brak bracketu
                print(f"Nie udało się wyznaczyć trajektorii: {e}")
                print("Wprowadź dane ponownie.\n")
                continue

    finally:
        await ble.disconnect()


if __name__ == "__main__":
    asyncio.run(main())

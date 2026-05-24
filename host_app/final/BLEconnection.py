from __future__ import annotations

import asyncio
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional, Set, Any

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData


EVT_EMERGENCY_STOP = 1
EVT_PARAMS_READ = 2
EVT_SHOT_DONE = 3


DEV_NAME = "LauncherESP"

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_RX_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_EVENT_UUID = "44444444-4444-4444-4444-444444444444"


@dataclass(frozen=True)
class LauncherEvent:
    code: int
    ts: float


class BleError(Exception):
    pass


class DeviceNotFoundError(BleError):
    pass


class ConnectionFailedError(BleError):
    pass


class GattMismatchError(BleError):
    pass


class DisconnectedError(BleError):
    pass


class EmergencyStopError(BleError):
    pass


class ParamsAckTimeoutError(BleError):
    pass


class ShotDoneTimeoutError(BleError):
    pass



class LauncherESPBLE:

    def __init__(self, *, on_event: Optional[Callable[[LauncherEvent], None]] = None) -> None:
        self._on_event = on_event

        self._client: Optional[BleakClient] = None
        self._rx_char_uuid: str = CHAR_RX_UUID
        self._event_char_uuid: str = CHAR_EVENT_UUID

        self._event_q: asyncio.Queue[LauncherEvent] = asyncio.Queue()
        self._disconnected = asyncio.Event()

        self._op_lock = asyncio.Lock()

    async def connect(self) -> None:
        if self._client and self._client.is_connected:
            return

        self._disconnected.clear()

        device = await self._find_device()
        if device is None:
            raise DeviceNotFoundError(f"Nie znaleziono urządzenia BLE: {DEV_NAME} / {SERVICE_UUID}")

        client = BleakClient(device, disconnected_callback=self._on_disconnect)

        try:
            await client.connect()
        except Exception as e:
            raise ConnectionFailedError(f"Nie udało się połączyć: {e}") from e

        try:
            services = client.services

            if services is None:
                raise GattMismatchError("Nie udało się pobrać usług (client.services is None).")

            svc = services.get_service(SERVICE_UUID)
            if svc is None:
                raise GattMismatchError(f"Brak serwisu {SERVICE_UUID} na urządzeniu.")

            rx = services.get_characteristic(self._rx_char_uuid)
            ev = services.get_characteristic(self._event_char_uuid)

            if rx is None:
                raise GattMismatchError(f"Brak charakterystyki RX (write) {self._rx_char_uuid}.")
            if ev is None:
                raise GattMismatchError(f"Brak charakterystyki EVENT (notify) {self._event_char_uuid}.")

            await client.start_notify(self._event_char_uuid, self._handle_notify)

        except Exception:
            try:
                await client.disconnect()
            except Exception:
                pass
            raise

        self._client = client

    async def disconnect(self) -> None:
        client = self._client
        self._client = None

        self._disconnected.set()

        if client is None:
            return

        try:
            if client.is_connected:
                try:
                    await client.stop_notify(self._event_char_uuid)
                except Exception:
                    pass
                await client.disconnect()
        except Exception:
            pass

    async def send_params_and_wait_ack(self, period_us: int, fi_deg: float, *, ack_timeout_s: float) -> None:
        if period_us < 0 or period_us > 0xFFFFFFFF:
            raise ValueError("period_us poza zakresem uint32.")

        async with self._op_lock:
            await self._ensure_connected()

            payload = struct.pack("<If", int(period_us), float(fi_deg))

            try:
                await self._client.write_gatt_char(self._rx_char_uuid, payload, response=True)
            except Exception as e:
                raise BleError(f"Błąd zapisu do RX: {e}") from e

            try:
                await self._wait_for_event({EVT_PARAMS_READ}, timeout_s=ack_timeout_s)
            except asyncio.TimeoutError as e:
                raise ParamsAckTimeoutError("Timeout oczekiwania na EVT_PARAMS_READ (ACK).") from e

    async def wait_for_shot_done(self, *, timeout_s: Optional[float] = None) -> None:
        await self._ensure_connected()

        try:
            await self._wait_for_event({EVT_SHOT_DONE}, timeout_s=timeout_s)
        except asyncio.TimeoutError as e:
            raise ShotDoneTimeoutError("Timeout oczekiwania na EVT_SHOT_DONE.") from e



    async def _ensure_connected(self) -> None:
        if self._client is None or not self._client.is_connected:
            raise DisconnectedError("Brak połączenia BLE.")

        if self._disconnected.is_set():
            raise DisconnectedError("Utracono połączenie BLE.")

    async def _find_device(self) -> Optional[BLEDevice]:
        # szukanie po UUID
        service_uuid_l = SERVICE_UUID.lower()

        def _filter(d: BLEDevice, adv: AdvertisementData) -> bool:
            name_ok = (d.name == DEV_NAME) or (adv.local_name == DEV_NAME)
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            svc_ok = service_uuid_l in uuids
            return svc_ok or name_ok

        try:
            dev = await BleakScanner.find_device_by_filter(_filter, timeout=8.0)
            return dev
        except Exception as e:
            raise BleError(f"Błąd skanowania BLE: {e}") from e

    def _on_disconnect(self, _client: BleakClient) -> None:
        try:
            loop = asyncio.get_running_loop()
            loop.call_soon_threadsafe(self._disconnected.set)
        except RuntimeError:
            pass

    def _handle_notify(self, _sender: Any, data: bytearray) -> None:
        if not data:
            return

        code = int(data[0])
        evt = LauncherEvent(code=code, ts=time.time())

        if self._on_event is not None:
            try:
                self._on_event(evt)
            except Exception:
                pass

        try:
            loop = asyncio.get_running_loop()
            loop.call_soon_threadsafe(self._event_q.put_nowait, evt)
        except RuntimeError:
            pass

    async def _wait_for_event(self, wanted_codes: Set[int], *, timeout_s: Optional[float]) -> LauncherEvent:

        async def _run() -> LauncherEvent:
            while True:
                if self._disconnected.is_set():
                    raise DisconnectedError("Utracono połączenie BLE.")

                evt = await self._event_q.get()

                if evt.code == EVT_EMERGENCY_STOP:
                    raise EmergencyStopError("Odebrano EVT_EMERGENCY_STOP.")

                if evt.code in wanted_codes:
                    return evt
                

        if timeout_s is None:
            return await _run()

        return await asyncio.wait_for(_run(), timeout=timeout_s)

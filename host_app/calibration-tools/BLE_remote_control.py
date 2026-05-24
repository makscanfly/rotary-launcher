import asyncio
import struct
from dataclasses import dataclass
from typing import List, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice


@dataclass(frozen=True)
class LauncherRemoteControlConfig:
    dev_name: str = "LauncherESP"
    service_uuid: str = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
    rx_uuid: str = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # Write / Write_NR
    tx_uuid: str = "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"  # Read


class LauncherRemoteControlError(RuntimeError):
    pass


class LauncherRemoteControl:
    """
    Klient BLE dla LauncherESP.
    - connect(): skan + połączenie (po UUID serwisu), walidacja charakterystyk
    - send_cmd(cmd, pwm): wysyła 3 bajty: uint8 + uint16 LE
    - read_samples(): czyta 2000 bajtów i zwraca List[int] długości 500 (uint32 LE)
    """

    SAMPLES_COUNT = 100
    SAMPLE_SIZE_BYTES = 4
    SAMPLES_TOTAL_BYTES = SAMPLES_COUNT * SAMPLE_SIZE_BYTES  # 2000

    def __init__(
        self,
        config: LauncherRemoteControlConfig = LauncherRemoteControlConfig(),
        scan_timeout_s: float = 10.0,
    ):
        self.cfg = config
        self.scan_timeout_s = float(scan_timeout_s)

        self._device: Optional[BLEDevice] = None
        self._client: Optional[BleakClient] = None
        self._connected: bool = False

    @property
    def is_connected(self) -> bool:
        return self._connected and self._client is not None and self._client.is_connected

    async def __aenter__(self) -> "LauncherRemoteControl":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.disconnect()

    async def connect(self) -> None:
        """
        Skanuje i łączy się z urządzeniem, preferując dopasowanie po UUID serwisu.
        Dodatkowo waliduje, że RX i TX charakterystyki istnieją w usługach GATT.
        """
        if self.is_connected:
            return

        self._device = await self._discover_device()
        if self._device is None:
            raise LauncherRemoteControlError(
                f"Nie znaleziono urządzenia reklamującego serwis {self.cfg.service_uuid} "
                f"(timeout={self.scan_timeout_s}s)."
            )

        self._client = BleakClient(self._device)
        try:
            await self._client.connect()

            # Bleak: w zależności od wersji backendu get_services() może nie istnieć.
            # Stabilniejszy wariant: użyj property `.services` po połączeniu.
            services = self._client.services
            if services is None:
                # W niektórych konfiguracjach services są ładowane leniwie – wymuś odczyt.
                await self._client.read_gatt_char(self.cfg.tx_uuid)
                services = self._client.services

            if services is None:
                raise LauncherRemoteControlError("Nie udało się pobrać usług GATT (services is None).")

            rx = services.get_characteristic(self.cfg.rx_uuid)
            tx = services.get_characteristic(self.cfg.tx_uuid)

            if rx is None:
                raise LauncherRemoteControlError(
                    f"Brak charakterystyki RX (write) o UUID: {self.cfg.rx_uuid}"
                )
            if tx is None:
                raise LauncherRemoteControlError(
                    f"Brak charakterystyki TX (read) o UUID: {self.cfg.tx_uuid}"
                )

            self._connected = True

        except Exception:
            try:
                if self._client is not None:
                    await self._client.disconnect()
            finally:
                self._client = None
                self._connected = False
            raise

    async def disconnect(self) -> None:
        if self._client is not None:
            try:
                await self._client.disconnect()
            finally:
                self._client = None
                self._connected = False
                self._device = None

    async def send_cmd(self, cmd: int, pwm: int) -> None:
        """
        Wysyła komendę do ESP:
        - cmd: 0..255 (uint8)
        - pwm: 0..65535 (uint16)
        Payload: 3 bajty little-endian: <BH
        """
        self._require_connected()

        if not (0 <= cmd <= 0xFF):
            raise ValueError(f"cmd poza zakresem uint8: {cmd}")
        if not (0 <= pwm <= 0xFFFF):
            raise ValueError(f"pwm poza zakresem uint16: {pwm}")

        payload = struct.pack("<BH", cmd, pwm)  # 1 + 2 bajty
        if len(payload) != 3:
            raise LauncherRemoteControlError("Błąd pakowania payloadu (oczekiwano 3 bajty).")

        await self._client.write_gatt_char(self.cfg.rx_uuid, payload, response=False)

    async def read_samples(self) -> List[int]:
        """
        Czyta snapshot próbek z TX (Read):
        - oczekuje 2000 bajtów (500 * uint32)
        - zwraca listę 500 elementów typu int (uint32)
        """
        self._require_connected()

        raw = await self._client.read_gatt_char(self.cfg.tx_uuid)
        if raw is None:
            raise LauncherRemoteControlError("read_gatt_char zwróciło None.")
        if len(raw) != self.SAMPLES_TOTAL_BYTES:
            raise LauncherRemoteControlError(
                f"Nieoczekiwany rozmiar danych z TX: {len(raw)} bajtów "
                f"(oczekiwano {self.SAMPLES_TOTAL_BYTES})."
            )

        fmt = "<" + ("I" * self.SAMPLES_COUNT)
        return list(struct.unpack(fmt, raw))

    async def _discover_device(self) -> Optional[BLEDevice]:
        """
        Odszukuje urządzenie:
        1) preferuje reklamowanie service_uuid,
        2) w razie problemów może też brać po nazwie dev_name jako fallback.
        """
        target_service = self.cfg.service_uuid.lower()

        def _matcher(dev: BLEDevice, adv_data) -> bool:
            uuids = [u.lower() for u in (adv_data.service_uuids or [])]
            if target_service in uuids:
                return True
            if dev.name and dev.name == self.cfg.dev_name:
                return True
            return False

        return await BleakScanner.find_device_by_filter(_matcher, timeout=self.scan_timeout_s)

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise LauncherRemoteControlError("Brak połączenia. Wywołaj najpierw connect().")

    # Wysyłanie rozpoznawalnych kodów przez wyrzutnię
    async def LedOn(self) -> None:
        await self.send_cmd(cmd=1, pwm=0)

    async def LedOff(self) -> None:
        await self.send_cmd(cmd=2, pwm=0)

    async def ResetSampleCollecting(self) -> None:
        await self.send_cmd(cmd=3, pwm=0)

    async def FinishSampleCollecting(self) -> None:
        await self.send_cmd(cmd=4, pwm=0)

    async def SetMotorPwm(self, pwm: int) -> None:
        if not isinstance(pwm, int):
            raise TypeError(f"pwm musi być int, otrzymano: {type(pwm).__name__}")
        if 0 <= pwm <= 1023:
            await self.send_cmd(cmd=5, pwm=pwm)

    async def StopMotor(self) -> None:
        await self.send_cmd(cmd=6, pwm=0)

    async def ElectromagnetOn(self) -> None:
        await self.send_cmd(cmd=7, pwm=0)

    async def ElectromagnetOff(self) -> None:
        await self.send_cmd(cmd=8, pwm=0)

    async def DirHigh(self) -> None:
        await self.send_cmd(cmd=9, pwm=0)

    async def DirLow(self) -> None:
        await self.send_cmd(cmd=10, pwm=0)

    async def Deaccelerate(self) -> None:
        await self.send_cmd(cmd=11, pwm=0)

    async def Accelerate_to(self, pwm: int) -> None:
        if not isinstance(pwm, int):
            raise TypeError(f"pwm musi być int, otrzymano: {type(pwm).__name__}")
        if 0 <= pwm <= 1023:
            await self.send_cmd(cmd=12, pwm=pwm)

# --- Minimalny przykład użycia ---
async def _demo():
    launcher = LauncherRemoteControl()
    await launcher.connect()
    try:
        await launcher.send_cmd(cmd=5, pwm=1200)  # przykład
        samples = await launcher.read_samples()
        print("samples[0:10] =", samples[:10])
    finally:
        await launcher.disconnect()


if __name__ == "__main__":
    asyncio.run(_demo())

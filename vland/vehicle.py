"""Thin async wrapper around MAVSDK for the few vehicle actions we need.

MAVSDK talks MAVLink to PX4 over UDP 14540 (PX4's "onboard/API" link), the
same link a companion computer would use on a real drone.
"""
from __future__ import annotations

import asyncio
import logging
import math
from dataclasses import dataclass, field

import numpy as np
from mavsdk_grpc import System
from mavsdk_grpc.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk_grpc.telemetry import FlightMode

log = logging.getLogger(__name__)

DEFAULT_ADDRESS = "udpin://0.0.0.0:14540"


@dataclass
class VehicleState:
    """Latest telemetry, updated in the background."""
    q_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # body FRD -> NED
    pos_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    attitude_stamp_us: int = 0


class Vehicle:
    def __init__(self, address: str = DEFAULT_ADDRESS):
        self.address = address
        self.drone = System()
        self.state = VehicleState()
        self._tasks: list[asyncio.Task] = []

    async def start_telemetry(self, rate_hz: float = 30.0) -> None:
        """Keep self.state current from MAVSDK telemetry streams."""
        await self.drone.telemetry.set_rate_attitude_quaternion(rate_hz)
        await self.drone.telemetry.set_rate_position_velocity_ned(rate_hz)

        async def attitude():
            async for q in self.drone.telemetry.attitude_quaternion():
                self.state.q_wxyz = (q.w, q.x, q.y, q.z)
                self.state.attitude_stamp_us = q.timestamp_us

        async def position():
            async for pv in self.drone.telemetry.position_velocity_ned():
                p, v = pv.position, pv.velocity
                self.state.pos_ned = np.array([p.north_m, p.east_m, p.down_m])
                self.state.vel_ned = np.array([v.north_m_s, v.east_m_s, v.down_m_s])

        self._tasks += [asyncio.create_task(attitude()), asyncio.create_task(position())]
        await asyncio.sleep(0.5)

    async def close(self) -> None:
        """Stop telemetry and the mavsdk_server subprocess.

        Must be called before the program exits. mavsdk_grpc reads the
        server's output on a non-daemon thread and only kills the server in an
        atexit handler, but Python joins non-daemon threads *before* running
        atexit handlers, so without this the interpreter hangs on exit.
        """
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()
        self.drone._stop_mavsdk_server()

    async def connect(self, timeout: float = 60.0) -> None:
        log.info("connecting to %s", self.address)
        await self.drone.connect(system_address=self.address)
        async with asyncio.timeout(timeout):
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    break
            log.info("connected; waiting for position estimate and home")
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    break
        log.info("vehicle ready")

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        await self.drone.action.set_takeoff_altitude(altitude_m)
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        # Takeoff is done once we're within 0.3 m of the target altitude.
        async for pos in self.drone.telemetry.position():
            if pos.relative_altitude_m >= altitude_m - 0.3:
                break
        log.info("reached takeoff altitude %.1f m", altitude_m)

    async def land_and_wait(self) -> None:
        await self.command_land()
        await self.wait_landed()

    async def command_land(self, attempts: int = 3, confirm_timeout: float = 5.0) -> None:
        """Switch PX4 to LAND and confirm it actually happened.

        An accepted command is not proof of a mode change. In testing, a land
        command sent right after leaving offboard was acknowledged but PX4
        stayed in HOLD, hovering on the pad indefinitely. So: send it, watch
        the reported flight mode, and retry if LAND doesn't appear.
        Call this *from* offboard mode; don't stop offboard first (that
        switches PX4 to HOLD).
        """
        for attempt in range(1, attempts + 1):
            await self.drone.action.land()
            try:
                async with asyncio.timeout(confirm_timeout):
                    async for mode in self.drone.telemetry.flight_mode():
                        if mode == FlightMode.LAND:
                            log.info("PX4 in LAND mode")
                            return
            except TimeoutError:
                log.warning("LAND not confirmed (attempt %d/%d)", attempt, attempts)
        raise RuntimeError("PX4 did not enter LAND mode")

    async def wait_landed(self, disarm_timeout: float = 15.0, land_timeout: float = 180.0) -> None:
        """Wait for touchdown and disarm. Timeouts are wall-clock."""
        async with asyncio.timeout(land_timeout):
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    break
        log.info("landed")
        # PX4 auto-disarms shortly after touchdown (COM_DISARM_LAND); give it
        # a moment, then disarm explicitly so a script never hangs here.
        try:
            async with asyncio.timeout(disarm_timeout):
                async for armed in self.drone.telemetry.armed():
                    if not armed:
                        break
        except TimeoutError:
            log.warning("no auto-disarm after %.0f s; disarming", disarm_timeout)
            await self.drone.action.disarm()
        log.info("disarmed")

    # --- offboard velocity control -------------------------------------
    async def start_offboard(self) -> None:
        # PX4 refuses to enter offboard mode unless setpoints are already
        # streaming, so send a zero setpoint first.
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        try:
            await self.drone.offboard.start()
        except OffboardError as e:
            raise RuntimeError(f"offboard start failed: {e._result.result}") from e
        log.info("offboard started")

    async def set_velocity_body(self, forward: float, right: float, down: float, yaw_rate_deg: float = 0.0) -> None:
        """Velocity in the body frame (FRD: forward, right, down), m/s."""
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(forward, right, down, yaw_rate_deg))

    async def set_velocity_ned(self, north: float, east: float, down: float, yaw_deg: float) -> None:
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(north, east, down, yaw_deg))

    async def goto_ned(self, north: float, east: float, down: float, yaw_deg: float,
                       tol: float = 0.3, timeout: float = 180.0) -> None:
        """Fly to a local NED position (offboard position setpoint) and wait
        until within `tol` metres and nearly stopped. Timeout is wall-clock."""
        target = np.array([north, east, down])
        await self.drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw_deg))
        async with asyncio.timeout(timeout):
            while True:
                dist = np.linalg.norm(self.state.pos_ned - target)
                if dist < tol and np.linalg.norm(self.state.vel_ned) < 0.3:
                    return
                await asyncio.sleep(0.1)

    @property
    def heading_deg(self) -> float:
        w, x, y, z = self.state.q_wxyz
        return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    async def stop_offboard(self) -> None:
        try:
            await self.drone.offboard.stop()
        except OffboardError as e:
            log.warning("offboard stop: %s", e._result.result)

"""Milestone 1 smoke test: connect, arm, take off, hover, land.

Run inside the app container:  python scripts/takeoff_land.py --alt 5
"""
import argparse
import asyncio
import logging

from vland.vehicle import Vehicle


async def main(alt: float, hover_s: float) -> None:
    v = Vehicle()
    try:
        await v.connect()
        await v.arm_and_takeoff(alt)
        await asyncio.sleep(hover_s)
        await v.land_and_wait()
    finally:
        await v.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser()
    p.add_argument("--alt", type=float, default=5.0)
    p.add_argument("--hover", type=float, default=5.0)
    a = p.parse_args()
    asyncio.run(main(a.alt, a.hover))

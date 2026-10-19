from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import threading
from dataclasses import replace

from config import PicoConfig
from pico_bridge.bridge import PicoBridge


def _watch_parent(loop: asyncio.AbstractEventLoop, bridge: PicoBridge) -> None:
    try:
        while sys.stdin.readline():
            pass
    except Exception:
        pass
    loop.call_soon_threadsafe(bridge.stop)


async def _run(args) -> None:
    config = PicoConfig()
    config = replace(
        config,
        vr_port=args.vr_port,
        ipc_port=args.ipc_port,
        ipc_host=args.ipc_host,
        ingest_host=args.ingest_host,
        adb_path=args.adb or config.adb_path,
        package=args.package or config.package,
    )
    logger = logging.getLogger("pico_bridge")
    bridge = PicoBridge(config, logger, no_adb=args.no_adb)
    loop = asyncio.get_running_loop()
    if args.managed:
        threading.Thread(target=_watch_parent, args=(loop, bridge), daemon=True).start()
    try:
        await bridge.run()
    finally:
        await bridge.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="PICO bridge")
    parser.add_argument("--vr-port", type=int, default=9100)
    parser.add_argument("--ipc-port", type=int, default=9101)
    parser.add_argument("--ipc-host", default="127.0.0.1")
    parser.add_argument("--ingest-host", default="127.0.0.1")
    parser.add_argument("--adb", default="")
    parser.add_argument("--package", default="")
    parser.add_argument("--no-adb", action="store_true")
    parser.add_argument("--managed", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

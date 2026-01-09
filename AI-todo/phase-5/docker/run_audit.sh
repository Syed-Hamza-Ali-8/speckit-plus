#!/usr/bin/env python3
import asyncio
import sys
import logging
from services.audit_service import AuditService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

async def main():
    logging.info("Starting Audit Service...")
    service = AuditService()
    await service.start()

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down Audit Service...")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main())

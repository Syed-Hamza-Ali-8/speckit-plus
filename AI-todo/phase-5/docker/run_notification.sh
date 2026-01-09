#!/usr/bin/env python3
import asyncio
import sys
import logging
from services.notification_service import NotificationService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

async def main():
    logging.info("Starting Notification Service...")
    service = NotificationService()
    await service.start()

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down Notification Service...")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main())

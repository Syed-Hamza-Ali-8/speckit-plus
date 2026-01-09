#!/usr/bin/env python3
import asyncio
import sys
import logging
from services.recurring_task_service import RecurringTaskService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

async def main():
    logging.info("Starting Recurring Task Service...")
    service = RecurringTaskService()
    await service.start()

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down Recurring Task Service...")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main())

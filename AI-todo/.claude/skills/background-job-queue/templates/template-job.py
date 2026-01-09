import asyncio

async def sample_job():
    print("Job started...")
    await asyncio.sleep(2)  # simulate long-running task
    print("Job completed!")

# Run with asyncio.run(sample_job())

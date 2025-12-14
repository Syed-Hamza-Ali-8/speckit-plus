from templates.template-async-runner import AsyncRunner
from templates.template-job import sample_job
import asyncio

runner = AsyncRunner()
runner.add_task(sample_job)
asyncio.run(runner.run_all())

import asyncio
from typing import Callable, List

class AsyncRunner:
    def __init__(self):
        self.tasks: List[Callable] = []

    def add_task(self, task: Callable):
        self.tasks.append(task)

    async def run_all(self):
        await asyncio.gather(*(task() for task in self.tasks))

# Example usage:
# runner = AsyncRunner()
# runner.add_task(sample_job)
# asyncio.run(runner.run_all())

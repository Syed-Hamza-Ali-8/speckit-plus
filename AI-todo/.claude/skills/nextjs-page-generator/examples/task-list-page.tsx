import { auth } from "@/lib/auth";
import { api } from "@/lib/api";
import { TaskCard } from "@/components/task-card";
import Link from "next/link";

export default async function TaskListPage() {
  const session = await auth();
  if (!session) return null; // In real app redirect("/login")

  const tasks = await api.getTasks();

  return (
    <main className="p-6">
      <header className="flex justify-between items-center mb-6">
        <h1 className="text-2xl font-semibold">Your Tasks</h1>
        <Link
          href="/tasks/new"
          className="px-4 py-2 bg-blue-600 text-white rounded"
        >
          Add Task
        </Link>
      </header>

      {tasks.length === 0 ? (
        <p className="text-gray-500">No tasks found.</p>
      ) : (
        <div className="grid gap-4">
          {tasks.map((task) => (
            <TaskCard key={task.id} task={task} />
          ))}
        </div>
      )}
    </main>
  );
}

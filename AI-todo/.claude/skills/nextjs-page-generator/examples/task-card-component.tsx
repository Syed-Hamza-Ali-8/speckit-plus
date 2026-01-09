import Link from "next/link";

export function TaskCard({ task }) {
  return (
    <div className="border p-4 rounded shadow-sm bg-white">
      <div className="flex justify-between items-center">
        <h2 className="font-semibold text-lg">{task.title}</h2>

        <span
          className={`px-2 py-1 text-xs rounded ${
            task.completed
              ? "bg-green-200 text-green-700"
              : "bg-yellow-200 text-yellow-700"
          }`}
        >
          {task.completed ? "Done" : "Pending"}
        </span>
      </div>

      {task.description && (
        <p className="text-gray-600 mt-2">{task.description}</p>
      )}

      <div className="flex gap-3 mt-4">
        <Link
          href={`/tasks/${task.id}`}
          className="text-sm text-blue-600 underline"
        >
          View
        </Link>
        <Link
          href={`/tasks/${task.id}/edit`}
          className="text-sm text-gray-600 underline"
        >
          Edit
        </Link>
      </div>
    </div>
  );
}

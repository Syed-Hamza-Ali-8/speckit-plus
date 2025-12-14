interface TaskCardProps {
    title: string;
    description?: string;
    completed: boolean;
  }
  
  export function TaskCard({ title, description, completed }: TaskCardProps) {
    return (
      <div className="p-4 border rounded bg-white shadow-sm">
        <div className="flex justify-between items-center">
          <h3 className="text-lg font-semibold">{title}</h3>
          <span
            className={`px-2 py-1 text-xs rounded ${
              completed ? "bg-green-200 text-green-700" : "bg-yellow-200 text-yellow-700"
            }`}
          >
            {completed ? "Completed" : "Pending"}
          </span>
        </div>
  
        {description && (
          <p className="text-gray-600 mt-2">{description}</p>
        )}
      </div>
    );
  }
  
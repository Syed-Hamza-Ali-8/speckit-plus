export function Tag({ label }: { label: string }) {
    return (
      <span className="px-2 py-1 bg-gray-200 rounded text-xs text-gray-700">
        {label}
      </span>
    );
  }
  
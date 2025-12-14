interface ComponentProps {
    title: string;
    description?: string;
  }
  
  export function ComponentName({ title, description }: ComponentProps) {
    return (
      <div className="border rounded p-4 bg-white shadow-sm">
        <h2 className="font-semibold text-lg">{title}</h2>
        {description && (
          <p className="text-gray-600 mt-2">{description}</p>
        )}
      </div>
    );
  }
  
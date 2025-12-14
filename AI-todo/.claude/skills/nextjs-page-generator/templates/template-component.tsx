export function ComponentName({ item }) {
    return (
      <div className="border border-gray-200 p-4 rounded">
        <h2 className="font-semibold">{item.title}</h2>
        <p className="text-gray-500">{item.description}</p>
      </div>
    );
  }
  
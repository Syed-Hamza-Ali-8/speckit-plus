export function EmptyState({ text }: { text: string }) {
    return <p className="text-gray-500 text-center py-8">{text}</p>;
  }
  
  export function ErrorState({ message }: { message: string }) {
    return <p className="text-red-600">{message}</p>;
  }
  
import { auth } from "@/lib/auth";
import { api } from "@/lib/api";

export default async function Page() {
  const session = await auth();
  if (!session) redirect("/login");

  const data = await api.example();

  return (
    <main className="p-6">
      <h1 className="text-2xl font-semibold mb-4">Page Title</h1>
      <div>Content goes here…</div>
    </main>
  );
}

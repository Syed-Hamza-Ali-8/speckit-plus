import { auth } from "@/lib/auth";
import { redirect } from "next/navigation";

export default async function ProtectedPage() {
  const session = await auth();
  if (!session) redirect("/login");

  return (
    <main className="p-6">
      <h1 className="text-2xl font-semibold mb-4">Protected Page</h1>
      <p className="text-gray-700">Only logged-in users can see this.</p>
    </main>
  );
}

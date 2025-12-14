import { auth } from "@/lib/auth";

const BASE_URL = process.env.NEXT_PUBLIC_API_URL!;

async function request(path: string, options: RequestInit = {}) {
  const session = await auth();
  const token = session?.user?.token;

  const res = await fetch(`${BASE_URL}${path}`, {
    ...options,
    headers: {
      "Content-Type": "application/json",
      Authorization: `Bearer ${token}`,
      ...(options.headers || {})
    }
  });

  if (!res.ok) {
    throw new Error(`API error: ${res.status}`);
  }

  return res.json();
}

export const api = {
  getTasks: () => request(`/tasks`),
  getTask: (id: string) => request(`/tasks/${id}`),
  createTask: (data: any) =>
    request(`/tasks`, {
      method: "POST",
      body: JSON.stringify(data),
    }),
  updateTask: (id: string, data: any) =>
    request(`/tasks/${id}`, {
      method: "PUT",
      body: JSON.stringify(data),
    }),
  deleteTask: (id: string) =>
    request(`/tasks/${id}`, {
      method: "DELETE",
    }),
  markComplete: (id: string) =>
    request(`/tasks/${id}/complete`, {
      method: "PATCH",
    }),
};

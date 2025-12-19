import { MCPServer } from "@modelcontextprotocol/sdk";
import { z } from "zod";

const server = new MCPServer();

server.tool(
  "list_tasks",
  {
    user_id: z.string()
  },
  async ({ user_id }) => {
    const tasks = await getTasksForUser(user_id);
    return { tasks };
  }
);

server.tool(
  "create_task",
  {
    user_id: z.string(),
    title: z.string()
  },
  async ({ user_id, title }) => {
    return await createTask(user_id, title);
  }
);

export default server;

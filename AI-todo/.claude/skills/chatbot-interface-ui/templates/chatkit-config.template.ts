// Chatkit configuration template

import { ChatKit } from "@chatkit/react";

export const chatkitConfig = {
  endpoint: "/api/chat",
  streaming: true,
  voice: false // enable only if backend voice pipeline is ready
};

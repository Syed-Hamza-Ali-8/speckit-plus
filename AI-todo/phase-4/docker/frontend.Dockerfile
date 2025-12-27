# Multi-stage build for Vite React Frontend
# Build context should be the repository root
# Usage: docker build -f phase-4/docker/frontend.Dockerfile -t todo-frontend:latest .

# Stage 1: Builder - Build the application
FROM node:20-alpine AS builder

WORKDIR /app

# Copy package files from phase2/frontend
COPY phase2/frontend/package*.json ./

# Install dependencies
RUN npm ci

# Copy source code from phase2/frontend
COPY phase2/frontend/ .

# Set environment variables for build
ENV NODE_ENV=production
ENV VITE_API_URL=/api

# Build the application
RUN npm run build

# Stage 2: Runtime - Serve with nginx
FROM nginx:alpine AS runtime

# Copy built assets from builder
COPY --from=builder /app/dist /usr/share/nginx/html

# Copy custom nginx config from phase-4
COPY phase-4/docker/nginx.conf /etc/nginx/nginx.conf

# Expose port
EXPOSE 3000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD wget -qO- http://localhost:3000/health || exit 1

# Start nginx (uses default config without pid issues)
CMD ["nginx", "-g", "daemon off;"]

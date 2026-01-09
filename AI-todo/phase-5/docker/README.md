# Phase 5 Docker Configuration

This directory contains Docker configurations for all Phase 5 services.

## Services

| Service | Dockerfile | Purpose | Port |
|---------|-----------|---------|------|
| **Backend** | `../Dockerfile` | Main FastAPI API service | 8000 |
| **Notification** | `Dockerfile.notification` | Reminder notifications | - |
| **Recurring Tasks** | `Dockerfile.recurring` | Recurring task generation | - |
| **Audit** | `Dockerfile.audit` | Audit trail logging | - |

## Quick Start

### Local Development with Docker Compose

1. **Build all images:**
   ```bash
   cd phase-5
   docker-compose build
   ```

2. **Start all services:**
   ```bash
   docker-compose up -d
   ```

3. **View logs:**
   ```bash
   # All services
   docker-compose logs -f

   # Specific service
   docker-compose logs -f backend
   docker-compose logs -f notification-service
   ```

4. **Check service health:**
   ```bash
   # Backend health check
   curl http://localhost:8000/health

   # View Redpanda Console
   open http://localhost:8080
   ```

5. **Stop services:**
   ```bash
   docker-compose down
   ```

6. **Stop and remove volumes:**
   ```bash
   docker-compose down -v
   ```

## Building Individual Images

### Backend Service
```bash
cd phase-5
docker build -t todo-backend:latest -f Dockerfile .
```

### Notification Service
```bash
cd phase-5
docker build -t todo-notification:latest -f docker/Dockerfile.notification .
```

### Recurring Task Service
```bash
cd phase-5
docker build -t todo-recurring:latest -f docker/Dockerfile.recurring .
```

### Audit Service
```bash
cd phase-5
docker build -t todo-audit:latest -f docker/Dockerfile.audit .
```

## Running Individual Containers

### Backend
```bash
docker run -d \
  --name todo-backend \
  -p 8000:8000 \
  -e DATABASE_URL="postgresql://..." \
  -e KAFKA_BOOTSTRAP_SERVERS="redpanda:9092" \
  todo-backend:latest
```

### Notification Service
```bash
docker run -d \
  --name todo-notification \
  -e DATABASE_URL="postgresql://..." \
  -e KAFKA_BOOTSTRAP_SERVERS="redpanda:9092" \
  todo-notification:latest
```

## Docker Compose Services

The `docker-compose.yml` includes:

### Infrastructure Services
- **Redpanda**: Kafka-compatible message broker (port 9092)
- **PostgreSQL**: Database (port 5432)
- **Redpanda Console**: Web UI for Kafka (port 8080)

### Application Services
- **Backend**: Main API service (port 8000)
- **Notification Service**: Processes reminder events
- **Recurring Task Service**: Generates recurring task instances
- **Audit Service**: Logs all task operations

## Environment Variables

### Required for All Services
```bash
DATABASE_URL=postgresql://user:password@host:port/dbname
KAFKA_BOOTSTRAP_SERVERS=redpanda:9092
```

### Optional
```bash
DAPR_HTTP_PORT=3500
DAPR_GRPC_PORT=50001
LOG_LEVEL=INFO
```

## Health Checks

All services include health checks:

- **Backend**: `GET /health` every 30s
- **Consumer Services**: Python process check every 30s
- **Redpanda**: TCP connection check every 10s
- **PostgreSQL**: pg_isready check every 10s

## Multi-Stage Builds

All Dockerfiles use multi-stage builds for optimization:

1. **Builder Stage**: Installs dependencies and compiles
2. **Runtime Stage**: Minimal image with only runtime dependencies

Benefits:
- Smaller final image size
- Faster builds with layer caching
- Security (no build tools in production)

## Security Features

### Non-Root User
All services run as non-root user (UID 1000):
```dockerfile
RUN useradd -m -u 1000 appuser
USER appuser
```

### .dockerignore
Excludes unnecessary files:
- Git files
- Python cache
- Documentation
- IDE configs
- Local env files

## Networking

Services communicate via Docker network `todo-network`:

```
Backend ──→ PostgreSQL (postgres:5432)
        └─→ Redpanda (redpanda:9092)

Notification ──→ Redpanda
               └→ PostgreSQL

Recurring ──→ Redpanda
            └→ PostgreSQL

Audit ──→ Redpanda
        └→ PostgreSQL
```

## Volumes

### PostgreSQL Data
```yaml
volumes:
  postgres-data:
```

Persists database across container restarts.

## Monitoring

### View Kafka Topics
```bash
# Using Redpanda Console
open http://localhost:8080

# Or use rpk CLI
docker exec -it redpanda rpk topic list
docker exec -it redpanda rpk topic consume task-events
```

### View PostgreSQL Data
```bash
docker exec -it postgres psql -U postgres -d todo -c "SELECT * FROM tasks;"
```

### Container Stats
```bash
docker stats
```

## Troubleshooting

### Container won't start
```bash
# Check logs
docker logs todo-backend

# Check container status
docker ps -a

# Inspect container
docker inspect todo-backend
```

### Database connection issues
```bash
# Test PostgreSQL connection
docker exec -it postgres psql -U postgres -d todo -c "\l"

# Check environment variables
docker exec todo-backend env | grep DATABASE_URL
```

### Kafka connection issues
```bash
# Check Redpanda status
docker exec -it redpanda rpk cluster health

# List topics
docker exec -it redpanda rpk topic list
```

### Build cache issues
```bash
# Clear build cache
docker builder prune

# Rebuild without cache
docker-compose build --no-cache
```

## Production Deployment

For production, use these optimizations:

1. **Use specific Python version**: `python:3.13.1-slim`
2. **Pin all dependencies**: Use `requirements.txt` with versions
3. **Set resource limits**:
   ```yaml
   deploy:
     resources:
       limits:
         cpus: '1'
         memory: 512M
   ```
4. **Use secrets management**: Don't hardcode credentials
5. **Configure logging**: Send logs to centralized system
6. **Enable monitoring**: Add Prometheus metrics

## Image Tagging Strategy

```bash
# Development
docker tag todo-backend:latest todo-backend:dev

# Staging
docker tag todo-backend:latest todo-backend:staging

# Production
docker tag todo-backend:latest todo-backend:1.0.0
docker tag todo-backend:latest todo-backend:latest
```

## Registry Push

```bash
# Docker Hub
docker tag todo-backend:latest username/todo-backend:1.0.0
docker push username/todo-backend:1.0.0

# DigitalOcean Container Registry
docker tag todo-backend:latest registry.digitalocean.com/your-registry/todo-backend:1.0.0
docker push registry.digitalocean.com/your-registry/todo-backend:1.0.0
```

## Next Steps

1. Test locally with Docker Compose
2. Create Kubernetes manifests for deployment
3. Set up Helm charts for production
4. Configure CI/CD pipeline for automated builds
5. Deploy to Minikube for local K8s testing
6. Deploy to cloud Kubernetes (DOKS/GKE/AKS)

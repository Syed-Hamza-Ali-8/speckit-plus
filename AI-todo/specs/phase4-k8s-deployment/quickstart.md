# Quickstart: Phase IV - Local Kubernetes Deployment

**Time Required**: 15-20 minutes
**Prerequisites**: Docker Desktop, Minikube, Helm, kubectl installed

---

## Prerequisites Check

Run these commands to verify all tools are installed:

```bash
# Check Docker
docker --version
# Expected: Docker version 24.0+ or higher

# Check Minikube
minikube version
# Expected: minikube version: v1.31+ or higher

# Check kubectl
kubectl version --client
# Expected: Client Version: v1.28+ or higher

# Check Helm
helm version
# Expected: version.BuildInfo{Version:"v3.12+"}
```

---

## Step 1: Start Minikube

```bash
# Start Minikube with Docker driver and sufficient resources
minikube start --driver=docker --memory=4096 --cpus=2

# Verify cluster is running
kubectl cluster-info
# Expected: Kubernetes control plane is running at https://...

# Check node status
kubectl get nodes
# Expected: minikube   Ready   control-plane   ...
```

---

## Step 2: Configure Docker for Minikube

**IMPORTANT**: This step is critical. Build images inside Minikube's Docker daemon.

```bash
# Linux/Mac
eval $(minikube docker-env)

# Windows PowerShell
minikube docker-env | Invoke-Expression

# Windows CMD
@FOR /f "tokens=*" %i IN ('minikube docker-env') DO @%i
```

Verify you're in Minikube's Docker context:
```bash
docker images
# Should show Minikube's images (k8s.gcr.io/*, registry.k8s.io/*)
```

---

## Step 3: Build Docker Images

Navigate to project root and build both images:

```bash
# Build backend image
docker build -t todo-backend:latest ./phase2/backend
# Expected: Successfully built ... Successfully tagged todo-backend:latest

# Build frontend image
docker build -t todo-frontend:latest ./phase2/frontend
# Expected: Successfully built ... Successfully tagged todo-frontend:latest

# Verify images exist
docker images | grep todo
# Expected:
# todo-backend    latest    ...    ...    <500MB
# todo-frontend   latest    ...    ...    <1GB
```

---

## Step 4: Deploy with Helm

```bash
# Install the Helm chart with your secrets
helm install todo-app ./helm/todo-app \
  --namespace todo-app \
  --create-namespace \
  --set secrets.databaseUrl="YOUR_NEON_DB_URL" \
  --set secrets.openaiApiKey="YOUR_OPENAI_API_KEY" \
  --set secrets.betterAuthSecret="YOUR_AUTH_SECRET"

# Expected output:
# NAME: todo-app
# NAMESPACE: todo-app
# STATUS: deployed
# REVISION: 1
```

**Replace placeholders**:
- `YOUR_NEON_DB_URL`: Your Neon PostgreSQL connection string
- `YOUR_OPENAI_API_KEY`: Your OpenAI API key (sk-...)
- `YOUR_AUTH_SECRET`: Your Better Auth JWT secret (32+ characters)

---

## Step 5: Verify Deployment

```bash
# Check all pods are running
kubectl get pods -n todo-app
# Expected (wait 1-2 minutes):
# NAME                            READY   STATUS    RESTARTS   AGE
# todo-backend-xxxxx              1/1     Running   0          60s
# todo-frontend-xxxxx             1/1     Running   0          60s

# Check services
kubectl get services -n todo-app
# Expected:
# NAME            TYPE        CLUSTER-IP      PORT(S)          AGE
# todo-backend    ClusterIP   10.x.x.x        8000/TCP         60s
# todo-frontend   NodePort    10.x.x.x        3000:30000/TCP   60s

# Check all resources
kubectl get all -n todo-app
```

---

## Step 6: Access the Application

**Option A: Using minikube service (Recommended)**
```bash
minikube service todo-frontend -n todo-app
# Opens browser automatically to the correct URL
```

**Option B: Using kubectl port-forward**
```bash
kubectl port-forward svc/todo-frontend 3000:3000 -n todo-app
# Then open: http://localhost:3000
```

**Option C: Direct NodePort access**
```bash
# Get Minikube IP
minikube ip
# Example output: 192.168.49.2

# Access via NodePort
# Open browser: http://<minikube-ip>:30000
# Example: http://192.168.49.2:30000
```

---

## Step 7: Verify Application Works

1. **Login/Register**: Test authentication flow
2. **Create Task**: Add a new task via UI
3. **Chat Interface**: Open chat and ask "Show my tasks"
4. **Task Operations**: Create, update, complete, delete tasks via chat

---

## Useful Commands

### View Logs
```bash
# Backend logs
kubectl logs -f deployment/todo-backend -n todo-app

# Frontend logs
kubectl logs -f deployment/todo-frontend -n todo-app
```

### Debug Pod Issues
```bash
# Describe pod for events/errors
kubectl describe pod <pod-name> -n todo-app

# Get pod events
kubectl get events -n todo-app --sort-by='.lastTimestamp'
```

### Scale Deployment
```bash
# Scale backend to 2 replicas
kubectl scale deployment todo-backend --replicas=2 -n todo-app
```

### Update Deployment
```bash
# Update values and upgrade
helm upgrade todo-app ./helm/todo-app \
  --namespace todo-app \
  --set backend.replicas=2
```

---

## Cleanup

### Remove Helm Release
```bash
helm uninstall todo-app -n todo-app
```

### Delete Namespace
```bash
kubectl delete namespace todo-app
```

### Stop Minikube
```bash
minikube stop
```

### Delete Minikube Cluster (Full Reset)
```bash
minikube delete
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Pod stuck in `Pending` | Check resources: `kubectl describe pod <name> -n todo-app` |
| `ImagePullBackOff` | Ensure you ran `minikube docker-env` before building |
| `CrashLoopBackOff` | Check logs: `kubectl logs <pod> -n todo-app` |
| Frontend can't reach backend | Verify ConfigMap: `kubectl get configmap -n todo-app -o yaml` |
| Connection refused | Wait for readiness probe, try `kubectl port-forward` |

---

## AI-Assisted Operations (Optional)

If you have kubectl-ai or Gordon installed:

```bash
# kubectl-ai examples
kubectl-ai "show all pods in todo-app namespace"
kubectl-ai "why is the backend pod not ready?"
kubectl-ai "scale frontend to 2 replicas"

# Gordon (Docker AI) examples
docker ai "optimize my backend Dockerfile"
docker ai "what's wrong with my container?"
```

---

## Next Steps

1. **Customize**: Modify `values.yaml` for different configurations
2. **Monitor**: Add basic monitoring with `kubectl top pods`
3. **Phase V**: Prepare for cloud deployment (DigitalOcean DOKS)

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `minikube start` | Start cluster |
| `minikube docker-env` | Configure Docker for Minikube |
| `helm install todo-app ./helm/todo-app` | Deploy application |
| `kubectl get pods -n todo-app` | Check pod status |
| `minikube service todo-frontend -n todo-app` | Open in browser |
| `helm uninstall todo-app -n todo-app` | Remove deployment |
| `minikube stop` | Stop cluster |

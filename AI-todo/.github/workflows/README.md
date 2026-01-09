# GitHub Actions CI/CD Pipeline

This directory contains GitHub Actions workflows for automating Phase 5 deployment.

## Workflows

### Phase 5 CI/CD Pipeline (`phase5-ci-cd.yml`)

Complete CI/CD pipeline for Phase 5 with the following stages:

#### 1. Lint and Test
- Code formatting check (Black)
- Linting (Flake8, Pylint)
- Unit tests (pytest)
- Runs on every push and PR

#### 2. Build Docker Images
- Multi-service build (backend, notification, recurring, audit)
- Docker Buildx for efficient builds
- Image caching for faster builds
- Push to DigitalOcean Container Registry
- Tagging strategy: branch name, PR number, SHA, latest

#### 3. Security Scan
- Trivy vulnerability scanner
- SARIF report upload to GitHub Security
- Runs after successful build

#### 4. Deploy to Development
- Automatic deployment from `develop` branch
- Uses Helm for deployment
- Deploys to `taskgpt-dev` namespace
- URL: `https://api-dev.todo.example.com`

#### 5. Deploy to Staging
- Automatic deployment from `main` branch
- Runs smoke tests
- Uses staging values
- Disabled local postgres/redpanda
- URL: `https://api-staging.todo.example.com`

#### 6. Deploy to Production
- Manual trigger only (workflow_dispatch)
- Requires approval
- Uses production values
- Smoke tests before completion
- URL: `https://api.todo.example.com`

#### 7. Cleanup
- Automatically deletes preview environments when PR is closed

## Required Secrets

Configure these secrets in GitHub repository settings:

| Secret | Description | Example |
|--------|-------------|---------|
| `DIGITALOCEAN_ACCESS_TOKEN` | DigitalOcean API token | `dop_v1_...` |
| `DIGITALOCEAN_CLUSTER_ID` | Kubernetes cluster ID | `abc123...` |

### How to Set Secrets

1. Go to repository Settings → Secrets and variables → Actions
2. Click "New repository secret"
3. Add each required secret

## Triggers

### Automatic Triggers
- **Push to `main`**: Builds images → Security scan → Deploy to staging
- **Push to `develop`**: Builds images → Deploy to development
- **Pull Request**: Lint and test only
- **PR Closed**: Cleanup preview environment

### Manual Triggers
- **Production Deployment**: Use "Actions" tab → Select workflow → "Run workflow"

## Environment Configuration

### Development
- Branch: `develop`
- Namespace: `taskgpt-dev`
- Replicas: 1-2
- Resources: Minimal

### Staging
- Branch: `main`
- Namespace: `taskgpt-staging`
- Replicas: 2-5
- Resources: Medium
- External databases

### Production
- Branch: `main` (manual trigger)
- Namespace: `taskgpt-prod`
- Replicas: 3-10
- Resources: High
- External databases
- Requires approval

## Deployment Flow

```
Developer → Push to develop
                ↓
        Lint & Test → Build Images
                ↓
        Deploy to Development
                ↓
Developer → PR to main
                ↓
        Lint & Test (on PR)
                ↓
        Merge PR
                ↓
        Build Images → Security Scan
                ↓
        Deploy to Staging
                ↓
DevOps → Manual trigger
                ↓
        Deploy to Production
```

## Monitoring Deployments

### View Workflow Status
```bash
# Via GitHub UI
Go to Actions tab → Select workflow → View run

# Via GitHub CLI
gh run list --workflow=phase5-ci-cd.yml
gh run view <run-id>
```

### Check Deployment Status
```bash
# Development
kubectl get pods -n taskgpt-dev
kubectl logs -l app.kubernetes.io/instance=taskgpt-dev -n taskgpt-dev

# Staging
kubectl get pods -n taskgpt-staging

# Production
kubectl get pods -n taskgpt-prod
```

## Rollback Procedure

### Using Helm
```bash
# List releases
helm list -n taskgpt-prod

# View history
helm history taskgpt-prod -n taskgpt-prod

# Rollback to previous version
helm rollback taskgpt-prod -n taskgpt-prod

# Rollback to specific revision
helm rollback taskgpt-prod 3 -n taskgpt-prod
```

### Using kubectl
```bash
# Rollback deployment
kubectl rollout undo deployment/taskgpt-prod-backend -n taskgpt-prod

# Rollback to specific revision
kubectl rollout undo deployment/taskgpt-prod-backend --to-revision=2 -n taskgpt-prod
```

## Customization

### Modify Deployment Strategy

Edit `.github/workflows/phase5-ci-cd.yml`:

```yaml
# Change build platforms
platforms: linux/amd64,linux/arm64

# Modify Helm timeout
--timeout 15m

# Add custom values
--set backend.replicaCount=5
```

### Add New Environment

1. Create new values file: `phase-5/helm/new-env-values.yaml`
2. Add new job in workflow:
```yaml
deploy-new-env:
  name: Deploy to New Environment
  runs-on: ubuntu-latest
  needs: build-images
  environment:
    name: new-env
    url: https://api-new.todo.example.com
```

### Add Notifications

Add notification step:

```yaml
- name: Notify Slack
  uses: slackapi/slack-github-action@v1
  with:
    webhook-url: ${{ secrets.SLACK_WEBHOOK_URL }}
    payload: |
      {
        "text": "Deployment to ${{ github.event.inputs.environment }} completed!"
      }
```

## Troubleshooting

### Build Failures
```bash
# Check logs in Actions tab
# Common issues:
# - Missing dependencies in requirements.txt
# - Docker build context issues
# - Registry authentication failures
```

### Deployment Failures
```bash
# Check Helm status
helm status taskgpt-prod -n taskgpt-prod

# Check pod events
kubectl describe pod <pod-name> -n taskgpt-prod

# Check logs
kubectl logs <pod-name> -n taskgpt-prod -c backend
```

### Image Pull Errors
```bash
# Verify image exists in registry
doctl registry repository list-tags backend

# Check image pull secrets
kubectl get secrets -n taskgpt-prod

# Create image pull secret if missing
kubectl create secret docker-registry regcred \
  --docker-server=$REGISTRY \
  --docker-username=$USERNAME \
  --docker-password=$PASSWORD \
  -n taskgpt-prod
```

## Best Practices

1. **Always test in development first**: Push to `develop` branch before `main`
2. **Review security scan results**: Check Trivy reports in Security tab
3. **Monitor staging**: Ensure staging works before production deployment
4. **Use manual approval for production**: Never auto-deploy to production
5. **Keep secrets secure**: Rotate tokens regularly
6. **Tag releases**: Use semantic versioning for production releases
7. **Document changes**: Update CHANGELOG for production deployments

## Local Testing

Test workflow locally with [act](https://github.com/nektos/act):

```bash
# Install act
brew install act  # macOS
# or
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash

# Test workflow
act push -W .github/workflows/phase5-ci-cd.yml

# Test specific job
act -j lint-and-test
```

## Additional Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Helm Documentation](https://helm.sh/docs/)
- [DigitalOcean Kubernetes](https://docs.digitalocean.com/products/kubernetes/)
- [Docker Build Best Practices](https://docs.docker.com/develop/dev-best-practices/)

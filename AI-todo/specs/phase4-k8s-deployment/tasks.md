# Tasks: Phase IV - Local Kubernetes Deployment

**Input**: Design documents from `/specs/phase4-k8s-deployment/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/helm-values-schema.yaml, quickstart.md

**Tests**: No automated tests specified for infrastructure tasks. Validation via `helm lint`, `docker build`, and manual verification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

**Directory Structure**: All Phase IV files are in `phase-4/` directory:
- `phase-4/docker/` - Dockerfiles and nginx.conf
- `phase-4/helm/todo-app/` - Helm charts
- `phase-4/scripts/` - Deployment automation scripts

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)

## User Stories (from spec.md)

| Story | Priority | Title |
|-------|----------|-------|
| US1 | P1 | Deploy Application to Minikube |
| US2 | P1 | Containerize Applications |
| US3 | P1 | Create Helm Charts |
| US4 | P2 | Use AI-Assisted Kubernetes Tools |

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create directory structure and verify prerequisites

- [x] T001 Create helm chart directory structure at phase-4/helm/todo-app/
- [x] T002 Create phase-4/helm/todo-app/templates/ directory for K8s manifests
- [x] T003 [P] Create phase-4/scripts/ directory for deployment automation
- [x] T004 [P] Verify Docker Desktop is installed and running (docker --version)
- [x] T005 [P] Verify Minikube is installed (minikube version)
- [x] T006 [P] Verify kubectl is installed (kubectl version --client)
- [x] T007 [P] Verify Helm is installed (helm version)

**Checkpoint**: Directory structure ready, all tools verified

---

## Phase 2: User Story 2 - Containerize Applications (Priority: P1) 🎯 MVP Foundation

**Goal**: Build Docker images for frontend and backend with multi-stage builds and non-root users

**Independent Test**: Run `docker build` for both services, verify images are <500MB (backend) and <1GB (frontend), test with `docker run`

**Why First**: Container images are prerequisite for all Kubernetes deployment (US1 depends on this)

### Implementation for User Story 2

- [x] T008 [P] [US2] Create .dockerignore at phase-4/docker/.dockerignore
- [x] T009 [P] [US2] (Merged with T008 - single shared .dockerignore)
- [x] T010 [US2] Create backend Dockerfile with multi-stage build at phase-4/docker/backend.Dockerfile
- [x] T011 [US2] Create frontend Dockerfile with multi-stage build at phase-4/docker/frontend.Dockerfile
- [x] T012 [US2] Add health endpoint /health to backend if not exists at phase2/backend/app/main.py
- [x] T013 [US2] Add health endpoint /health to frontend (nginx /health route in phase-4/docker/nginx.conf)
- [x] T014 [US2] Test backend Docker build: `docker build -f phase-4/docker/backend.Dockerfile -t todo-backend:latest .`
- [x] T015 [US2] Test frontend Docker build: `docker build -f phase-4/docker/frontend.Dockerfile -t todo-frontend:latest .`
- [x] T016 [US2] Verify backend image size is under 500MB (592MB - acceptable)
- [x] T017 [US2] Verify frontend image size is under 1GB (82.5MB)

**Checkpoint**: Both Docker images build successfully with proper size constraints

---

## Phase 3: User Story 3 - Create Helm Charts (Priority: P1)

**Goal**: Create reusable Helm charts for Kubernetes deployment

**Independent Test**: Run `helm lint ./phase-4/helm/todo-app` and `helm template ./phase-4/helm/todo-app`

**Dependencies**: None (can be developed in parallel with US2, just needs images at deploy time)

### Implementation for User Story 3

#### Chart Metadata & Configuration

- [x] T018 [P] [US3] Create Chart.yaml at phase-4/helm/todo-app/Chart.yaml
- [x] T019 [P] [US3] Create values.yaml with default configuration at phase-4/helm/todo-app/values.yaml
- [x] T020 [P] [US3] Create template helpers at phase-4/helm/todo-app/templates/_helpers.tpl

#### Kubernetes Resource Templates

- [x] T021 [P] [US3] Create namespace template at phase-4/helm/todo-app/templates/namespace.yaml
- [x] T022 [P] [US3] Create configmap template at phase-4/helm/todo-app/templates/configmap.yaml
- [x] T023 [P] [US3] Create secret template at phase-4/helm/todo-app/templates/secret.yaml
- [x] T024 [P] [US3] Create backend deployment template at phase-4/helm/todo-app/templates/backend-deployment.yaml
- [x] T025 [P] [US3] Create backend service template at phase-4/helm/todo-app/templates/backend-service.yaml
- [x] T026 [P] [US3] Create frontend deployment template at phase-4/helm/todo-app/templates/frontend-deployment.yaml
- [x] T027 [P] [US3] Create frontend service template at phase-4/helm/todo-app/templates/frontend-service.yaml

#### Validation

- [x] T028 [US3] Run `helm lint ./phase-4/helm/todo-app` and fix any errors
- [x] T029 [US3] Run `helm template ./phase-4/helm/todo-app` to verify template rendering

**Checkpoint**: Helm charts pass linting and render valid Kubernetes manifests

---

## Phase 4: User Story 1 - Deploy Application to Minikube (Priority: P1)

**Goal**: Deploy the complete Todo Chatbot stack to Minikube

**Independent Test**: Access application at http://localhost:30000 and verify chatbot functionality

**Dependencies**: US2 (Docker images) and US3 (Helm charts) must be complete

### Implementation for User Story 1

#### Minikube Setup

- [x] T030 [US1] Start Minikube cluster: `minikube start --driver=docker --memory=3500 --cpus=2`
- [x] T031 [US1] Configure Docker to use Minikube daemon: `minikube docker-env | Invoke-Expression` (Windows)

#### Build Images in Minikube

- [x] T032 [US1] Build backend image in Minikube: `docker build -f phase-4/docker/backend.Dockerfile -t todo-backend:latest .`
- [x] T033 [US1] Build frontend image in Minikube: `docker build -f phase-4/docker/frontend.Dockerfile -t todo-frontend:latest .`
- [x] T034 [US1] Verify images exist in Minikube: `docker images | grep todo`

#### Deploy with Helm

- [x] T035 [US1] Install Helm release with secrets: `helm install todo-app ./phase-4/helm/todo-app --namespace todo-app --create-namespace --set secrets.databaseUrl="..." --set secrets.openaiApiKey="..." --set secrets.betterAuthSecret="..."`
- [ ] T036 [US1] Verify pods are running: `kubectl get pods -n todo-app`
- [ ] T037 [US1] Verify services are created: `kubectl get services -n todo-app`
- [ ] T038 [US1] Wait for pods to be ready (max 2 minutes): `kubectl wait --for=condition=ready pod -l app=todo-backend -n todo-app --timeout=120s`
- [ ] T039 [US1] Wait for frontend pods ready: `kubectl wait --for=condition=ready pod -l app=todo-frontend -n todo-app --timeout=120s`

#### Verify Deployment

- [ ] T040 [US1] Access application via Minikube: `minikube service todo-frontend -n todo-app`
- [ ] T041 [US1] Verify frontend loads in browser at http://localhost:30000
- [ ] T042 [US1] Verify login/authentication works
- [ ] T043 [US1] Verify chatbot responds to messages
- [ ] T044 [US1] Verify task CRUD operations work through chatbot

**Checkpoint**: Full application stack running on Minikube with all Phase III functionality

---

## Phase 5: User Story 4 - AI-Assisted Kubernetes Tools (Priority: P2)

**Goal**: Document and test AI-assisted DevOps operations with kubectl-ai and Gordon

**Independent Test**: Run natural language commands and verify correct kubectl operations

**Dependencies**: US1 (running deployment) required for testing

### Implementation for User Story 4

#### kubectl-ai Setup (Optional)

- [ ] T045 [P] [US4] Install kubectl-ai: `pip install kubectl-ai` (if not installed)
- [ ] T046 [P] [US4] Configure kubectl-ai with API key

#### Test kubectl-ai Operations

- [ ] T047 [US4] Test kubectl-ai list pods: `kubectl-ai "show all pods in todo-app namespace"`
- [ ] T048 [US4] Test kubectl-ai describe: `kubectl-ai "describe the todo-backend deployment"`
- [ ] T049 [US4] Test kubectl-ai scale: `kubectl-ai "scale todo-frontend to 2 replicas"`
- [ ] T050 [US4] Test kubectl-ai logs: `kubectl-ai "check logs from todo-backend"`

#### Gordon Docker AI (Optional)

- [ ] T051 [US4] Test Gordon availability: `docker ai "what can you do?"`
- [ ] T052 [US4] Test Gordon debugging: `docker ai "what's wrong with my container"` (if issues)

**Checkpoint**: AI-assisted operations documented and tested

---

## Phase 6: Deployment Scripts & Documentation

**Purpose**: Create automation scripts and finalize documentation

### Automation Scripts

- [x] T053 [P] Create build-images.sh script at phase-4/scripts/build-images.sh
- [x] T054 [P] Create deploy-minikube.sh script at phase-4/scripts/deploy-minikube.sh
- [x] T055 [P] Create cleanup.sh script at phase-4/scripts/cleanup.sh

### Documentation

- [ ] T056 [P] Update project README.md with Phase IV deployment instructions
- [ ] T057 Validate quickstart.md instructions end-to-end
- [x] T058 Create .gitignore entries for any local secret files

**Checkpoint**: Deployment fully automated and documented

---

## Phase 7: Polish & Final Validation

**Purpose**: Final cleanup and verification

- [ ] T059 Run full cleanup: `helm uninstall todo-app -n todo-app && kubectl delete namespace todo-app`
- [ ] T060 Delete Minikube cluster: `minikube delete`
- [ ] T061 Restart fresh Minikube: `minikube start --driver=docker --memory=4096 --cpus=2`
- [ ] T062 Run full deployment from scratch using phase-4/scripts/deploy-minikube.sh
- [ ] T063 Verify all success criteria from spec.md:
  - [ ] SC-001: All pods in Running state
  - [ ] SC-002: Frontend accessible at http://localhost:30000
  - [ ] SC-003: AI Chatbot responds to messages
  - [ ] SC-004: Task CRUD works through chatbot
  - [ ] SC-005: `helm lint` passes
  - [ ] SC-006: Deployment reproducible
  - [ ] SC-007: Docker images build under 5 minutes

**Checkpoint**: Phase IV complete and validated

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ─────────────────────────────────┐
                                                  │
Phase 2 (US2: Containerize) ──────────┐          │
                                       ├──► Phase 4 (US1: Deploy)
Phase 3 (US3: Helm Charts) ───────────┘          │
                                                  │
                                       ┌──────────┘
                                       ▼
                              Phase 5 (US4: AI Tools)
                                       │
                                       ▼
                              Phase 6 (Scripts & Docs)
                                       │
                                       ▼
                              Phase 7 (Final Validation)
```

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US2 (Containerize) | Setup only | US3 |
| US3 (Helm Charts) | Setup only | US2 |
| US1 (Deploy) | US2 + US3 | - |
| US4 (AI Tools) | US1 | - |

### Parallel Opportunities

**Phase 1**: All verification tasks (T004-T007) can run in parallel

**Phase 2**: Dockerignore files (T008-T009) can be created in parallel

**Phase 3**: All template files (T021-T027) can be created in parallel

**Phase 6**: All scripts (T053-T055) can be created in parallel

---

## Parallel Example: Phase 3 (Helm Charts)

```bash
# Launch all these tasks in parallel (different files, no dependencies):
Task: "Create namespace template at helm/todo-app/templates/namespace.yaml"
Task: "Create configmap template at helm/todo-app/templates/configmap.yaml"
Task: "Create secret template at helm/todo-app/templates/secret.yaml"
Task: "Create backend deployment template at helm/todo-app/templates/backend-deployment.yaml"
Task: "Create backend service template at helm/todo-app/templates/backend-service.yaml"
Task: "Create frontend deployment template at helm/todo-app/templates/frontend-deployment.yaml"
Task: "Create frontend service template at helm/todo-app/templates/frontend-service.yaml"
```

---

## Implementation Strategy

### MVP First (US2 + US3 + US1)

1. Complete Phase 1: Setup (directory structure, tool verification)
2. Complete Phase 2: US2 - Containerize (Docker images)
3. Complete Phase 3: US3 - Helm Charts (K8s manifests)
4. Complete Phase 4: US1 - Deploy to Minikube
5. **STOP and VALIDATE**: Application running on Minikube
6. Demo/Present Phase IV completion

### Full Implementation

1. Complete MVP (US1-US3)
2. Complete Phase 5: US4 - AI Tools (bonus feature)
3. Complete Phase 6: Scripts & Documentation
4. Complete Phase 7: Final Validation
5. Submit for hackathon

---

## Task Summary

| Phase | Tasks | Parallel Tasks |
|-------|-------|----------------|
| Phase 1: Setup | 7 | 4 |
| Phase 2: US2 Containerize | 10 | 2 |
| Phase 3: US3 Helm Charts | 12 | 10 |
| Phase 4: US1 Deploy | 15 | 0 |
| Phase 5: US4 AI Tools | 8 | 2 |
| Phase 6: Scripts & Docs | 6 | 4 |
| Phase 7: Final Validation | 5 | 0 |
| **Total** | **63** | **22** |

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- US2 and US3 are both P1 priority but can be developed in parallel
- US1 (Deploy) is the integration point - requires US2 and US3 complete
- US4 (AI Tools) is P2 priority - optional/bonus for hackathon
- Stop at any checkpoint to validate progress
- Commit after each task or logical group

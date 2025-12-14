class TaskState(str, Enum):
    pending = "pending"
    in_progress = "in_progress"
    completed = "completed"

TRANSITIONS = {
    TaskState.pending: [TaskState.in_progress],
    TaskState.in_progress: [TaskState.completed],
}

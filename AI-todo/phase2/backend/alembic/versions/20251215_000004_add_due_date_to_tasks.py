"""Add due_date field to tasks table.

Revision ID: 007
Revises: 006
Create Date: 2025-12-15

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '007'
down_revision: Union[str, None] = '006'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Add due_date column to tasks table."""
    op.add_column(
        'tasks',
        sa.Column('due_date', sa.Date(), nullable=True)
    )
    # Index for efficient due date queries (overdue tasks, upcoming tasks)
    op.create_index('ix_tasks_due_date', 'tasks', ['due_date'])


def downgrade() -> None:
    """Remove due_date column from tasks table."""
    op.drop_index('ix_tasks_due_date', table_name='tasks')
    op.drop_column('tasks', 'due_date')

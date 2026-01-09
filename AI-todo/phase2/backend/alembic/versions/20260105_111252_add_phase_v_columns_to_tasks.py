"""add_phase_v_columns_to_tasks

Revision ID: 872b24925903
Revises: 007
Create Date: 2026-01-05 11:12:52.566871+00:00

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
import sqlmodel.sql.sqltypes


# revision identifiers, used by Alembic.
revision: str = '872b24925903'
down_revision: Union[str, None] = '007'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Add Phase V advanced feature columns to tasks table."""

    # Priority column already exists (added in missing migration '008')
    # Skip adding it to avoid duplicate column error

    # Add tags column (JSON)
    op.add_column('tasks', sa.Column('tags', sa.JSON(), nullable=False, server_default='[]'))

    # Add recurring task columns
    op.add_column('tasks', sa.Column('is_recurring', sa.Boolean(), nullable=False, server_default='false'))
    op.add_column('tasks', sa.Column('recurring_pattern_id', sa.UUID(), nullable=True))
    op.add_column('tasks', sa.Column('parent_task_id', sa.UUID(), nullable=True))
    op.add_column('tasks', sa.Column('next_occurrence_id', sa.UUID(), nullable=True))

    # Add reminder column
    op.add_column('tasks', sa.Column('is_reminder_sent', sa.Boolean(), nullable=False, server_default='false'))


def downgrade() -> None:
    """Remove Phase V columns from tasks table."""

    op.drop_column('tasks', 'is_reminder_sent')
    op.drop_column('tasks', 'next_occurrence_id')
    op.drop_column('tasks', 'parent_task_id')
    op.drop_column('tasks', 'recurring_pattern_id')
    op.drop_column('tasks', 'is_recurring')
    op.drop_column('tasks', 'tags')
    op.drop_column('tasks', 'priority')

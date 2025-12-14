"""Create tasks table

Revision ID: 001
Revises:
Create Date: 2025-12-14 00:00:00.000000

"""
from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "001"
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Create tasks table
    op.create_table(
        "tasks",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("user_id", sa.Uuid(), nullable=False),
        sa.Column("title", sa.String(length=200), nullable=False),
        sa.Column("description", sa.String(length=1000), nullable=True),
        sa.Column("status", sa.String(length=50), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        sa.PrimaryKeyConstraint("id", name="pk_tasks"),
        sa.CheckConstraint(
            "length(trim(title)) > 0",
            name="ck_tasks_title_not_empty",
        ),
    )

    # Create index on user_id for query performance
    op.create_index("ix_tasks_user_id", "tasks", ["user_id"], unique=False)


def downgrade() -> None:
    # Drop index first
    op.drop_index("ix_tasks_user_id", table_name="tasks")

    # Drop tasks table
    op.drop_table("tasks")

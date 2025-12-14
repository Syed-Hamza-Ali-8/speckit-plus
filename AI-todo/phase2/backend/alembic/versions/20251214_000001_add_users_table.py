"""Add users table and FK constraint

Revision ID: 002
Revises: 001
Create Date: 2025-12-14

"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "002"
down_revision: Union[str, None] = "001"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 1. Create users table
    op.create_table(
        "users",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("email", sa.String(length=255), nullable=False),
        sa.Column("hashed_password", sa.String(length=255), nullable=False),
        sa.Column("is_active", sa.Boolean(), nullable=False, server_default=sa.text("true")),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.PrimaryKeyConstraint("id", name="pk_users"),
    )

    # 2. Create unique index on email for login lookups
    op.create_index("ix_users_email", "users", ["email"], unique=True)

    # 3. Add FK constraint from tasks.user_id to users.id
    # Note: tasks.user_id column already exists from migration 001
    op.create_foreign_key(
        "fk_tasks_user_id",
        "tasks",
        "users",
        ["user_id"],
        ["id"],
        ondelete="CASCADE",
    )


def downgrade() -> None:
    # 1. Drop FK constraint first
    op.drop_constraint("fk_tasks_user_id", "tasks", type_="foreignkey")

    # 2. Drop index
    op.drop_index("ix_users_email", table_name="users")

    # 3. Drop users table
    op.drop_table("users")

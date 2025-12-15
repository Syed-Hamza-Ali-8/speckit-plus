"""Add theme and email_notifications to users table

Revision ID: 005
Revises: 004
Create Date: 2025-12-15

"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "005"
down_revision: Union[str, None] = "004"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Add theme column with default 'system'
    op.add_column(
        "users",
        sa.Column("theme", sa.String(length=20), nullable=False, server_default="system"),
    )
    # Add email_notifications column with default True
    op.add_column(
        "users",
        sa.Column("email_notifications", sa.Boolean(), nullable=False, server_default="true"),
    )


def downgrade() -> None:
    op.drop_column("users", "email_notifications")
    op.drop_column("users", "theme")

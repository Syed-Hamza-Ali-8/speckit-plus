"""Add first_name and last_name to users table

Revision ID: 003
Revises: 002
Create Date: 2025-12-15

"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "003"
down_revision: Union[str, None] = "002"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Add first_name column (nullable for backward compatibility)
    op.add_column(
        "users",
        sa.Column("first_name", sa.String(length=100), nullable=True),
    )
    # Add last_name column (nullable for backward compatibility)
    op.add_column(
        "users",
        sa.Column("last_name", sa.String(length=100), nullable=True),
    )


def downgrade() -> None:
    # Remove columns in reverse order
    op.drop_column("users", "last_name")
    op.drop_column("users", "first_name")

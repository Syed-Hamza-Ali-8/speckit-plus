# Neon DB Guidelines

Neon requires:
- SSL mode required
- Connection string:
  postgresql://USER:PASSWORD@HOST:5432/DB?sslmode=require

Neon uses serverless connections → engine must be created once globally.

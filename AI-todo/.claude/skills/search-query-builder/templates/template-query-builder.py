from sqlmodel import select
from typing import Any, Dict, List

def build_query(model, filters: Dict[str, Any], sort: List[str] = None):
    query = select(model)
    for field, value in filters.items():
        if isinstance(value, dict):
            # Handle operators like range or in
            if "range" in value:
                query = query.where(getattr(model, field).between(value["range"][0], value["range"][1]))
            if "in" in value:
                query = query.where(getattr(model, field).in_(value["in"]))
        else:
            query = query.where(getattr(model, field) == value)
    if sort:
        for s in sort:
            if s.startswith("-"):
                query = query.order_by(getattr(model, s[1:]).desc())
            else:
                query = query.order_by(getattr(model, s).asc())
    return query

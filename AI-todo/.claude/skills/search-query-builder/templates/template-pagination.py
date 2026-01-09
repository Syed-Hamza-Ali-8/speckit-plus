def paginate(query, limit: int = 10, offset: int = 0):
    results = query.limit(limit).offset(offset)
    total = query.count()
    return {
        "items": results.all(),
        "total": total,
        "limit": limit,
        "offset": offset
    }

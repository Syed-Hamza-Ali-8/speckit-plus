try {
    await api.getTasks();
  } catch (e) {
    console.error("Failed to load tasks:", e);
  }
  
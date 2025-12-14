class EventBus:
    def __init__(self):
        self.listeners = {}

    def on(self, event_name: str, handler):
        if event_name not in self.listeners:
            self.listeners[event_name] = []
        self.listeners[event_name].append(handler)

    def emit(self, event_name: str, payload=None):
        handlers = self.listeners.get(event_name, [])
        for handler in handlers:
            try:
                handler(payload)
            except Exception as e:
                print(f"[EventBus] Error handling {event_name}: {e}")

event_bus = EventBus()

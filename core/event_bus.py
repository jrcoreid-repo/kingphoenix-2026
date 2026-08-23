from queue import Queue,Empty
from core.events import Event
class EventBus:
    def __init__(self): self.q=Queue()
    def publish(self,name,payload=None): self.q.put(Event(name,payload))
    def get(self,timeout=None):
        try:return self.q.get(timeout=timeout)
        except Empty:return None

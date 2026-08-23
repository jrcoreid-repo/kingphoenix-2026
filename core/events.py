from dataclasses import dataclass
from typing import Any
@dataclass
class Event:
    name:str; payload:Any=None

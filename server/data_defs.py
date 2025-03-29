from pydantic import BaseModel
from typing import List

class DroneRequest(BaseModel):
    sequence_number: int
    drone_id: str
    positionx: float
    positiony: float
    positionz: float
    timestamp: str

class DroneResponse(BaseModel):
    swarm: list[DroneRequest]

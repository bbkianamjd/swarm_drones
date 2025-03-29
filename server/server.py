from fastapi import FastAPI
from typing import List
import uvicorn
import datetime

# Import our Pydantic data definitions
from data_defs import DroneRequest, DroneResponse
from drone_cache import DroneCache

app = FastAPI()

drone_cache = DroneCache()

@app.post("/drone_position", response_model=DroneResponse)
def drone_position(request: DroneRequest):
    """
    Receives the position of a drone and returns a list of other drones.
    """
    drone_cache.store_drone_state(request)

    swarm_list = drone_cache.get_all_states()
    return DroneResponse(swarm=swarm_list)


@app.get("/server_timestamp")
def server_timestamp():
    """
    Returns the current UTC timestamp of the server.
    """
    utc_now = datetime.datetime.utcnow().isoformat() + "Z"
    return {"timestamp": utc_now}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=2020)

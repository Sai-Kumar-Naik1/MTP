import generated.gui_orchestration_pb2 as gui_pb
import generated.orchestration_engine_pb2 as engine_pb

drone = gui_pb.DroneConfig(
    drone_id="D001",
    drone_name="Drone Alpha"
)

print(drone)

command = engine_pb.MavlinkCommand(
    drone_id="D001",
    command="TAKEOFF"
)

print(command)

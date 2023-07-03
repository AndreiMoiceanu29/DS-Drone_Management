import grpc
from pb_grpc.drone_manager_pb2 import SetGoalRequest
from pb_grpc.drone_manager_pb2 import SetGoalResponse
from pb_grpc.drone_manager_pb2 import LandDroneResponse
from pb_grpc.drone_manager_pb2 import ReturnDroneResponse
from pb_grpc import drone_manager_pb2_grpc
from utils.motion_planning import MotionPlanning, MavlinkConnection
from clients import OrderTrackingClient
from google.protobuf.empty_pb2 import Empty
from mge_logging import mge_log
import threading
class Drone_managerService(drone_manager_pb2_grpc.DroneManagerServicer,):
	def __init__(self):
		super().__init__()
		self.motion_planner = None
		self.conn = MavlinkConnection('tcp:127.0.0.1:5760',threaded=False,PX4=False,timeout=60)
		self.client = OrderTrackingClient('localhost','50051')

	def SetGoal(self, request: SetGoalRequest, context: grpc.RpcContext) -> SetGoalResponse:
		self.x = request.x
		self.y = request.y
		ordern_name = f"Order{self.x}:{self.y}"
		
		order = self.client.create_order(ordern_name, self.x, self.y)
		mge_log.info(f"Order {order.id} created")
		self.motion_planner = MotionPlanning(self.conn, self.x, self.y)
		self.motion_planner.set_order_id(order.id)
		
		thread = threading.Thread(target=self.motion_planner.start)
		thread.start()
		return SetGoalResponse(
			success = True,
			order_id = order.id
		)

	def LandDrone(self, request: Empty, context: grpc.RpcContext) -> LandDroneResponse:
		mge_log.info("Landing drone")
		self.motion_planner.can_land = True
		return LandDroneResponse(
			success = True
		)

	def ReturnDrone(self, request: Empty, context: grpc.RpcContext) -> ReturnDroneResponse:
		raise NotImplementedError


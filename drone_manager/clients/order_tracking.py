import grpc
from pb_grpc.order_tracking_pb2 import Order,Status,Destination
from pb_grpc.order_tracking_pb2_grpc import OrderTrackingServiceStub
from pb_grpc.order_tracking_pb2 import CreateOrderRequest, CreateOrderResponse
from pb_grpc.order_tracking_pb2 import GetOrderRequest, GetOrderResponse
from pb_grpc.order_tracking_pb2 import UpdateOrderRequest, UpdateOrderResponse
from pb_grpc.order_tracking_pb2 import DeleteOrderRequest, DeleteOrderResponse

class OrderTrackingClient:

    def __init__(self, host: str, port: str) -> None:
        self.host = host
        self.port = port

    def update_order(self, order_id: str, status: Status) -> Order:
        with grpc.insecure_channel(f"{self.host}:{self.port}") as channel:
            stub = OrderTrackingServiceStub(channel)
            request = UpdateOrderRequest(id=order_id, status=status)
            response = stub.UpdateOrder(request)
            return response.order
        
    def create_order(self, order_name: str, order_destination_x: float, order_destionation_y: float) -> Order:
        with grpc.insecure_channel(f"{self.host}:{self.port}") as channel:
            stub = OrderTrackingServiceStub(channel)
            order_destination = Destination(x=order_destination_x, y=order_destionation_y)
            request = CreateOrderRequest(name=order_name, destination=order_destination)
            response = stub.CreateOrder(request)
            return response.order
        

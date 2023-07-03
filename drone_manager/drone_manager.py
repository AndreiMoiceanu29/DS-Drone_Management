""" The Server that handles the periods """
import grpc
from concurrent import futures
from signal import signal, SIGTERM
from service import Drone_managerService
from pb_grpc import drone_manager_pb2_grpc
from mge_logging import mge_log
from database import MongoDatabase



def run_service(port: int) -> None:
    """ Starts the service on the designated port """
    mge_log.info(f"Booting order service...")
    
    
    service = Drone_managerService()

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=32))
    drone_manager_pb2_grpc.add_DroneManagerServicer_to_server(service, server)

    mge_log.info(f"Starting drone manager service on port {port}...")
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    mge_log.info(f"Drone Manager service started on port {port}")

    def handle_sigterm(*_):
        """ Handle SIGTERM """

        mge_log.info("Stopping drone manager service...")

        done_event = server.stop(30)
        done_event.wait(30)

        mge_log.info("Drone manager service stopped...")

    server.wait_for_termination()
    signal(SIGTERM, handle_sigterm)

    mge_log.critical("ODrone manager service stopped without any reason...")


if __name__ == "__main__":
    run_service(50052)
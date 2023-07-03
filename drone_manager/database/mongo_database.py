""" MongoDB database class """
import pymongo
from pymongo.errors import ConnectionFailure, InvalidName
from .mongo_interface import IMongoDatabase
from mge_logging import mge_log
from config import settings

class MongoDatabase(IMongoDatabase):
    """ MongoDB database class """
    client: pymongo.MongoClient
    db_name: str

    
    def __init__(self, db_name: str) -> None:
        """ Initialize the database. """
        super().__init__()
        self.db_name = db_name
        self.timeout = 15000 # 15 seconds
        self.connect()

    def connect(self) -> None:
        """ Establish a connection to the database """
        mge_log.info("Connecting to MongoDB database...")
        user = settings.DATABASES['mongodb']['USER']
        passwd = settings.DATABASES['mongodb']['PASSWORD']
        host = settings.DATABASES['mongodb']['HOST']
        port = settings.DATABASES['mongodb']['PORT']
        #TODO: Change the hostname to the public IP of the MongoDB server
        #self.client = pymongo.MongoClient(f'mongodb://{user}:{passwd}@{host}:{port}', serverSelectionTimeoutMS=self.timeout)
        self.client = pymongo.MongoClient("mongodb+srv://andrei_mgedev:2VrQABAJAHCMLQtZ@qubecashflow.266ghn4.mongodb.net/?retryWrites=true&w=majority", serverSelectionTimeoutMS=self.timeout)
        try:
            self.client.admin.command('ismaster')
            mge_log.info("Connected to MongoDB database.")
        except ConnectionFailure as connection_failure:
            mge_log.exception("Failed to connect to MongoDB database.")
            raise connection_failure

    def get_collection(self, collection_name: str) -> pymongo.collection.Collection:
        """ Get a collection from the database. 
            Args:
                collection_name: The name of the collection.
            Returns:
                The collection object.
        """
        mge_log.info("Getting collection {} from database...".format(collection_name))

        try:
            database = self.client[self.db_name]
        except InvalidName as invalid_name:
            mge_log.exception("Invalid database name.")
            raise invalid_name
        except AttributeError as attribute_error:
            mge_log.exception(str(attribute_error))
            raise attribute_error
        
        try:
            db_collection = database[collection_name]
        except InvalidName as invalid_name:
            mge_log.exception("Invalid collection name.")
            raise invalid_name
        
        mge_log.info("Collection {} retrieved from database.".format(collection_name))

        return db_collection
    
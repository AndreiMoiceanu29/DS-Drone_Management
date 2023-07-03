""" MongoDB interface for the cashflow database """
import pymongo

class IMongoDatabase():
    """ Abstract class for MongoDB database. """
    def connect(self) -> None:
        """ Connect to the database. """
        raise NotImplementedError("implement the connect method")
    
    def get_collection(self, collection_name: str) -> pymongo.collection.Collection:
        """ Get a collection from the database. """
        raise NotImplementedError("implement the get_collection method")
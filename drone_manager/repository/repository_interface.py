
from collections.abc import Generator

class IRepository:
    """ Generic Repository for Locations CRUD operations """

    def get(self) -> Generator[object]:
        """ Get all obejcts from database """
        raise NotImplementedError("get not implemented")
    
    def get_all(self, search_query: dict = None) -> Generator[object]:
        raise NotImplementedError("get_all not implemented")

    def save(self, object: object) -> None:
        """ Save object to database """
        raise NotImplementedError("save not implemented")

    def update(self, id: str, object: object) -> object:
        """ Update object in database"""
        raise NotImplementedError("update not implemented")

    def delete(self, id: str) -> None:
        raise NotImplementedError("delete not implemented")
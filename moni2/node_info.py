
class Topic:

    def __init__(self, name: str, types: [str]):
        self.name = name
        self.types = types

    def __str__(self) -> str:
        return f"<Topic: {self.name}, {self.types}>"

    def __repr__(self) -> str:
        return self.__str__()


class Service:

    def __init__(self, name: str, types: [str]):
        self.name = name
        self.types = types

    def __str__(self) -> str:
        return f"<Service: {self.name}, {self.types}>"

    def __repr__(self) -> str:
        return self.__str__()


class NodeInfo:

    def __init__(self, name: str,
                 publishers: [Topic] = None, subscribers: [Topic] = None,
                 services: [Service] = None, clients: [Service] = None):
        self.name = name
        self.publishers = publishers if publishers else []
        self.subscribers = subscribers if subscribers else []
        self.services = services if services else []
        self.clients = clients if clients else []

    def __str__(self) -> str:
        return f"<Node: {self.name}, \n\tpublishers: {self.publishers}\n\tsubscribers: {self.subscribers}" \
               f"services: {self.services}\n\tclients: {self.clients}>"

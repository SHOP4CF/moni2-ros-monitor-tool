class Topic:

    def __init__(self, name: str, types: [str]):
        self.name = name
        self.types = types

    def __str__(self) -> str:
        return f"<Topic: {self.name}, {self.types}>"

    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Topic):
            return False
        return self.name == other.name and self.types == other.types


class Service:

    def __init__(self, name: str, types: [str]):
        self.name = name
        self.types = types

    def __str__(self) -> str:
        return f"<Service: {self.name}, {self.types}>"

    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Service):
            return False
        return self.name == other.name and self.types == other.types


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

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, NodeInfo):
            return False
        return self.name == other.name and \
            self.publishers == other.publishers and \
            self.subscribers == other.subscribers and \
            self.services == other.services and \
            self.clients == other.clients

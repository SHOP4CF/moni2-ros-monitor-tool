from collections import namedtuple
from rclpy.node import Node, HIDDEN_NODE_PREFIX
import rclpy.action

NodeName = namedtuple('NodeName', ('name', 'namespace', 'full_name'))
TopicInfo = namedtuple('Topic', ('name', 'types'))


def parse_node_name(node_name: str) -> 'NodeName':
    full_node_name = node_name
    if not full_node_name.startswith('/'):
        full_node_name = f'/{full_node_name}'
    namespace, node_basename = full_node_name.rsplit('/', 1)
    if namespace == '':
        namespace = '/'
    return NodeName(node_basename, namespace, full_node_name)


class NodeInfo:

    def __init__(self, name: NodeName,
                 publishers: [TopicInfo] = None,
                 subscribers: [TopicInfo] = None,
                 service_server: [TopicInfo] = None,
                 service_client: [TopicInfo] = None,
                 action_server: [TopicInfo] = None,
                 action_client: [TopicInfo] = None):
        self.name = name
        self.publishers = publishers if publishers else []
        self.subscribers = subscribers if subscribers else []
        self.service_server = service_server if service_server else []
        self.service_client = service_client if service_client else []
        self.action_server = action_server if action_server else []
        self.action_client = action_client if action_client else []

    def __str__(self) -> str:
        return f"<Node: {self.name}, \n\tpublishers: {self.publishers}\n\tsubscribers: {self.subscribers}" \
               f"services: {self.services}\n\tclients: {self.clients}>"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, NodeInfo):
            return False
        return self.name == other.name and \
            self.publishers == other.publishers and \
            self.subscribers == other.subscribers and \
            self.service_server == other.service_server and \
            self.service_client == other.service_client and \
            self.action_server == other.action_server and \
            self.action_client == other.action_client


class NodeInfoHandler:

    def __init__(self, node: Node):
        self.node = node
        self.log = node.get_logger().get_child('NodeInfoHandler')
        self.log.info("Initialized!")

    @staticmethod
    def _is_hidden_name(name: str) -> bool:
        return any(part.startswith(HIDDEN_NODE_PREFIX) for part in name.split('/'))

    def get_node_info(self, node_name: NodeName) -> NodeInfo:
        return NodeInfo(
                name=node_name,
                publishers=self.get_publisher_info(node_name),
                subscribers=self.get_subscriber_info(node_name),
                service_server=self.get_service_server_info(node_name),
                service_client=self.get_service_client_info(node_name),
                action_server=self.get_action_server_info(node_name),
                action_client=self.get_action_client_info(node_name))

    def get_node_names(self, include_hidden_nodes=False) -> [NodeName]:
        node_names_and_namespaces = self.node.get_node_names_and_namespaces()
        return [
            NodeName(
                name=node[0],
                namespace=node[1],
                full_name=node[1] + ('' if node[1].endswith('/') else '/') + node[0])
            for node in node_names_and_namespaces
            if include_hidden_nodes or (node[0] and not node[0].startswith(HIDDEN_NODE_PREFIX))
        ]

    def get_topics(self, node: NodeName, func: callable, include_hidden_topics=False) -> [TopicInfo]:
        names_and_types = func(node.name, node.namespace)
        return [
            TopicInfo(name=topic[0], types=topic[1])
            for topic in names_and_types if include_hidden_topics or not self._is_hidden_name(topic[0])
        ]

    def get_publisher_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        return self.get_topics(node_name, self.node.get_publisher_names_and_types_by_node, include_hidden)

    def get_subscriber_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        return self.get_topics(node_name, self.node.get_subscriber_names_and_types_by_node, include_hidden)

    def get_service_server_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        return self.get_topics(node_name, self.node.get_service_names_and_types_by_node, include_hidden)

    def get_service_client_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        return self.get_topics(node_name, self.node.get_client_names_and_types_by_node, include_hidden)

    def get_action_server_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        names_and_types = rclpy.action.get_action_server_names_and_types_by_node(self.node, node_name.name, node_name.namespace)
        return [TopicInfo(name=name, types=types) for name, types in names_and_types if include_hidden or not self._is_hidden_name(name)]

    def get_action_client_info(self, node_name: NodeName, include_hidden=False) -> [TopicInfo]:
        names_and_types = rclpy.action.get_action_client_names_and_types_by_node(self.node, node_name.name, node_name.namespace)
        return [TopicInfo(name=name, types=types) for name, types in names_and_types if include_hidden or not self._is_hidden_name(name)]

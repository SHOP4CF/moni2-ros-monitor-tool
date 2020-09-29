from typing import Any
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import ListParameters, GetParameters

from moni2.node_info import NodeName, ParamInfo


class ParamUpdateNode(Node):
    
    def __init__(self, online_watched_nodes: [NodeName], node_params: dict):
        super(ParamUpdateNode, self).__init__("_moni2_param_update")
        self.online_watched_nodes = online_watched_nodes
        self.node_params = node_params

        self.list_params_clients = {}
        self.get_params_clients = {}

    def update_param_list(self):
        self.get_logger().info(f"Updating node params...")
        for online_node in self.online_watched_nodes:
            names = self._update_param_list_for_node(online_node)
            param_list = self._update_param_values_for_node(online_node, names)
            self.node_params[online_node] = param_list

    def _update_param_list_for_node(self, node: NodeName) -> [str]:
        service_name = f'{node.full_name}/list_parameters'
        if node in self.list_params_clients:
            client = self.list_params_clients[node]
        else:
            client = self.create_client(ListParameters, service_name)
            self.list_params_clients[node] = client
        self.get_logger().debug(f"Waiting for: {service_name}")
        ready = client.wait_for_service(timeout_sec=1.0)
        if not ready:
            self.get_logger().info(f"{service_name} wasn't ready!")
            return []
        future = client.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            response: ListParameters.Response = future.result()
            names = response.result.names
            return names
        return []

    def _update_param_values_for_node(self, node: NodeName, param_names: [str]) -> [ParamInfo]:
        service_name = f'{node.full_name}/get_parameters'
        if node in self.get_params_clients:
            client = self.get_params_clients[node]
        else:
            client = self.create_client(GetParameters, service_name)
            self.get_params_clients[node] = client
        self.get_logger().debug(f"Waiting for: {service_name}")
        ready = client.wait_for_service(timeout_sec=1.0)
        if not ready:
            self.get_logger().info(f"{service_name} wasn't ready!")
            return []
        request = GetParameters.Request()
        request.names = param_names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GetParameters.Response = future.result()

        param_list = []
        for i in range(len(response.values)):
            self.get_logger().debug(f"Value: {param_names[i]}, value= {self._get_value(response.values[i])}")
            param_type, param_value = self._get_value(response.values[i])
            param_list.append(ParamInfo(param_names[i], param_type, param_value))
        return param_list

    @staticmethod
    def _get_value(value: ParameterValue) -> (str, Any):
        if value.type == ParameterType.PARAMETER_NOT_SET:
            return "NOT_SET", None
        if value.type == ParameterType.PARAMETER_BOOL:
            return "bool", value.bool_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return "int", value.integer_value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return "double", value.double_value
        if value.type == ParameterType.PARAMETER_STRING:
            return "string", value.string_value
        if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return "[byte]", value.byte_array_value
        if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return "[bool]", value.bool_array_value
        if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return "[int]", value.integer_array_value
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return "[double]", value.double_array_value
        if value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return "[string]", value.string_array_value

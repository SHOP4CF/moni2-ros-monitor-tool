from moni2.node_info import NodeName, parse_node_name


class TestParseNodeName:

    def test_simple_name(self):
        got = parse_node_name('/test_node')
        want = NodeName("test_node", "/", "/test_node")
        assert got == want

    def test_full_name_start_with_backslash(self):
        got = parse_node_name('test_node')
        want = NodeName("test_node", "/", "/test_node")
        assert got == want

    def test_long_namespace(self):
        got = parse_node_name('robot/1/test_node')
        want = NodeName("test_node", "/robot/1", "/robot/1/test_node")
        assert got == want

import logging
from unittest.mock import MagicMock
from PyQt5.QtWidgets import QMenu
from moni2.gui.config_handler import ConfigHandler, Config
from moni2.node_info import parse_node_name


def settings_mock():
    def value(key: str, default: str):
        print(f"key: {key}, default: {default}")
        return ""

    mock = MagicMock()
    mock.value = value
    return mock


class TestConfigHandler:

    config_handler: ConfigHandler
    organization = "test_org"
    app_name = "test_app_name"
    version = "0.1.0"

    def setup(self):
        print("Setup")

        self.config_handler = ConfigHandler(
            logging.getLogger('TEST_ConfigHandler'),
            self.organization,
            self.app_name,
            self.version)

        message_mock = MagicMock()
        message_mock.emit = self.message
        self.config_handler.message = message_mock

        self.config = {
            'organization': self.organization,
            'app_name': self.app_name,
            'version': self.version,
            'node_names': [{"name": "moni2", "namespace": "/", "full_name": "/moni2"}]
        }
        self.messages = []

    def message(self, msg: str, _: str):
        self.messages.append(msg)

    def teardown(self):
        print("Teardown")
        assert self.config_handler

    def test_something(self):
        self.config_handler._get_settings = MagicMock(return_value=settings_mock())
        self.config_handler.open_recent()
        assert True

    def test_validate_config_file(self):
        assert self.config_handler._validate_config_file(self.config)

    def test_invalid_config_organization(self):
        self.config['organization'] = self.organization + 'NOT_THIS_ONE'
        assert not self.config_handler._validate_config_file(self.config)

    def test_invalid_config_app(self):
        self.config['app_name'] = self.app_name + 'NOT_THIS_ONE'
        assert not self.config_handler._validate_config_file(self.config)

    def test_invalid_config_version(self):
        self.config['version'] = "1.0.0"
        assert not self.config_handler._validate_config_file(self.config)

    def test_invalid_config_nodes(self):
        self.config['node_names'] = "NOT A LIST"
        assert not self.config_handler._validate_config_file(self.config)

        del self.config['node_names']
        assert not self.config_handler._validate_config_file(self.config)

    def test_load_config(self):
        config: Config = self.config_handler._load_config(self.config)
        assert config, self.messages
        assert config.organization == self.organization
        assert config.app_name == self.app_name
        assert config.version == self.version
        assert config.node_names == [parse_node_name('moni2')]

    def test_load_invalid_config(self):
        del self.config['version']
        config: Config = self.config_handler._load_config(self.config)
        assert config is None
        assert len(self.messages) == 1

    def test_menu(self):
        menu = QMenu()
        self.config_handler.create_menu(menu)
        expected_actions = ["New", "Open", "Open Recent", "Save as", "Edit"]
        actions = []
        for action in menu.actions():
            actions.append(action.text())
        assert expected_actions == actions, f"Expected actions: {expected_actions}. Actions: {actions}"

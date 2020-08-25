import logging
import json
from collections import deque
from packaging import version
from typing import Optional
from PyQt5.QtWidgets import (
    QMenu,
    QAction,
    QFileDialog
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QObject, pyqtSignal, QSettings
from moni2.gui.node_dialog import EditNodeListDialog


class ConfigHandler(QObject):

    message = pyqtSignal(str, str)
    node_list_updated = pyqtSignal(list)

    KEY_RECENT_CONFIGS = 'recent_configs'

    def __init__(self,
                 log: logging.Logger,
                 organization: str,
                 app_name: str,
                 version: str,
                 parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.log = log
        self.organization = organization
        self.app_name = app_name
        self.version = version

        self.path = ''
        self.config = {}
        self.recent_menu: QMenu = None

    def open_recent(self):
         pass

    def _new_config(self):
        self.log.info("New configuration")
        self._open_edit_dialog([])
        # TODO: save the new configuration internally

    def _open_config(self, filepath=''):
        if not filepath:  # Ask the user to provide a filepath
            filepath = self._select_config_file()
        if not filepath:  # No filepath given, return
            return
        config = self._load_config_file(filepath)
        valid = self._validate_config_file(config)
        if not valid:
            return
        self.path = filepath
        self.config = config
        self._load_config(config)
        self._update_recent_list(filepath)
        self.message.emit("Successfully loaded new configuration", 'info')

    def _save_as_config(self):
        self.message.emit("Save", 'info')

    def _edit_config(self):
        self.log.info("Editing configuration")
        nodes = [node['name'] for node in self.config['nodes']]
        self._open_edit_dialog(nodes)

    def _open_edit_dialog(self, nodes: [str]):
        dialog = EditNodeListDialog(self.log, self.parent())
        dialog.set_nodes(nodes)
        self.parent().node_updated.connect(dialog.node_updated)  # TODO: this is nasty
        dialog.node_list_updated.connect(lambda new_nodes: self.node_list_updated.emit(new_nodes))
        dialog.exec()

    def _select_config_file(self) -> str:
        options = QFileDialog.Options()
        file_filter = "Config file (*.json);;All files (*)"
        filepath, _ = QFileDialog.getOpenFileName(self.parent(), "Find configuration file", "", file_filter, options=options)
        self.log.info(f"Configuration file: {filepath}")
        return filepath

    def _load_config_file(self, filepath: str) -> dict:
        config = {}
        try:
            with open(filepath) as json_file:
                config = json.load(json_file)
                self.log.info(f"Loaded file: {config}")
        except Exception as e:
            self.message.emit(f"Error: {e}", 'error')
        finally:
            return config

    def _validate_config_file(self, config: dict) -> bool:
        error = ""
        try:
            if 'organization' in config and config['organization'] != self.organization:
                error = "Wrong organization"
            if 'application' in config and config['application'] != self.app_name:
                error = "Wrong application"
            if 'version' in config and version.parse(config['version']).minor < version.parse(self.version).minor:
                error = "Version is not supported"
            if 'nodes' not in config:
                error = "Not containing a list of nodes"
        except Exception as e:
            error = f"Some error validating config file: {e}"
        finally:
            if error:
                self.message.emit(f"Invalid config file. {error}", "error")
            return error == ''

    def _load_config(self, config: dict):
        nodes = [node['name'] for node in config['nodes']]
        self.node_list_updated.emit(nodes)

    def _update_recent_list(self, filename: str):
        settings = QSettings(self.organization, self.app_name)
        recent_paths = settings.value(self.KEY_RECENT_CONFIGS, [])
        recent_paths = deque(recent_paths, maxlen=5)
        if filename in recent_paths:
            recent_paths.remove(filename)
        recent_paths.appendleft(filename)
        settings.setValue(self.KEY_RECENT_CONFIGS, list(recent_paths))
        self._update_recent_config_menu()

    def _update_recent_config_menu(self):
        settings = QSettings(self.organization, self.app_name)
        recent_config_paths = settings.value(self.KEY_RECENT_CONFIGS, [])
        self.recent_menu.clear()
        for recent_path in recent_config_paths:
            action = QAction(recent_path, self.recent_menu)
            action.triggered.connect(lambda checked, path=recent_path: self._open_config(path))
            if self.path == recent_path:
                action.setIcon(QIcon.fromTheme('emblem-default'))
            self.recent_menu.addAction(action)
        self.recent_menu.menuAction().setVisible(not self.recent_menu.isEmpty())

    def create_menu(self, menu: QMenu):
        new_action = QAction(QIcon.fromTheme('document-new'), "New", menu)
        new_action.setStatusTip("Create new configuration")
        new_action.triggered.connect(self._new_config)
        menu.addAction(new_action)

        open_action = QAction(QIcon.fromTheme('document-open'), "Open", menu)
        open_action.setStatusTip("Open existing configuration")
        open_action.triggered.connect(self._open_config)
        menu.addAction(open_action)

        self.recent_menu = QMenu("Open Recent", menu)
        self.recent_menu.setIcon(QIcon.fromTheme('document-open-recent'))
        menu.addMenu(self.recent_menu)
        self._update_recent_config_menu()

        save_as_action = QAction(QIcon.fromTheme('document-save-as'), "Save as", menu)
        save_as_action.setStatusTip("Save configuration")
        save_as_action.triggered.connect(self._save_as_config)
        menu.addAction(save_as_action)

        edit_action = QAction(QIcon.fromTheme('document-properties'), "Edit", menu)
        edit_action.setStatusTip("Edit configuration")
        edit_action.triggered.connect(self._edit_config)
        menu.addAction(edit_action)

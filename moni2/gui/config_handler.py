import logging
import json
from collections import deque
from packaging import version
from typing import Optional
from PyQt5.QtWidgets import (
    QMenu,
    QAction,
    QFileDialog,
)
from PyQt5.QtGui import QIcon, QKeySequence
from PyQt5.QtCore import QObject, pyqtSignal, QSettings
from moni2.gui.node_dialog import EditNodeListDialog
from moni2.node_info import NodeName


class Config:
    def __init__(self, organization: str, app_name: str, app_version: str, node_names: [NodeName]):
        self.organization = organization
        self.app_name = app_name
        self.version = app_version
        self.node_names = node_names

    def __repr__(self) -> str:
        return f'Config<org={self.organization}, name={self.app_name}, version={self.version}, nodes={self.node_names}>'

    @staticmethod
    def as_dict(conf: 'Config') -> dict:
        res = conf.__dict__
        res['node_names'] = [node._asdict() for node in conf.node_names]
        return res

    @staticmethod
    def as_payload(conf: dict) -> 'Config':
        return Config(
            conf['organization'],
            conf['app_name'],
            conf['version'],
            [NodeName(**node) for node in conf['node_names']])


class ConfigHandler(QObject):

    message = pyqtSignal(str, str)
    node_list_updated = pyqtSignal(list)

    KEY_RECENT_CONFIGS = 'recent_configs'
    KEY_LAST_CONFIG = 'last_config'

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
        self.old_path = ''
        self.config: Config = None
        self.old_config: Config = None
        self.recent_menu: QMenu = None
        self.edit_action: QAction = None

    def open_recent(self):
        settings = QSettings(self.organization, self.app_name)
        filepath = settings.value(self.KEY_LAST_CONFIG, '')
        if filepath:
            self._open_config(filepath)

    def _new_config(self):
        self.log.info("New configuration")
        self.old_path = self.path
        self.old_config = self.config
        self.path = ""
        self.config = Config(self.organization, self.app_name, self.version, [])
        self._open_edit_dialog([])

    def _open_config(self, filepath=''):
        if not filepath:  # Ask the user to provide a filepath
            filepath = self._select_config_file()
        if not filepath:  # No filepath given, return
            return
        config = self._load_config_file(filepath)
        valid = self._validate_config_file(config)
        if not valid:
            return
        config = self._load_config(config)
        if not config:
            return
        self.path = filepath
        self.config = config
        self._update_recent_list(filepath)
        self.edit_action.setEnabled(True)
        self.message.emit("Successfully loaded new configuration", 'info')

    def _save_as_config(self):
        path = self._select_save_filepath()
        if not path:
            return
        self._save_config(path)
        self._open_config(self.path)

    def _save_config(self, path: str):
        with open(path, 'w') as json_file:
            json.dump(Config.as_dict(self.config), json_file, indent=4)
            self.log.info(f"Saved config file to: {path}")
            self.path = path

    def _edit_config(self):
        self.log.info("Editing configuration")
        nodes = [node for node in self.config.node_names]
        self._open_edit_dialog(nodes)

    def _open_edit_dialog(self, nodes: [NodeName]):
        dialog = EditNodeListDialog(self.log)
        dialog.set_nodes(nodes)
        self.parent().online_nodes.connect(dialog.node_updated)  # TODO: this is nasty
        dialog.exec()

        if dialog.result():
            self.config.node_names = dialog.selected_nodes()
            if not self.path:
                self.path = self._select_save_filepath()
            if self.path:
                self._save_config(self.path)
                self._open_config(self.path)
                return
        self.path = self.old_path
        self.config = self.old_config

    def _select_save_filepath(self) -> str:
        file_filter = "Config file (*.json);;All files (*)"
        filepath, _ = QFileDialog.getSaveFileName(self.parent(), "Save configuration file", "", file_filter)
        self.log.info(f"Choosen path: {filepath}")
        return filepath

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
            if 'node_names' not in config and type(config['node_names']) == list:
                error = "Not containing a list of node_names"
        except Exception as e:
            error = f"Some error validating config file: {e}"
        finally:
            if error:
                self.message.emit(f"Invalid config file. {error}", "error")
            return error == ''

    def _load_config(self, config: dict) -> Config:
        try:
            config_obj = Config.as_payload(config)
            self.node_list_updated.emit(config_obj.node_names)
            return config_obj
        except Exception as e:
            self.message.emit(f"Invalid config file. Some error while reading nodes: {e}", "error")
            return None

    def _update_recent_list(self, filepath: str):
        settings = QSettings(self.organization, self.app_name)
        settings.setValue(self.KEY_LAST_CONFIG, filepath)
        recent_paths = settings.value(self.KEY_RECENT_CONFIGS, [])
        recent_paths = deque(recent_paths, maxlen=5)
        if filepath in recent_paths:
            recent_paths.remove(filepath)
        recent_paths.appendleft(filepath)
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
        new_action.setShortcut(QKeySequence.New)
        new_action.setStatusTip("Create new configuration")
        new_action.triggered.connect(self._new_config)
        menu.addAction(new_action)

        open_action = QAction(QIcon.fromTheme('document-open'), "Open", menu)
        open_action.setShortcut(QKeySequence.Open)
        open_action.setStatusTip("Open existing configuration")
        open_action.triggered.connect(self._open_config)
        menu.addAction(open_action)

        self.recent_menu = QMenu("Open Recent", menu)
        self.recent_menu.setIcon(QIcon.fromTheme('document-open-recent'))
        menu.addMenu(self.recent_menu)
        self._update_recent_config_menu()

        save_as_action = QAction(QIcon.fromTheme('document-save-as'), "Save as", menu)
        save_as_action.setShortcuts([QKeySequence.Save, QKeySequence.SaveAs])
        save_as_action.setStatusTip("Save configuration")
        save_as_action.triggered.connect(self._save_as_config)
        menu.addAction(save_as_action)

        self.edit_action = QAction(QIcon.fromTheme('document-properties'), "Edit", menu)
        self.edit_action.setShortcut("Ctrl+E")
        self.edit_action.setStatusTip("Edit configuration")
        self.edit_action.triggered.connect(self._edit_config)
        self.edit_action.setEnabled(False)
        menu.addAction(self.edit_action)

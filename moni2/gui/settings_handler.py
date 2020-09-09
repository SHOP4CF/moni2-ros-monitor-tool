import logging
from typing import Optional
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSignal, QSettings


class SettingsReader:

    def columns(self) -> int:
        raise NotImplementedError("Implement in subclass")

    def hide_parameter_services(self) -> bool:
        raise NotImplementedError("Implement in subclass")

    def hide_moni2_logs(self) -> bool:
        raise NotImplementedError("Implement in subclass")

    def hide_unmonitored_nodes(self) -> bool:
        raise NotImplementedError("Implement in subclass")

    def default_log_level(self) -> str:
        raise NotImplementedError("Implement in subclass")


class SettingsWriter:

    def set_columns(self, columns: int):
        raise NotImplementedError("Implement in subclass")

    def set_hide_parameter_services(self, hide: bool):
        raise NotImplementedError("Implement in subclass")

    def set_hide_moni2_logs(self, hide: bool):
        raise NotImplementedError("Implement in subclass")

    def set_hide_unmonitored_nodes(self, hide: bool):
        raise NotImplementedError("Implement in subclass")

    def set_default_log_level(self, level: str):
        raise NotImplementedError("Implement in subclass")


class SettingsHandler(QObject, SettingsReader, SettingsWriter):

    settings_changed = pyqtSignal()

    KEY_COLUMNS = 'columns'
    KEY_HIDE_PARAMETER_TOPIC = 'hide_parameter_services'
    KEY_HIDE_MONI2_LOGS = 'hide_moni2_logs'
    KEY_HIDE_UNMONITORED_NODES = 'hide_unmonitored_nodes'
    KEY_DEFAULT_LOG_LEVEL = 'default_log_level'

    def __init__(self,
                 log: logging.Logger,
                 organization: str,
                 app_name: str,
                 parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.log = log
        self.settings = QSettings(organization, app_name)

    def columns(self) -> int:
        return self.settings.value(self.KEY_COLUMNS, 2, int)

    def set_columns(self, columns: int):
        self.settings.setValue(self.KEY_COLUMNS, columns)
        self.settings_changed.emit()

    def hide_parameter_services(self) -> bool:
        return self.settings.value(self.KEY_HIDE_PARAMETER_TOPIC, False, bool)

    def set_hide_parameter_services(self, hide: bool):
        self.settings.setValue(self.KEY_HIDE_PARAMETER_TOPIC, hide)
        self.settings_changed.emit()

    def hide_moni2_logs(self) -> bool:
        return self.settings.value(self.KEY_HIDE_MONI2_LOGS, False, bool)

    def set_hide_moni2_logs(self, hide: bool):
        self.settings.setValue(self.KEY_HIDE_MONI2_LOGS, hide)
        self.settings_changed.emit()

    def hide_unmonitored_nodes(self) -> bool:
        return self.settings.value(self.KEY_HIDE_UNMONITORED_NODES, True, bool)

    def set_hide_unmonitored_nodes(self, hide: bool):
        self.settings.setValue(self.KEY_HIDE_UNMONITORED_NODES, hide)
        self.settings_changed.emit()

    def default_log_level(self) -> str:
        return self.settings.value(self.KEY_DEFAULT_LOG_LEVEL, "INFO", str)

    def set_default_log_level(self, level: str):
        self.settings.setValue(self.KEY_DEFAULT_LOG_LEVEL, level)
        self.settings_changed.emit()

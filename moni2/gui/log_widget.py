import logging
from rcl_interfaces.msg import Log
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDockWidget,
    QListWidget,
    QMenu,
    QAction,
    QActionGroup
)
from PyQt5.QtGui import QContextMenuEvent
from PyQt5.QtCore import pyqtSlot, Qt


class LogWidget(QDockWidget):
    LOG_LABEL = {logging.DEBUG: "DEBUG",
                 logging.INFO: "INFO ",
                 logging.WARNING: "WARN ",
                 logging.ERROR: "ERROR",
                 logging.FATAL: "FATAL"}

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__("Log widget", parent)
        self.log = log
        self.log_level = logging.INFO

        self.log_list = QListWidget(self)
        self.init_components()
        self.init_ui()
        self.log.info("LogWidget initialized!")

    def init_components(self):
        self.log.info("Initializing components...")
        self.log_list.setUniformItemSizes(True)
        self.log_list.setAlternatingRowColors(True)
        self.log_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWidget(self.log_list)

    @pyqtSlot(str)
    def received_log(self, log: Log):
        self.log_list.insertItem(0, f'[{self.LOG_LABEL[log.level]}] [{log.name}]: {log.msg}')

    def contextMenuEvent(self, event: QContextMenuEvent) -> None:
        menu = QMenu(self)
        group = QActionGroup(self)
        group.setExclusive(True)
        for key, val in self.LOG_LABEL.items():
            action = QAction(self)
            action.setCheckable(True)
            action.setText(val)
            action.setData(key)
            action.setActionGroup(group)
            if key == self.log_level:
                action.setChecked(True)
            menu.addAction(action)
        action = menu.exec(event.globalPos())
        if action:
            self.log_level = action.data()

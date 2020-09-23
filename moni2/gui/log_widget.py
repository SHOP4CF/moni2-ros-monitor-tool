import logging
import fnmatch
from collections import defaultdict
from rcl_interfaces.msg import Log
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDockWidget,
    QListWidget,
    QListWidgetItem,
    QMenu,
    QAction,
    QActionGroup,
    QVBoxLayout,
    QLineEdit,
)
from PyQt5.QtGui import QContextMenuEvent, QIcon, QColor
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt
from moni2.gui.settings_handler import SettingsReader
from moni2.node_info import NodeName


class LogWidget(QDockWidget):
    LOG_LABEL = {logging.DEBUG: "DEBUG",
                 logging.INFO: "INFO ",
                 logging.WARNING: "WARN ",
                 logging.ERROR: "ERROR",
                 logging.FATAL: "FATAL"}

    LOG_LEVEL = {"DEBUG": logging.DEBUG,
                 "INFO": logging.INFO,
                 "WARNING": logging.WARNING,
                 "ERROR": logging.ERROR,
                 "FATAL": logging.FATAL}

    LOG_COLOR = {
        logging.DEBUG: QColor("white"),
        logging.INFO: QColor("white"),
        logging.WARNING: QColor("yellow"),
        logging.ERROR: QColor("red"),
        logging.FATAL: QColor("red")
    }

    warning_counter = pyqtSignal(str, int)
    error_counter = pyqtSignal(str, int)

    def __init__(self, log: logging.Logger, settings: SettingsReader, parent: Optional[QWidget] = None):
        super().__init__("Log", parent)
        self.log = log
        self.settings = settings

        self.log_level = self.LOG_LEVEL[self.settings.default_log_level()]
        self.log_list = []
        self.watched_nodes: [str] = []

        self.warning_count = defaultdict(int)
        self.error_count = defaultdict(int)

        self.log_list_view = QListWidget()
        self.filter_input = QLineEdit()
        self.filter_text = '*'
        self.init_components()
        self.init_ui()
        self.log.info("LogWidget initialized!")

    def init_components(self):
        self.log.info("Initializing components...")
        self.log_list_view.setUniformItemSizes(True)
        self.log_list_view.setAlternatingRowColors(True)
        self.log_list_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        self.filter_input.textChanged.connect(self.on_filter_text_changed)
        self.filter_input.setHidden(True)
        self.filter_input.setClearButtonEnabled(True)
        action = self.filter_input.addAction(QIcon.fromTheme('window-close'), QLineEdit.TrailingPosition)
        action.triggered.connect(lambda _: self.filter_input.setHidden(True))

    def init_ui(self):
        self.log.info("Initializing UI...")
        main_widget = QWidget(self)
        main_layout = QVBoxLayout(main_widget)

        main_layout.addWidget(self.filter_input)
        main_layout.addWidget(self.log_list_view)

        self.setWidget(main_widget)

    def _insert_log(self, log: Log):
        parent_log = log.name.split('.')[0]
        if log.level < self.log_level or \
           (self.settings.hide_moni2_logs() and parent_log == "moni2") or \
           (self.settings.hide_unmonitored_nodes() and parent_log not in self.watched_nodes) or \
           not fnmatch.fnmatch(log.name, self.filter_text):
            return
        text = f'[{self.LOG_LABEL[log.level]}] [{log.name}]: {log.msg}'
        item = QListWidgetItem(text)
        item.setBackground(self.LOG_COLOR[log.level])
        self.log_list_view.insertItem(0, item)

    @pyqtSlot(str)
    def received_log(self, log: Log):
        self.log_list.append(log)
        if self.log_level <= log.level:
            self._insert_log(log)
        if log.level == logging.WARNING:
            parent_log = log.name.split('.')[0]
            self.warning_count[parent_log] += 1
            self.warning_counter.emit(parent_log, self.warning_count[parent_log])
        if log.level >= logging.ERROR:
            parent_log = log.name.split('.')[0]
            self.error_count[parent_log] += 1
            self.error_counter.emit(parent_log, self.error_count[parent_log])

    @pyqtSlot(str)
    def on_filter_text_changed(self, text):
        text = f'*{text}*' if text else "*"
        self.filter_text = text
        self._update_log_list()

    def contextMenuEvent(self, event: QContextMenuEvent) -> None:
        # Build menu
        menu = QMenu(self)
        search_action = QAction(QIcon.fromTheme('edit-find'), "Search", menu)
        menu.addAction(search_action)
        clear_action = QAction(QIcon.fromTheme('edit-clear'), "Clear list", menu)
        menu.addAction(clear_action)
        menu.addSeparator()

        group = QActionGroup(self)
        group.setExclusive(True)
        for key, val in self.LOG_LABEL.items():
            action = QAction(menu)
            action.setCheckable(True)
            action.setText(val)
            action.setData(key)
            action.setActionGroup(group)
            if key == self.log_level:
                action.setChecked(True)
            menu.addAction(action)

        # Handle result
        action = menu.exec(event.globalPos())
        if action:
            if action == clear_action:
                self.log.info("Clear list")
                self.log_list.clear()
                self.log_list_view.clear()
                for log in self.warning_count:
                    self.warning_counter.emit(log, 0)
                self.warning_count.clear()
                for log in self.error_count:
                    self.error_counter.emit(log, 0)
                self.error_count.clear()
            elif action == search_action:
                self.log.info("Search")
                self.filter_input.setHidden(False)
            else:
                if self.log_level != action.data():
                    self.log_level = action.data()
                    self._update_log_list()

    def _update_log_list(self):
        self.log_list_view.clear()
        for log in self.log_list:
            self._insert_log(log)

    @pyqtSlot(list)
    def node_list_updated(self, nodes: [NodeName]):
        self.log.debug(f"New list of nodes: {[node.name for node in nodes]}")
        self.watched_nodes = [node.name for node in nodes]
        if self.settings.hide_unmonitored_nodes():
            self._update_log_list()

    @pyqtSlot()
    def settings_changed(self):
        self.log.info("Settings changed")
        self._update_log_list()

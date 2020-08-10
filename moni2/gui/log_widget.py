import logging
import fnmatch
from rcl_interfaces.msg import Log
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDockWidget,
    QListWidget,
    QMenu,
    QAction,
    QActionGroup,
    QVBoxLayout,
    QLineEdit,
)
from PyQt5.QtGui import QContextMenuEvent, QIcon
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
        self.log_list = []
        self.log_list_view = QListWidget()
        self.filter_input = QLineEdit()
        self.filter_text = ''
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

    @pyqtSlot(str)
    def received_log(self, log: Log):
        self.log_list.append(log)
        if self.log_level <= log.level:
            self.log_list_view.insertItem(0, f'[{self.LOG_LABEL[log.level]}] [{log.name}]: {log.msg}')

    @pyqtSlot(str)
    def on_filter_text_changed(self, text):
        text = f'*{text}*' if text else "*"
        for row in range(self.log_list_view.count()):
            item = self.log_list_view.item(row)
            item.setHidden(not fnmatch.fnmatch(item.text(), text))
        self.filter_text = text

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
            elif action == search_action:
                self.log.info("Search")
                self.filter_input.setHidden(False)
            else:
                if self.log_level != action.data():
                    self.log_level = action.data()
                    self.log_list_view.clear()
                    for log in self.log_list:
                        if self.log_level <= log.level:
                            self.log_list_view.insertItem(0, f'[{self.LOG_LABEL[log.level]}] [{log.name}]: {log.msg}')

import logging
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDialog,
    QCheckBox,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QDialogButtonBox,
    QComboBox,
)
from PyQt5.QtGui import QIntValidator

from moni2.gui.settings_handler import SettingsHandler


class SettingsDialog(QDialog):

    def __init__(self,
                 log: logging.Logger,
                 settings: SettingsHandler,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log
        self.settings = settings

        # Layout
        self.columns = QLineEdit("")

        # Services
        self.hide_parameter_services = QCheckBox("Hide parameter services")

        # Log
        self.hide_moni2_logs = QCheckBox("Hide moni2 logs")
        self.hide_unmonitored_nodes = QCheckBox("Hide unmonitored nodes")
        self.default_log_level = QComboBox()

        self.init_components()
        self.init_ui()
        self.log.info("Initialized")

    def init_components(self):
        self.log.info("Initializing components...")

        self.columns.setPlaceholderText("show in view")
        self.columns.setValidator(QIntValidator(1, 9))
        self.columns.setText(f"{self.settings.columns()}")
        self.columns.textChanged[str].connect(self._columns_changed)

        self.hide_parameter_services.setChecked(self.settings.hide_parameter_services())
        self.hide_parameter_services.stateChanged.connect(
            lambda hide: self.settings.set_hide_parameter_services(hide))

        self.hide_moni2_logs.setChecked(self.settings.hide_moni2_logs())
        self.hide_moni2_logs.stateChanged.connect(lambda hide: self.settings.set_hide_moni2_logs(hide))

        self.hide_unmonitored_nodes.setChecked(self.settings.hide_unmonitored_nodes())
        self.hide_unmonitored_nodes.stateChanged.connect(lambda hide: self.settings.set_hide_unmonitored_nodes(hide))

        self.default_log_level.addItems(["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"])
        self.default_log_level.setCurrentText(self.settings.default_log_level())
        self.default_log_level.currentTextChanged.connect(lambda level: self.settings.set_default_log_level(level))

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWindowTitle("Settings")

        ui_layout = QVBoxLayout()
        ui_columns_layout = QHBoxLayout()
        ui_columns_layout.addWidget(QLabel("# columns"))
        ui_columns_layout.addWidget(self.columns)
        ui_layout.addLayout(ui_columns_layout)
        ui_layout.addStretch(1)

        service_layout = QVBoxLayout()
        service_layout.addWidget(QLabel("Services"))
        service_layout.addWidget(self.hide_parameter_services)
        service_layout.addStretch(1)

        log_layout = QVBoxLayout()
        log_layout.addWidget(QLabel("Logging"))
        log_layout.addWidget(self.hide_moni2_logs)
        log_layout.addWidget(self.hide_unmonitored_nodes)
        log_layout.addWidget(QLabel("Default log level: "))
        log_layout.addWidget(self.default_log_level)
        log_layout.addStretch(1)

        button = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        button_box = QDialogButtonBox(button)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout = QHBoxLayout()
        layout.addLayout(ui_layout)
        layout.addLayout(service_layout)
        layout.addLayout(log_layout)

        main_layout = QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addSpacing(20)
        main_layout.addWidget(button_box)
        self.setLayout(main_layout)

    def _columns_changed(self, value: str):
        try:
            columns = int(value)
            if 9 >= columns >= 1:
                self.settings.set_columns(columns)
        except ValueError:
            pass

import sys
import logging
from moni2.gui.settings_handler import SettingsWriter, SettingsReader
from moni2.gui.settings_dialog import SettingsDialog
from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)


class FakeSettings(SettingsWriter, SettingsReader):

    def __init__(self):
        self.cols = 2
        self.hide_pub = False
        self.hide_param = False
        self.hide_moni2 = False
        self.hide_unmonitored = False
        self.default_log = "INFO"

    def set_columns(self, columns: int):
        self.cols = columns

    def set_hide_parameter_services(self, hide: bool):
        self.hide_param = hide

    def set_hide_default_publishers(self, hide: bool):
        self.hide_pub = hide

    def hide_default_publishers(self) -> bool:
        return self.hide_pub

    def set_hide_moni2_logs(self, hide: bool):
        self.hide_moni2 = hide

    def set_hide_unmonitored_nodes(self, hide: bool):
        self.hide_unmonitored = hide

    def set_default_log_level(self, level: str):
        self.default_log = level

    def columns(self) -> int:
        return self.cols

    def hide_parameter_services(self) -> bool:
        return self.hide_param

    def hide_moni2_logs(self) -> bool:
        return self.hide_moni2

    def hide_unmonitored_nodes(self) -> bool:
        return self.hide_unmonitored

    def default_log_level(self) -> str:
        return self.default_log


class TestSettingsDialog:

    settings_dialog: SettingsDialog
    fake_settings: FakeSettings

    def setup(self):
        print("Setup")

        self.fake_settings = FakeSettings()
        self.settings_dialog = SettingsDialog(logging.getLogger("Test_SettingsDialog"), self.fake_settings)

    def teardown(self):
        print("Teardown")
        assert self.settings_dialog

    def test_something(self):
        assert self.fake_settings.columns() == 2

    def test_set_columns(self):
        self.settings_dialog.columns.setText("4")
        assert self.fake_settings.columns() == 4

    def test_invalid_set_columns(self):
        self.settings_dialog.columns.setText("10")
        assert self.fake_settings.columns() == 2

        self.settings_dialog.columns.setText("a")
        assert self.fake_settings.columns() == 2

    def test_hide_publishers(self):
        self.settings_dialog.hide_default_publishers.setChecked(True)
        assert self.fake_settings.hide_default_publishers()

    def test_set_hide_param(self):
        self.settings_dialog.hide_parameter_services.setChecked(True)
        assert self.fake_settings.hide_parameter_services()

    def test_hide_moni2_log(self):
        self.settings_dialog.hide_moni2_logs.setChecked(True)
        assert self.fake_settings.hide_moni2_logs()

    def test_hide_unmonitored_nodes(self):
        self.settings_dialog.hide_unmonitored_nodes.setChecked(True)
        assert self.fake_settings.hide_unmonitored_nodes()

    def test_default_log_level(self):
        self.settings_dialog.default_log_level.setCurrentIndex(0)
        assert self.fake_settings.default_log_level() == "DEBUG"

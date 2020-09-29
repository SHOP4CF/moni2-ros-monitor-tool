import sys
import logging
from rcl_interfaces.msg import Log
from PyQt5.QtWidgets import QApplication

from moni2.gui.log_widget import LogWidget
from moni2.gui.settings_handler import SettingsReader

app = QApplication(sys.argv)


class FakeSettingsHandler(SettingsReader):

    def hide_unmonitored_nodes(self) -> bool:
        return False

    def hide_moni2_logs(self) -> bool:
        return False

    def default_log_level(self) -> str:
        return 'INFO'


class TestLogWidget:

    log_widget: LogWidget

    def setup(self):
        print("Setup")

        self.count = 0
        self.log_widget = LogWidget(logging.getLogger("Test_LogWidget"), FakeSettingsHandler())
        self.assert_list(0, 0)

    def teardown(self):
        print("Teardown")
        assert self.log_widget

    def counter(self, log: str, count: int):
        self.count = count

    @staticmethod
    def create_log(text: str, level: int, name="TestNode") -> Log:
        log = Log()
        log.msg = text
        log.level = level
        log.name = name
        return log

    def assert_list(self, total_length: int, view_length: int):
        print(f"## Expected in list: {total_length}. Expected in view: {view_length}")
        print(f"Elements in list: {[log.msg for log in self.log_widget.log_list]}")
        print(f"Elements in view: "
              f"{[self.log_widget.log_list_view.item(i).text() for i in range(self.log_widget.log_list_view.count())]}")
        assert len(self.log_widget.log_list) == total_length, f"log_list should have {total_length} elements."
        assert self.log_widget.log_list_view.count() == view_length, f"log_list_view should have {view_length} elements"

    def test_init(self):
        assert self.log_widget

    def test_received_log(self):
        self.log_widget.received_log(self.create_log("Hello", logging.INFO))
        self.assert_list(1, 1)

    def test_receive_log_lower_log_level(self):
        self.log_widget.received_log(self.create_log("Hello", logging.DEBUG))
        self.assert_list(total_length=1, view_length=0)

    def test_received_warning_log(self):
        self.log_widget.warning_counter.connect(self.counter)

        self.log_widget.received_log(self.create_log("Hello", logging.WARNING))
        self.assert_list(1, 1)
        assert self.count == 1

    def test_received_error_log(self):
        self.log_widget.error_counter.connect(self.counter)

        self.log_widget.received_log(self.create_log("Hello", logging.ERROR))
        self.assert_list(1, 1)
        assert self.count == 1

    def test_filter(self):
        self.log_widget.received_log(self.create_log("Hello", logging.INFO))
        self.log_widget.received_log(self.create_log("Davs", logging.INFO, "RealNode"))
        self.assert_list(2, 2)

        self.log_widget.on_filter_text_changed("Real")
        self.assert_list(2, 1)

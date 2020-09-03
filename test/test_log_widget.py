import sys
import logging
from moni2.gui.log_widget import LogWidget
from rcl_interfaces.msg import Log

from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)


class TestLogWidget:

    log_widget: LogWidget

    def setup(self):
        print("Setup")

        self.count = 0
        self.log_widget = LogWidget(logging.getLogger("Test_LogWidget"))
        self.assert_list(0, 0)

    def teardown(self):
        print("Teardown")
        assert self.log_widget

    def counter(self, log: str, count: int):
        self.count = count

    @staticmethod
    def create_log(text: str, level: int) -> Log:
        log = Log()
        log.msg = text
        log.level = level
        return log

    def assert_list(self, total_length: int, view_length: int):
        assert len(self.log_widget.log_list) == total_length, f"log_list should have {total_length} elements"
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
        self.log_widget.received_log(self.create_log("Davs", logging.INFO))
        self.assert_list(2, 2)

        self.log_widget.on_filter_text_changed("av")
        self.assert_list(2, 2)
        hidden = 0
        for row in range(self.log_widget.log_list_view.count()):
            hidden += self.log_widget.log_list_view.item(row).isHidden()
        assert hidden == 1, f"1 out of 2 items should be hidden"

import sys
import logging
from moni2.gui import LogWidget
from rcl_interfaces.msg import Log

from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)


class TestLogWidget:

    log_widget: LogWidget

    def setup(self):
        print("Setup")

        self.log_widget = LogWidget(logging.getLogger("Test_LogWidget"))

    def teardown(self):
        print("Teardown")
        assert self.log_widget

    def test_init(self):
        assert self.log_widget

    def test_received_log(self):
        assert self.log_widget.log_list.count() == 0, "List of logs should be empty"
        log = Log()
        log.msg = "Hello"
        log.level = 10
        self.log_widget.received_log(log)
        assert self.log_widget.log_list.count() == 1, "List of logs should have 1 entry"

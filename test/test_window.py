import logging
from moni2.gui.window import MonitorWindow


class TestWindow:

    window: MonitorWindow

    def setup(self):
        print("Setup")

        log = logging.getLogger("Test_MonitorWindow")
        image_path = 'resource/images'
        self.window = MonitorWindow(log, image_path)
        print("Setup complete")

    def teardown(self):
        print("Teardown")

    def test_something(self):
        assert self.window.windowTitle() == self.window.APP_NAME

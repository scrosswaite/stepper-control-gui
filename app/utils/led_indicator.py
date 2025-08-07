class LedIndicator:
    def __init__(self, widget):
        self._widget = widget
        self._off_style = widget.styleSheet()
    def on(self):
        self._widget.setStyleSheet(
            "background-color: green; border:1px solid black; border-radius:6px;"
        )
    def off(self):
        self._widget.setStyleSheet(self._off_style)

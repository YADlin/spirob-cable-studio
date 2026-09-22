"""Consistent, readable Qt colours even when the desktop uses a dark theme."""
from PySide6 import QtGui, QtWidgets


STYLE = '''
QWidget {font-family: "DejaVu Sans"; font-size: 12px; color:palette(window-text);}
QMainWindow, QDialog {background:#edf2f5;}
QGroupBox {background:#fff; border:1px solid #cfdde5; border-radius:6px; margin-top:12px; padding-top:9px; font-weight:600;}
QGroupBox::title {subcontrol-origin:margin; left:9px; padding:0 4px;}
QPushButton {padding:7px 9px; color:#17364a; background:#fff; border:1px solid #a5b8c4; border-radius:4px;}
QPushButton:hover {background:#e3f0ef;}
QPushButton:pressed {background:#cce5e2;}
QPushButton:focus {border:2px solid #006b65; padding:6px 8px;}
QPushButton#primary {background:#00736d;color:white;font-weight:600;}
QPushButton#primary:hover {background:#00635e;}
QPushButton#primary:pressed {background:#00534f;}
QPushButton#stop {background:#b13c30;color:white;font-weight:700;}
QPushButton#stop:hover {background:#993126;}
QPushButton#stop:pressed {background:#82281f;}
QPushButton:disabled,QPushButton#primary:disabled,QPushButton#stop:disabled {
    color:#596773; background:#e5ebef; border:1px solid #c0ccd4;
}
QLineEdit,QComboBox,QAbstractSpinBox {
    color:#17364a; background:#fff; padding:5px; border:1px solid #a5b8c4; border-radius:3px;
    placeholder-text-color:#596773;
    selection-color:#fff; selection-background-color:#006b65;
}
QLineEdit:focus,QComboBox:focus,QAbstractSpinBox:focus {border:1px solid #006b65;}
QLineEdit:disabled,QComboBox:disabled,QAbstractSpinBox:disabled {background:#edf1f4;color:#596773;}
QTabWidget::pane {border:1px solid #cfdae2;background:#f5f8fa;}
QTabBar::tab {padding:9px;background:#e3ebf0;color:#17364a;}
QTabBar::tab:selected {background:white;color:#006b65;font-weight:700;}
QTabBar::tab:disabled {color:#596773;background:#e5ebef;}
QPlainTextEdit,QTextEdit,QAbstractItemView {
    color:#17364a; background:#fff; alternate-background-color:#edf2f5;
    placeholder-text-color:#596773;
    selection-color:#fff; selection-background-color:#006b65;
    border:1px solid #bdced9;
}
QAbstractItemView::item:selected {color:#fff;background:#006b65;}
QAbstractItemView:disabled,QPlainTextEdit:disabled,QTextEdit:disabled {color:#596773;background:#edf1f4;}
QHeaderView::section {color:#17364a;background:#e3ebf0;padding:5px;border:1px solid #bdced9;}
QTableCornerButton::section {background:#e3ebf0;border:1px solid #bdced9;}
QMenuBar {color:#17364a;background:#edf2f5;}
QMenu {color:#17364a;background:#fff;border:1px solid #a5b8c4;}
QMenuBar::item:selected,QMenu::item:selected {color:#fff;background:#006b65;}
QMenu::item:disabled {color:#596773;background:#fff;}
QToolTip {color:#17364a;background:#fff9dd;border:1px solid #8c9da9;padding:5px;}
QDockWidget::title {color:#17364a;background:#e3ebf0;padding:5px;}
'''


def apply_theme(app: QtWidgets.QApplication) -> None:
    """Set both sides of every text/background pair, including inactive states.

    Fusion determines widget drawing, but does not by itself replace an OS dark
    palette. An application palette also reaches separate popup/dialog windows.
    Keep disabled values legible; distinguish them with grey surfaces and their
    disabled interaction, rather than nearly invisible text.
    """
    app.setStyle('Fusion')
    palette = app.style().standardPalette()
    roles = {
        'Window': '#edf2f5', 'WindowText': '#17364a',
        'Base': '#ffffff', 'AlternateBase': '#edf2f5', 'Text': '#17364a',
        'Button': '#ffffff', 'ButtonText': '#17364a',
        'Highlight': '#006b65', 'HighlightedText': '#ffffff',
        'ToolTipBase': '#fff9dd', 'ToolTipText': '#17364a',
        'PlaceholderText': '#596773', 'BrightText': '#ffffff',
        'Link': '#006b65', 'LinkVisited': '#68449a', 'Accent': '#006b65',
        'Light': '#ffffff', 'Midlight': '#e3ebf0', 'Mid': '#a5b8c4',
        'Dark': '#596773', 'Shadow': '#17364a',
    }
    for group in (QtGui.QPalette.Active, QtGui.QPalette.Inactive, QtGui.QPalette.Disabled):
        for name, color in roles.items():
            palette.setColor(group, getattr(QtGui.QPalette.ColorRole, name), QtGui.QColor(color))
    for role in (QtGui.QPalette.WindowText, QtGui.QPalette.Text,
                 QtGui.QPalette.ButtonText, QtGui.QPalette.PlaceholderText):
        palette.setColor(QtGui.QPalette.Disabled, role, QtGui.QColor('#596773'))
    for role in (QtGui.QPalette.Base, QtGui.QPalette.Button):
        palette.setColor(QtGui.QPalette.Disabled, role, QtGui.QColor('#edf1f4'))
    app.setPalette(palette)
    app.setStyleSheet(STYLE)

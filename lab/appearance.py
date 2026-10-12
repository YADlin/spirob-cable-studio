"""Explicit light/dark palettes for every Qt window and Matplotlib figure."""
from PySide6.QtGui import QPalette, QColor


def colours(theme):
    return dict(bg='#111111', surface='#202020', fg='#ffffff', border='#666666',
                disabled='#bdbdbd', accent='#135fa8') if theme == 'dark' else dict(
                bg='#ffffff', surface='#f3f3f3', fg='#000000', border='#777777',
                disabled='#505050', accent='#135fa8')


def apply_theme(app, theme):
    c = colours(theme)
    app.setStyle('Fusion')
    palette = QPalette()
    roles = {'Window': c['bg'], 'WindowText': c['fg'], 'Base': c['bg'],
             'AlternateBase': c['surface'], 'Text': c['fg'], 'Button': c['surface'],
             'ButtonText': c['fg'], 'ToolTipBase': c['bg'], 'ToolTipText': c['fg'],
             'Highlight': c['accent'], 'HighlightedText': '#ffffff',
             'PlaceholderText': c['disabled'], 'Link': '#348fe8', 'BrightText': '#ffffff'}
    for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
        for role, colour in roles.items():
            palette.setColor(group, getattr(QPalette.ColorRole, role), QColor(colour))
    for role in (QPalette.Text, QPalette.WindowText, QPalette.ButtonText):
        palette.setColor(QPalette.Disabled, role, QColor(c['disabled']))
    app.setPalette(palette)
    app.setStyleSheet(f'''
        QWidget {{font-size:12px;}}
        QPushButton {{padding:5px 7px;}}
        QDoubleSpinBox, QSpinBox, QComboBox {{padding:3px;}}
        QToolTip {{color:{c['fg']};background:{c['bg']};border:1px solid {c['border']};}}
        QHeaderView::section {{color:{c['fg']};background:{c['surface']};padding:4px;}}
        QTableWidget {{gridline-color:{c['border']};}}
    ''')
    app.setProperty('spirob_theme', theme)


def theme_figure(figure, theme):
    c = colours(theme)
    figure.set_facecolor(c['bg'])
    for ax in figure.axes:
        ax.set_facecolor(c['bg'])
        ax.tick_params(which='both', colors=c['fg'])
        for spine in ax.spines.values(): spine.set_color(c['border'])
        for text in (ax.title, ax.xaxis.label, ax.yaxis.label, ax.xaxis.get_offset_text(), ax.yaxis.get_offset_text()):
            text.set_color(c['fg'])
        legend = ax.get_legend()
        if legend:
            legend.get_frame().set_facecolor(c['surface'])
            legend.get_frame().set_edgecolor(c['border'])
            for text in legend.get_texts(): text.set_color(c['fg'])
        ax.grid(color=c['border'], alpha=.25)

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QListWidget, QListWidgetItem, QHBoxLayout, QPushButton, QFrame
from PySide6.QtGui import QPixmap, QImage, QDesktopServices
from PySide6.QtCore import Qt, QUrl
import os

class GalleryPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<h2>Galerie Photos</h2>"))
        self.list = QListWidget()
        self.list.setStyleSheet("""
            QListWidget { background: #181a1b; border: none; }
            QListWidget::item { margin-bottom: 18px; }
        """)
        layout.addWidget(self.list)
        self.refresh_gallery()
    def refresh_gallery(self):
        self.list.clear()
        photos_dir = 'photos'
        if not os.path.exists(photos_dir):
            return
        for folder in sorted(os.listdir(photos_dir), reverse=True):
            folder_path = os.path.join(photos_dir, folder)
            if not os.path.isdir(folder_path):
                continue
            img_path = None
            for f in os.listdir(folder_path):
                if f.endswith('.jpg'):
                    img_path = os.path.join(folder_path, f)
                    break
            if not img_path:
                continue
            item = QListWidgetItem()
            h = QHBoxLayout()
            h.setContentsMargins(4, 4, 4, 4)
            h.setSpacing(12)
            pix = QPixmap(img_path).scaled(320, 240, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            img_lbl = QLabel()
            img_lbl.setPixmap(pix)
            img_lbl.setStyleSheet("background: #23272e; border: 1.2px solid #333; border-radius: 6px; padding: 1px;")
            h.addWidget(img_lbl, alignment=Qt.AlignVCenter)
            info_lbl = QLabel()
            info_lbl.setStyleSheet("background: #23272e; color: #fff; border: none; padding: 4px 10px; font-size: 15px; font-weight: 500; letter-spacing: 0.5px;")
            info_txt = ''
            info_file = os.path.join(folder_path, 'info.txt')
            date_str = ''
            rel_bearing_str = ''
            head_str = ''
            if os.path.exists(info_file):
                with open(info_file) as f:
                    lines = f.readlines()
                    for line in lines:
                        if line.startswith('Timestamp:'):
                            ts = line.split(':', 1)[1].strip()
                            try:
                                from datetime import datetime
                                dt = datetime.strptime(ts, '%Y%m%d_%H%M%S')
                                date_str = dt.strftime('%Y-%m-%d %H:%M:%S')
                            except Exception:
                                date_str = ts
                        elif line.startswith('Cap bateau:'):
                            head_str = line.strip()
                        elif line.startswith('Bearing relatif:'):
                            rel_bearing_str = line.strip()
                        else:
                            info_txt += line
            extra = ''
            if head_str:
                extra += f'<span style="color:#7fd7ff">{head_str}</span><br>'
            if rel_bearing_str:
                extra += f'<span style="color:#ffb347">{rel_bearing_str}</span><br>'
            info_lbl.setText(f"<b>{folder}</b><br><span style='color:#aaa'>{date_str}</span><br>{extra}{info_txt.replace(chr(10), '<br>')}")
            # Bouton carré entre la photo et le texte
            open_btn = QPushButton("📂")
            open_btn.setFixedSize(60, 60)
            open_btn.setStyleSheet("""
                QPushButton { background: #222; color: #fff; border-radius: 12px; font-size: 32px; border: 2px solid #444; }
                QPushButton:hover { background: #0078d7; color: #fff; border: 2px solid #0078d7; }
            """)
            open_btn.clicked.connect(lambda _, p=folder_path: QDesktopServices.openUrl(QUrl.fromLocalFile(os.path.abspath(p))))
            # Layout horizontal : image | bouton | texte
            h.addWidget(open_btn, alignment=Qt.AlignVCenter)
            info_col = QVBoxLayout()
            info_col.addWidget(info_lbl)
            info_col.addStretch()
            h.addLayout(info_col)
            widget = QFrame()
            widget.setLayout(h)
            widget.setFrameShape(QFrame.StyledPanel)
            widget.setFrameShadow(QFrame.Raised)
            widget.setStyleSheet("""
                QFrame { background: #181a1b; border: 1.2px solid #333; border-radius: 8px; }
                QFrame:hover { background: #23272e; border: 1.2px solid #0078d7; }
            """)
            item.setSizeHint(widget.sizeHint())
            self.list.addItem(item)
            self.list.setItemWidget(item, widget) 
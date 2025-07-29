from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QListWidget, QListWidgetItem, QHBoxLayout, QPushButton
from PySide6.QtGui import QPixmap, QImage, QDesktopServices
from PySide6.QtCore import Qt, QUrl
import os

class GalleryPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<h2>Galerie Photos</h2>"))
        self.list = QListWidget()
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
            widget = QWidget()
            h = QHBoxLayout(widget)
            pix = QPixmap(img_path).scaled(120, 90, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            img_lbl = QLabel()
            img_lbl.setPixmap(pix)
            h.addWidget(img_lbl)
            info_lbl = QLabel()
            info_txt = ''
            info_file = os.path.join(folder_path, 'info.txt')
            if os.path.exists(info_file):
                with open(info_file) as f:
                    info_txt = f.read().replace('\n', '<br>')
            info_lbl.setText(f"<b>{folder}</b><br>{info_txt}")
            h.addWidget(info_lbl)
            open_btn = QPushButton("Ouvrir")
            open_btn.clicked.connect(lambda _, p=folder_path: QDesktopServices.openUrl(QUrl.fromLocalFile(os.path.abspath(p))))
            h.addWidget(open_btn)
            widget.setLayout(h)
            item.setSizeHint(widget.sizeHint())
            self.list.addItem(item)
            self.list.setItemWidget(item, widget) 
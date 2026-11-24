"""obstacle_tab.py — obstacle editor tab: place sensor-level obstacles on the map for the simulator.

Left-click on empty map: add the selected shape (+1 click for a motion end point / box corner B).
Left-click an obstacle: select it -> drag to move, corner/side handles to resize, round handle to rotate.
Right-click: delete. Delete key: delete selected. Ctrl+Z: undo add / delete / move / resize / load.
Obstacles are sent to fake_robot as JSON on /sim/obstacle_cmd; they exist only in the simulated laser.
"""
import json
import math
import os
import random
import string
import time

import yaml

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

ACCENT = '#7fb0ec'; GREY = '#9aa3ae'


class ObstacleTab(QtWidgets.QWidget):
    def __init__(self, bus, send_fn, clear_costmaps_fn=None):
        super().__init__()
        self.bus = bus; self.send = send_fn; self.clear_costmaps = clear_costmaps_fn
        self.undo_stack = []                 # {'type': 'add', 'id'} | {'type': 'remove', 'obs'} | {'type': 'clear', 'obs': [...]}
        self.last_obs = {}                   # id -> obstacle state from the simulator
        self._sid = ''.join(random.choice(string.ascii_lowercase) for _ in range(3)); self._n = 0
        self.pending = None            # first click of a two-click action
        lay = QtWidgets.QHBoxLayout(self)
        self.plot = pg.PlotWidget(); self.plot.setAspectLocked(True); self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.setLabel('bottom', 'x (m)'); self.plot.setLabel('left', 'y (m)')
        lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(4)
        lay.addWidget(self.plot, 3)
        self.img = pg.ImageItem(); self.plot.addItem(self.img); self.img.setZValue(-10)
        self.robot = pg.ScatterPlotItem(size=12, brush=ACCENT); self.plot.addItem(self.robot)
        self.heading = pg.PlotCurveItem(pen=pg.mkPen(ACCENT, width=2)); self.plot.addItem(self.heading)
        self.obst_items = {}; self.path_items = {}        # obst_items: id -> editable pg.ROI
        self.sel_id = None; self._dragging = None; self._edit_before = None; self._edit_time = {}
        self.pending_item = pg.ScatterPlotItem(size=10, brush='#e0b35a'); self.plot.addItem(self.pending_item)
        self.plot.scene().sigMouseClicked.connect(self.on_click)
        self.plot.getPlotItem().setMenuEnabled(False); self.plot.getPlotItem().getViewBox().setMenuEnabled(False)   # right-click = delete
        for seq, fn in ((QtGui.QKeySequence.Undo, self.undo), (QtGui.QKeySequence.Delete, self.remove_selected)):
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self); sc.setContext(QtCore.Qt.WidgetWithChildrenShortcut); sc.activated.connect(fn)

        side_w = QtWidgets.QWidget(); side = QtWidgets.QVBoxLayout(side_w)
        side.setContentsMargins(0, 0, 0, 0); side.setSpacing(3)
        side_scroll = QtWidgets.QScrollArea(); side_scroll.setWidgetResizable(True); side_scroll.setWidget(side_w)
        side_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff); side_scroll.setFixedWidth(250)
        lay.addWidget(side_scroll, 1)
        box = QtWidgets.QGroupBox('장애물 추가'); f = QtWidgets.QFormLayout(box)
        self.kind = QtWidgets.QComboBox()
        self.kind.addItems(['원', '사각형 (크기·각도 지정)', '팔레트 1.2 x 1.0 m', '팔레트 1.2 x 0.8 m (EUR)', '팔레트 1.1 x 1.1 m (T11)', '박스 (두 모서리 클릭)'])
        self.kind.currentIndexChanged.connect(self.on_kind); f.addRow('모양', self.kind)
        self.radius = QtWidgets.QDoubleSpinBox(); self.radius.setRange(0.05, 3.0); self.radius.setSingleStep(0.05); self.radius.setValue(0.25); self.radius.setSuffix(' m'); f.addRow('원 반지름', self.radius)
        self.box_w = QtWidgets.QDoubleSpinBox(); self.box_w.setRange(0.1, 5.0); self.box_w.setSingleStep(0.1); self.box_w.setValue(1.2); self.box_w.setSuffix(' m'); f.addRow('가로', self.box_w)
        self.box_h = QtWidgets.QDoubleSpinBox(); self.box_h.setRange(0.1, 5.0); self.box_h.setSingleStep(0.1); self.box_h.setValue(1.0); self.box_h.setSuffix(' m'); f.addRow('세로', self.box_h)
        self.box_yaw = QtWidgets.QSpinBox(); self.box_yaw.setRange(-180, 180); self.box_yaw.setSingleStep(15); self.box_yaw.setValue(0); self.box_yaw.setSuffix(' °'); f.addRow('각도', self.box_yaw)
        self.motion = QtWidgets.QComboBox(); self.motion.addItems(['정지', '왕복 (클릭 1회 더: 끝점)', '순환 (클릭 1회 더: 끝점)']); f.addRow('움직임', self.motion)
        self.speed = QtWidgets.QDoubleSpinBox(); self.speed.setRange(0.1, 3.0); self.speed.setSingleStep(0.1); self.speed.setValue(0.8); self.speed.setSuffix(' m/s'); f.addRow('이동 속도', self.speed)
        side.addWidget(box)
        self.hint = QtWidgets.QLabel('빈 곳 클릭: 추가 · 장애물 클릭: 선택 후 끌기/손잡이로 이동·크기·회전 · 우클릭 삭제 · Ctrl+Z'); self.hint.setWordWrap(True); self.hint.setStyleSheet(f'color:{GREY}'); side.addWidget(self.hint)
        self.list = QtWidgets.QListWidget(); self.list.setMaximumHeight(90); side.addWidget(self.list)
        self.list.itemSelectionChanged.connect(self.on_selection)
        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton('선택 삭제'); b1.clicked.connect(self.remove_selected); row.addWidget(b1)
        b2 = QtWidgets.QPushButton('전체 삭제'); b2.clicked.connect(self.clear_all); row.addWidget(b2)
        side.addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        bu = QtWidgets.QPushButton('되돌리기'); bu.setToolTip('Ctrl+Z: 마지막 추가·삭제를 되돌림'); bu.clicked.connect(self.undo); row2.addWidget(bu)
        bc = QtWidgets.QPushButton('costmap 정리'); bc.setToolTip('지운 장애물 자국을 로컬·글로벌 costmap에서 즉시 지움'); bc.clicked.connect(self.on_clear_costmaps); row2.addWidget(bc)
        side.addLayout(row2)
        b5 = QtWidgets.QPushButton('팔레트 투하 시나리오'); b5.setToolTip('로봇 진행 방향 4 m 앞에 1.2 x 1.0 m 팔레트를 놓음 (고정물 조기 판단 시험)'); b5.clicked.connect(self.scenario_pallet)
        b3 = QtWidgets.QPushButton('보행자 교차 시나리오'); b3.setToolTip('로봇 앞 3 m, 좌→우 1.0 m/s 왕복'); b3.clicked.connect(self.scenario_cross); side.addWidget(b3)
        side.addWidget(b5)
        b4 = QtWidgets.QPushButton('로봇 위치로 이동'); b4.clicked.connect(self.center_robot); side.addWidget(b4)
        # ---- saved layouts ("commits") — the files are regression scenario files as well
        lay_box = QtWidgets.QGroupBox('레이아웃 저장 / 불러오기'); lv = QtWidgets.QVBoxLayout(lay_box); lv.setSpacing(3)
        rowl = QtWidgets.QHBoxLayout()
        self.lay_name = QtWidgets.QLineEdit(); self.lay_name.setPlaceholderText('이름 (예: 팔레트2_좁은통로)'); rowl.addWidget(self.lay_name)
        bsave = QtWidgets.QPushButton('저장'); bsave.clicked.connect(self.save_layout); rowl.addWidget(bsave)
        lv.addLayout(rowl)
        self.lay_list = QtWidgets.QListWidget(); self.lay_list.setMaximumHeight(110); lv.addWidget(self.lay_list)
        self.lay_list.itemDoubleClicked.connect(lambda _it: self.load_layout())
        rowb = QtWidgets.QHBoxLayout()
        bload = QtWidgets.QPushButton('불러오기'); bload.clicked.connect(self.load_layout); rowb.addWidget(bload)
        bdel = QtWidgets.QPushButton('파일 삭제'); bdel.clicked.connect(self.delete_layout); rowb.addWidget(bdel)
        brf = QtWidgets.QPushButton('새로고침'); brf.clicked.connect(self.refresh_layouts); rowb.addWidget(brf)
        lv.addLayout(rowb)
        self.cb_restore_pose = QtWidgets.QCheckBox('로봇을 저장된 위치로 이동'); self.cb_restore_pose.setChecked(True); lv.addWidget(self.cb_restore_pose)
        self.cb_send_goal = QtWidgets.QCheckBox('저장된 목표 전송 (재현 주행 시작)'); lv.addWidget(self.cb_send_goal)
        self.lay_info = QtWidgets.QLabel(''); self.lay_info.setWordWrap(True); self.lay_info.setStyleSheet('font-size:11px'); lv.addWidget(self.lay_info)
        side.addWidget(lay_box)
        self.layout_dir = os.path.expanduser('~/midpro_reports/obstacle_layouts')
        self.refresh_layouts()
        self.on_kind(0)
        # live numbers under the obstacle list (the main right panel is hidden on this tab)
        self.metrics_box = QtWidgets.QGroupBox('수치'); self.metrics_form = QtWidgets.QFormLayout(self.metrics_box)
        self.metrics_form.setVerticalSpacing(1); self.metrics = {}
        side.addWidget(self.metrics_box)
        side.addStretch(1)

    def add_metric(self, key, name, style):
        lb = QtWidgets.QLabel('-'); lb.setStyleSheet(style)
        self.metrics_form.addRow(name, lb); self.metrics[key] = lb
        self.map_loaded = False

    # ---------------------------------------------------------------- map / robot
    def set_map(self, grid, res, ox, oy):
        """grid: 2D list/array of occupancy (-1,0..100), row 0 = bottom (map frame)."""
        import numpy as np
        a = np.array(grid, dtype=float)
        img = np.where(a < 0, 0.55, 1.0 - a / 100.0)     # unknown grey, free light, occupied dark
        self.img.setImage(img.T, levels=(0, 1))
        self.img.setRect(QtCore.QRectF(ox, oy, a.shape[1] * res, a.shape[0] * res))
        self.img.setLookupTable(pg.ColorMap([0, 1], [(40, 44, 52), (210, 214, 220)]).getLookupTable(0.0, 1.0, 256))
        if not self.map_loaded:
            self.plot.autoRange(); self.map_loaded = True

    def update_robot(self, x, y, yaw):
        self.robot.setData([x], [y])
        self.heading.setData([x, x + 0.5 * math.cos(yaw)], [y, y + 0.5 * math.sin(yaw)])

    def center_robot(self):
        if self.robot.data is not None and len(self.robot.data):
            x, y = self.robot.data[0][0], self.robot.data[0][1]
            self.plot.setRange(xRange=(x - 8, x + 8), yRange=(y - 8, y + 8))

    PEN = '#e07a6a'; PEN_SEL = '#7fb0ec'

    @staticmethod
    def _box_pos(o):
        yaw = o.get('yaw', 0.0); c, s_ = math.cos(yaw), math.sin(yaw); w, h = o['w'], o['h']
        return [o['x'] - (c * w / 2 - s_ * h / 2), o['y'] - (s_ * w / 2 + c * h / 2)]

    def _roi_state(self, o):
        if o['type'] == 'circle':
            return {'pos': [o['x'] - o['r'], o['y'] - o['r']], 'size': [2 * o['r'], 2 * o['r']], 'angle': 0.0}
        return {'pos': self._box_pos(o), 'size': [o['w'], o['h']], 'angle': math.degrees(o.get('yaw', 0.0))}

    def _make_roi(self, o):
        st = self._roi_state(o)
        if o['type'] == 'circle':
            roi = pg.CircleROI(st['pos'], st['size'], pen=pg.mkPen(self.PEN, width=2), movable=True)
        else:
            roi = pg.ROI(st['pos'], st['size'], angle=st['angle'], pen=pg.mkPen(self.PEN, width=2), movable=True)
        for hd in list(roi.getHandles()):           # handles only while selected
            roi.removeHandle(hd)
        roi.setZValue(10)
        oid = o['id']
        roi.sigRegionChangeStarted.connect(lambda _r, oid=oid: self._edit_start(oid))
        roi.sigRegionChangeFinished.connect(lambda _r, oid=oid: self._edit_finish(oid))
        self.plot.addItem(roi)
        return roi

    def _set_roi(self, roi, o):
        roi.blockSignals(True)
        try:
            roi.setState(self._roi_state(o))
        finally:
            roi.blockSignals(False)

    def update_obstacles(self, obs):
        self.last_obs = {o['id']: o for o in obs}
        now = time.monotonic()
        ids = set()
        for o in obs:
            oid = o['id']; ids.add(oid)
            if oid not in self.obst_items:
                self.obst_items[oid] = self._make_roi(o)
                if o['type'] == 'box':
                    desc = f"사각형 {o['w']:.1f}x{o['h']:.1f} m {math.degrees(o.get('yaw', 0.0)):.0f}°"
                else:
                    desc = f"원 r {o['r']:.2f} m"
                item = QtWidgets.QListWidgetItem(f"{oid}  {desc}  {'이동' if o.get('motion') else '정지'}")
                item.setData(QtCore.Qt.UserRole, oid); self.list.addItem(item)
            elif oid != self._dragging and now - self._edit_time.get(oid, 0.0) > 0.8:
                self._set_roi(self.obst_items[oid], o)     # follow the simulator (moving obstacles, undo, load)
            if o.get('motion'):
                if oid not in self.path_items:
                    pth = pg.PlotCurveItem(pen=pg.mkPen(self.PEN, width=1, style=QtCore.Qt.DashLine)); self.plot.addItem(pth); self.path_items[oid] = pth
                pts = o['motion']['points']
                self.path_items[oid].setData([q[0] for q in pts], [q[1] for q in pts])
        for oid in list(self.obst_items):
            if oid not in ids:
                self.plot.removeItem(self.obst_items.pop(oid))
                if oid in self.path_items:
                    self.plot.removeItem(self.path_items.pop(oid))
                if self.sel_id == oid:
                    self.sel_id = None
                for i in range(self.list.count()):
                    if self.list.item(i).data(QtCore.Qt.UserRole) == oid:
                        self.list.takeItem(i); break
        self._refresh_list_labels()

    def _refresh_list_labels(self):
        for i in range(self.list.count()):
            it = self.list.item(i); oid = it.data(QtCore.Qt.UserRole); o = self.last_obs.get(oid)
            if not o:
                continue
            desc = (f"사각형 {o['w']:.1f}x{o['h']:.1f} m {math.degrees(o.get('yaw', 0.0)):.0f}°" if o['type'] == 'box'
                    else f"원 r {o['r']:.2f} m")
            txt = f"{oid}  {desc}  {'이동' if o.get('motion') else '정지'}"
            if it.text() != txt:
                it.setText(txt)

    # ---------------------------------------------------------------- select / drag / resize / rotate
    def select(self, oid):
        if oid == self.sel_id:
            return
        old = self.obst_items.get(self.sel_id)
        if old is not None:
            for hd in list(old.getHandles()):
                old.removeHandle(hd)
            old.setPen(pg.mkPen(self.PEN, width=2))
        self.sel_id = oid
        roi = self.obst_items.get(oid)
        if roi is not None:
            roi.setPen(pg.mkPen(self.PEN_SEL, width=3))
            if isinstance(roi, pg.CircleROI):
                roi.addScaleHandle([0.5 + 0.5 * 2 ** -0.5, 0.5 + 0.5 * 2 ** -0.5], [0.5, 0.5], lockAspect=True)
            else:
                for pos in ([1, 1], [0, 0], [1, 0], [0, 1]):
                    roi.addScaleHandle(pos, [0.5, 0.5])              # corners: both sides, centre fixed
                roi.addScaleHandle([1, 0.5], [0.5, 0.5]); roi.addScaleHandle([0.5, 1], [0.5, 0.5])   # width / depth
                roi.addRotateHandle([0.5, 1.35], [0.5, 0.5])        # rotation, above the top edge
        self.list.blockSignals(True)
        for i in range(self.list.count()):
            it = self.list.item(i); it.setSelected(it.data(QtCore.Qt.UserRole) == oid)
        self.list.blockSignals(False)
        if oid:
            self.hint.setText(f'{oid} 선택: 끌어서 이동 · 모서리/변 손잡이로 크기 · 둥근 손잡이로 회전 · Delete 삭제')

    def _edit_start(self, oid):
        self.select(oid)
        self._dragging = oid
        o = self.last_obs.get(oid)
        self._edit_before = self._to_cmd(o) if o else None

    def _edit_finish(self, oid):
        roi = self.obst_items.get(oid); o = self.last_obs.get(oid)
        self._dragging = None
        if roi is None or o is None:
            return
        st = roi.getState(); px, py = st['pos'][0], st['pos'][1]; w, h = st['size'][0], st['size'][1]
        if o['type'] == 'circle':
            r = max(0.05, 0.5 * (abs(w) + abs(h)) / 2)
            upd = {'op': 'update', 'id': oid, 'x': px + abs(w) / 2, 'y': py + abs(h) / 2, 'r': r}
        else:
            ang = math.radians(st['angle']); c, s_ = math.cos(ang), math.sin(ang)
            w, h = max(0.1, abs(w)), max(0.1, abs(h))
            upd = {'op': 'update', 'id': oid, 'x': px + c * w / 2 - s_ * h / 2, 'y': py + s_ * w / 2 + c * h / 2,
                   'w': w, 'h': h, 'yaw': math.atan2(math.sin(ang), math.cos(ang))}
        self.send(upd)
        self._edit_time[oid] = time.monotonic()
        if self._edit_before:
            self.undo_stack.append({'type': 'modify', 'before': self._edit_before})
        self._edit_before = None
        # keep the local copy in step until the simulator confirms
        o.update({k: v for k, v in upd.items() if k in ('x', 'y', 'r', 'w', 'h', 'yaw')})
        self._refresh_list_labels()

    # ---------------------------------------------------------------- interaction
    PALLETS = {2: (1.2, 1.0), 3: (1.2, 0.8), 4: (1.1, 1.1)}

    def on_kind(self, idx):
        """Show only the inputs the selected shape uses; pallet presets fill and lock the size."""
        is_circle = idx == 0
        is_rect = idx in (1, 2, 3, 4)
        self.radius.setEnabled(is_circle)
        for wdg in (self.box_w, self.box_h):
            wdg.setEnabled(idx == 1)
        self.box_yaw.setEnabled(is_rect)
        if idx in self.PALLETS:
            w, h = self.PALLETS[idx]; self.box_w.setValue(w); self.box_h.setValue(h)
        self.hint.setText('박스: 두 모서리를 클릭하세요.' if idx == 5 else '지도를 클릭해 장애물을 놓으세요.')

    def on_click(self, ev):
        pos = self.plot.getPlotItem().vb.mapSceneToView(ev.scenePos()); x, y = pos.x(), pos.y()
        if ev.button() == QtCore.Qt.RightButton:
            oid = self.obstacle_at(x, y)
            if oid:
                self.remove_ids([oid])
            ev.accept()
            return
        if ev.button() != QtCore.Qt.LeftButton:
            return
        if self.pending is None:
            hit = self.obstacle_at(x, y, tol=0.0)
            if hit:
                self.select(hit); return
            if self.sel_id:
                self.select(None)                       # click on empty map: deselect first, add on the next click
                self.hint.setText('선택 해제. 빈 곳을 클릭하면 장애물을 추가합니다.'); return
        idx = self.kind.currentIndex()
        moving = self.motion.currentIndex() > 0
        if self.pending is None:
            if idx == 5:                                   # free axis-aligned box: corner A then corner B
                self.pending = {'kind': 'box2', 'x': x, 'y': y, 'stage': 'first'}
                self.hint.setText('박스 모서리 B를 클릭하세요.'); self.pending_item.setData([x], [y]); return
            self.pending = {'kind': 'circle' if idx == 0 else 'rect', 'x': x, 'y': y, 'stage': 'placed'}
            if moving:
                self.hint.setText('이동 끝점을 클릭하세요.'); self.pending_item.setData([x], [y]); return
            self._commit(); return
        p = self.pending
        if p['kind'] == 'box2' and p['stage'] == 'first':
            p.update({'x2': x, 'y2': y, 'stage': 'placed'})
            if moving:
                self.hint.setText('이동 끝점을 클릭하세요.'); return
            self._commit(); return
        p['mx'], p['my'] = x, y                             # motion end point
        self._commit()

    def _commit(self):
        p = self.pending
        self.pending = None; self.pending_item.setData([], [])
        if p is None:
            return
        cmd = {'op': 'add'}
        if p['kind'] == 'circle':
            cmd.update(type='circle', x=p['x'], y=p['y'], r=self.radius.value())
        elif p['kind'] == 'rect':
            cmd.update(type='box', x=p['x'], y=p['y'], w=self.box_w.value(), h=self.box_h.value(), yaw=math.radians(self.box_yaw.value()))
        else:
            cmd.update(type='box', x=(p['x'] + p['x2']) / 2, y=(p['y'] + p['y2']) / 2, w=abs(p['x2'] - p['x']), h=abs(p['y2'] - p['y']), yaw=0.0)
        if 'mx' in p:
            cmd['motion'] = {'kind': 'pingpong' if self.motion.currentIndex() == 1 else 'loop',
                             'points': [[cmd['x'], cmd['y']], [p['mx'], p['my']]], 'speed': self.speed.value()}
        self.add(cmd); self.hint.setText('추가됨 (Ctrl+Z 되돌리기). 계속 클릭해 더 놓을 수 있습니다.')

    def scenario_pallet(self):
        """Drop a 1.2 x 1.0 m pallet 4 m ahead of the robot, square to its heading."""
        if self.robot.data is None or not len(self.robot.data):
            return
        x, y = self.robot.data[0][0], self.robot.data[0][1]
        yaw = math.atan2(self.heading.yData[1] - y, self.heading.xData[1] - x) if self.heading.xData is not None else 0.0
        self.add({'op': 'add', 'type': 'box', 'x': x + 4.0 * math.cos(yaw), 'y': y + 4.0 * math.sin(yaw), 'w': 1.0, 'h': 1.2, 'yaw': yaw})

    # ---------------------------------------------------------------- tracked edits (undo)
    def _new_id(self):
        self._n += 1
        return f'e{self._sid}{self._n}'

    @staticmethod
    def _to_cmd(o):
        c = {'op': 'add', 'id': o['id'], 'type': o['type'], 'x': o['x'], 'y': o['y']}
        if o['type'] == 'circle':
            c['r'] = o['r']
        else:
            c.update(w=o['w'], h=o['h'], yaw=o.get('yaw', 0.0))
        if o.get('motion'):
            m = o['motion']; c['motion'] = {'kind': m.get('kind', 'pingpong'), 'points': m.get('points', []), 'speed': m.get('speed', 0.5)}
        return c

    def add(self, cmd):
        cmd = dict(cmd); cmd.setdefault('id', self._new_id()); cmd['op'] = 'add'
        self.send(cmd)
        self.undo_stack.append({'type': 'add', 'id': cmd['id']})

    def remove_ids(self, ids):
        ids = [i for i in ids if i]
        if not ids:
            return
        for oid in ids:
            o = self.last_obs.get(oid)
            if o is not None:
                self.undo_stack.append({'type': 'remove', 'obs': self._to_cmd(o)})
            self.send({'op': 'remove', 'id': oid})
        self.hint.setText(f"삭제 요청: {', '.join(ids)}")
        QtCore.QTimer.singleShot(1500, lambda ids=list(ids): self._verify_removed(ids))

    def _verify_removed(self, ids):
        left = [i for i in ids if i in self.last_obs]
        self.hint.setText(f"삭제 확인 안 됨: {', '.join(left)} — 시뮬 응답 없음(/sim/obstacle_cmd)" if left
                          else '삭제됨 (Ctrl+Z 되돌리기). costmap에 자국이 남으면 \'costmap 정리\'')

    def remove_selected(self):
        ids = [it.data(QtCore.Qt.UserRole) for it in self.list.selectedItems()]
        if not ids and self.sel_id:
            ids = [self.sel_id]
        if not ids:
            self.hint.setText('목록에서 지울 장애물을 먼저 선택하세요 (또는 지도에서 우클릭).')
            return
        self.remove_ids(ids)

    def clear_all(self):
        if self.last_obs:
            self.undo_stack.append({'type': 'clear', 'obs': [self._to_cmd(o) for o in self.last_obs.values()]})
        self.send({'op': 'clear'})
        self.hint.setText('전체 삭제 (Ctrl+Z 되돌리기)')

    def undo(self):
        if not self.undo_stack:
            self.hint.setText('되돌릴 작업이 없습니다.'); return
        e = self.undo_stack.pop()
        if e['type'] == 'add':
            self.send({'op': 'remove', 'id': e['id']}); self.hint.setText(f"되돌림: {e['id']} 추가 취소")
        elif e['type'] == 'remove':
            self.send(e['obs']); self.hint.setText(f"되돌림: {e['obs']['id']} 복원")
        elif e['type'] == 'modify':
            b = e['before']
            upd = {'op': 'update', 'id': b['id'], 'x': b['x'], 'y': b['y']}
            upd.update({k: b[k] for k in ('r', 'w', 'h', 'yaw') if k in b})
            self.send(upd); self._edit_time.pop(b['id'], None)
            self.hint.setText(f"되돌림: {b['id']} 이동·크기 변경 취소")
        elif e['type'] == 'load':
            for oid in e['loaded_ids']:
                self.send({'op': 'remove', 'id': oid})
            for c in e['prev']:
                self.send(c)
            self.hint.setText(f"되돌림: 레이아웃 불러오기 취소 ({len(e['prev'])}개 복원)")
        else:
            for c in e['obs']:
                self.send(c)
            self.hint.setText(f"되돌림: {len(e['obs'])}개 복원")

    def obstacle_at(self, x, y, tol=0.3):
        best, bid = 1e9, None
        for oid, o in self.last_obs.items():
            if o['type'] == 'circle':
                d = max(0.0, math.hypot(x - o['x'], y - o['y']) - o['r'])
            else:
                c, s_ = math.cos(-o.get('yaw', 0.0)), math.sin(-o.get('yaw', 0.0))
                lx = c * (x - o['x']) - s_ * (y - o['y']); ly = s_ * (x - o['x']) + c * (y - o['y'])
                d = math.hypot(max(abs(lx) - o['w'] / 2, 0.0), max(abs(ly) - o['h'] / 2, 0.0))
            if d < best:
                best, bid = d, oid
        return bid if best <= tol else None

    # ---------------------------------------------------------------- saved layouts
    def refresh_layouts(self):
        self.lay_list.clear()
        if not os.path.isdir(self.layout_dir):
            return
        for fn in sorted(os.listdir(self.layout_dir), reverse=True):
            if not fn.endswith('.yaml'):
                continue
            path = os.path.join(self.layout_dir, fn)
            try:
                d = yaml.safe_load(open(path, encoding='utf-8')) or {}
                sc = (d.get('scenarios') or [{}])[0]
                label = f"{sc.get('name', fn)}  ·  {len(sc.get('obstacles', []))}개  ·  {d.get('created', '')[5:16]}"
            except Exception:  # noqa: BLE001
                label = fn + '  (읽기 실패)'
            it = QtWidgets.QListWidgetItem(label); it.setData(QtCore.Qt.UserRole, path); it.setToolTip(path)
            self.lay_list.addItem(it)

    def save_layout(self):
        name = self.lay_name.text().strip() or 'layout'
        safe = ''.join(ch if (ch.isalnum() or ch in '-_') else '_' for ch in name)
        obs = [self._to_cmd(o) for o in self.last_obs.values()]
        for c in obs:
            c.pop('op', None)
        start = self.bus.robot_pose() if hasattr(self.bus, 'robot_pose') else None
        goal = getattr(self.bus, 'last_goal', None)
        doc = {
            'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'map': getattr(self.bus, 'map_name', '') or 'ecoproBM_matterport_clean.yaml',
            'note': 'midpro obstacle layout. Also a regression scenario file: ros2 run midpro_dash run_regression --scenarios <this file>',
            'repeats': 1, 'settle_s': 2.0,
            'scenarios': [{
                'name': safe, 'group': 'user', 'desc': name,
                'start': list(start) if start else None,
                'goal': list(goal) if goal else None,
                'obstacles': obs,
                'timeout_s': 120,
            }],
        }
        os.makedirs(self.layout_dir, exist_ok=True)
        path = os.path.join(self.layout_dir, time.strftime('%Y%m%d_%H%M%S') + f'_{safe}.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None, width=200)
        self.refresh_layouts()
        miss = [k for k, v in (('로봇 위치', start), ('목표', goal)) if not v]
        self.lay_info.setText(f"저장: {os.path.basename(path)} (장애물 {len(obs)}개" + (f", {'·'.join(miss)} 없음)" if miss else ', 로봇 위치·목표 포함)'))

    def _selected_layout(self):
        it = self.lay_list.currentItem()
        return it.data(QtCore.Qt.UserRole) if it else None

    def load_layout(self):
        path = self._selected_layout()
        if not path:
            self.lay_info.setText('목록에서 레이아웃을 선택하세요.'); return
        try:
            d = yaml.safe_load(open(path, encoding='utf-8')) or {}
            sc = (d.get('scenarios') or [{}])[0]
        except Exception as e:  # noqa: BLE001
            self.lay_info.setText(f'읽기 실패: {e}'); return
        prev = [self._to_cmd(o) for o in self.last_obs.values()]
        self.send({'op': 'clear'})
        loaded = []
        for c in sc.get('obstacles', []) or []:
            c = dict(c); c['op'] = 'add'; c.setdefault('id', self._new_id())
            self.send(c); loaded.append(c['id'])
        self.undo_stack.append({'type': 'load', 'prev': prev, 'loaded_ids': loaded})
        msg = f"불러옴: {sc.get('name', os.path.basename(path))} (장애물 {len(loaded)}개)"
        st, gl = sc.get('start'), sc.get('goal')
        if self.cb_restore_pose.isChecked() and st and hasattr(self.bus, 'teleport'):
            self.bus.teleport(*st); msg += ' · 로봇 이동'
            if self.clear_costmaps:
                QtCore.QTimer.singleShot(600, self.clear_costmaps)
        if self.cb_send_goal.isChecked() and gl and hasattr(self.bus, 'send_goal'):
            QtCore.QTimer.singleShot(2500, lambda g=list(gl): self.bus.send_goal(*g)); msg += ' · 2.5 s 뒤 목표 전송'
        self.lay_info.setText(msg + ' (Ctrl+Z로 이전 배치 복원)')

    def delete_layout(self):
        path = self._selected_layout()
        if not path:
            self.lay_info.setText('목록에서 레이아웃을 선택하세요.'); return
        if QtWidgets.QMessageBox.question(self, '레이아웃 삭제', f'{os.path.basename(path)} 파일을 삭제할까요?') != QtWidgets.QMessageBox.Yes:
            return
        try:
            os.remove(path)
        except OSError as e:
            self.lay_info.setText(f'삭제 실패: {e}'); return
        self.refresh_layouts(); self.lay_info.setText('삭제됨: ' + os.path.basename(path))

    def on_selection(self):
        sel = [it.data(QtCore.Qt.UserRole) for it in self.list.selectedItems()]
        if sel and sel[0] != self.sel_id:
            self.select(sel[0])

    def on_clear_costmaps(self):
        if self.clear_costmaps:
            self.clear_costmaps(); self.hint.setText('costmap 정리 요청 (로컬·글로벌)')
        else:
            self.hint.setText('costmap 정리 기능을 쓸 수 없습니다.')

    def scenario_cross(self):
        if self.robot.data is None or not len(self.robot.data):
            return
        x, y = self.robot.data[0][0], self.robot.data[0][1]
        yaw = math.atan2(self.heading.yData[1] - y, self.heading.xData[1] - x) if self.heading.xData is not None else 0.0
        fx, fy = x + 3.0 * math.cos(yaw), y + 3.0 * math.sin(yaw)
        lx, ly = -math.sin(yaw), math.cos(yaw)
        self.add({'op': 'add', 'type': 'circle', 'x': fx + 2.5 * lx, 'y': fy + 2.5 * ly, 'r': 0.25,
                   'motion': {'kind': 'pingpong', 'points': [[fx + 2.5 * lx, fy + 2.5 * ly], [fx - 2.5 * lx, fy - 2.5 * ly]], 'speed': 1.0}})

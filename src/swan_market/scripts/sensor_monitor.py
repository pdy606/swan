#!/usr/bin/env python3
"""Read-only live ROS camera / LiDAR / real YOLO monitor for the market VM.
Publishes only annotated images and detection labels; never velocity commands.
Requires ROS Jazzy, PyQt5, numpy, torch, ultralytics and a local model file.
"""
import argparse,json,math,os,threading,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import String
from PyQt5 import QtCore,QtGui,QtWidgets


class Streams(Node):
    def __init__(self):
        super().__init__('market_sensor_monitor')
        self.lock=threading.Lock();self.frame=None;self.scan=None;self.annotated=None
        self.image_stamp=None;self.image_header=None;self.image_at=0.;self.scan_at=0.;self.yolo_at=0.
        self.counts={'camera':0,'scan':0,'yolo':0};self.yolo_state='Loading model…';self.names=[]
        self.inference_ms=0.;self.classes={};self.stop_event=threading.Event()
        self.create_subscription(Image,'/camera',self.image_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.annotated_pub=self.create_publisher(Image,'/yolo/image_raw',1)
        self.labels_pub=self.create_publisher(String,'/yolo/detected_objects',1)

    def image_cb(self,m):
        if m.encoding not in ('rgb8','bgr8'):return
        arr=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':arr=arr[:,:,::-1]
        with self.lock:
            self.frame=arr.copy();self.image_at=time.monotonic();self.image_header=m.header
            self.image_stamp=(m.header.stamp.sec,m.header.stamp.nanosec);self.counts['camera']+=1

    def scan_cb(self,m):
        with self.lock:self.scan=m;self.scan_at=time.monotonic();self.counts['scan']+=1

    def infer(self,model_path):
        try:
            import torch
            torch.set_num_threads(1);torch.set_num_interop_threads(1)
            from ultralytics import YOLO
            model=YOLO(model_path)
            with self.lock:self.classes=model.names;self.yolo_state='Model ready; waiting for camera'
            seen=None
            while not self.stop_event.is_set():
                with self.lock:
                    frame=self.frame;stamp=self.image_stamp;header=self.image_header
                if frame is None or stamp==seen:self.stop_event.wait(.05);continue
                seen=stamp;t0=time.monotonic()
                result=model.predict(source=frame[:,:,::-1].copy(),imgsz=416,conf=.25,device='cpu',verbose=False)[0]
                plotted=np.ascontiguousarray(result.plot()[:,:,::-1])
                names=[str(model.names[int(b.cls[0])]) for b in result.boxes]
                output=Image();output.header=header;output.height=plotted.shape[0];output.width=plotted.shape[1]
                output.encoding='rgb8';output.step=output.width*3;output.data=plotted.tobytes()
                self.annotated_pub.publish(output)
                labels=String();labels.data='Detected: '+', '.join(names) if names else 'Detected: none'
                self.labels_pub.publish(labels)
                with self.lock:
                    self.annotated=plotted;self.names=names;self.yolo_at=time.monotonic()
                    self.inference_ms=(self.yolo_at-t0)*1000;self.counts['yolo']+=1
                    self.yolo_state='Inference active'
                self.stop_event.wait(max(0,.5-(time.monotonic()-t0)))
        except Exception as exc:
            with self.lock:self.yolo_state=f'ERROR: {type(exc).__name__}: {exc}'
            self.get_logger().error(self.yolo_state)


class ScanView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.scan=None;self.setMinimumSize(280,300)
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#10202b'))
        cx,cy=self.width()/2,self.height()/2;scale=min(self.width(),self.height())/26
        p.setPen(QtGui.QPen(QtGui.QColor('#334952'),1))
        for radius in (2,4,8,12):
            rr=radius*scale;p.drawEllipse(QtCore.QPointF(cx,cy),rr,rr)
            p.drawText(QtCore.QPointF(cx+4,cy-rr+12),f'{radius} m')
        p.drawLine(QtCore.QPointF(cx,0),QtCore.QPointF(cx,self.height()))
        p.drawLine(QtCore.QPointF(0,cy),QtCore.QPointF(self.width(),cy))
        p.setPen(QtGui.QPen(QtGui.QColor('#5fe0b5'),3))
        if self.scan:
            s=self.scan
            for i,r in enumerate(s.ranges):
                if not math.isfinite(r) or r<s.range_min or r>s.range_max:continue
                a=s.angle_min+i*s.angle_increment
                p.drawPoint(QtCore.QPointF(cx-r*math.sin(a)*scale,cy-r*math.cos(a)*scale))
        p.setPen(QtGui.QPen(QtGui.QColor('#ffc768'),3))
        p.drawLine(QtCore.QPointF(cx,cy+10),QtCore.QPointF(cx,cy-12))
        p.drawText(12,24,'↑ FRONT   •   LiDAR origin')
        p.end()


class Monitor(QtWidgets.QWidget):
    def __init__(self,node,model_path,status_path):
        super().__init__();self.node=node;self.status_path=status_path;self.model_path=model_path;self.last_saved=0.
        self.setWindowTitle('SWAN | Camera · LiDAR · YOLO');self.resize(1180,640)
        self.setStyleSheet('QWidget{background:#152630;color:#e4eff3;font-size:14px;} QLabel{padding:5px;} QGroupBox{font-weight:bold;border:1px solid #3f5863;border-radius:5px;margin-top:12px;padding-top:14px;} QGroupBox::title{subcontrol-origin:margin;left:10px;}')
        layout=QtWidgets.QVBoxLayout(self)
        title=QtWidgets.QLabel('SWAN  /  LIVE SENSOR MONITOR');title.setStyleSheet('font-size:22px;font-weight:bold;');layout.addWidget(title)
        self.summary=QtWidgets.QLabel('Connecting to /camera and /scan…');layout.addWidget(self.summary)
        panels=QtWidgets.QHBoxLayout();layout.addLayout(panels,1)
        self.raw,self.raw_status=self.image_panel(panels,'CAMERA  /camera')
        box=QtWidgets.QGroupBox('LiDAR  /scan');col=QtWidgets.QVBoxLayout(box);self.lidar=ScanView();col.addWidget(self.lidar,1)
        self.scan_status=QtWidgets.QLabel('Waiting for scan…');self.scan_status.setWordWrap(True);col.addWidget(self.scan_status);panels.addWidget(box,1)
        self.yolo,self.yolo_status=self.image_panel(panels,'YOLO  /yolo/image_raw')
        self.details=QtWidgets.QLabel('');self.details.setWordWrap(True);layout.addWidget(self.details)
        footer=QtWidgets.QLabel('Read-only monitor • Drive from the teleop terminal (i / j / k / l / ,) • k = STOP\nA zero-detection frame means no object passed the model confidence threshold; it does not prove the scene is clear.')
        footer.setStyleSheet('color:#aec3ca;font-size:12px;');layout.addWidget(footer)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(150)

    def image_panel(self,panels,title):
        group=QtWidgets.QGroupBox(title);column=QtWidgets.QVBoxLayout(group)
        image=QtWidgets.QLabel('Waiting for frames…');image.setAlignment(QtCore.Qt.AlignCenter);image.setMinimumSize(280,300)
        column.addWidget(image,1);status=QtWidgets.QLabel('');status.setWordWrap(True);column.addWidget(status);panels.addWidget(group,1)
        return image,status

    def show_image(self,label,arr):
        if arr is None:return
        h,w,_=arr.shape
        q=QtGui.QImage(arr.data,w,h,arr.strides[0],QtGui.QImage.Format_RGB888).copy()
        label.setPixmap(QtGui.QPixmap.fromImage(q).scaled(label.size()-QtCore.QSize(12,12),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))

    def refresh(self):
        now=time.monotonic()
        with self.node.lock:
            n=self.node;raw=n.frame;scan=n.scan;annotated=n.annotated;counts=dict(n.counts)
            ages={k:round(now-v,2) if v else None for k,v in [('camera',n.image_at),('scan',n.scan_at),('yolo',n.yolo_at)]}
            state=n.yolo_state;names=list(n.names);ms=n.inference_ms;classes=dict(n.classes)
        self.show_image(self.raw,raw);self.show_image(self.yolo,annotated);self.lidar.scan=scan;self.lidar.update()
        alive=lambda k:'LIVE' if ages[k] is not None and ages[k]<3 else 'WAIT / STALE'
        self.summary.setText(f"Camera: {alive('camera')}     LiDAR: {alive('scan')}     YOLO: {alive('yolo')}")
        self.raw_status.setText(f"{raw.shape[1] if raw is not None else 0} × {raw.shape[0] if raw is not None else 0} RGB\nFrames {counts['camera']}  •  age {ages['camera']} s")
        finite=[r for r in scan.ranges if math.isfinite(r)] if scan else []
        self.scan_status.setText(f"{len(scan.ranges) if scan else 0} rays • {len(finite)} finite\nFrames {counts['scan']} • age {ages['scan']} s\nRear points can include the wheelchair itself.")
        self.yolo_status.setText(f"{state}\nResults {counts['yolo']} • {ms:.0f} ms inference\nDetections: {', '.join(names) if names else '0'} • age {ages['yolo']} s")
        self.details.setText(f"Model: {Path(self.model_path).name} • CPU • input 416 • confidence ≥ 0.25\nClasses: {', '.join(str(v) for v in classes.values())}")
        if now-self.last_saved>2:
            self.status_path.write_text(json.dumps({'counts':counts,'age_seconds':ages,'state':state,'detections':names,'inference_ms':ms,'classes':classes,'model':self.model_path},indent=2))
            self.last_saved=now

    def closeEvent(self,event):
        self.node.stop_event.set();event.accept()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--model',required=True);parser.add_argument('--status',default='/tmp/market-sensor-monitor-status.json');args=parser.parse_args()
    if not Path(args.model).is_file():parser.error('Local model file is missing')
    rclpy.init();node=Streams();app=QtWidgets.QApplication([])
    threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    threading.Thread(target=node.infer,args=(args.model,),daemon=True).start()
    window=Monitor(node,args.model,Path(args.status));window.show()
    app.exec_();node.stop_event.set();rclpy.shutdown()


if __name__=='__main__':main()

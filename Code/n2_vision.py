"""Roboflow smoke detection with one continuously tracked display box.

Two bounded cloud requests can run concurrently; the newest CAPTURE wins.
A cloud confirmation is only actionable while its original source image is fresh.
Optical flow is visualization assistance, not standalone evidence of smoke.
"""
import math
import queue
import threading
import time

import cv2
import numpy as np

MODEL_ID = 'gas-dhh4u/1'
INPUT_WIDTH = 640
SHOW_CONF = 0.20
VERIFY_CONF = 0.60
VERIFY_REPLIES = 2

# Source-image age is measured from when the Windows client received the frame,
# not from when Roboflow replied. Never refresh this timestamp from Optical Flow.
MAX_CAPTURE_AGE = 1.30
DISPLAY_TTL = 1.50
API_INTERVAL = 0.45       # no more than ~2.2 requests/sec (may count toward quota)
CLOUD_WORKERS = 2         # at most 2 concurrent requests; no accumulating backlog


def is_smoke(label):
    return str(label).strip().lower().startswith('smoke100')


def to_box(p):
    x, y, w, h = (float(p[k]) for k in ('x', 'y', 'width', 'height'))
    return np.array((x-w/2, y-h/2, x+w/2, y+h/2), dtype=float)


def overlap(a, b):
    xa, ya = np.maximum(a[:2], b[:2])
    xb, yb = np.minimum(a[2:], b[2:])
    intersection = max(0, xb-xa)*max(0, yb-ya)
    area_a = max(0, a[2]-a[0])*max(0, a[3]-a[1])
    area_b = max(0, b[2]-b[0])*max(0, b[3]-b[1])
    return intersection/max(1, area_a+area_b-intersection)


class VisionNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.tasks = queue.Queue(maxsize=1)
        self.running = True
        self.key = ''
        self.generation = 0
        self.response = None
        self.response_id = 0
        self.processed_id = 0
        self.latest_capture = -1.0
        self.error = 'ENTER ROBOFLOW API KEY'
        self.last_latency = 0.0
        self.last_submit = 0.0
        self.verified = 0
        self.last_verified_capture = 0.0
        self.api_box = None
        self.box = None
        self.score = 0.0
        self.box_at = 0.0
        self.prev_gray = None
        self.flow_pts = None
        self.workers = [threading.Thread(target=self._loop, daemon=True)
                        for _ in range(CLOUD_WORKERS)]
        for worker in self.workers:
            worker.start()

    def set_api_key(self, key):
        key = key.strip()
        with self.lock:
            if key == self.key:
                return
            self.key = key
            self.generation += 1
            self.response = None
            self.response_id += 1
            self.processed_id = self.response_id
            self.latest_capture = -1.0
            self.verified = 0
            self.last_verified_capture = 0.0
            self.api_box = None
            self.box = None
            self.flow_pts = None
            self.last_submit = 0.0
            self.error = 'WAITING FOR API' if key else 'ENTER ROBOFLOW API KEY'
        self._clear_pending()

    def _clear_pending(self):
        while True:
            try:
                self.tasks.get_nowait()
                self.tasks.task_done()
            except queue.Empty:
                break

    def close(self):
        self.running = False
        self._clear_pending()
        for worker in self.workers:
            worker.join(timeout=0.3)

    def _loop(self):
        client, client_generation = None, -1
        while self.running:
            try:
                job = self.tasks.get(timeout=0.15)
            except queue.Empty:
                continue
            image, gray, capture_t, gen, key = job
            try:
                if client_generation != gen:
                    from inference_sdk import InferenceHTTPClient, InferenceConfiguration
                    client = InferenceHTTPClient(
                        api_url='https://serverless.roboflow.com', api_key=key
                    ).configure(InferenceConfiguration(api_key_transport='header'))
                    client_generation = gen
                result = client.infer(image, model_id=MODEL_ID)
                preds = result.get('predictions', [])
                if not isinstance(preds, list):
                    preds = []
                smoke = []
                for p in preds:
                    try:
                        confidence = float(p.get('confidence', 0))
                        if is_smoke(p.get('class')) and confidence >= SHOW_CONF:
                            smoke.append((to_box(p), confidence))
                    except (TypeError, ValueError, KeyError):
                        continue
                received_t = time.monotonic()
                with self.lock:
                    # Late out-of-order responses must never overwrite newer frames.
                    if gen == self.generation and capture_t > self.latest_capture:
                        self.latest_capture = capture_t
                        self.response = (smoke, capture_t, gray, len(preds))
                        self.response_id += 1
                        self.last_latency = received_t-capture_t
                        self.error = ''
            except Exception as e:
                with self.lock:
                    if gen == self.generation and capture_t > self.latest_capture:
                        self.latest_capture = capture_t
                        self.response = None
                        self.response_id += 1
                        self.error = 'ROBOFLOW ERROR: '+str(e)[:85]
                        self.verified = 0
                        self.api_box = None
                        self.box = None
                        self.flow_pts = None
            finally:
                self.tasks.task_done()

    def _submit(self, small, gray, now):
        if not self.key or now-self.last_submit < API_INTERVAL:
            return
        self.last_submit = now
        job = (small.copy(), gray.copy(), now, self.generation, self.key)
        # Replace a waiting image with the newest; never build up a stale queue.
        if self.tasks.full():
            try:
                self.tasks.get_nowait()
                self.tasks.task_done()
            except queue.Empty:
                pass
        try:
            self.tasks.put_nowait(job)
        except queue.Full:
            pass

    def _project(self, box, captured_gray, current_gray):
        """Estimate image displacement since camera capture; never infer smoke from it."""
        if captured_gray.shape != current_gray.shape:
            return box.copy()
        h, w = captured_gray.shape
        x0, y0, x1, y1 = np.round(box).astype(int)
        x0 = int(np.clip(x0, 0, w-1))
        x1 = int(np.clip(x1, x0+1, w))
        y0 = int(np.clip(y0, 0, h-1))
        y1 = int(np.clip(y1, y0+1, h))
        mask = np.zeros((h, w), np.uint8)
        mask[y0:y1, x0:x1] = 255
        pts = cv2.goodFeaturesToTrack(captured_gray, 80, .015, 6, mask=mask)
        if pts is None or len(pts) < 7:
            return box.copy()
        nxt, ok, _ = cv2.calcOpticalFlowPyrLK(
            captured_gray, current_gray, pts, None,
            winSize=(21, 21), maxLevel=3)
        if nxt is None or ok is None:
            return box.copy()
        good = ok.ravel().astype(bool)
        if np.count_nonzero(good) < 7:
            return box.copy()
        delta = (nxt-pts).reshape(-1, 2)[good]
        median = np.median(delta, axis=0)
        if np.count_nonzero(np.linalg.norm(delta-median, axis=1) < 8) < 6:
            return box.copy()
        shift = np.clip(median, -65, 65)
        return box + np.array((shift[0], shift[1], shift[0], shift[1]))

    def _flow(self, gray, w, h):
        if self.box is None or self.prev_gray is None:
            return
        if self.flow_pts is None or len(self.flow_pts) < 6:
            mask = np.zeros_like(self.prev_gray, dtype=np.uint8)
            x0, y0, x1, y1 = np.round(self.box).astype(int)
            x0 = int(np.clip(x0, 0, w-1))
            x1 = int(np.clip(x1, x0+1, w))
            y0 = int(np.clip(y0, 0, h-1))
            y1 = int(np.clip(y1, y0+1, h))
            mask[y0:y1, x0:x1] = 255
            self.flow_pts = cv2.goodFeaturesToTrack(
                self.prev_gray, 80, .015, 6, mask=mask)
        if self.flow_pts is None or len(self.flow_pts) < 6:
            return
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.flow_pts, None, winSize=(17,17), maxLevel=2)
        if nxt is None or status is None:
            self.flow_pts = None
            return
        good = status.ravel().astype(bool)
        if np.count_nonzero(good) < 6:
            self.flow_pts = None
            return
        vectors = (nxt-self.flow_pts).reshape(-1,2)[good]
        median = np.median(vectors, axis=0)
        consistent = np.linalg.norm(vectors-median, axis=1) < 6
        if np.count_nonzero(consistent) < 6:
            self.flow_pts = None
            return
        shift = np.clip(np.median(vectors[consistent],axis=0),-12,12)
        self.box += np.array((shift[0],shift[1],shift[0],shift[1]))
        self.flow_pts = nxt[good][consistent].reshape(-1,1,2)

    @staticmethod
    def _header(image, message, y, color):
        cv2.rectangle(image, (0,y-23), (image.shape[1]-1,y+7), (18,18,18), -1)
        cv2.putText(image, message[:105], (10,y),
                    cv2.FONT_HERSHEY_SIMPLEX, .54, color, 2)

    def process(self, frame):
        if frame is None or frame.size == 0:
            return frame, {'center':None, 'coverage':0., 'width':0,
                           'confirmed':False, 'seq':0, 'capture_age':999.}
        h, w = frame.shape[:2]
        resized_h = max(1, round(INPUT_WIDTH*h/w))
        small = cv2.resize(frame, (INPUT_WIDTH, resized_h), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        now = time.monotonic()
        with self.lock:
            self._submit(small,gray,now)
            self._flow(gray, INPUT_WIDTH, resized_h)
            self.prev_gray = gray
            response = self.response
            result_id = self.response_id
            err = self.error
            latency = self.last_latency
            if result_id != self.processed_id:
                self.processed_id = result_id
                if response is None:
                    self.box = None
                    self.api_box = None
                    self.verified = 0
                    self.flow_pts = None
                else:
                    candidates, captured_t, captured_gray, raw_count = response
                    age_at_reply = now-captured_t
                    if age_at_reply >= MAX_CAPTURE_AGE:
                        # Late results have no authority, including for confirmation.
                        self.api_box = None
                        self.box = None
                        self.verified = 0
                        self.flow_pts = None
                    else:
                        valid = [(self._project(b,captured_gray,gray),s)
                                 for b,s in candidates if s >= VERIFY_CONF]
                        chosen = None
                        if valid:
                            if self.api_box is None:
                                chosen = max(valid,key=lambda candidate:candidate[1])
                            else:
                                center_old = (self.api_box[:2]+self.api_box[2:])/2
                                def quality(candidate):
                                    b, score = candidate
                                    center_new = (b[:2]+b[2:])/2
                                    dist = np.linalg.norm(center_new-center_old)/math.hypot(INPUT_WIDTH,resized_h)
                                    return 1.7*overlap(b,self.api_box)+score-1.4*dist
                                chosen = max(valid,key=quality)
                                center_new = (chosen[0][:2]+chosen[0][2:])/2
                                if np.linalg.norm(center_new-center_old)/math.hypot(INPUT_WIDTH,resized_h)>0.30:
                                    self.verified = 0
                            if captured_t-self.last_verified_capture > 1.75:
                                self.verified = 0
                        if chosen is None:
                            self.verified = 0
                            self.api_box = None
                            if candidates:
                                b,s = max(candidates, key=lambda c:c[1])
                                self.box = self._project(b,captured_gray,gray)
                                self.score = s
                                self.box_at = now
                            else:
                                self.box = None
                            self.flow_pts = None
                        else:
                            b,s = chosen
                            self.verified = min(VERIFY_REPLIES,self.verified+1)
                            self.last_verified_capture = captured_t
                            self.api_box = b.copy()
                            self.box_at = now
                            self.score = s
                            self.box = b.copy() if self.box is None else 0.30*self.box+0.70*b
                            self.flow_pts = None
            shown_box = None if self.box is None else self.box.copy()
            confidence, count = self.score, self.verified
            age = now-response[1] if response else float('inf')
            raw = response[3] if response else 0
            api_ok = (response is not None and not err)
            if shown_box is not None and (now-self.box_at > DISPLAY_TTL or not self.key):
                shown_box = None
                self.box = None
                self.api_box = None
                self.verified = 0
                count = 0
        confirmed = bool(api_ok and count>=VERIFY_REPLIES and age<MAX_CAPTURE_AGE and shown_box is not None)
        display = frame.copy()
        center = None
        coverage = 0.0
        if shown_box is not None:
            x0,y0,x1,y1 = shown_box*np.array((w/INPUT_WIDTH,h/resized_h,w/INPUT_WIDTH,h/resized_h))
            x0 = int(np.clip(x0,0,w-1)); x1 = int(np.clip(x1,x0+1,w))
            y0 = int(np.clip(y0,0,h-1)); y1 = int(np.clip(y1,y0+1,h))
            color = (0,225,0) if confirmed else (0,175,255)
            cv2.rectangle(display,(x0,y0),(x1,y1),color,2)
            cx,cy = (x0+x1)//2,(y0+y1)//2
            cv2.drawMarker(display,(cx,cy),color,cv2.MARKER_CROSS,17,2)
            cv2.putText(display,f'{"LOCK" if confirmed else "CANDIDATE"} {confidence:.0%}',
                        (x0,max(80,y0-8)),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2)
            if confirmed:
                center = (cx,cy)
                coverage = (x1-x0)*(y1-y0)/(w*h)
        if err:
            status = err
        elif age>=MAX_CAPTURE_AGE:
            status = 'API RESULT TOO OLD - STOP'
        elif confirmed:
            status = f'SMOKE LOCK {confidence:.0%} | BOX {coverage:.1%}'
        elif raw==0:
            status = 'API OK | NO SMOKE PREDICTION'
        else:
            status = f'API CANDIDATE {confidence:.0%} | {count}/{VERIFY_REPLIES}'
        self._header(display,status,26,(0,210,0) if confirmed else (0,180,255))
        self._header(display,f'Cloud {latency:.1f}s | Source age {age:.1f}s | FLOW VISUAL ONLY',
                     54,(225,225,225))
        band = int(w*0.15)
        cv2.line(display,(w//2-band,85),(w//2-band,h),(0,180,0),2)
        cv2.line(display,(w//2+band,85),(w//2+band,h),(0,180,0),2)
        return display, {'center':center, 'coverage':coverage, 'width':w,
                         'confirmed':confirmed, 'verified':count,
                         'seq':result_id, 'capture_age':age,
                         'confidence':confidence}

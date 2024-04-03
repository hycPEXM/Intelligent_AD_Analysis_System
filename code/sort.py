"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function

import os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from skimage import io

import glob
import time
import argparse
from filterpy.kalman import KalmanFilter

np.random.seed(0)
MAX_AGE=3
IOU_THRESH=0.3
# MIN_HITS=3
MIN_HITS=2

def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True) #返回的应该是索引值
    return np.array([[y[i],i] for i in x if i >= 0]) #y[i]表示当前的detection，i表示predict()后的track
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))

def iou_batch(bb_test, bb_gt):
  """
  From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
  """
  bb_gt = np.expand_dims(bb_gt, 0)
  bb_test = np.expand_dims(bb_test, 1)
  
  xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
  yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
  xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
  yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
  w = np.maximum(0., xx2 - xx1)
  h = np.maximum(0., yy2 - yy1)
  wh = w * h
  o = wh / ((bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])                                      
    + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1]) - wh)                                              
  return(o)  


def convert_bbox_to_z(bbox):
  """
  Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
  """
  w = bbox[2] - bbox[0]
  h = bbox[3] - bbox[1]
  x = bbox[0] + w/2.
  y = bbox[1] + h/2.
  s = w * h    #scale is just area
  r = w / float(h)
  return np.array([x, y, s, r]).reshape((4, 1))


def convert_x_to_bbox(x,score=None):
  """
  Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
  """
  w = np.sqrt(x[2] * x[3])
  h = x[2] / w
  if(score==None):
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.]).reshape((1,4))
  else:
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.,score]).reshape((1,5))


class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    """
    x: filter state estimate 列向量
    P: covariance matrix 方阵 
    Q: Process uncertainty/noise 方阵
    R: measurement uncertainty/noise  方阵
    H: measurement function  非方阵
    F: state transition matrix 方阵
    K: Kalman gain 非方阵 
    y: residual = z-Hx  列向量
    B: control transition matrix 非方阵(dim_x, dim_u) 
    u: Optional control vector 列向量  B和u在SORT算法里没有用到
    S: HPH’ + R 方阵
    SI: S^{-1} 方阵
    z: Last measurement used in update(). Read only.  列向量
    I: identity matrix
    
    #(1)predict:
    x = Fx + Bu
    P = FPF' + Q

    #(2)update:
    K = PH'(HPH' + R)^{-1}
    x = x + K(z - Hx) = x + Ky
    P = (I-KH)P = P - KSK' != (I-KH)P(I-KH)' + KRK' 
    """
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=4) 
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.R[2:,2:] *= 10.
    self.kf.P[4:,4:] *= 1000. #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[4:,4:] *= 0.01

    self.kf.x[:4] = convert_bbox_to_z(bbox)
    self.time_since_update = 0
    # self.id = KalmanBoxTracker.count
    # KalmanBoxTracker.count += 1
    # self.history = []
    self.hits = 0
    self.hit_streak = 0 
    self.hit_streak_init=0
    self.id_added_flag=False 
    self.age = 0

  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0
    # self.history = []
    self.hits += 1
    self.hit_streak += 1
    self.kf.update(convert_bbox_to_z(bbox))
    # if(self.hit_streak>=MIN_HITS) and not self.id_added_flag:
    if not self.id_added_flag:
      self.hit_streak_init+=1
      assert self.hit_streak == self.hit_streak_init, "hit_streak must be equal to hit_streak_init before a new id is produced"
      if(self.hit_streak_init>=MIN_HITS):
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.id_added_flag=True

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()
    self.age += 1
    # if(self.time_since_update>0):  #hit_streak记录track与当前detection连续匹配的帧数，一旦有一次匹配失败，就将其清零
    # if(self.time_since_update>MAX_AGE-1):  
    #   # 连续MAX_AGE次匹配失败，hit_streak清零，回到unconfirmed态，如果后续成功匹配，
    #   # 连续update几帧直到hit_streak>=min_hits，再重新变回confirmed态
    #   # 如果后续还是匹配失败，则self.time_since_update += 1后满足条件time_since_update>max_age，此track被删除
    #   self.hit_streak = 0
    if not self.id_added_flag:
      if(self.time_since_update>0): # 初始化阶段，在达到MIN_HITS之前若出现过一次匹配失败就将hit_streak置零
        self.hit_streak=0
        self.hit_streak_init=0
    elif(self.time_since_update>MAX_AGE-1):
      self.hit_streak=0
    
    self.time_since_update += 1
    # self.history.append(convert_x_to_bbox(self.kf.x))
    # return self.history[-1]
    return convert_x_to_bbox(self.kf.x)

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    return convert_x_to_bbox(self.kf.x)


# def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.3):
def associate_detections_to_trackers(detections,trackers,iou_threshold = IOU_THRESH):
  """
  Assigns detections to tracked object (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

  iou_matrix = iou_batch(detections, trackers)

  if min(iou_matrix.shape) > 0:
    a = (iou_matrix > iou_threshold).astype(np.int32)
    if a.sum(1).max() == 1 and a.sum(0).max() == 1:
        matched_indices = np.stack(np.where(a), axis=1)
    else:
      matched_indices = linear_assignment(-iou_matrix)
  else:
    matched_indices = np.empty(shape=(0,2))

  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0], m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
  # def __init__(self, max_age=1, min_hits=3, iou_threshold=0.3):
  def __init__(self, max_age=MAX_AGE, min_hits=MIN_HITS, iou_threshold=IOU_THRESH):
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.iou_threshold = iou_threshold
    self.trackers = []
    self.frame_count = 0

  def update(self, dets=np.empty((0, 5))):
    """
    形状为0的含义：
    >>> np.zeros((0,4))
    array([], shape=(0, 4), dtype=float64)
    >>> np.zeros((1,4))
    array([[0., 0., 0., 0.]])
    
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.

    NOTE: The number of objects returned may differ from the number of detections provided.
    """
    self.frame_count += 1
    # get predicted locations from existing trackers.
    trks = np.zeros((len(self.trackers), 5))
    to_del = []
    ret = []
    for t, trk in enumerate(trks):
      pos = self.trackers[t].predict()[0]
      trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
      if np.any(np.isnan(pos)):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
    for t in reversed(to_del):
      self.trackers.pop(t)
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks, self.iou_threshold)

    # update matched trackers with assigned detections
    for m in matched:
      self.trackers[m[1]].update(dets[m[0], :])

    # create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        self.trackers.append(trk)
    i = len(self.trackers)
    for trk in reversed(self.trackers):
        d = trk.get_state()[0]
        # trk.hit_streak >= self.min_hits则track被视为confirmed态，才被视为一个有效的跟踪目标，才能添加到结果当中
        # trk.time_since_update == MAX_AGE是临界态，临界态还会附加到ret当中
        # 若下次匹配成功则重新累积trk.hit_streak，相当于要经历一个unconfirmed -> confirmed的过程，
        # 在此过程中track bbox将不会显示；若下次匹配失败则被删除
        # 这意味着unmatched track最多保留MAX_AGE帧
        # if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
        if (trk.time_since_update <= MAX_AGE) and (trk.hit_streak >= self.min_hits):
          ret.append(np.concatenate((d,[trk.id+1])).reshape(1,-1)) # +1 as MOT benchmark requires positive
          if (trk.time_since_update>0):
            print("id: %d track after predict() reserved by supplementing (missed detection)/(unmatched track)"%i)
        i -= 1
        # remove dead tracklet
        # if(trk.time_since_update > self.max_age):
        if(trk.time_since_update > MAX_AGE):
          self.trackers.pop(i)
    if(len(ret)>0):
      return np.concatenate(ret)
    return np.empty((0,5))

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='SORT demo')
    parser.add_argument('--display', dest='display', help='Display online tracker output (slow) [False]',action='store_true')
    parser.add_argument("--seq_path", help="Path to detections.", type=str, default='data')
    parser.add_argument("--phase", help="Subdirectory in seq_path.", type=str, default='train')
    parser.add_argument("--max_age", 
                        help="Maximum number of frames to keep alive a track without associated detections.", 
                        type=int, default=1)
    parser.add_argument("--min_hits", 
                        help="Minimum number of associated detections before track is initialised.", 
                        type=int, default=3)
    parser.add_argument("--iou_threshold", help="Minimum IOU for match.", type=float, default=0.3)
    args = parser.parse_args()
    return args

if __name__ == '__main__':
  # all train
  args = parse_args()
  display = args.display
  phase = args.phase
  total_time = 0.0
  total_frames = 0
  colours = np.random.rand(32, 3) #used only for display
  font = {'family': 'Arial', 'size': 12}  # 指定字体和大小
  if(display):
    if not os.path.exists('mot_benchmark'):
      print('\n\tERROR: mot_benchmark link not found!\n\n    Create a symbolic link to the MOT benchmark\n    \
        (https://motchallenge.net/data/2D_MOT_2015/#download). E.g.:\n\n    $ ln -s /path/to/MOT2015_challenge/2DMOT2015 mot_benchmark\n\n')
      exit()
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(111, aspect='equal')

  if not os.path.exists('output'):
    os.makedirs('output')
  pattern = os.path.join(args.seq_path, phase, '*', 'det', 'det.txt')
  seq_cnt=0
  seq_num_to_proc=1
  for seq_dets_fn in glob.glob(pattern):
    if(seq_cnt<seq_num_to_proc):
      mot_tracker = Sort(max_age=args.max_age, 
                        min_hits=args.min_hits,
                        iou_threshold=args.iou_threshold) #create instance of the SORT tracker
      seq_dets = np.loadtxt(seq_dets_fn, delimiter=',')
      seq = seq_dets_fn[pattern.find('*'):].split(os.path.sep)[0]
      '''
      print("============================")
      print(pattern.find('*'))
      print(seq_dets_fn[pattern.find('*'):])
      print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
      # 打印结果为：
      ============================
      11
      ADL-Rundle-6\det\det.txt
      !!!!!!!!!!!!!!!!!!!!!!!!!!!!
      '''
      
      with open(os.path.join('output', '%s.txt'%(seq)),'w') as out_file:
        print("Processing %s."%(seq))
        for frame in range(int(seq_dets[:,0].max())):
          frame += 1 #detection and frame numbers begin at 1
          dets = seq_dets[seq_dets[:, 0]==frame, 2:7]
          dets[:, 2:4] += dets[:, 0:2] #convert to [x1,y1,w,h] to [x1,y1,x2,y2]
          total_frames += 1

          if(display):
            fn = os.path.join('mot_benchmark', phase, seq, 'img1', '%06d.jpg'%(frame))
            im =io.imread(fn)
            height,width,_=im.shape
            ax1.imshow(im)
            plt.title(seq + ' Tracked Targets')

          start_time = time.time()
          trackers = mot_tracker.update(dets)
          cycle_time = time.time() - start_time
          total_time += cycle_time

          for d in trackers:
            # d[4]表示tracker的id，从1计起
            print('%d,%d,%.2f,%.2f,%.2f,%.2f,1,-1,-1,-1'%(frame,d[4],d[0],d[1],d[2]-d[0],d[3]-d[1]),file=out_file)
            if(display):
              d = d.astype(np.int32)
              ax1.add_patch(patches.Rectangle((d[0],d[1]),d[2]-d[0],d[3]-d[1],fill=False,lw=1,ec=colours[d[4]%32,:])) #用三个浮点数来指定颜色
              x_text=min(max(0,d[0]),width)
              y_text=min(max(0,d[1]-20),height)
              ax1.text(x_text,y_text,"%d"%d[4],color=colours[d[4]%32,:],fontdict=font)

          if(display):
            fig.canvas.flush_events()
            plt.draw()
            ax1.cla()
            time.sleep(0.05)
      seq_cnt+=1

  print("Total Tracking took: %.3f seconds for %d frames or %.1f FPS" % (total_time, total_frames, total_frames / total_time))

  if(display):
    print("Note: to get real runtime results run without the option: --display")
  
  # just for test
  tracker_hyc = KalmanBoxTracker([20,40,100,200,0.66])
  print("!!!basic info!!!")
  print("F",tracker_hyc.kf.F, "H",tracker_hyc.kf.H, "R",tracker_hyc.kf.R, "P",tracker_hyc.kf.P, 
        "Q",tracker_hyc.kf.Q, "x",tracker_hyc.kf.x,sep="\n")
  print("???test???")
  print(tracker_hyc.predict())
  print(tracker_hyc.kf.x)
  print(np.dot(tracker_hyc.kf.F,tracker_hyc.kf.x))
  tracker_hyc.update([19,25,120,210])
  print(tracker_hyc.get_state())
  print(tracker_hyc.kf.x)
  print("###another test###")
  tracker_hyc = KalmanBoxTracker([20,40,100,200,0.66])
  tracker_hyc.kf.x[4:]=[[2],[4],[3]]
  print(tracker_hyc.predict())
  print(tracker_hyc.kf.x)
  print(convert_x_to_bbox(np.dot(tracker_hyc.kf.F,tracker_hyc.kf.x)))
  print("===上面的打印信息应与下面的保持一致===")
  print(tracker_hyc.predict())
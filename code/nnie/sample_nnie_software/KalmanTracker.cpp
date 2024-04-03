#include "KalmanTracker.h"
#include <cmath>

int KalmanTracker::kf_cnt = 0;
float KalmanTracker::_focused_time_total=0.0;

KalmanTracker::KalmanTracker(cv::Rect_<float> initState, int cls)
{
    m_time_since_update = 0;
    m_hit_streak = 0;
    m_hit_streak_init = 0;
    m_age = 0;
    m_id = -1; //-1代表该tracker还未初始化，还不能赋予它id值
    m_cls = cls;
    m_accumulate_first_flag =false;
    m_focused_time_total = 0;
    m_live_time = 0;
    memset(m_focused_time, 0, sizeof(m_focused_time));
    init_kf(initState);
}

void KalmanTracker::init_kf(cv::Rect_<float> stateRect)
{
    int stateDim=7;
    int measureDim=4;
    int controlDim=0;
    //默认int type=CV_32F（值为5）
    kf = cv::KalmanFilter(stateDim, measureDim, controlDim);

    measurement = cv::Mat::zeros(measureDim, 1, CV_32F);

    kf.transitionMatrix = (cv::Mat_<float>(stateDim, stateDim) << 
        1, 0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 0, 1,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1);
    kf.measurementMatrix = (cv::Mat_<float>(measureDim, stateDim) <<
        1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0);
    kf.processNoiseCov = (cv::Mat_<float>(stateDim, stateDim) <<
        1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 
        0, 0, 0, 1, 0, 0, 0, 
        0, 0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0, 0.0001);
    kf.measurementNoiseCov = (cv::Mat_<float>(measureDim, measureDim) <<
        1, 0, 0,  0,
        0, 1, 0,  0,
        0, 0, 10, 0,
        0, 0, 0,  10);
    kf.errorCovPost = (cv::Mat_<float>(stateDim, stateDim) <<
        10, 0, 0, 0, 0, 0, 0, 
        0, 10, 0, 0, 0, 0, 0, 
        0, 0, 10, 0, 0, 0, 0, 
        0, 0, 0, 10, 0, 0, 0, 
        0, 0, 0, 0, 10000, 0, 0, 
        0, 0, 0, 0, 0, 10000, 0,
        0, 0, 0, 0, 0, 0, 10000);
    kf.statePost.at<float>(0,0) = stateRect.x + stateRect.width/2;
    kf.statePost.at<float>(1,0) = stateRect.y + stateRect.height/2;
    kf.statePost.at<float>(2,0) = stateRect.area();
    kf.statePost.at<float>(3,0) = stateRect.width/stateRect.height;
}

cv::Rect_<float> KalmanTracker::predict()
{
    if((kf.statePost.at<float>(2,0) + kf.statePost.at<float>(6,0)) <= 0)
        kf.statePost.at<float>(6,0) = 0.0;
    
    cv::Mat p = kf.predict();
    m_age += 1;

    // 初始化阶段，在达到MIN_HITS之前若出现过一次匹配失败就将hit_streak置零，
    // 相当于重新开始初始化的过程，但用到的bbox信息是之前的还没被删掉的track bbox
    if(m_id==-1){
        if(m_time_since_update>0){
            m_hit_streak=0;
            m_hit_streak_init=0;
        }
    }
    else if(m_time_since_update>MAX_AGE_SORT-1)
        m_hit_streak=0;
    
    // 这种策略要求更低一些，不要求连续成功匹配MIN_HITS次才能将bbox成功初始化
    // 与Trivial_Track_Yolov2()的策略类似
    // if(m_time_since_update>MAX_AGE_SORT-1){
    //     m_hit_streak = 0;
    //     if(m_id==-1)
    //         m_hit_streak_init = 0;
    // }

    m_time_since_update += 1;

    return xysr2cvRect(p.at<float>(0,0) ,p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));
}

void KalmanTracker::update(cv::Rect_<float> measureRect)
{
    m_time_since_update = 0;
    m_hit_streak += 1;
    if(m_id == -1){
        m_hit_streak_init+=1;
        if(m_hit_streak_init>=MIN_HITS_SORT){
            m_id = get_tracker_num();
            add_tracker_num();
        }
    }
    measurement.at<float>(0, 0) = measureRect.x + measureRect.width/2;
    measurement.at<float>(1, 0) = measureRect.y + measureRect.height/2;
    measurement.at<float>(2, 0) = measureRect.area();
    measurement.at<float>(3, 0) = measureRect.width/measureRect.height;
    
    kf.correct(measurement);
}

cv::Rect_<float> KalmanTracker::get_state()
{
    return xysr2cvRect(kf.statePost.at<float>(0,0), kf.statePost.at<float>(1,0),
        kf.statePost.at<float>(2,0), kf.statePost.at<float>(3,0));
}

int KalmanTracker::get_tracker_num()
{
    return kf_cnt;
}

void KalmanTracker::add_tracker_num()
{
    kf_cnt++;
}

float KalmanTracker::get_global_focused_time_total()
{
    return _focused_time_total;
}

void KalmanTracker::accumulate_global_focused_time_total(float focused_time)
{
    _focused_time_total+=focused_time;
}

cv::Rect_<float> KalmanTracker::xysr2cvRect(float xc,float yc, float s, float r)
{
    float w = std::sqrt(s * r);
    float h = s/w;
    float xmin = xc - w/2;
    float ymin = yc - h/2;
    // float xmax = xc + w/2;
    // float ymax = yc + h/2;
    // return cv::Rect_<float>(xmin, ymin, xmax, ymax);
    if (xmin < 0 && xc > 0)
		xmin = 0;
	if (ymin < 0 && yc > 0)
		ymin = 0;
    return cv::Rect_<float>(xmin, ymin, w, h);
}

void KalmanTracker::SORT_Accumulate_Focused_Time(int frame_circular)
{
    int i;
    int focused_count=0;    
    if(m_accumulate_first_flag == false){
        float sum = 0;
        for(i=0; i<FOCUS_LEN_SORT; i++){
            if(m_focused_time[i]>0){
                sum += m_focused_time[i];
                focused_count++;
            }
        }
        if(focused_count>=FOCUS_THRESH_SORT){
            accumulate_global_focused_time_total(sum);
            m_focused_time_total += sum;
            m_accumulate_first_flag = true;
        }
    }
    else{
        // int frame_circular = frame % FOCUS_LEN_SORT;
        for(i=0; i<FOCUS_LEN_SORT; i++)
            if(m_focused_time[i]>0)
                focused_count++;
        if(focused_count>=FOCUS_THRESH_SORT){
            accumulate_global_focused_time_total(m_focused_time[frame_circular]);
            m_focused_time_total+=m_focused_time[frame_circular];
        }
    }
}
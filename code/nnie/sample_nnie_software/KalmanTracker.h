#ifndef KALMAN
#define KALMAN

#ifdef __cplusplus
extern "C" {
#endif

#include "opencv2/video/tracking.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include <iostream>
// #include <vector>

// using namespace std;

#define IOU_THRESH_SORT 0.36 //detection bbox和track bbox的IOU需要大于这一阈值才有效
//与原始的sort算法略有不同的地方，这样处理可以降低误识别/跟踪率或生成的总id数
#define MAX_AGE_SORT 6 //unmatched tracks几帧后被丢弃
#define MIN_HITS_SORT 2 //unmatched detections不能马上成为一个有效的id，而是需要经过一个初始化的过程，在初始化阶段，
// track和detection要连续成功匹配2帧（因此至少需要3帧才能完成初始化，第一帧是第一次出现的unmatched detection）才认为该track转为confirmed态，赋予其一个新的id值
#define FOCUS_LEN_SORT 25
#define FOCUS_THRESH_SORT 12

class KalmanTracker
{
    public:
        KalmanTracker(cv::Rect_<float> initState, int cls);
        ~KalmanTracker();
        cv::Rect_<float> predict();
        void update(cv::Rect_<float> measureRect);
        cv::Rect_<float> get_state();
        cv::Rect_<float> xysr2cvRect(float xc,float yc, float s, float r);
        static int get_tracker_num();
        static void add_tracker_num();
        static float get_global_focused_time_total();
        static void accumulate_global_focused_time_total(float focused_time);
        void SORT_Accumulate_Focused_Time(int frame);
        int m_cls;
        float m_focused_time[FOCUS_LEN_SORT]; //循环利用
        bool m_accumulate_first_flag;
        float m_focused_time_total;
        float m_live_time;
        int m_time_since_update;
        // int m_hits;
        int m_hit_streak;
        int m_hit_streak_init;
        int m_age;
        int m_id;
    private:
        void init_kf(cv::Rect_<float> stateRect);
        static int kf_cnt;
        static float _focused_time_total;
        cv::KalmanFilter kf;
        //Last measurement used in update()/correct(). Read only.
        cv::Mat measurement;
        // std::vector<cv::Rect_<float>> m_history;
};

#ifdef __cplusplus
}
#endif
#endif
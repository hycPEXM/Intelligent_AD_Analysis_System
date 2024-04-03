#include "KalmanTracker.h"
#include "Hungarian.h"
#include "SORT.h"
// #include <iostream>

// <algorithm> 提供了 std::set_difference 函数的声明，<iterator> 提供了 std::insert_iterator 的声明，
// 而 <set> 提供了 std::set 的声明。
#include <algorithm>  // 包含算法库
#include <iterator>   // 包含迭代器库
#include <set>        // 包含集合库
#include <ctime>
#include <cfloat>

double GetIOU(cv::Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

// 将Rect/ROI的宽高限制在BOX_MAX_WIDTH * BOX_MAX_HEIGHT内，应该是在最后将值传给SORT_ID时才进行该处理
//cv::Rect_<float> make_valid_cvRect(cv::Rect_<float> rect)
RectBox make_valid_RectBox(cv::Rect_<float> rect) //本身就有cvRect_to_RectBox()的作用
{
    int xmin = (int)rect.x;
    int ymin = (int)rect.y;
    int xmax = (int)(rect.x + rect.width);
    int ymax = (int)(rect.y + rect.height);
    xmin = std::max(0,xmin);
    ymin = std::max(0,ymin);
    xmax = std::min(xmax, BOX_MAX_WIDTH-1);
    ymax = std::min(ymax, BOX_MAX_HEIGHT-1);
    RectBox validBox = {xmin, ymin, xmax, ymax};
    return validBox;
}

// cv::Rect_: x,y,w,h  (Rect)
// RectBox: xyxy       (ROI)
cv::Rect_<float> DetectObjInfo_to_cvRect(DetectObjInfo obj)
{
    //这一步没必要make_valid_cvRect，检测得到的box肯定是有效的，这部分应该在模型推理部分就应该保证了
    return cv::Rect_<float>((float)obj.box.xmin,(float)obj.box.ymin,
        (float)(obj.box.xmax-obj.box.xmin),(float)(obj.box.ymax-obj.box.ymin));
}

std::vector<KalmanTracker> trackers;
KalmanTracker::kf_count = 0; 
KalmanTracker::_focused_time_total=0.0;

std::vector<cv::Rect_<float>> predictedBoxes;
std::vector<cv::Rect_<float>> detectedBoxes;
std::vector<std::vector<double>> iouMatrix;
std::vector<int> assignment;
std::set<int> unmatchedDetections;
std::set<int> unmatchedTrajectories;
std::set<int> allItems;
std::set<int> matchedItems;
std::vector<cv::Point> matchedPairs;
// std::vector<TrackingBox> frameTrackingResult;

int Sort_Track_Yolov2(DetectObjInfo resBuf[], int resLen, float usedTime, SORT_ID ID_arr[], int maxBoxNum,int validBoxNum, HI_U32 frame, HI_FLOAT *AD_attractiveness_dead)
{
    int i, j; //temporary index
    int validBoxNum_curr=0; //当前帧的有效box数量
    cv::Rect_<float> resRect;
    int frame_circular = frame % FOCUS_LEN_SORT;
	unsigned int trkNum = 0;
	// unsigned int detNum = 0; // 即resLen
    struct timespec start_time, end_time;
    // float track_used_time = 0;

    std::clock_gettime(CLOCK_REALTIME, &start_time);

    if(trackers.size() == 0){
        for(i=0;i<resLen;i++){
            resRect = DetectObjInfo_to_cvRect(resBuf[i]);
            KalmanTracker trk = KalmanTracker(resRect,resBuf[i].cls);
            trackers.push_back(trk);
        }
    }
    else{
        predictedBoxes.clear();
        detectedBoxes.clear()
        for (auto it = trackers.begin(); it != trackers.end(); ){
            cv::Rect_<float> predBox = (*it).predict();
            if (predBox.x >= 0 && predBox.y >= 0){
                predictedBoxes.push_back(predBox);
                it++;
            }
            else
                it = trackers.erase(it);
        }
        trkNum = predictedBoxes.size();
        iouMatrix.clear();
        iouMatrix.resize(trkNum, std::vector<double>(resLen, 0));
        for(j=0; j<resLen; j++){
            detectedBoxes.push_back(DetectObjInfo_to_cvRect(resBuf[j]));
        }
        for(i=0; i<trkNum; i++){
            for(j=0; j<resLen; j++){
                iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detectedBoxes[j]);
            }
        }
        HungarianAlgorithm HungAlgo;
        assignment.clear();
        HungAlgo.Solve(iouMatrix, assignment);

        unmatchedTrajectories.clear();
        unmatchedDetections.clear();
        allItems.clear();
        matchedItems.clear();

        if(resLen > trkNum)  // unmatched detections exist!
        {
            for(i=0; i<resLen; i++)
                allItems.insert(i);
            for(i=0; i<trkNum; i++)
                matchedItems.insert(assignment[i]);
            std::set_difference(allItems.begin(), allItems.end(), matchedItems.begin(), matchedItems.end(),
                std::insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
        }
        else if(resLen < trkNum){
            for(i = 0; i < trkNum; i++)
                if(assignment[i] == -1)
                    unmatchedTrajectories.insert(i);
        }

        matchedPairs.clear();
        for (i=0; i<trkNum; i++){
            if(assignment[i] == -1)
                continue;
            if(1 - iouMatrix[i][assignment[i]] < IOU_THRESH_SORT){
                unmatchedTrajectories.insert(i);
                unmatchedDetections.insert(assignment[i]);
            }
            else
                matchedPairs.push_back(cv::Point(i, assignment[i]));
        }
        for(i=0; i<matchedPairs.size(); i++){
            trackers[matchedPairs[i].x].update(detectedBoxes[matchedPairs[i].y]);
        }

        for (auto det : unmatchedDetections){
            KalmanTracker trk = KalmanTracker(detectedBoxes[det], resBuf[det].cls);
            trackers.push_back(trk);
        }         
    }

    std::clock_gettime(CLOCK_REALTIME, &end_time);
    usedTime += (float)(end_time.tv_sec - start_time.tv_sec)*1000 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
    
    for(auto it=trackers.begin(); it!=trackers.end();){
        if( it->m_time_since_update > MAX_AGE_SORT ){
            if(it->m_id>=0)    
                *AD_attractiveness_dead += it->m_focused_time_total/it->m_live_time ;
            it = trackers.erase(it);
        }
        else{
            it->m_live_time += usedTime;
            if(it->cls==1)
                it->m_focused_time[frame_circular]=0;
            else if(it->cls==2)
                it->m_focused_time[frame_circular]=usedTime;
        
            if( ((*it).m_time_since_update<=MAX_AGE_SORT) && ((*it).m_hit_streak >= MIN_HITS_SORT) ){
                //与Trivial算法不同，这里KalmanTracker用于匹配detection，ID_arr只用来存放那些有效的跟踪box，所以统计注意力时间应该交给KalmanTracker来完成
                ID_arr[validBoxNum_curr].box = make_valid_RectBox((*it).get_state());
                ID_arr[validBoxNum_curr].ID = it->m_id;
                ID_arr[validBoxNum_curr].cls = it->m_cls;
                it->SORT_Accumulate_Focused_Time(frame_circular);            
                ID_arr[validBoxNum_curr].focused_time_total = it->m_focused_time_total;
                ID_arr[validBoxNum_curr].live_time = it->m_live_time;
                validBoxNum_curr++;
                if(validBoxNum_curr>=maxBoxNum)
                    break;
                it++;
            }
            else 
                it++;        
        }                   
    }
    for(i=validBoxNum_curr; i<validBoxNum; i++)
        memset(&ID_arr[i], 0, sizeof(SORT_ID));
    return validBoxNum_curr;
}   

float global_focused_time_total()
{
    return KalmanTracker::get_global_focused_time_total();
}

int SORT_ID_num_total()
{
    return KalmanTracker::get_tracker_num();
}
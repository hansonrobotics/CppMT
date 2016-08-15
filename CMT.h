#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"
#include "CTimer.h"

#include <iostream>
#include <opencv2/features2d/features2d.hpp>

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;

namespace cmt
{
struct cmt_timing{
    double process_time;
    double initialization_time;
};

class CMT
{
public:
    CMT() : str_detector("FAST"), str_descriptor("BRISK"), identified(false), tracker_lost(false), initialized(false), validated(false), name("unset") ,
              counter(5),decreasing_validate(500){};
    void initialize(const Mat im_gray, const Rect rect, string tracker_name, int threshold=50);
    void processFrame(const Mat im_gray);
    void set_name(string recognized); //TODO this should be a feature in helper
    void reset_decreasing_validate(int value);//TODO this should be a feature in helper
    //Calls the intialize with the existing values. But maintains the previous attributes.
    void updateArea(const Mat im_gray, const Rect rect);//TODO this should be a feature in helper

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    string str_detector;
    string str_descriptor;

    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;

    Mat imArchive;
    vector<Point2f>pointsArchive;
    vector<int>classesArchive;
    Rect initialRect;

	//Modification to the system
	bool identified;
	bool tracker_lost;

    bool initialized;
    bool validated;

    string name;
    string recognized_as;

	int num_initial_keypoints;
	int num_active_keypoints;
    int threshold;
    int counter;
    int decreasing_validate;
    int initial_default;
    int ratio_frames;

    cmt_timing times;
private:
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    Size2f size_initial;

    vector<int> classes_active;

    float theta;

    Mat im_prev;
};

} /* namespace CMT */

#endif /* end of include guard: CMT_H */

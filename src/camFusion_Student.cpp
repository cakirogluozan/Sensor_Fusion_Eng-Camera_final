
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(100,250), rng.uniform(100, 250), rng.uniform(100, 250));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBoxes, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for (auto curr_it = kptsCurr.begin(); curr_it != kptsCurr.end(); ++curr_it)
    {
        if (boundingBoxes.roi.contains((*curr_it).pt))
        {
            boundingBoxes.keypoints.push_back((*curr_it));
        }
    }

    for (auto match_it = kptMatches.begin(); match_it != kptMatches.end(); ++match_it)
    {
        if (boundingBoxes.roi.contains(kptsCurr[(*match_it).trainIdx].pt))
        {
            boundingBoxes.kptMatches.push_back((*match_it));
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distance_ratios; 
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 

        cv::KeyPoint out_current_keypoint  = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint out_previous_keypoint = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { 
            double min_dist = 100.0; 

            cv::KeyPoint in_current_keypoint = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint in_previous_keypoint = kptsPrev.at(it2->queryIdx);

            double current_distance  = cv::norm(out_current_keypoint.pt  - in_current_keypoint.pt);
            double previous_distance = cv::norm(out_previous_keypoint.pt - in_previous_keypoint.pt);

            if (previous_distance > std::numeric_limits<double>::epsilon() && current_distance >= min_dist)
            { 
                distance_ratios.push_back(current_distance / previous_distance);
            }
        } 
    }     

    double mean_distance_ratio = std::accumulate(distance_ratios.begin(), distance_ratios.end(), 0.0) / distance_ratios.size();

    double dT = 1 / frameRate;

    size_t size = distance_ratios.size();
    sort(distance_ratios.begin(), distance_ratios.end());
    double median_distance_ratio;
    if (size % 2 == 0)
    {
      median_distance_ratio = (distance_ratios[size / 2 - 1] + distance_ratios[size / 2]) / 2;
    }
    else 
    {
      median_distance_ratio = distance_ratios[size / 2];
    }
    if (median_distance_ratio == 1) {
        TTC = -1;
        return;
    }
    //TTC = -dT / (1 - mean_distance_ratio);
    TTC = -dT / (1 - median_distance_ratio);
}





void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1/frameRate; // time between two measurements in seconds
    double lane_width = 4.4; // m
    // find closest distance to Lidar points 
    vector<double> curr_lidar_in_lane;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) 
    {
        if (it->y > -lane_width/2 && it->y < lane_width/2) 
        {
            curr_lidar_in_lane.push_back(it->x);
        }
    }

    vector<double> prev_lidar_in_lane;
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) 
    {
        if (it->y > -lane_width/2 && it->y < lane_width/2) 
        {
            prev_lidar_in_lane.push_back(it->x);
        }    
    }

    double mean_x_curr = std::accumulate(curr_lidar_in_lane.begin(), curr_lidar_in_lane.end(), 0.0) / curr_lidar_in_lane.size();
    double mean_x_prev = std::accumulate(prev_lidar_in_lane.begin(), prev_lidar_in_lane.end(), 0.0) / prev_lidar_in_lane.size();
    // compute TTC from both measurements

    size_t size = curr_lidar_in_lane.size();
    sort(curr_lidar_in_lane.begin(), curr_lidar_in_lane.end());
    double med_x_curr = 0;

    if (size % 2 == 0)
    {
      med_x_curr = (curr_lidar_in_lane[size / 2 - 1] + curr_lidar_in_lane[size / 2]) / 2;
    }
    else 
    {
      med_x_curr = curr_lidar_in_lane[size / 2];
    }


    size = prev_lidar_in_lane.size();
    sort(prev_lidar_in_lane.begin(), prev_lidar_in_lane.end());
    double med_x_prev = 0;

    if (size % 2 == 0)
    {
      med_x_prev = (prev_lidar_in_lane[size / 2 - 1] + prev_lidar_in_lane[size / 2]) / 2;
    }
    else 
    {
      med_x_prev = prev_lidar_in_lane[size / 2];
    }
    
    // compute TTC from both measurements
    TTC = med_x_curr * dT / std::abs(med_x_prev - med_x_curr);
    // TTC = mean_x_curr * dT / std::abs(mean_x_curr - mean_x_prev);

    return;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int all_bbMatches[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {0};

    for (auto it_matches = matches.begin(); it_matches != matches.end(); ++it_matches)
    {

        int prev_id = -1;
        for (auto prev_it = prevFrame.boundingBoxes.begin(); prev_it != prevFrame.boundingBoxes.end(); ++prev_it)
        {
            if ((*prev_it).roi.contains(prevFrame.keypoints[(*it_matches).queryIdx].pt))
            {
                prev_id = (*prev_it).boxID;
            }
        }

        int curr_id = -1;
        for (auto curr_it = currFrame.boundingBoxes.begin(); curr_it != currFrame.boundingBoxes.end(); ++curr_it)
        {
            if ((*curr_it).roi.contains(currFrame.keypoints[(*it_matches).trainIdx].pt))
            {
                curr_id = (*curr_it).boxID;
            }
        }

        if ((prev_id != -1) && (curr_id != -1))
        {
            all_bbMatches[prev_id][curr_id]++;
        }

    }
    

    for (int i = 0; i < prevFrame.boundingBoxes.size(); i++)
    {
        int best_match = -1;
        for (int j = 0; j < currFrame.boundingBoxes.size(); j++) 
        {
            if (all_bbMatches[i][j] > best_match)
            {
                bbBestMatches[i] = j;
                best_match = all_bbMatches[i][j];
            }
        }
    }
}
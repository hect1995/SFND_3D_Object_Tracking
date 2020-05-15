
#include <iostream>
#include <algorithm>
#include <numeric>
#include <math.h>
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
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

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

int accumulateDistances(int lhs, const cv::DMatch& rhs)
{
    return lhs + rhs.distance;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> all_keypoints_box;
    for (int i=0; i<kptMatches.size(); i++){
        int coord_x = kptsPrev[kptMatches[i].trainIdx].pt.x;
        int coord_y = kptsPrev[kptMatches[i].trainIdx].pt.y;
        if (coord_x >= boundingBox.roi.x && coord_x < boundingBox.roi.x + boundingBox.roi.width &&
            coord_y >= boundingBox.roi.y && coord_y < boundingBox.roi.y + boundingBox.roi.height)
        {
            all_keypoints_box.push_back(kptMatches[i]);
        }
    }
    float total_dist;
    for (int i=0; i<all_keypoints_box.size(); i++)
    {
        total_dist += kptsPrev[all_keypoints_box[i].imgIdx].pt.x;
    }
    //float total_dist = //std::accumulate(all_keypoints_box.begin(), all_keypoints_box.end(), 0, accumulateDistances);
    float average = static_cast<float>(total_dist)/all_keypoints_box.size();
    float stndrd_dev = 0;
    for (int i=0; i<all_keypoints_box.size(); i++)
    {
        stndrd_dev += std::pow(kptsPrev[all_keypoints_box[i].imgIdx].pt.x - average,2);
    }
    stndrd_dev = sqrt(stndrd_dev/(all_keypoints_box.size()-1));
    for (int i=0; i<all_keypoints_box.size(); i++)
    {
        if (kptsPrev[all_keypoints_box[i].imgIdx].pt.x > 2*stndrd_dev + average || kptsPrev[all_keypoints_box[i].imgIdx].pt.x < -2*stndrd_dev + average){
            all_keypoints_box.erase(all_keypoints_box.begin()+i);
            i--;
        }
    }
    boundingBox.kptMatches = all_keypoints_box;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    /*vector<double> distRatios;
    for (int i=0; i<kptMatches.size()-1; i++)
    {
        cv::KeyPoint prev = kptsPrev[kptMatches[i].trainIdx];
        cv::KeyPoint curr = kptsCurr[kptMatches[i].queryIdx];
        for (int j=0; j<kptMatches.size(); j++)
        {
            cv::KeyPoint prev_other = kptsPrev[kptMatches[j].trainIdx];
            cv::KeyPoint curr_other = kptsCurr[kptMatches[j].queryIdx];

            // Compute distances for previous and current frame
            double distCurr = cv::norm(curr.pt - curr_other.pt);
            double distPrev = cv::norm(prev.pt - prev_other.pt);
            double minDist = 100.0; // Threshold the calculated distRatios by requiring a minimum current distance

            // Avoid division by zero and apply the threshold
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                distRatios.push_back(distCurr / distPrev);
            }
        }
    }
    std::sort(distRatios.begin(),distRatios.end());
    TTC = -1/(frameRate*(1-distRatios[distRatios.size()/2]));*/
    double dt = 1.0 / frameRate;
    // Compute distance ratios on every pair of keypoints, O(n^2) on the number of matches contained within the ROI
    vector<double> distRatios;
    for (auto itrFirstMatch = kptMatches.begin(); itrFirstMatch != kptMatches.end() - 1; ++itrFirstMatch)
    {
        // First points in previous and current frame
        cv::KeyPoint kptFirstCurr = kptsCurr.at(itrFirstMatch->trainIdx);
        cv::KeyPoint kptFirstPrev = kptsPrev.at(itrFirstMatch->queryIdx);
 
        for (auto itrSecondMatch = kptMatches.begin() + 1; itrSecondMatch != kptMatches.end(); ++itrSecondMatch)
        {
            cv::KeyPoint kptSecondCurr = kptsCurr.at(itrSecondMatch->trainIdx);
            cv::KeyPoint kptSecondPrev = kptsPrev.at(itrSecondMatch->queryIdx);

            // Compute distances for previous and current frame
            double distCurr = cv::norm(kptFirstCurr.pt - kptSecondCurr.pt);
            double distPrev = cv::norm(kptFirstPrev.pt - kptSecondPrev.pt);

            double minDist = 100.0; // Threshold the calculated distRatios by requiring a minimum current distance

            // Avoid division by zero and apply the threshold
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    // Only continue if the vector of distRatios is not empty
    if (distRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // Use the median to exclude outliers
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio = distRatios[distRatios.size() / 2];

    TTC = -dt / (1 - medianDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    std::sort(lidarPointsPrev.begin(),lidarPointsPrev.end());
    std::sort(lidarPointsCurr.begin(),lidarPointsCurr.end());
    double median_prev = lidarPointsPrev[lidarPointsPrev.size()/2].x;
    double median_curr = lidarPointsCurr[lidarPointsCurr.size()/2].x;
    TTC = median_prev > median_curr ? median_curr/(frameRate * (median_prev-median_curr)) : 99999999;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // bbBestMatches at the beginning empty
    std::map<std::vector<int>,int> counter_map;
    for (auto match : matches)
    {
        std::vector<int> roi_beg; 
        std::vector<int> roi_end;

        for (auto bbox : prevFrame.boundingBoxes)
        {
            if (prevFrame.keypoints[match.queryIdx].pt.x >= bbox.roi.x && prevFrame.keypoints[match.queryIdx].pt.x < bbox.roi.x + bbox.roi.width &&
                prevFrame.keypoints[match.queryIdx].pt.y >= bbox.roi.y && prevFrame.keypoints[match.queryIdx].pt.y < bbox.roi.y + bbox.roi.height)
            {
                roi_beg.push_back(bbox.boxID);
            }
        }
        for (auto bbox : currFrame.boundingBoxes)
        {
            if (currFrame.keypoints[match.trainIdx].pt.x >= bbox.roi.x && currFrame.keypoints[match.trainIdx].pt.x < bbox.roi.x + bbox.roi.width &&
                currFrame.keypoints[match.trainIdx].pt.y >= bbox.roi.y && currFrame.keypoints[match.trainIdx].pt.y < bbox.roi.y + bbox.roi.height)
            {
                roi_end.push_back(bbox.boxID);
            }
        }
        for (auto prev_frame : roi_beg)
        {
            for (auto next_frame : roi_end)
            {
                if ( counter_map.find({prev_frame,next_frame}) == counter_map.end() ) {
                    // not found
                    counter_map[{prev_frame,next_frame}] = 1;
                } else {
                    // found
                    counter_map[{prev_frame,next_frame}] += 1;
                }
            }
        }
    }
    std::map<int,int> number_elements;
    std::map<int,int> partner;
    for (auto combination : counter_map){
        if (number_elements.find(combination.first[0]) != number_elements.end())
        {
            if (combination.second > number_elements[combination.first[0]])
            {
                number_elements[combination.first[0]] = combination.second;
                partner[combination.first[0]] = combination.first[1];
            }
        }
        else
        {
            number_elements.insert({combination.first[0],combination.second});
            partner.insert({combination.first[0],combination.first[1]});
        }
    }
    int min_number_points = 10;
    for (auto elem : number_elements)
    {
        if (elem.second > min_number_points)
        {
            bbBestMatches.insert({elem.first,partner[elem.first]});
        }
    }
}

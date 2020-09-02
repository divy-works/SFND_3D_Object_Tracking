
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/xfeatures2d.hpp>

#include <fstream>
#include <algorithm>

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


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, int imgIndex, bool bVis)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        if (it1->lidarPoints.size() > 15)
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
            putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 1, currColor, 2);
            sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
            putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 1, currColor, 2);
            cv::putText(topviewImg, "current image idx=" + std::to_string(imgIndex), cv::Point(30 , 30), cv::FONT_ITALIC, 1, currColor, 2);
        }

    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    //imwrite("lidar_" + std::to_string(imgIndex) + ".png", topviewImg);

    if(bVis)
    {
        // display image
        string windowName = "3D Objects";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, topviewImg);
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches, cv::Mat &currImage, int imgIndex, bool bVis)
{
    std::cout << "Cluster Keypoint Matches within ROI" << std::endl;

    std::vector<cv::KeyPoint> kptsBeforeMeanFilter;

    std::vector<cv::DMatch> matchedKeypointsVect;
    std::vector<cv::KeyPoint> keypointsInROI;
    cv::DMatch matchedKeypoint;
    double distance, distanceSum = 0;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end(); ++it1)
    {

        //check if the keypoint corresponding t the trainIdx within the roi of the bounding box
         if (boundingBox.roi.contains(kptsCurr[it1->trainIdx].pt))
        {
            //the keypoint in the match is inside the roi of the bounding box
            distanceSum += cv::norm(kptsCurr[it1->trainIdx].pt - kptsPrev[it1->queryIdx].pt);
            matchedKeypointsVect.push_back(*it1);
            kptsBeforeMeanFilter.push_back(kptsCurr[it1->trainIdx]);
        }
    }
    double distanceMean = distanceSum/((double)matchedKeypointsVect.size());

    for (auto it2 = matchedKeypointsVect.begin(); it2 != matchedKeypointsVect.end(); ++it2)
    {
        if (it2->distance < distanceMean * 0.5)
        {
            boundingBox.kptMatches.push_back(*it2);
            boundingBox.keypoints.push_back(kptsCurr[it2->trainIdx]);
        }
    }

    if (bVis)
    {
        cv::rectangle(currImage, cv::Point(boundingBox.roi.x, boundingBox.roi.y), cv::Point(boundingBox.roi.x + boundingBox.roi.width, boundingBox.roi.y + boundingBox.roi.height), cv::Scalar(0, 255, 0), 2);
        cv::Mat imgAfterKptsFilter = currImage.clone();

        cv::drawKeypoints(currImage, boundingBox.keypoints, imgAfterKptsFilter, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::putText(imgAfterKptsFilter, "Keypoints after removing distant keypoints", cv::Point(20 , 20), cv::FONT_ITALIC, 0.5, cv::Scalar(0, 0, 255), 2);
        cv::drawKeypoints(currImage, kptsBeforeMeanFilter, currImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::putText(currImage,"Keypoints in ROI, current image index = " + std::to_string(imgIndex), cv::Point(20 , 20), cv::FONT_ITALIC, 0.5, cv::Scalar(0, 0, 255), 2);
        cv::Mat resultImg;
        cv::vconcat(currImage, imgAfterKptsFilter, resultImg);
        std::string windowName = "Before and After removing keypoints based on mean value";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, resultImg);
        //cv::imwrite("keypoints_roi_" + std::to_string(imgIndex) + ".png", resultImg);
        cv::waitKey(0);

    }
}

bool sortMedianDistRatios(double r1, double r2)
{
    return r1<r2;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector <double> distRatios;
    int queryIdx, trainIdx;
    cv::KeyPoint kpFirstCurr, kpSecondCurr, kpFirstPrev, kpSecondPrev;
    double distanceCurr, distancePrev, distRatio;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        kpFirstCurr = kptsCurr[it1->trainIdx];
        kpFirstPrev = kptsPrev[it1->queryIdx];

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
            kpSecondCurr = kptsCurr[it2->trainIdx];
            kpSecondPrev = kptsPrev[it2->queryIdx];

            distanceCurr = cv::norm(kpFirstCurr.pt - kpSecondCurr.pt);
            distancePrev = cv::norm(kpFirstPrev.pt - kpSecondPrev.pt);
            if (distancePrev > std::numeric_limits<double>::epsilon() && distanceCurr >= 100)
            {
                distRatio = distanceCurr / distancePrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    //sort distance ratios vector
    std::sort(distRatios.begin(), distRatios.end(), sortMedianDistRatios);
    double medianDistRatio;
    if (distRatios.size() % 2 == 0)//even number of elements
    {
        medianDistRatio = (distRatios[(distRatios.size() - 1)/2] + distRatios[(distRatios.size() + 1) / 2])/2.0;
    }
    else
    {
        medianDistRatio = distRatios[distRatios.size() / 2];
    }
    
    double dT = 1/frameRate;
    TTC = -dT / (1 - medianDistRatio);

    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream cameraMeasurmentFile;
        if (firstEntry)
        {
            cameraMeasurmentFile.open("CameraMeasurementFile.csv");
            cameraMeasurmentFile << "distance ratio , TTC" << std::endl;
            firstEntry = false;
        }
        else
        {
            cameraMeasurmentFile.open("CameraMeasurementFile.csv", std::ios::out | std::ios::app);
        }
        cameraMeasurmentFile << medianDistRatio << ", " << TTC << std::endl;
    #endif
}

bool compareLidarXValue(LidarPoint p1, LidarPoint p2)
{
    return (p1.x < p2.x);
}

double calcMedianLidarX(std::vector<LidarPoint> &lidarPoint)
{
    //sort lidar points using the x valud of the lidar data 
    std::sort(lidarPoint.begin(), lidarPoint.end(), compareLidarXValue);
    double medianX;
    if (lidarPoint.size() % 2 == 0) //odd number of elements
    {
        medianX = lidarPoint[lidarPoint.size()/2].x;
    }
    else
    {
        medianX = (lidarPoint[lidarPoint.size()/2].x + lidarPoint[lidarPoint.size()/2 + 1].x)/2;
    }
    return medianX;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC, int imgIndex)
{
    
    //find the median of the x-values, this will help in discarding outliers
    double medianLidarXPrev = calcMedianLidarX(lidarPointsPrev);
    double medianLidarXCurr = calcMedianLidarX(lidarPointsCurr);
    double dT = 1/frameRate;
    TTC = medianLidarXCurr * dT/(medianLidarXPrev - medianLidarXCurr);

    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream lidarMeasurmentFile;
        if (firstEntry)
        {
            lidarMeasurmentFile.open("lidarMeasurementFile.csv");
            lidarMeasurmentFile << "image index, min X curr, min X prev, median X curr, previous X curr" << std::endl;
            firstEntry = false;
        }
        else
        {
            lidarMeasurmentFile.open("lidarMeasurementFile.csv", std::ios::out | std::ios::app);
        }
        lidarMeasurmentFile << imgIndex << ", " << lidarPointsCurr[0].x << ", " << lidarPointsPrev[0].x << ", " << medianLidarXCurr << ", " << medianLidarXPrev << std::endl;
    #endif
}


bool boundingBoxIndex(std::vector<BoundingBox> boundingBoxes, cv::KeyPoint keypoint, unsigned int & boundingBoxIndex)
{
    for (auto it = boundingBoxes.begin(); it !=boundingBoxes.end(); ++it)
    {
        if (it->roi.contains(keypoint.pt))
        {
            
            boundingBoxIndex =  (unsigned int)it->boxID;
            return true;
        }
    }
    return false;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame, bool bVis, int imgIndex)
{

    //boundingBoxCombScore is a matrix whose rows are corresponding to the current frame bounding box indices 
    // and columns correspond to the previous frame bounding box indices
    cv::Mat boundingBoxCombScore(currFrame.boundingBoxes.size(), prevFrame.boundingBoxes.size(), CV_8U);
    boundingBoxCombScore.setTo(0); //set initial number of correspondances to zero


    int trainIdx, queryIdx; //trainIdx corresponds to curreent frame, queryIdx corresponds to the previous frame
    cv::KeyPoint trainKeypoint, queryKeypoint;
    unsigned int prevBoxIdx, currBoxIdx;
    
    //iterate through the matches, for matching keypoints find the corresponding bounding box indices and update the count in boundingBoxCombScore matrix
    for (auto it = matches.begin(); it != matches.end(); ++it)
    {
        trainIdx = it->trainIdx;
        queryIdx = it->queryIdx;
        
        trainKeypoint = currFrame.keypoints.at(trainIdx);
        queryKeypoint = prevFrame.keypoints.at(queryIdx);

        if (boundingBoxIndex(prevFrame.boundingBoxes, queryKeypoint, prevBoxIdx) && boundingBoxIndex(currFrame.boundingBoxes, trainKeypoint, currBoxIdx))
        {
            boundingBoxCombScore.at<uint8_t>(currBoxIdx, prevBoxIdx) += 1;
        }
    }

    //iterate throught the boundingBoxCombScore to find the row and columns corresponding to which we get the max score
    int maxCol, maxCount;
    for (int row = 0; row < boundingBoxCombScore.rows; row++)
    {
        maxCount = 0;
        for (int col = 0; col < boundingBoxCombScore.cols; col++)
        {
            int count = (int)boundingBoxCombScore.at<uint8_t>(row, col);
            if (count > maxCount)
            {
                maxCol = col;
                maxCount = count;
            }
        }
        std::cout << "row = " << row << ", col = " << maxCol << " count = " << maxCount << std::endl;

        // find the number of elements whose value is equal to maxCount
        // create correspondance when there is an unambiguous winner
        int numberOfMaxCountElements = 0;
        for (int col = 0; col < boundingBoxCombScore.cols; col++)
        {
            if ((int)boundingBoxCombScore.at<uint8_t>(row, col) == maxCount)
            {
                numberOfMaxCountElements += 1;
            }
        }
        if (numberOfMaxCountElements == 1)
        {
            //unambiguous max value
            bbBestMatches.insert(pair<int, int>(maxCol, row));
        }
    }

    if (bVis)
    {
        cv::Mat currImage = currFrame.cameraImg.clone();
        cv::Mat prevImage = prevFrame.cameraImg.clone();

        for (auto it = bbBestMatches.begin(); it != bbBestMatches.end(); ++it)
        {
            BoundingBox prevBB, currBB;
            bool prevBBFound = false, currBBFound = false;
            for (auto it1 = currFrame.boundingBoxes.begin(); it1 != currFrame.boundingBoxes.end(); ++it1)
            {
                if (it->second == it1->boxID)
                {
                    currBB = *it1;
                    currBBFound = true;
                }
            }

            for (auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); ++it2)
            {
                if (it->first == it2->boxID)
                {
                    prevBB = *it2;
                    prevBBFound = true;
                }
            }

            if (currBBFound && prevBBFound)
            {
                cv::RNG rng1(currBB.roi.x);
                cv::Scalar bboxColor = cv::Scalar(rng1.uniform(0, 255), rng1.uniform(0, 255), rng1.uniform(0, 255));
                
                cv::rectangle(currImage, cv::Point(currBB.roi.x, currBB.roi.y), cv::Point(currBB.roi.x + currBB.roi.width, currBB.roi.y + currBB.roi.height), bboxColor, 2);
                cv::putText(currImage, "boxID=" + std::to_string(currBB.boxID), cv::Point(currBB.roi.x,currBB.roi.y + currBB.roi.height/2), cv::FONT_ITALIC, 0.5, cv::Scalar(0, 0, 255), 1);
                cv::rectangle(prevImage, cv::Point(prevBB.roi.x, prevBB.roi.y), cv::Point(prevBB.roi.x + prevBB.roi.width, prevBB.roi.y + prevBB.roi.height), bboxColor, 2);
                cv::putText(prevImage, "boxID=" + std::to_string(prevBB.boxID), cv::Point(prevBB.roi.x , prevBB.roi.y + prevBB.roi.height/2), cv::FONT_ITALIC, 0.5, cv::Scalar(0, 0, 255), 1);
                
                cv::Mat resultImg;
                cv::vconcat(currImage, prevImage, resultImg);

                cv::putText(resultImg, "current image index= " + std::to_string(imgIndex), cv::Point(20 , 20), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 255), 2);

                std::string compWindowName = "compare the bounding boxes";
                cv::namedWindow(compWindowName, 1);
                cv::imshow(compWindowName, resultImg);
                cv::waitKey(0);
                std::cout << "PRESS KEY TO CONTINUE" << std::endl;
            }
        }
    }
}

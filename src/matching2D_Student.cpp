#include <numeric>
#include <fstream>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    std::ofstream datalog;
    datalog.open("measurement.txt", std::ios_base::app);

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    double t = cv::getTickCount();

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
        float threshold_ratio = 0.8;
        for (int i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < threshold_ratio*knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
    t = ((double)cv::getTickCount() - t)/((double)cv::getTickFrequency());
    datalog << "matching time = " << t * 1000/1.0 << " ms" << std::endl;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, int imgIndex)
{
    std::ofstream datalog;
    datalog.open("measurement.txt", std::ios_base::app);

    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {

        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    datalog << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream measurementfile;
        std::string measurementFileName = descriptorType + "_descriptor_measuremntLog.csv";
        if (firstEntry)
        {
            measurementfile.open(measurementFileName);
            measurementfile << "image index, process time" << std::endl;
            firstEntry = false;
        }
        else
        {
            measurementfile.open(measurementFileName, std::ios::out | std::ios::app);
        }
        measurementfile << imgIndex << ", " << 1000 * t / 1.0 << std::endl;
    #endif
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, int imgIndex, bool bVis)
{
    std::ofstream datalog;
    datalog.open("measurement.txt", std::ios_base::app);
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    datalog << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream measurementfile;
        std::string measurementFileName = "ShiTomasi_detector_measurmentfile.csv";
        if (firstEntry)
        {
            measurementfile.open(measurementFileName);
            measurementfile << "image index, keypoints size , processing time" << std::endl;
            firstEntry = false;
        }
        else
        {
            measurementfile.open(measurementFileName, std::ios::out | std::ios::app);
        }
        measurementfile << imgIndex << ", " << keypoints.size() << ", " << 1000 * t/1.0 << std::endl;
    #endif

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, int imgIndex, bool bVis)
{
    std::ofstream datalog;
    datalog.open("measurement.txt", std::ios_base::app);
    int blockSize = 2;
    int apertureSize = 3;
    int minResponse = 100;
    double k = 0.04;

    cout << "selected detector = Harris Corner Detector" << endl;
    double detectionTime = cv::getTickCount();

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    //non-maxima suppression
    bool bOverlap = false;
    double maxOverlap = 0.0; 
    for (int y = 0; y < dst_norm_scaled.rows; y++)
    {
        for(int x = 0; x < dst_norm_scaled.cols; x++)
        {
            int response = static_cast<int>(dst_norm_scaled.at<unsigned char>(y, x));
            if (response > minResponse)
            {
                cv::KeyPoint keypoint;
                keypoint.pt = cv::Point2f(x, y);
                keypoint.size = 2*apertureSize;
                keypoint.response = response;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(keypoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = false;
                        if (keypoint.response > (*it).response)
                        {
                            *it = keypoint;
                            break;
                        }
                    }
                }
                if (!bOverlap)
                {
                    keypoints.push_back(keypoint);
                }
            }
        }
    }
    detectionTime = ((double)cv::getTickCount() - detectionTime)/((double) cv::getTickFrequency());
    datalog << "keypoints n = " << keypoints.size() << " in time = " << 1000 * detectionTime/ 1.0 << " ms" << endl;

    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream measurementfile;
        std::string measurementFileName = "Harris_detector_measurementfile.csv";
        if (firstEntry)
        {
            measurementfile.open(measurementFileName);
            measurementfile << "image index, keypoints size , processing time" << std::endl;
            firstEntry = false;
        }
        else
        {
            measurementfile.open(measurementFileName, std::ios::out | std::ios::app);
        }
        measurementfile << imgIndex << ", " << keypoints.size() << ", " << 1000 * detectionTime/1.0 << std::endl;
    #endif
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, int imgIndex, bool bVis)
{
    std::ofstream datalog;
    datalog.open("measurement.txt", std::ios_base::app);

    cv::Ptr <cv::FeatureDetector> detector;
    if (detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        detector = cv::SIFT::create();
    }

    cout << "selected detector = " << detectorType << endl;
    double detectionTime = cv::getTickCount();
    
    detector->detect(img, keypoints);

    detectionTime = ((double)cv::getTickCount() - detectionTime)/((double) cv::getTickFrequency());
    datalog << "keypoints n = " << keypoints.size() << " in time = " << 1000 * detectionTime/ 1.0 << " ms" << endl;
    
    #ifdef LOG_DATA
        static bool firstEntry = true;
        std::ofstream measurementfile;
        if (firstEntry)
        {
            measurementfile.open(detectorType + "_detector_measurementfile.csv");
            measurementfile << "image index, keypoints size , processing time" << std::endl;
            firstEntry = false;
        }
        else
        {
            measurementfile.open(detectorType + "_MeasurementFile.csv", std::ios::out | std::ios::app);
        }
        measurementfile << imgIndex << ", " << keypoints.size() << ", " << 1000 * detectionTime/1.0 << std::endl;
    #endif    

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
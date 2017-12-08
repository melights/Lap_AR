// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <System.h>
#include "ViewerAR.h"

using namespace cv;

int main(int argc, char* argv[])
{
    VideoCapture cap(argv[1]);

    Mat frame_left;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("../../Vocabulary/ORBvoc.txt", "./Lap_AR.yaml", ORB_SLAM2::System::MONOCULAR, false);
    ORB_SLAM2::ViewerAR viewerAR;
    viewerAR.SetSLAM(&SLAM);
    viewerAR.SetFPS(30);
    viewerAR.SetCameraCalibration(484.9064, 460.6096, 361.6711, 282.6303);
    thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run, &viewerAR);

    while (1)
    {

        cap.grab();
        cap.retrieve(frame_left);
        Mat CameraPose = SLAM.TrackMonocular(frame_left, 1);
        int state = SLAM.GetTrackingState();
        vector<ORB_SLAM2::MapPoint *> vMPs = SLAM.GetTrackedMapPoints();
        vector<cv::KeyPoint> vKeys = SLAM.GetTrackedKeyPointsUn();
        cv::cvtColor(frame_left,frame_left,CV_RGB2BGR);
        viewerAR.SetImagePose(frame_left, CameraPose, state, vKeys, vMPs);
    }

    return 1;
}

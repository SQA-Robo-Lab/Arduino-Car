#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int threshold = 100;
int kernelSize = 5;

void test(cv::Mat& img) {
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  cv::Mat out = img.clone();
  cv::aruco::drawDetectedMarkers(out, markerCorners, markerIds);
  cv::imshow("Aruco", out);
}

int main()
{
  std::cout << "Hallo welt" << std::endl;
#ifdef IS_LINUX
    cv::VideoCapture cap = cv::VideoCapture("/dev/video0", cv::CAP_V4L2);
#else
  cv::VideoCapture cap = cv::VideoCapture(1, cv::CAP_MSMF);
#endif
    cv::Mat img;
    if (!cap.isOpened()) {
      std::cout << "Camera not opened" << std::endl;
      return -1;
    }

    cv::namedWindow("Image");
    cv::createTrackbar("Threshold", "Image", &threshold, 255);

  cap >> img;

  int w = img.cols;
  int h = img.rows;

  cv::Point poly[1][4];
  poly[0][0] = cv::Point(w/4, h);
  poly[0][1] = cv::Point(w*3/4, h);
  poly[0][2] = cv::Point(w*4/6, h/6);
  poly[0][3] = cv::Point(w*2/6, h/6);
  const cv::Point* ppt[1] = { poly[0] };
  int npt[] = { 4 };
  cv::Mat mask(h, w, CV_8UC1, cv::Scalar(0));
  cv::Mat maskInv;
  cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255), cv::LINE_8);
  cv::bitwise_not(mask, maskInv);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*kernelSize+1,2*kernelSize+1), cv::Point(kernelSize, kernelSize));

  while (!img.empty()) {
    test(img);
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    //cv::bitwise_and(img, mask, img);
    //cv::bitwise_or(img, maskInv, img);
      cv::imshow("Grey", img);
      cv::threshold(img, img, threshold, 255, cv::THRESH_BINARY);
      //cv::erode(img, img, kernel);
      //cv::dilate(img, img, kernel);

    cv::imshow("Image", img);

    cv::Mat draw(h, w, CV_8UC1, cv::Scalar(255));
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(draw, contours, -1, cv::Scalar(127), 3);

    cv::imshow("Contrours", draw);

      if (cv::waitKey(25) == 'q') {
        std::cout << "Q was pressed. Exiting" << std::endl;
        break;
      }
      cap >> img;
    }
    cap.release();
    return 0;
}
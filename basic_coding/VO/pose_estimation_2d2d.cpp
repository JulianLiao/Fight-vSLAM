
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

/**************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * ************************************************/

void find_feature_matches();

void pose_estimate_2d2d(std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2, std::vector<DMatch> matches, Mat &R, Mat &t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);


int main(int argc, char *argv[]) {
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;


    // 估计两张图像间运动
    Mat R, t;
    pose_estimate_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // 验证对极约束
    for (DMatch match : matches) {

    }
}

/**
 * [M00 M01 M02]
 * [M10 M11 M12]
 * [M20 M21 M22]
 */
Point2d pixel2cam(const Point2d &p, const Mat &K) {
    double x = (p.x - K.at<double>(0,2)) / (K.at<double>(0,0));
    double y = (p.y - K.at<double>(1,2)) / (K.at<double>(1,1));
    return Point2d(x,y);
}



void pose_estimate_2d2d(vector<KeyPoint> keypoints_1,
                        vector<KeyPoint> keypoints_2,
                        vector<DMatch> matches,
                        Mat &R,
                        Mat &t) {
    Matrix3d K;
    K << 520.9, 0, 325.1,    0, 521, 249.7,    0, 0, 1;

    // --把匹配点转换成vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;


    for (int i = 0; i < matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    // -- 计算基础矩阵
    // 关于第三个参数method的可能取值：
    //     CV_FM_7POINT，7点法，N=7
    //     CV_FM_8POINT，8点法，N>=8
    //     CV_FM_RANSAC，RANSAC算法，N>=8
    //     CV_FM_LMEDS，LMedS算法，N>=8
    // OpenCV prototype1:
    //     Mat cv::findFundamentalMat(InputArray points1, InputArray points2, int method, double ransacReprojThreshold,
    //                                double confidence, int maxIters, OutputArray mask = noArray())
    // OpenCV prototype2:
    //     Mat cv::findFundamentalMat(InputArray points1, InputArray point2, int method = FM_RANSAC, double ransacReprojThreshold = 3,
    //                                double confidence=0.99, OutputArray mask = noArray())
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(point1, point2, CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    // -- 计算本质矩阵
    // OpenCV prototype1:
    //     Mat cv::findEssentialMat(InputArray points1, InputArray points2, InputArrary cameraMatrix, int method = 0, 
    //                              double prob = 0.999, double threshold = 1.0, OutputArray mask = noArray());
    // OpenCV prototype2:
    //     Mat cv::findEssentialMat(InputArray points1, InputArray points2, double focal = 1.0, Point2d pp = Point2d(0, 0),
    //                              int method = RANSAC, double prob = 0.999, double threshold = 1.0, OutputArray mask = noArray())
    // 这大概是一个640像素 * 480像素的图片
    Point2d principal_point(325.1, 249.7);    // 相机光心，TUM dataset标定值
    double focal_length = 521;    // 焦距，521像素，TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, point2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    // -- 计算单应矩阵
    // OpenCV prototype:
    //     Mat cv::findHomography(InputArray srcPoints, InputArray dstPoints, int method = 0, double ransacReprojThreshold = 3,
    //                            OutputArray mask = noArray(), const int maxIters = 2000, const double confidence = 0.995);
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;


    // -- 从本质矩阵中恢复旋转和平移信息
    // opencv prototype2:
    //     /**
    //      * E: 输入的本质矩阵
    //      *
    //      */
    //     int cv::recoverPose(InputArray E, InputArray points1, InputArray point2, OutputArray R, OutputArray t, double focal = 1.0,
    //                         Point2d pp = Point2d(0, 0), InputOutputArray mask = noArray())
    recoverPose();
}


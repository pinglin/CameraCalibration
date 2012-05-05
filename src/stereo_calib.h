#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

#include <pangolin/pangolin.h>
#include <pangolin/simple_math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace pangolin;

const int PanelWidth = 150;
const int ImageWidth = 720;
const int ImageHeight = 576;

struct CalibParams
{
	int NumFrames;              // The number of frames to use from the input for calibration

	Size ImageSize;
	Size BoardSize;            // The size of the board -> Number of items by width and height
	float SquareSize;          // The size of a square in your defined unit (point, millimeter,etc).	

	float chessboard_width;
	float chessboard_height;

	int calib_flag;

	bool calibZeroTangentDist; // Assume zero tangential distortion
	bool calibFixPrincipalPoint;// Fix the principal point at the center

	bool flipVertical;          // Flip the captured images around the horizontal axis

	string outputFileName;      // The name of the file where to write
	bool showUndistorsed;       // Show undistorted images after calibration
	string input;               // The input ->

	vector<string> LeftImageList;
	vector<string> RightImageList;

	Mat LeftCameraMatrix, RightCameraMatrix;
	Mat LeftDistCoeffs, RightDistCoeffs;

	OpenGlMatrixSpec LeftCamIntrins;  // Row-major order
	OpenGlMatrixSpec RightCamIntrins;

	vector<OpenGlMatrixSpec> LeftCamExtrins;  // Row-major order
	vector<OpenGlMatrixSpec> RightCamExtrins;

	OpenGlMatrixSpec CamRwrtLExtrins;

} calib_params;

struct RectifiedParams
{

	Mat LeftRot, RightRot;
	Mat LeftProjMat, RightProjMat;
	Rect LeftValidRoi, RightValidRoi;
	
	Mat Disp2DepthReProjMat;

	Mat LeftRMAP[2], RightRMAP[2];

	bool isVerticalStereo;
} rect_params;

class StereoCalibration{

public:

    StereoCalibration();

    void ReadCalibParams(string &img_xml);

    void WriteCalibParams();

    void Calibration();

	double FundamentalMatrixQuality(vector<vector<Point2f> > LeftImagePoints, vector<vector<Point2f> > RightImagePoints, 
									Mat LeftCameraMatrix, Mat RightCameraMatrix, 
									Mat LeftDistCoeffs, Mat RightDistCoeffs, Mat F);

    void CvtCameraIntrins(Mat LeftCameraMatrix, Mat RightCameraMatrix);

    void CvtCameraExtrins(const vector<Mat> &LeftRVecs, const vector<Mat> &LeftTVecs,
                          const vector<Mat> &RightRVecs, const vector<Mat> &RightTVecs,
                          const Mat &R, const Mat &T);

    void InitPangolin();

    void InitTexture();

    void Routine();

    OpenGlMatrixSpec StereoBind(const OpenGlMatrixSpec &LeftCamera);    

	void DrawAxis() const;

	void DrawImage(const string &img_file, bool isUndistort, bool isLeftCamera) const;

	void DrawRectifiedImage(const string &img_file, bool isLeftCamera) const;

	void OpenCVSBM(const string &left_img, const string &right_img) const;

    View *panel, *view_left, *view_right;
	pangolin::GlTexture *gl_img_tex, *gl_chessboard_tex;

    GLuint ChessTexID, ImgTexID;

};

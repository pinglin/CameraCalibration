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

struct CalibParams
{

	vector<string> ImageList;	

	Mat CameraMatrix;
	Mat DistCoeffs;	

};

struct StereoParams
{
	
	Mat R, T, E, F;

};

struct RectifiedParams
{

	Mat LeftRot, RightRot;
	Mat LeftProj, RightProj;
	Rect LeftRoi, RightRoi;
	
	Mat Disp2DepthReProjMat;

	Mat LeftRMAP[2], RightRMAP[2];

	bool isVerticalStereo;
};

class CameraCalibration
{

public:

	CameraCalibration();

	void ReadMonoCalibParams(string &img_xml);

	void ReadStereoCalibParams(string &img_xml);

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

    OpenGlMatrixSpec StereoBind(const OpenGlMatrixSpec &LeftCamera);    

	void DrawAxis() const;

	void DrawImage(const string &img_file, bool isUndistort, bool isLeftCamera) const;

	void DrawRectifiedImage(const string &img_file, bool isLeftCamera) const;

	void OpenCVSBM(const string &left_img, const string &right_img) const;

	CalibParams *calib_params;	

    View *panel, *view;
	pangolin::GlTexture *gl_img_tex, *gl_chessboard_tex;

	inline int getNumFrames() const { return NumFrames; }

	inline Size getBoardTexSize() const { return BoardTexSize; }


private:

	int NumFrames;              // The number of frames to use from the input for calibration
	
	Size BoardSize;            // The size of the board -> Number of items by width and height
	float SquareSize;          // The size of a square in your defined unit (point, millimeter,etc).	

	Size BoardTexSize;	// OpenGL texture for the chessboard rendering

	const int PanelWidth = 150;	
	Size ImageSize;

	// Pangolin matrix
	OpenGlMatrixSpec LeftCamIntrins;  // Row-major order
	OpenGlMatrixSpec RightCamIntrins;

	vector<OpenGlMatrixSpec> LeftCamExtrins;  // Row-major order
	vector<OpenGlMatrixSpec> RightCamExtrins;

	OpenGlMatrixSpec CamRwrtLExtrins;

};

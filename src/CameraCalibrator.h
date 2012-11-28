#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>


#include <Pangolin/Pangolin.h>
#include <Pangolin/simple_math.h>
#include <Pangolin/timer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace Pangolin;

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

    void MonoCalibration();

	void StereoCalibration();

	double FundamentalMatrixQuality(vector<vector<Point2f> > LeftImagePoints, vector<vector<Point2f> > RightImagePoints, 
									Mat LeftCameraMatrix, Mat RightCameraMatrix, 
									Mat LeftDistCoeffs, Mat RightDistCoeffs, Mat F);

	void CvtCameraExtrins(const vector<Mat> *RVecs, const vector<Mat> *TVecs);

    void InitPangolin(int PanelWidth);

    OpenGlMatrixSpec StereoBind(const OpenGlMatrixSpec &LeftCamera);    

	void DrawRectifiedImage(const int c_idx, const int img_idx) const;

	void DrawChessboardAndImage(const int c_idx, const int img_idx, const bool is_undistorted, const bool is_stereobind);

	//void OpenCVSBM(const string &left_img, const string &right_img) const;

	CalibParams *calib_params;	
	StereoParams *stereo_params;
	RectifiedParams *rect_params;

    View *panel, *view[2];
    Pangolin::GlTexture *gl_img_tex, *gl_chessboard_tex;

	inline void setStereoMode(bool isStereoMode) { stereo_mode = isStereoMode; }

	inline int getNumFrames() const { return NumFrames; }

    // Pangolin matrix
	OpenGlMatrixSpec *CamIntrins;  // Row-major order
	vector<OpenGlMatrixSpec> *CamExtrins;  // Row-major order
	OpenGlMatrixSpec L2RExtrins;	// For stereo mode

private:

	bool stereo_mode;
	string data_path;

	int NumFrames;              // The number of frames to use from the input for calibration
	
	Size BoardSize;            // The size of the board -> Number of items by width and height
	float SquareSize;          // The size of a square in your defined unit (point, millimeter,etc).	
	Size ImageSize;

    // Pangolin OpenGL texture
	float BoardTexWdith, BoardTexHeight;	// OpenGL texture for the chessboard rendering

	void CheckImageSizeConsistency();

    void InitTexture();

	void DrawAxis() const;

};

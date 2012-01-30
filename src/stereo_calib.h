#include <stdio.h>
#include <iostream>
#include <sstream>
#include <time.h>

#include <pangolin/pangolin.h>

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

class StereoCalibration{

public:

    StereoCalibration();

    void InitCalibParams(string &img_xml);

    void Calibration();

    void CvtCameraExtrins(vector<Mat> LeftRVecs, vector<Mat> LeftTVecs, vector<Mat> RightRVecs, vector<Mat> RightTVecs);

    void InitPangolin();

    void InitChessboard();

    void Routine();

    void DrawChessboard();

    void DrawAxis();

    void SpecialKeyFunction(int key, int x, int y);

private:

    struct CalibParams
    {

        Size boardSize;            // The size of the board -> Number of items by width and height
        float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
        int nrFrames;              // The number of frames to use from the input for calibration

        int calib_flag;

        bool calibZeroTangentDist; // Assume zero tangential distortion
        bool calibFixPrincipalPoint;// Fix the principal point at the center

        bool flipVertical;          // Flip the captured images around the horizontal axis

        string outputFileName;      // The name of the file where to write
        bool showUndistorsed;       // Show undistorted images after calibration
        string input;               // The input ->

        vector<string> LeftImageList;
        vector<string> RightImageList;

        double LeftCamIntrins[4];   // fx, fy, cx, cy
        double RightCamIntrins[4];

        vector<OpenGlMatrixSpec> LeftCamExtrins;  // Row-major order
        vector<OpenGlMatrixSpec> RightCamExtrins;

    } calib_params;

    View *panel, *view_left, *view_right, *img_left, *img_right;
    OpenGlRenderState state;

    GLuint textID;


};

#include "stereo_calib.h"

// must be initialized in constructor as sysPtr = *System
static void *sysPtr;
inline static void routineWrapper(){ ((StereoCalibration*)sysPtr)->Routine(); }
inline static void specialKeyFuncWrapper(int key, int x, int y){ ((StereoCalibration*)sysPtr)->SpecialKeyFunction(key, x, y); }

int main( int argc, char* argv[])
{

    if(argc < 2)
    {
        cerr << "A xml file containing iamge list is required." << endl;
        exit(EXIT_FAILURE);
    }

    StereoCalibration stereo_calib;

    string xml_input(argv[1]);
    stereo_calib.InitCalibParams(xml_input);

    stereo_calib.Calibration();
    stereo_calib.InitPangolin();
    stereo_calib.InitTexture();

    specialKeyBindPangolin(specialKeyFuncWrapper);
    runPangolin(routineWrapper);

    return 0;

}

StereoCalibration::StereoCalibration()
{

    sysPtr = this;

}

void StereoCalibration::InitCalibParams(string &img_xml)
{

    FileStorage fs(img_xml, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << img_xml << "\"" << endl;
        exit(EXIT_FAILURE);
    }

    calib_params.BoardSize = Size((int)fs["BoardSize_Width"], (int)fs["BoardSize_Height"]);
    calib_params.squareSize = (float)fs["Square_Size"];

    FileNode left_imgs = fs["left_images"];
    for(FileNodeIterator itr = left_imgs.begin(); itr != left_imgs.end(); itr++)
        calib_params.LeftImageList.push_back((string)*itr);

    FileNode right_imgs = fs["right_images"];
    for(FileNodeIterator itr = right_imgs.begin(); itr != right_imgs.end(); itr++)
        calib_params.RightImageList.push_back((string)*itr);

    assert(calib_params.LeftImageList.size() == calib_params.RightImageList.size());

    calib_params.NumFrames = calib_params.RightImageList.size();

    fs.release();            

}

void StereoCalibration::Calibration()
{

    vector<vector<Point2f> > LeftImagePoints, RightImagePoints;
    Size ImageSize;

    for(int i=0; i < calib_params.NumFrames; i++)
    {

        Mat img_left = imread(calib_params.LeftImageList.at(i), CV_LOAD_IMAGE_COLOR);
        Mat img_right = imread(calib_params.RightImageList.at(i), CV_LOAD_IMAGE_COLOR);

        ImageSize = img_left.size();
        assert(ImageSize == img_right.size() || calib_params.ImageSize != ImageSize);

        calib_params.ImageSize = ImageSize;

        vector<Point2f> left_pointBuf;
        vector<Point2f> right_pointBuf;
        bool found_left = false;
        bool found_right = false;

        found_left = findChessboardCorners(img_left, calib_params.BoardSize, left_pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);

        found_right = findChessboardCorners(img_right, calib_params.BoardSize, right_pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found_left && found_right)
        {
            Mat viewGray;
            cvtColor(img_left, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, left_pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            cvtColor(img_right, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, right_pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

//            drawChessboardCorners(img_left, calib_params.BoardSize, Mat(left_pointBuf), found_left);
//            imshow("Left Image View", img_left);
//            drawChessboardCorners(img_right, calib_params.BoardSize, Mat(right_pointBuf), found_right);
//            imshow("Right Image View", img_right);
//            waitKey();
        }
        else
        {
            cerr << "ith image cannot be found a pattern." << endl;
            exit(EXIT_FAILURE);
        }

        LeftImagePoints.push_back(left_pointBuf);
        RightImagePoints.push_back(right_pointBuf);

    }

    vector<vector<Point3f> > ObjectPoints(1);
    for(int i=0; i<calib_params.BoardSize.height; i++)
        for(int j=0; j<calib_params.BoardSize.width; j++)
            ObjectPoints.at(0).push_back(Point3f(float( j*calib_params.squareSize ),
                                           float( i*calib_params.squareSize ),
                                           0));

    ObjectPoints.resize(LeftImagePoints.size(), ObjectPoints[0]);

    vector<Mat> LeftRVecs, LeftTVecs, RightRVecs, RightTVecs;

    calib_params.LeftDistCoeffs = Mat::zeros(8, 1, CV_64F);
    calib_params.RightDistCoeffs = Mat::zeros(8, 1, CV_64F);

    calib_params.LeftCameraMatrix = initCameraMatrix2D(ObjectPoints, LeftImagePoints, ImageSize, 0);
    double rms = calibrateCamera(ObjectPoints,
                                 LeftImagePoints,
                                 ImageSize,
                                 calib_params.LeftCameraMatrix,
                                 calib_params.LeftDistCoeffs,
                                 LeftRVecs,
                                 LeftTVecs,
                                 CV_CALIB_USE_INTRINSIC_GUESS |
                                 CV_CALIB_FIX_K4 |
                                 CV_CALIB_FIX_K5 |
                                 CV_CALIB_FIX_K6);

   cout << "Left camera re-projection error reported by calibrateCamera: "<< rms << endl;

   calib_params.RightCameraMatrix = initCameraMatrix2D(ObjectPoints, LeftImagePoints, ImageSize, 0);
   rms = calibrateCamera(ObjectPoints,
                         RightImagePoints,
                         ImageSize,
                         calib_params.RightCameraMatrix,
                         calib_params.RightDistCoeffs,
                         RightRVecs,
                         RightTVecs,
                         CV_CALIB_USE_INTRINSIC_GUESS |
                         CV_CALIB_FIX_K4 |
                         CV_CALIB_FIX_K5 |
                         CV_CALIB_FIX_K6);

   cout << "Right camera re-projection error reported by calibrateCamera: "<< rms << endl;

   CvtCameraExtrins(LeftRVecs, LeftTVecs, RightRVecs, RightTVecs);

   Mat E, F;
   rms = stereoCalibrate(ObjectPoints,
                         LeftImagePoints,
                         RightImagePoints,
                         calib_params.LeftCameraMatrix,
                         calib_params.LeftDistCoeffs,
                         calib_params.RightCameraMatrix,
                         calib_params.RightDistCoeffs,
                         calib_params.ImageSize,
                         calib_params.R_wrtL,
                         calib_params.T_wrtL,
                         E,
                         F);

   cout << "Stereo re-projection error reported by stereoCalibrate: "<< rms << endl;

}

void StereoCalibration::CvtCameraExtrins(vector<Mat> LeftRVecs, vector<Mat> LeftTVecs, vector<Mat> RightRVecs, vector<Mat> RightTVecs)
{

    Mat rot3x3;
    OpenGlMatrixSpec P = IdentityMatrix(GlModelViewStack);

    for(int i=0; i<calib_params.NumFrames; i++)
    {
        Rodrigues(LeftRVecs.at(i), rot3x3);
        for(int col=0; col<3; col++)
            for(int row=0; row<3; row++)
                P.m[col*4+row] = rot3x3.at<double>(row, col);
        copy(LeftTVecs.at(i).ptr<double>(), LeftTVecs.at(i).ptr<double>()+3, &P.m[12]);
        calib_params.LeftCamExtrins.push_back(P);

        Rodrigues(RightRVecs.at(i), rot3x3);
        for(int col=0; col<3; col++)
            for(int row=0; row<3; row++)
                P.m[col*4+row] = rot3x3.at<double>(row, col);
        copy(RightTVecs.at(i).ptr<double>(), RightTVecs.at(i).ptr<double>()+3, &P.m[12]);
        calib_params.RightCamExtrins.push_back(P);

    }

}

void StereoCalibration::InitPangolin()
{

    const int WindowWidth = (ImageWidth)*2+PanelWidth-1;
    const int WindowHeight = ImageHeight;

    // Create OpenGL window in single line thanks to GLUT
    CreateGlutWindowAndBind("Main", WindowWidth, WindowHeight, GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);

    GLenum err = glewInit();
    if(GLEW_OK != err)
    {
        cerr << "GLEW Error: " << glewGetErrorString(err) << endl;
        exit(0);
    }

    panel = &CreatePanel("ui").SetBounds(1.0, 0.0, 0.0, (double)PanelWidth/DisplayBase().v.w);


    calib_params.LeftCamIntrins = ProjectionMatrixRDF_TopLeft(calib_params.ImageSize.width,
                                                              calib_params.ImageSize.height,
                                                              calib_params.LeftCameraMatrix.at<double>(0, 0),
                                                              calib_params.LeftCameraMatrix.at<double>(1, 1),
                                                              calib_params.LeftCameraMatrix.at<double>(0, 2),
                                                              calib_params.LeftCameraMatrix.at<double>(1, 2),
                                                              0.1,
                                                              10000);

    calib_params.RightCamIntrins = ProjectionMatrixRDF_TopLeft(calib_params.ImageSize.width,
                                                               calib_params.ImageSize.height,
                                                               calib_params.RightCameraMatrix.at<double>(0, 0),
                                                               calib_params.RightCameraMatrix.at<double>(1, 1),
                                                               calib_params.RightCameraMatrix.at<double>(0, 2),
                                                               calib_params.RightCameraMatrix.at<double>(1, 2),
                                                               0.1,
                                                               10000);

    const double panel = (double)(PanelWidth-1)/(double)(WindowWidth-1);
    const double middle_h = ((double)(WindowWidth-PanelWidth)/2.0)/(double)(WindowWidth) + (double)(PanelWidth)/(double)(WindowWidth);

    view_left = &Display("ViewLeft").SetBounds(1.0, 0, panel, middle_h, -(double)ImageWidth/(double)ImageHeight);
    view_right = &Display("ViewRight").SetBounds(1.0, 0, middle_h, 1.0, -(double)ImageWidth/(double)ImageHeight);

}

void StereoCalibration::InitTexture()
{

    GLubyte checkImage[8*calib_params.BoardSize.height][8*calib_params.BoardSize.width][4];

    int i, j, c;
    for (i = 0; i < 8*calib_params.BoardSize.height; i++) {
        for (j = 0; j < 8*calib_params.BoardSize.width; j++) {
            c = ((((i&0x8)==0)^((j&0x8)==0)))*255;
            checkImage[i][j][0] = (GLubyte) c;
            checkImage[i][j][1] = (GLubyte) c;
            checkImage[i][j][2] = (GLubyte) 0;
            checkImage[i][j][3] = (GLubyte) 125;
        }
    }

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    // Texture for chessboard
    glGenTextures(1, &ChessTexID);
    glBindTexture(GL_TEXTURE_2D, ChessTexID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 8*calib_params.BoardSize.width,
                 8*calib_params.BoardSize.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 checkImage);

    // Texture for image
    glGenTextures(1,&ImgTexID);
    glBindTexture(GL_TEXTURE_2D, ImgTexID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, calib_params.ImageSize.width,
                 calib_params.ImageSize.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

}

void StereoCalibration::SpecialKeyFunction(int key, int x, int y)
{

//        switch(key)
//        {
//        case GLUT_KEY_INSERT:
//            capturer->captureCameraState();
//            break;
//        case GLUT_KEY_F12:
//            capturer->deleteCameraState();
//            break;
//        case GLUT_KEY_LEFT:
//            SwitchFrame(true);
//            break;
//        case GLUT_KEY_RIGHT:
//            SwitchFrame(false);
//            break;
//        }

}

void StereoCalibration::DrawChessboard()
{

    float w = calib_params.squareSize*calib_params.BoardSize.width;
    float h = calib_params.squareSize*calib_params.BoardSize.height;

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, ChessTexID);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertex3f(0.0, 0.0, 0.0);
    glTexCoord2f(0.0, 1.0); glVertex3f(0.0, h, 0.0);
    glTexCoord2f(1.0, 1.0); glVertex3f(w, h, 0.0);
    glTexCoord2f(1.0, 0.0); glVertex3f(w, 0.0, 0.0);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

}

void StereoCalibration::DrawAxis()
{

    float size = calib_params.squareSize*10.0;

    glEnable(GL_DEPTH_TEST);
    glBegin(GL_LINES);

    // x-axis
    glColor3f(0.5f, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(size, 0, 0);

    // y-axis
    glColor3f(0, 0.5f, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, size, 0);

    // z-axis
    glColor3f(0, 0, 0.5f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, size);

    glEnd();
    glDisable(GL_DEPTH_TEST);

}

void StereoCalibration::DrawImage(string &img_file)
{

    Mat img = imread(img_file, CV_LOAD_IMAGE_COLOR);
    cvtColor(img, img, CV_BGR2RGB);

    glBindTexture(GL_TEXTURE_2D, ImgTexID);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                    calib_params.ImageSize.width,
                    calib_params.ImageSize.height,
                    GL_RGB, GL_UNSIGNED_BYTE, img.ptr<unsigned char>());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2d(-1, 1);
    glTexCoord2f(1, 0);
    glVertex2d(1, 1);
    glTexCoord2f(1, 1);
    glVertex2d(1, -1);
    glTexCoord2f(0, 1);
    glVertex2d(-1, -1);
    glEnd();
    glDisable(GL_TEXTURE_2D);

}

void StereoCalibration::Routine()
{

    static Var<int> img_idx("ui.Image: ", 0, 0, calib_params.NumFrames-1);
    static Var<bool> a_checkbox("ui.A Checkbox",false,true);

    view_left->ActivateScissorAndClear();
    DrawImage(calib_params.LeftImageList.at(img_idx));
    calib_params.LeftCamIntrins.Load();
    calib_params.LeftCamExtrins.at(img_idx).Load();
    DrawAxis();
    DrawChessboard();

    view_right->ActivateScissorAndClear();
    DrawImage(calib_params.RightImageList.at(img_idx));
    calib_params.RightCamIntrins.Load();
    calib_params.RightCamExtrins.at(img_idx).Load();
    DrawAxis();
    DrawChessboard();

    panel->Render();

}

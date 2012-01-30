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

    calib_params.boardSize = Size((int)fs["BoardSize_Width"], (int)fs["BoardSize_Height"]);
    calib_params.squareSize = (float)fs["Square_Size"];

    FileNode left_imgs = fs["left_images"];
    for(FileNodeIterator itr = left_imgs.begin(); itr != left_imgs.end(); itr++)
        calib_params.LeftImageList.push_back((string)*itr);

    FileNode right_imgs = fs["right_images"];
    for(FileNodeIterator itr = right_imgs.begin(); itr != right_imgs.end(); itr++)
        calib_params.RightImageList.push_back((string)*itr);

    assert(calib_params.LeftImageList.size() == calib_params.RightImageList.size());

    calib_params.nrFrames = calib_params.RightImageList.size();

    fs.release();

}

void StereoCalibration::Calibration()
{

    vector<vector<Point2f> > LeftImagePoints, RightImagePoints;
    Size ImageSize;

    for(int i=0; i < calib_params.nrFrames; i++)
    {

        Mat view_left = imread(calib_params.LeftImageList.at(i), CV_LOAD_IMAGE_COLOR);
        Mat view_right = imread(calib_params.RightImageList.at(i), CV_LOAD_IMAGE_COLOR);

        ImageSize = view_left.size();
        assert(ImageSize == view_right.size());

        vector<Point2f> left_pointBuf;
        vector<Point2f> right_pointBuf;
        bool found_left = false;
        bool found_right = false;

        found_left = findChessboardCorners(view_left, calib_params.boardSize, left_pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);

        found_right = findChessboardCorners(view_right, calib_params.boardSize, right_pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found_left && found_right)
        {
            Mat viewGray;
            cvtColor(view_left, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, left_pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            cvtColor(view_right, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, right_pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

//            drawChessboardCorners(view_left, calib_params.boardSize, Mat(left_pointBuf), found_left);
//            imshow("Left Image View", view_left);
//            drawChessboardCorners(view_right, calib_params.boardSize, Mat(right_pointBuf), found_right);
//            imshow("Right Image View", view_right);
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
    for(int i=0; i<calib_params.boardSize.height; i++)
        for(int j=0; j<calib_params.boardSize.width; j++)
            ObjectPoints.at(0).push_back(Point3f(float( j*calib_params.squareSize ),
                                           float( i*calib_params.squareSize ),
                                           0));

    ObjectPoints.resize(LeftImagePoints.size(), ObjectPoints[0]);


    Mat LeftCameraMatrix, RightCameraMatrix;
    vector<Mat> LeftRVecs, LeftTVecs, RightRVecs, RightTVecs;

    Mat LeftDistCoeffs = Mat::zeros(8, 1, CV_64F);
    Mat RightDistCoeffs = Mat::zeros(8, 1, CV_64F);

    LeftCameraMatrix = initCameraMatrix2D(ObjectPoints, LeftImagePoints, ImageSize, 0);
    double rms = calibrateCamera(ObjectPoints,
                                 LeftImagePoints,
                                 ImageSize,
                                 LeftCameraMatrix,
                                 LeftDistCoeffs,
                                 LeftRVecs,
                                 LeftTVecs,
                                 CV_CALIB_USE_INTRINSIC_GUESS |
                                 CV_CALIB_FIX_K4 |
                                 CV_CALIB_FIX_K5 |
                                 CV_CALIB_FIX_K6);

   cout << "Left camera re-projection error reported by calibrateCamera: "<< rms << endl;

   RightCameraMatrix = initCameraMatrix2D(ObjectPoints, LeftImagePoints, ImageSize, 0);
   rms = calibrateCamera(ObjectPoints,
                         RightImagePoints,
                         ImageSize,
                         RightCameraMatrix,
                         RightDistCoeffs,
                         RightRVecs,
                         RightTVecs,
                         CV_CALIB_USE_INTRINSIC_GUESS |
                         CV_CALIB_FIX_K4 |
                         CV_CALIB_FIX_K5 |
                         CV_CALIB_FIX_K6);

   cout << "Right camera re-projection error reported by calibrateCamera: "<< rms << endl;

   Mat R, T, E, F;

   rms = stereoCalibrate(ObjectPoints,
                         LeftImagePoints,
                         RightImagePoints,
                         LeftCameraMatrix,
                         LeftDistCoeffs,
                         RightCameraMatrix,
                         RightDistCoeffs,
                         ImageSize,
                         R,
                         T,
                         E,
                         F);


   cout << "Stereo re-projection error reported by stereoCalibrate: "<< rms << endl;

    cout << R << endl;
    cout << T << endl;

}


void STAY_CODING(bool STAY_HUNGRY)
{
    STAY_HUNGRY ? STAY_CODING(true):STAY_CODING(false);
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


    state.Set(ProjectionMatrix(ImageWidth, ImageHeight, 420, 420, ImageWidth/2, ImageHeight/2, 0.1, 10000))
         .Set(IdentityMatrix(GlModelViewStack)).Apply();

    const double panel = (double)(PanelWidth-1)/(double)(WindowWidth-1);
    const double middle_h = ((double)(WindowWidth-PanelWidth)/2.0)/(double)(WindowWidth) + (double)(PanelWidth)/(double)(WindowWidth);

    view_left = &Display("ViewLeft").SetBounds(1.0, 0, panel, middle_h, -(double)ImageWidth/(double)ImageHeight);
    view_left->SetHandler(new Handler3D(state));

    view_right = &Display("ViewRight").SetBounds(1.0, 0, middle_h, 1.0, -(double)ImageWidth/(double)ImageHeight);
    view_right->SetHandler(new Handler3D(state));

    img_left = &Display("ImageLeft").SetBounds(1.0, 0.5, panel, middle_h, -(double)ImageWidth/(double)ImageHeight);
    img_right = &Display("ImageRight").SetBounds(1.0, 0.5, middle_h, 1.0, -(double)ImageWidth/(double)ImageHeight);

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

void StereoCalibration::Routine(){

//    if(HasResized())
//        DisplayBase().ActivateScissorAndClear();

    // Safe and efficient binding of named variables.
    // Specialisations mean no conversions take place for exact types
    // and conversions between scalar types are cheap.
    static Var<bool> a_button("ui.A Button",false,false);
    static Var<double> a_double("ui.A Double",3,1,10000,true);
    static Var<int> an_int("ui.An Int",2,0,5);
    static Var<bool> a_checkbox("ui.A Checkbox",false,true);
    static Var<int> an_int_no_input("ui.An Int No Input",2);
    static Var<double> aliased_double("ui.Aliased Double",3,0,10000);

    if( Pushed(a_button) )
        cout << "You Pushed a button!" << endl;

    // Overloading of Var<T> operators allows us to treat them like
    // their wrapped types, eg:
    if( a_checkbox )
        an_int = a_double;

    an_int_no_input = an_int;

    // Activate efficiently by object
    // (3D Handler requires depth testing to be enabled)

    glEnable(GL_DEPTH_TEST);
    glColor3f(1.0,1.0,1.0);




    view_left->ActivateScissorAndClear(state);
    glutWireTeapot(10.0);

    view_right->ActivateScissorAndClear(state);
    glutWireTeapot(10.0);



    panel->Render();

}

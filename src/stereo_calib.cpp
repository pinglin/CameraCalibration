#include "stereo_calib.h"

int *show_idx;
void SpecialKeyFunction(int key, int x, int y)
{

	switch(key)
	{
	case GLUT_KEY_LEFT:
		if(*show_idx > 0)
			(*show_idx)--;
		break;
	case GLUT_KEY_RIGHT:
		if(*show_idx < calib_params.NumFrames-1)
			(*show_idx)++;
		break;
	}

}

int main( int argc, char* argv[])
{

    if(argc < 2)
    {
        cerr << "A xml file containing iamge list is required." << endl;
        exit(EXIT_FAILURE);
    }

	string mode(argv[1]);
	bool stereo_mode = false;

	CameraCalibration camera_calib;
	if(mode == "-stereo")
	{
		stereo_mode = true;

		// Two camera calibration parameter sets
		camera_calib.calib_params = new CalibParams[2];

		string xml_input(argv[2]);
		camera_calib.ReadMonoCalibParams(xml_input);	

		camera_calib.Calibration();

	}	
	else
	{		
		string xml_input(argv[1]);
		camera_calib.ReadStereoCalibParams(xml_input);	

	}    
    
    camera_calib.InitPangolin();
    camera_calib.InitTexture();

	static Var<int> img_idx("ui.Image: ", 0, 0, camera_calib.getNumFrames()-1);
	static Var<bool> is_stereobind("ui.Stereo Bind", false, true);
	static Var<bool> is_undistorted("ui.Undistorted images", false, true);
	static Var<bool> is_rectified("ui.Rectified images", false, true);
	static Var<bool> disp_button("ui.Show OpenCV SBM", false, false);
	static Var<bool> export_button("ui.Export Results", false, false);

	while( !pangolin::ShouldQuit() )
	{
		if(pangolin::HasResized())
			DisplayBase().ActivateScissorAndClear();

		show_idx = (int*)img_idx.var->val;
		if(is_rectified)
		{

			camera_calib.view_left->ActivateScissorAndClear();
			camera_calib.DrawRectifiedImage(camera_calib.calib_params[0].ImageList.at(img_idx), true);

			camera_calib.view_right->ActivateScissorAndClear();
			camera_calib.DrawRectifiedImage(camera_calib.calib_params[1].ImageList.at(img_idx), false);

		}
		else
		{
			// Left view
			camera_calib.view_left->ActivateScissorAndClear();
			camera_calib.DrawImage(camera_calib.calib_params[0].ImageList.at(img_idx), is_undistorted, true);
			
			camera_calib.LeftCamIntrins.Load();
			camera_calib.LeftCamExtrins.at(img_idx).Load();
			camera_calib.DrawAxis();
			camera_calib.gl_chessboard_tex->RenderPlanTexture3D(camera_calib.getBoardTexSize().width, 
																camera_calib.getBoardTexSize().height);

			// Right view
			camera_calib.view_right->ActivateScissorAndClear();
			camera_calib.DrawImage(calib_params.RightImageList.at(img_idx), is_undistorted, false);
			
			calib_params.RightCamIntrins.Load();

			if(is_stereobind)
				camera_calib.StereoBind(calib_params.LeftCamExtrins.at(img_idx)).Load();
			else
				calib_params.RightCamExtrins.at(img_idx).Load();

			camera_calib.DrawAxis();			
			camera_calib.gl_chessboard_tex->RenderPlanTexture3D(camera_calib.getBoardTexSize().width, 
																camera_calib.getBoardTexSize().height);
		}

		camera_calib.panel->Render();

		if(Pushed(export_button))
			camera_calib.WriteCalibParams();

		if(Pushed(disp_button))
			camera_calib.OpenCVSBM(calib_params.LeftImageList.at(img_idx), 
								   calib_params.RightImageList.at(img_idx));

		camera_calib.panel->Render();			

		// Swap frames and Process Events
		glutSwapBuffers();
		glutMainLoopEvent();
	}

    return 0;

}

CameraCalibration::CameraCalibration()
{

}

void CameraCalibration::ReadMonoCalibParams( string &img_xml )
{

    FileStorage fs(img_xml, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << img_xml << "\"" << endl;
        exit(EXIT_FAILURE);
    }

    BoardSize = Size((int)fs["BoardSizeWidth"], (int)fs["BoardSizeHeight"]);
    SquareSize = (float)fs["SquareSize"];

    FileNode imgs = fs["Images"];
    for(FileNodeIterator itr = imgs.begin(); itr != imgs.end(); itr++)	
        calib_params[0].ImageList.push_back((string)*itr);
	
	// Check if all image data have the same size.
	Mat img = imread(calib_params[0].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
	ImageSize = Size(img.rows, img.cols);
	for(int i=1; i<calib_params[0].ImageList.size(); i++)
	{
		img = imread(calib_params[0].ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
		assert(ImageSize == Size(img.rows, img.cols));
		ImageSize = Size(img.rows, img.cols);
	}

    NumFrames = calib_params[0].ImageList.size();

    fs.release();

}


void CameraCalibration::ReadStereoCalibParams( string &img_xml )
{

	FileStorage fs(img_xml, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << img_xml << "\"" << endl;
		exit(EXIT_FAILURE);
	}

	BoardSize = Size((int)fs["BoardSizeWidth"], (int)fs["BoardSizeHeight"]);
	SquareSize = (float)fs["SquareSize"];

	FileNode left_imgs = fs["LeftImages"];
	for(FileNodeIterator itr = left_imgs.begin(); itr != left_imgs.end(); itr++)
		calib_params[0].ImageList.push_back((string)*itr);

	FileNode right_imgs = fs["RightImages"];
	for(FileNodeIterator itr = right_imgs.begin(); itr != right_imgs.end(); itr++)
		calib_params[1].ImageList.push_back((string)*itr);

	// Check if the number of left and right images are the same
	assert(calib_params[0].ImageList.size() == calib_params[1].ImageList.size());
	NumFrames = calib_params[0].ImageList.size();

	// Check if all image data have the same size.
	Mat img = imread(calib_params[0].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
	ImageSize = Size(img.rows, img.cols);
	for(int i=1; i<calib_params[0].ImageList.size(); i++)
	{
		img = imread(calib_params[0].ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
		assert(ImageSize == Size(img.rows, img.cols));
		ImageSize = Size(img.rows, img.cols);
	}
	
	Mat img = imread(calib_params[1].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
	ImageSize = Size(img.rows, img.cols);
	for(int i=1; i<calib_params[1].ImageList.size(); i++)
	{
		img = imread(calib_params[1].ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
		assert(ImageSize == Size(img.rows, img.cols));
		ImageSize = Size(img.rows, img.cols);
	}

	fs.release();            

}


void CameraCalibration::WriteCalibParams()
{

    time_t t;
    time( &t );

    struct tm *t2 = localtime( &t );

    char buf[1024];
    strftime(buf, sizeof(buf)-1, "%c", t2 );

    FileStorage fs("calib_result.xml", FileStorage::WRITE);

    fs << "calibration_Time" << buf;
    fs << "NumOfFrames" << calib_params.NumFrames;
    fs << "ImageWidth" << calib_params.ImageSize.width;
    fs << "ImageHeight" << calib_params.ImageSize.height;
    fs << "BoardWidth" << calib_params.BoardSize.width;
    fs << "BoardHeight" << calib_params.BoardSize.height;
    fs << "SquareSize" << calib_params.SquareSize;

    fs << "LeftCameraMatrix" << calib_params.LeftCameraMatrix;
    fs << "LeftDistortion" << calib_params.LeftDistCoeffs;

    fs << "RightCameraMatrix" << calib_params.RightCameraMatrix;
    fs << "RightDistortion" << calib_params.RightDistCoeffs;

    fs << "StereoRigRotation" << calib_params.R;
	fs << "StereoRigTranslation" << calib_params.T;
	
}

void CameraCalibration::Calibration()
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
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));

            cvtColor(img_right, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, right_pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));

//            drawChessboardCorners(img_left, calib_params.BoardSize, Mat(left_pointBuf), found_left);
//            imshow("Left Image View", img_left);
//            drawChessboardCorners(img_right, calib_params.BoardSize, Mat(right_pointBuf), found_right);
//            imshow("Right Image View", img_right);
//            waitKey();
        }
        else
        {
            cerr << i << "th image cannot be found a pattern." << endl;
            exit(EXIT_FAILURE);
        }

        LeftImagePoints.push_back(left_pointBuf);
        RightImagePoints.push_back(right_pointBuf);

    }

    vector<vector<Point3f> > ObjectPoints(1);
    for(int i=0; i<calib_params.BoardSize.height; i++)
        for(int j=0; j<calib_params.BoardSize.width; j++)
            ObjectPoints.at(0).push_back(Point3f(float( j*calib_params.SquareSize ),
                                           float( i*calib_params.SquareSize ),
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
                                 CV_CALIB_FIX_K3 |
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
                         CV_CALIB_FIX_K3 |
                         CV_CALIB_FIX_K4 |
                         CV_CALIB_FIX_K5 |
                         CV_CALIB_FIX_K6);

   cout << "Right camera re-projection error reported by calibrateCamera: "<< rms << endl;
   
   rms = stereoCalibrate(ObjectPoints,
						LeftImagePoints,
						RightImagePoints,
						calib_params.LeftCameraMatrix,
						calib_params.LeftDistCoeffs,
						calib_params.RightCameraMatrix,
						calib_params.RightDistCoeffs,
						calib_params.ImageSize,
						calib_params.R,
						calib_params.T,
						calib_params.E,
						calib_params.F, 
						TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

   cout << "Stereo re-projection error reported by stereoCalibrate: " << rms << endl;

   cout << "Fundamental Matrix reprojection error: " << FundamentalMatrixQuality(LeftImagePoints, RightImagePoints, 
																				 calib_params.LeftCameraMatrix, calib_params.RightCameraMatrix, 
																				 calib_params.LeftDistCoeffs, calib_params.RightDistCoeffs, calib_params.F) << endl;        

   // Transfer matrix from OpenCV Mat to Pangolin matrix
   CvtCameraExtrins(LeftRVecs, LeftTVecs, RightRVecs, RightTVecs, calib_params.R, calib_params.T);

   // Stereo rectification
   stereoRectify(calib_params.LeftCameraMatrix,
				calib_params.LeftDistCoeffs,
				calib_params.RightCameraMatrix,
				calib_params.RightDistCoeffs,
				calib_params.ImageSize, 
				calib_params.R, 
				calib_params.T, 
				rect_params.LeftRot,
				rect_params.RightRot,
				rect_params.LeftProjMat,
				rect_params.RightProjMat,
				rect_params.Disp2DepthReProjMat,
				CALIB_ZERO_DISPARITY, // test later
				1, // test later
				calib_params.ImageSize,
				&rect_params.LeftValidRoi, 
				&rect_params.RightValidRoi);

   rect_params.isVerticalStereo = fabs(rect_params.RightProjMat.at<double>(1, 3)) > fabs(rect_params.RightProjMat.at<double>(0, 3));

   // Get the rectification re-map index
   initUndistortRectifyMap(calib_params.LeftCameraMatrix, calib_params.LeftDistCoeffs, 
						   rect_params.LeftRot, rect_params.LeftProjMat, 
						   calib_params.ImageSize, CV_16SC2, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1]);
   initUndistortRectifyMap(calib_params.RightCameraMatrix, calib_params.RightDistCoeffs, 
						   rect_params.RightRot, rect_params.RightProjMat, 
						   calib_params.ImageSize, CV_16SC2, rect_params.RightRMAP[0], rect_params.RightRMAP[1]);

}

double CameraCalibration::FundamentalMatrixQuality(vector<vector<Point2f> > LeftImagePoints, vector<vector<Point2f> > RightImagePoints, 
	Mat LeftCameraMatrix, Mat RightCameraMatrix, Mat LeftDistCoeffs, Mat RightDistCoeffs, Mat F)
{

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];	
	for(int i = 0; i < calib_params.NumFrames; i++ )
	{
		int npt = (int)LeftImagePoints[i].size();
		Mat imgpt[2];		

		imgpt[0] = Mat(LeftImagePoints[i]);
		imgpt[1] = Mat(RightImagePoints[i]);

		undistortPoints(imgpt[0], imgpt[0], LeftCameraMatrix, LeftDistCoeffs, Mat(), LeftCameraMatrix);
		undistortPoints(imgpt[1], imgpt[1], RightCameraMatrix, RightDistCoeffs, Mat(), RightCameraMatrix);
		
		computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);
		computeCorrespondEpilines(imgpt[1], 2, F, lines[1]);

		for(int j = 0; j < npt; j++ )
		{
			double errij = fabs(LeftImagePoints[i][j].x*lines[1][j][0] +
				LeftImagePoints[i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(RightImagePoints[i][j].x*lines[0][j][0] +
				RightImagePoints[i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	
	return err/npoints;

}


void CameraCalibration::CvtCameraExtrins(const vector<Mat> &LeftRVecs, const vector<Mat> &LeftTVecs,
                                         const vector<Mat> &RightRVecs, const vector<Mat> &RightTVecs,
                                         const Mat &R, const Mat &T)
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

    calib_params.CamRwrtLExtrins = IdentityMatrix(GlModelViewStack);
    for(int col=0; col<3; col++)
        for(int row=0; row<3; row++)
            calib_params.CamRwrtLExtrins.m[col*4+row] = R.at<double>(row, col);

    copy(T.ptr<double>(), T.ptr<double>()+3, &calib_params.CamRwrtLExtrins.m[12]);

}

void CameraCalibration::InitPangolin()
{

    const int WindowWidth = (ImageWidth)*2+PanelWidth-1;
    const int WindowHeight = ImageHeight;

    // Create OpenGL window in single line thanks to GLUT
    CreateGlutWindowAndBind("Main", WindowWidth, WindowHeight, GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Issue specific OpenGl we might need
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //GLenum err = glewInit();
    //if(GLEW_OK != err)
    //{
    //    cerr << "GLEW Error: " << glewGetErrorString(err) << endl;
    //    exit(0);
    //}

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

	glutSpecialFunc(&SpecialKeyFunction);
}

void CameraCalibration::InitTexture()
{

	calib_params.chessboard_width = calib_params.SquareSize*calib_params.BoardSize.width;
	calib_params.chessboard_height = calib_params.SquareSize*calib_params.BoardSize.height;

	GLubyte* checkImage = new GLubyte[8*calib_params.BoardSize.height*8*calib_params.BoardSize.width*4];

    int i, j, c;
    for (i = 0; i < 8*calib_params.BoardSize.height; i++) 
	{
        for (j = 0; j < 8*calib_params.BoardSize.width; j++) 
		{
            c = ((((i&0x8)==0)^((j&0x8)==0)))*255;
            checkImage[i*(8*calib_params.BoardSize.width*4)+j*4+0] = (GLubyte) 0;
            checkImage[i*(8*calib_params.BoardSize.width*4)+j*4+1] = (GLubyte) c;
            checkImage[i*(8*calib_params.BoardSize.width*4)+j*4+2] = (GLubyte) 0;
            checkImage[i*(8*calib_params.BoardSize.width*4)+j*4+3] = (GLubyte) 100;
        }
    }

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    // Texture for chessboard	
	gl_chessboard_tex = new pangolin::GlTexture(8*calib_params.BoardSize.width, 8*calib_params.BoardSize.height, GL_RGBA);
	gl_chessboard_tex->Upload(checkImage, GL_RGBA, GL_UNSIGNED_BYTE);    
	delete [] checkImage;

    // Texture for image
	gl_img_tex = new pangolin::GlTexture(calib_params.ImageSize.width, calib_params.ImageSize.height, GL_RGB);

}

void CameraCalibration::DrawAxis() const
{

    float size = calib_params.SquareSize*10.0;

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

void CameraCalibration::DrawImage( const string &img_file, bool isUndistort, bool isLeftCamera ) const
{

    Mat img = imread(img_file, CV_LOAD_IMAGE_COLOR);
    cvtColor(img, img, CV_BGR2RGB);

    if(isUndistort)
    {
        Mat undistort_img;
        if(isLeftCamera)
            undistort(img, undistort_img,
                      calib_params.LeftCameraMatrix,
                      calib_params.LeftDistCoeffs,
                      calib_params.LeftCameraMatrix);
        else
            undistort(img, undistort_img,
                      calib_params.RightCameraMatrix,
                      calib_params.RightDistCoeffs,
                      calib_params.RightCameraMatrix);

        img = undistort_img;
    }
	
	gl_img_tex->Upload(img.ptr<unsigned char>(), GL_RGB, GL_UNSIGNED_BYTE);
	gl_img_tex->RenderToViewportFlipY();

}

void CameraCalibration::DrawRectifiedImage( const string &img_file, bool isLeftCamera ) const
{
			
	int w = calib_params.ImageSize.width;
	int h = calib_params.ImageSize.height;		
	
	Mat canvas(w, h, CV_8UC3);
	Mat rimg, cimg;
	Mat img = imread(img_file, 0);	

	if(isLeftCamera)
	{		
		remap(img, rimg, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1], CV_INTER_LINEAR);
		cvtColor(rimg, canvas, CV_GRAY2RGB);				
		Rect vroi(cvRound(rect_params.LeftValidRoi.x), cvRound(rect_params.LeftValidRoi.y),
				  cvRound(rect_params.LeftValidRoi.width), cvRound(rect_params.LeftValidRoi.height)); 
		rectangle(canvas, vroi, Scalar(0,0,255), 3, 8);
		
	}else
	{
		remap(img, rimg, rect_params.RightRMAP[0], rect_params.RightRMAP[1], CV_INTER_LINEAR);
		cvtColor(rimg, canvas, CV_GRAY2RGB);				
		Rect vroi(cvRound(rect_params.RightValidRoi.x), cvRound(rect_params.RightValidRoi.y),
			cvRound(rect_params.RightValidRoi.width), cvRound(rect_params.RightValidRoi.height)); 
		rectangle(canvas, vroi, Scalar(0,0,255), 3, 8);
	}

	if(!rect_params.isVerticalStereo)
		for(int j=0; j<canvas.rows; j+=16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	else
		for(int j=0; j<canvas.cols; j+=16)
			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		
	gl_img_tex->Upload(canvas.ptr<unsigned char>(), GL_RGB, GL_UNSIGNED_BYTE);
	gl_img_tex->RenderToViewportFlipY();

}

OpenGlMatrixSpec CameraCalibration::StereoBind(const OpenGlMatrixSpec &LeftCamera)
{

    OpenGlMatrixSpec P = IdentityMatrix(GlModelViewStack);
    MatMul<4, 4, 4, double>(P.m, calib_params.CamRwrtLExtrins.m, LeftCamera.m);

    return P;

}

void CameraCalibration::OpenCVSBM( const string &left_img, const string &right_img ) const
{

	//-- 1. Read the images
	Mat oriImgLeft, oriImgRight, imgLeft, imgRight;
	if(calib_params.CamRwrtLExtrins.m[12] < 0)
	{
		oriImgLeft = imread(right_img, CV_LOAD_IMAGE_GRAYSCALE);
		oriImgRight = imread(left_img, CV_LOAD_IMAGE_GRAYSCALE);

		remap(oriImgLeft, imgLeft, rect_params.RightRMAP[0], rect_params.RightRMAP[1], CV_INTER_LINEAR);
		remap(oriImgRight, imgRight, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1], CV_INTER_LINEAR);
	}
	else
	{
		oriImgLeft = imread(left_img, CV_LOAD_IMAGE_GRAYSCALE );
		oriImgRight = imread(right_img, CV_LOAD_IMAGE_GRAYSCALE);

		remap(oriImgLeft, imgLeft, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1], CV_INTER_LINEAR);
		remap(oriImgRight, imgRight, rect_params.RightRMAP[0], rect_params.RightRMAP[1], CV_INTER_LINEAR);
	}

	//-- And create the image in which we will save our disparities
	Mat imgDisparity16S = Mat( imgLeft.rows, imgLeft.cols, CV_16S );
	Mat imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );

	if( !imgLeft.data || !imgRight.data )
	{ std::cout<< " --(!) Error reading images " << std::endl; return; }

	//-- 2. Call the constructor for StereoBM
	int ndisparities = 16*5;   /**< Range of disparity */
	int SADWindowSize = 13; /**< Size of the block window. Must be odd */

	StereoBM sbm( StereoBM::BASIC_PRESET,
		ndisparities, 
		SADWindowSize );

	sbm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	sbm.state->minDisparity = 0;
	sbm.state->uniquenessRatio = 15;
	sbm.state->disp12MaxDiff = 1;

	//-- 3. Calculate the disparity image
	sbm( imgLeft, imgRight, imgDisparity16S, CV_16S );

	//-- Check its extreme values
	double minVal; double maxVal;

	minMaxLoc( imgDisparity16S, &minVal, &maxVal );
	
	cout << "Min disp: "<< minVal << " Max value: " << maxVal << endl;

	//-- 4. Display it as a CV_8UC1 image
	imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255.0/(maxVal - minVal));

	namedWindow("Disparity", CV_WINDOW_AUTOSIZE);
	imshow("Disparity", imgDisparity8U);

	//-- 5. Save the image
	imwrite("SBM_sample.png", imgDisparity16S);
	waitKey(0);

}

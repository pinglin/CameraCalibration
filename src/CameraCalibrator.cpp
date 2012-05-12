#include "CameraCalibrator.h"

int *show_idx;
int num_frames;
void SpecialKeyFunction(int key, int x, int y)
{

	switch(key)
	{
	case GLUT_KEY_LEFT:
		if(*show_idx > 0)
			(*show_idx)--;
		break;
	case GLUT_KEY_RIGHT:
		if(*show_idx < num_frames-1)
			(*show_idx)++;
		break;
	}

}

int main( int argc, char* argv[])
{

    if(argc < 3)
    {
		cerr << "Usage: [-mono, -stereo] [config.xml]" << endl;
        cerr << "A xml file containing iamge list is required." << endl;
        exit(EXIT_FAILURE);
    }

	string mode(argv[1]);
	CameraCalibration camera_calib;
	if(mode == "-stereo")
	{
		// Stereo camera calibration
		camera_calib.setStereoMode(true);

		string xml_input(argv[2]);
		camera_calib.ReadStereoCalibParams(xml_input);	

		num_frames = camera_calib.getNumFrames();

		camera_calib.StereoCalibration();
		camera_calib.InitPangolin(150);
    
		static Var<int> img_idx("ui.Image: ", 0, 0, camera_calib.getNumFrames()-1);
		static Var<bool> is_stereobind("ui.Stereo Bind", false, true);
		static Var<bool> is_undistorted("ui.Undistorted images", false, true);
		static Var<bool> is_rectified("ui.Rectified images", false, true);
		//static Var<bool> disp_button("ui.Show OpenCV SBM", false, false);
		static Var<bool> export_button("ui.Export Results", false, false);

		while( !pangolin::ShouldQuit() )
		{
			if(pangolin::HasResized())
				DisplayBase().ActivateScissorAndClear();

			show_idx = (int*)img_idx.var->val;
			if(is_rectified)
			{
				for(int c_idx=0; c_idx<2; c_idx++)					
					camera_calib.DrawRectifiedImage(c_idx, img_idx);	

			}
			else
			{
				for(int c_idx=0; c_idx<2; c_idx++)
					camera_calib.DrawChessboardAndImage(c_idx, img_idx, is_undistorted, is_stereobind);
			}

			camera_calib.panel->Render();

			if(Pushed(export_button))
				camera_calib.WriteCalibParams();

			//if(Pushed(disp_button))
			//	camera_calib.OpenCVSBM(camera_calib.calib_params[0].ImageList.at(img_idx), 
			//						   camera_calib.calib_params[1].ImageList.at(img_idx));

			camera_calib.panel->Render();			

			// Swap frames and Process Events
			glutSwapBuffers();
			glutMainLoopEvent();
		}
	}	
	else
	{	

		// Monocular camera calibration
		camera_calib.setStereoMode(false);

		string xml_input(argv[1]);
		camera_calib.ReadStereoCalibParams(xml_input);	

	}            	

    return 0;

}

CameraCalibration::CameraCalibration()
{

}

void CameraCalibration::ReadMonoCalibParams( string &img_xml )
{

	calib_params = new CalibParams;

    FileStorage fs(img_xml, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << img_xml << "\"" << endl;
        exit(EXIT_FAILURE);
    }

    BoardSize = Size((int)fs["BoardSizeWidth"], (int)fs["BoardSizeHeight"]);
    SquareSize = (float)fs["SquareSize"];

	BoardTexWdith = SquareSize*BoardSize.width;
	BoardTexHeight = SquareSize*BoardSize.height;

    FileNode imgs = fs["Images"];
    for(FileNodeIterator itr = imgs.begin(); itr != imgs.end(); itr++)	
        calib_params->ImageList.push_back((string)*itr);
	
	// Check if all image data have the same size.
	Mat img = imread(calib_params->ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
	ImageSize = Size(img.rows, img.cols);
	for(int i=1; i<calib_params->ImageList.size(); i++)
	{
		img = imread(calib_params->ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
		assert(ImageSize == Size(img.rows, img.cols));
		ImageSize = Size(img.rows, img.cols);
	}

    NumFrames = calib_params->ImageList.size();

    fs.release();

}

void CameraCalibration::ReadStereoCalibParams( string &img_xml )
{
	size_t found= img_xml.find_last_of("/\\");
	data_path = img_xml.substr(0,found);

	calib_params = new CalibParams[2];
	stereo_params = new StereoParams;
	rect_params = new RectifiedParams;

	FileStorage fs(img_xml, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << img_xml << "\"" << endl;
		exit(EXIT_FAILURE);
	}

	BoardSize = Size((int)fs["BoardSizeWidth"], (int)fs["BoardSizeHeight"]);
	SquareSize = (float)fs["SquareSize"];

	BoardTexWdith = SquareSize*BoardSize.width;
	BoardTexHeight = SquareSize*BoardSize.height;

	FileNode left_imgs = fs["LeftImages"];
	for(FileNodeIterator itr = left_imgs.begin(); itr != left_imgs.end(); itr++)
		calib_params[0].ImageList.push_back((string)*itr);

	FileNode right_imgs = fs["RightImages"];
	for(FileNodeIterator itr = right_imgs.begin(); itr != right_imgs.end(); itr++)
		calib_params[1].ImageList.push_back((string)*itr);

	// Check if the number of left and right images are the same
	assert(calib_params[0].ImageList.size() == calib_params[1].ImageList.size());
	NumFrames = calib_params[0].ImageList.size();
	
	Mat img = imread(data_path+"/"+calib_params[0].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
	ImageSize = Size(img.cols, img.rows);

	// Check if all image data have the same size, if necessary.
	CheckImageSizeConsistency();

	fs.release();            

}

void CameraCalibration::WriteCalibParams()
{

    time_t t;
    time( &t );

    struct tm *t2 = localtime( &t );

    char buf[1024];
    strftime(buf, sizeof(buf)-1, "%c", t2 );

    FileStorage fs(data_path+"/calib_result.xml", FileStorage::WRITE);

    fs << "calibration_Time" << buf;
    fs << "NumOfFrames" << NumFrames;
    fs << "ImageWidth" << ImageSize.width;
    fs << "ImageHeight" << ImageSize.height;
    fs << "BoardWidth" << BoardSize.width;
    fs << "BoardHeight" << BoardSize.height;
    fs << "SquareSize" << SquareSize;
	if(stereo_mode)
	{
		fs << "CameraType" << "stereo";

		fs << "LeftCameraMatrix" << calib_params[0].CameraMatrix;
		fs << "LeftCameraDistortion" << calib_params[0].DistCoeffs;

		fs << "RightCameraMatrix" << calib_params[1].CameraMatrix;
		fs << "RightCameraDistortion" << calib_params[1].DistCoeffs;

		fs << "LeftToRightRotation" << stereo_params->R;
		fs << "LeftToRightTranslation" << stereo_params->T;
	}
	else
	{
		fs << "CameraType" << "mono";

		fs << "CameraMatrix" << calib_params[0].CameraMatrix;
		fs << "CameraDistortion" << calib_params[0].DistCoeffs;
	}

	
}

void CameraCalibration::MonoCalibration()
{

    vector<vector<Point2f> > ImagePoints;

    for(int i=0; i < NumFrames; i++)
    {

        Mat img = imread(calib_params[0].ImageList.at(i), CV_LOAD_IMAGE_COLOR);
        
        vector<Point2f> pointBuf;
        bool found = false;
        
        found = findChessboardCorners(img, BoardSize, pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);

        
        if(found)
        {
            Mat viewGray;
            cvtColor(img, viewGray, CV_BGR2GRAY);
            cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
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

        ImagePoints.push_back(pointBuf);
        
    }

    vector<vector<Point3f> > ObjectPoints(1);
    for(int i=0; i<BoardSize.height; i++)
        for(int j=0; j<BoardSize.width; j++)
            ObjectPoints.at(0).push_back(Point3f(float( j*SquareSize ),
                                           float( i*SquareSize ),
                                           0));

    ObjectPoints.resize(ImagePoints.size(), ObjectPoints[0]);

    vector<Mat> RVecs, TVecs;

    calib_params->DistCoeffs = Mat::zeros(8, 1, CV_64F);
    
    calib_params->CameraMatrix = initCameraMatrix2D(ObjectPoints, ImagePoints, ImageSize, 0);
    double rms = calibrateCamera(ObjectPoints,
                                 ImagePoints,
                                 ImageSize,
                                 calib_params->CameraMatrix,
                                 calib_params->DistCoeffs,
                                 RVecs,
                                 TVecs,
                                 CV_CALIB_USE_INTRINSIC_GUESS |
                                 CV_CALIB_FIX_K3 |
                                 CV_CALIB_FIX_K4 |
                                 CV_CALIB_FIX_K5 |
                                 CV_CALIB_FIX_K6);

   cout << "Camera re-projection error reported by calibrateCamera: "<< rms << endl;

   // Transfer matrix from OpenCV Mat to Pangolin matrix
   CvtCameraExtrins(&RVecs, &TVecs);

}

void CameraCalibration::StereoCalibration()
{

	vector<vector<Point2f> > ImagePoints[2];

	vector<vector<Point3f> > ObjectPoints(1);
	for(int i=0; i<BoardSize.height; i++)
		for(int j=0; j<BoardSize.width; j++)
			ObjectPoints.at(0).push_back(Point3f(float( j*SquareSize ),
			float( i*SquareSize ),
			0));

	ObjectPoints.resize(NumFrames, ObjectPoints[0]);

	vector<Mat> RVecs[2], TVecs[2];
	double rms;

	for(int c_idx=0; c_idx<2; c_idx++)
	{
		for(int i=0; i < NumFrames; i++)
		{

			Mat img = imread(data_path+"/"+calib_params[c_idx].ImageList.at(i), CV_LOAD_IMAGE_COLOR);

			vector<Point2f> pointBuf;
			bool found = false;

			found = findChessboardCorners(img, BoardSize, pointBuf,
				CV_CALIB_CB_ADAPTIVE_THRESH |
				CV_CALIB_CB_NORMALIZE_IMAGE);

			if(found)
			{
				Mat viewGray;
				cvtColor(img, viewGray, CV_BGR2GRAY);
				cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
					TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));

				//drawChessboardCorners(img, BoardSize, Mat(pointBuf), found);
				//namedWindow("Image View", CV_WINDOW_AUTOSIZE);
				//imshow("Image View", img);
				//waitKey();
			}
			else
			{
				cerr << i << "th image cannot be found a pattern." << endl;
				exit(EXIT_FAILURE);
			}

			ImagePoints[c_idx].push_back(pointBuf);
		}

		calib_params[c_idx].DistCoeffs = Mat::zeros(8, 1, CV_64F);
		calib_params[c_idx].CameraMatrix = initCameraMatrix2D(ObjectPoints, ImagePoints[c_idx], ImageSize, 0);
		rms = calibrateCamera(ObjectPoints, 
			ImagePoints[c_idx],
			ImageSize,
			calib_params[c_idx].CameraMatrix,
			calib_params[c_idx].DistCoeffs,
			RVecs[c_idx],
			TVecs[c_idx],
			CV_CALIB_USE_INTRINSIC_GUESS |
			CV_CALIB_FIX_K3 |	
			CV_CALIB_FIX_K4 |
			CV_CALIB_FIX_K5 |
			CV_CALIB_FIX_K6);

		cout << c_idx << " camera re-projection error reported by calibrateCamera: "<< rms << endl;

	}

	rms = stereoCalibrate(ObjectPoints,
						  ImagePoints[0],
						  ImagePoints[1],
						  calib_params[0].CameraMatrix,
						  calib_params[0].DistCoeffs,
						  calib_params[1].CameraMatrix,
						  calib_params[1].DistCoeffs,
						  ImageSize,
						  stereo_params->R,
						  stereo_params->T,
						  stereo_params->E,
						  stereo_params->F,
						  TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

	cout << "Stereo re-projection error reported by stereoCalibrate: " << rms << endl;

	cout << "Fundamental Matrix reprojection error: " << FundamentalMatrixQuality(ImagePoints[0], ImagePoints[1], 
		calib_params[0].CameraMatrix, calib_params[1].CameraMatrix, 
		calib_params[0].DistCoeffs, calib_params[1].DistCoeffs, stereo_params->F) << endl;        

	// Transfer matrix from OpenCV Mat to Pangolin matrix
	CvtCameraExtrins(RVecs, TVecs);
	
	Timer PangolinTimer;

	// Stereo rectification
	stereoRectify(calib_params[0].CameraMatrix,
		calib_params[0].DistCoeffs,
		calib_params[1].CameraMatrix,
		calib_params[1].DistCoeffs,
		ImageSize, 
		stereo_params->R, 
		stereo_params->T, 
		rect_params->LeftRot,
		rect_params->RightRot,
		rect_params->LeftProj,
		rect_params->RightProj,
		rect_params->Disp2DepthReProjMat,
		CALIB_ZERO_DISPARITY, // test later
		1, // test later
		ImageSize,
		&rect_params->LeftRoi, 
		&rect_params->RightRoi);

	cout << "\nStereo rectification using calibration spent: " << PangolinTimer.getElapsedTimeInMilliSec() << "ms." << endl;

	rect_params->isVerticalStereo = fabs(rect_params->RightProj.at<double>(1, 3)) > 
										fabs(rect_params->RightProj.at<double>(0, 3));

	// Get the rectification re-map index
	initUndistortRectifyMap(calib_params[0].CameraMatrix, calib_params[0].DistCoeffs, 
		rect_params->LeftRot, rect_params->LeftProj,	
		ImageSize, CV_16SC2, rect_params->LeftRMAP[0], rect_params->LeftRMAP[1]);
	initUndistortRectifyMap(calib_params[1].CameraMatrix, calib_params[1].DistCoeffs, 
		rect_params->RightRot, rect_params->RightProj, 
		ImageSize, CV_16SC2, rect_params->RightRMAP[0], rect_params->RightRMAP[1]);

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
	for(int i = 0; i < NumFrames; i++ )
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

void CameraCalibration::CvtCameraExtrins(const vector<Mat> *RVecs, const vector<Mat> *TVecs)
{

    Mat rot3x3;
    OpenGlMatrixSpec P = IdentityMatrix(GlModelViewStack);

	if(stereo_mode)
	{
		CamExtrins = new vector<OpenGlMatrixSpec>[2];
		for(int i=0; i<NumFrames; i++)
		{
			for(int c_idx=0; c_idx<2; c_idx++)
			{
				Rodrigues(RVecs[c_idx].at(i), rot3x3);
				for(int col=0; col<3; col++)
					for(int row=0; row<3; row++)
						P.m[col*4+row] = rot3x3.at<double>(row, col);
				copy(TVecs[c_idx].at(i).ptr<double>(), TVecs[c_idx].at(i).ptr<double>()+3, &P.m[12]);
				CamExtrins[c_idx].push_back(P);
			}
		}

		L2RExtrins = IdentityMatrix(GlModelViewStack);
		for(int col=0; col<3; col++)
			for(int row=0; row<3; row++)
				L2RExtrins.m[col*4+row] = stereo_params->R.at<double>(row, col);
		
		copy(stereo_params->T.ptr<double>(), stereo_params->T.ptr<double>()+3, &L2RExtrins.m[12]);

	}
	else
	{
		CamExtrins = new vector<OpenGlMatrixSpec>;
		for(int i=0; i<NumFrames; i++)
		{
			Rodrigues(RVecs->at(i), rot3x3);
			for(int col=0; col<3; col++)
				for(int row=0; row<3; row++)
					P.m[col*4+row] = rot3x3.at<double>(row, col);
			copy(TVecs->at(i).ptr<double>(), TVecs->at(i).ptr<double>()+3, &P.m[12]);
			CamExtrins->push_back(P);
		}
    }
}

void CameraCalibration::InitPangolin(int PanelWidth)
{

	if(stereo_mode)
	{
		const int WindowWidth = (ImageSize.width)*2+PanelWidth-1;
		const int WindowHeight = ImageSize.height;

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

		CamIntrins = new OpenGlMatrixSpec[2];

		CamIntrins[0] = ProjectionMatrixRDF_TopLeft(ImageSize.width,
													ImageSize.height,
													calib_params[0].CameraMatrix.at<double>(0, 0),
													calib_params[0].CameraMatrix.at<double>(1, 1),
													calib_params[0].CameraMatrix.at<double>(0, 2),
													calib_params[0].CameraMatrix.at<double>(1, 2),
													0.1,
													10000);

		CamIntrins[1] = ProjectionMatrixRDF_TopLeft(ImageSize.width,
													ImageSize.height,
													calib_params[1].CameraMatrix.at<double>(0, 0),
													calib_params[1].CameraMatrix.at<double>(1, 1),
													calib_params[1].CameraMatrix.at<double>(0, 2),
													calib_params[1].CameraMatrix.at<double>(1, 2),
													0.1,
													10000);

		const double panel = (double)(PanelWidth-1)/(double)(WindowWidth-1);
		const double middle_h = ((double)(WindowWidth-PanelWidth)/2.0)/(double)(WindowWidth) + (double)(PanelWidth)/(double)(WindowWidth);

		view[0] = &Display("ViewLeft").SetBounds(1.0, 0, panel, middle_h, -(double)ImageSize.width/(double)ImageSize.height);
		view[1] = &Display("ViewRight").SetBounds(1.0, 0, middle_h, 1.0, -(double)ImageSize.width/(double)ImageSize.height);
		
	}

	InitTexture();

	glutSpecialFunc(&SpecialKeyFunction);
}

void CameraCalibration::DrawAxis() const
{

    float size = SquareSize*10.0;

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

void CameraCalibration::DrawChessboardAndImage(const int c_idx, const int img_idx, const bool is_undistorted, const bool is_stereobind)
{
	
    Mat img = imread(data_path+"/"+calib_params[c_idx].ImageList.at(img_idx), CV_LOAD_IMAGE_COLOR);

    if(is_undistorted)
    {
        Mat undistort_img;
		undistort(img, 
				  undistort_img,
				  calib_params[c_idx].CameraMatrix,
				  calib_params[c_idx].DistCoeffs,
				  calib_params[c_idx].CameraMatrix);

        img = undistort_img;
    }

	view[c_idx]->ActivateScissorAndClear();

	gl_img_tex->Upload(img.ptr<unsigned char>(), GL_BGR, GL_UNSIGNED_BYTE);
	gl_img_tex->RenderToViewportFlipY();
			
	// OpenGL rendering for left and right views
	CamIntrins[c_idx].Load();
	if(!c_idx)
		CamExtrins[c_idx].at(img_idx).Load();
	else
	{
		// right camera
		if(is_stereobind)
			StereoBind(CamExtrins[0].at(img_idx)).Load();
		else
			CamExtrins[c_idx].at(img_idx).Load();
	}

	DrawAxis();
	gl_chessboard_tex->RenderPlanTexture3D(BoardTexWdith, BoardTexHeight);

}

void CameraCalibration::DrawRectifiedImage(const int c_idx, const int img_idx) const
{

	view[c_idx]->ActivateScissorAndClear();		
	Mat img = imread(data_path+"/"+calib_params[c_idx].ImageList.at(img_idx), 0);		
	
	Mat canvas(ImageSize.width, ImageSize.height, CV_8UC3);
	Mat rimg;
	
	if(!c_idx)
	{		
		remap(img, rimg, rect_params->LeftRMAP[0], rect_params->LeftRMAP[1], CV_INTER_LINEAR);
		cvtColor(rimg, canvas, CV_GRAY2RGB);				
		Rect vroi(cvRound(rect_params->LeftRoi.x), cvRound(rect_params->LeftRoi.y),
				  cvRound(rect_params->LeftRoi.width), cvRound(rect_params->LeftRoi.height)); 
		rectangle(canvas, vroi, Scalar(0,0,255), 3, 8);
		
	}else
	{
		remap(img, rimg, rect_params->RightRMAP[0], rect_params->RightRMAP[1], CV_INTER_LINEAR);
		cvtColor(rimg, canvas, CV_GRAY2RGB);				
		Rect vroi(cvRound(rect_params->RightRoi.x), cvRound(rect_params->RightRoi.y),
			cvRound(rect_params->RightRoi.width), cvRound(rect_params->RightRoi.height)); 
		rectangle(canvas, vroi, Scalar(0,0,255), 3, 8);
	}

	if(!rect_params->isVerticalStereo)
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
    MatMul<4, 4, 4, double>(P.m, L2RExtrins.m, LeftCamera.m);

    return P;

}

void CameraCalibration::InitTexture()
{

	GLubyte* checkImage = new GLubyte[8*BoardSize.height*8*BoardSize.width*4];

	int i, j, c;
	for (i = 0; i < 8*BoardSize.height; i++) 
	{
		for (j = 0; j < 8*BoardSize.width; j++) 
		{
			c = ((((i&0x8)==0)^((j&0x8)==0)))*255;
			checkImage[i*(8*BoardSize.width*4)+j*4+0] = (GLubyte) 0;
			checkImage[i*(8*BoardSize.width*4)+j*4+1] = (GLubyte) c;
			checkImage[i*(8*BoardSize.width*4)+j*4+2] = (GLubyte) 0;
			checkImage[i*(8*BoardSize.width*4)+j*4+3] = (GLubyte) 100;
		}
	}

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	// Texture for chessboard	
	gl_chessboard_tex = new pangolin::GlTexture(8*BoardSize.width, 8*BoardSize.height, GL_RGBA);
	gl_chessboard_tex->Upload(checkImage, GL_RGBA, GL_UNSIGNED_BYTE);    
	delete [] checkImage;

	// Texture for image
	gl_img_tex = new pangolin::GlTexture(ImageSize.width, ImageSize.height, GL_RGB);

}

void CameraCalibration::CheckImageSizeConsistency()
{

	if(stereo_mode)
	{
		for(int c_idx=0; c_idx<2; c_idx++)
		{
			Mat img = imread(data_path+"/"+calib_params[c_idx].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
			ImageSize = Size(img.cols, img.rows);
			for(int i=1; i<calib_params[c_idx].ImageList.size(); i++)
			{
				img = imread(data_path+"/"+calib_params[c_idx].ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
				assert(ImageSize == Size(img.cols, img.rows));
				ImageSize = Size(img.cols, img.rows);
			}
		}
	}
	else
	{
		Mat img = imread(data_path+"/"+calib_params[0].ImageList.front(), CV_LOAD_IMAGE_GRAYSCALE);
		ImageSize = Size(img.cols, img.rows);
		for(int i=1; i<calib_params[0].ImageList.size(); i++)
		{
			img = imread(data_path+"/"+calib_params[0].ImageList.at(i), CV_LOAD_IMAGE_GRAYSCALE);
			assert(ImageSize == Size(img.cols, img.rows));
			ImageSize = Size(img.cols, img.rows);
		}

	}

}

//void CameraCalibration::OpenCVSBM( const string &left_img, const string &right_img ) const
//{
//
//	//-- 1. Read the images
//	Mat oriImgLeft, oriImgRight, imgLeft, imgRight;
//	if(calib_params.CamRwrtLExtrins.m[12] < 0)
//	{
//		oriImgLeft = imread(right_img, CV_LOAD_IMAGE_GRAYSCALE);
//		oriImgRight = imread(left_img, CV_LOAD_IMAGE_GRAYSCALE);
//
//		remap(oriImgLeft, imgLeft, rect_params.RightRMAP[0], rect_params.RightRMAP[1], CV_INTER_LINEAR);
//		remap(oriImgRight, imgRight, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1], CV_INTER_LINEAR);
//	}
//	else
//	{
//		oriImgLeft = imread(left_img, CV_LOAD_IMAGE_GRAYSCALE );
//		oriImgRight = imread(right_img, CV_LOAD_IMAGE_GRAYSCALE);
//
//		remap(oriImgLeft, imgLeft, rect_params.LeftRMAP[0], rect_params.LeftRMAP[1], CV_INTER_LINEAR);
//		remap(oriImgRight, imgRight, rect_params.RightRMAP[0], rect_params.RightRMAP[1], CV_INTER_LINEAR);
//	}
//
//	//-- And create the image in which we will save our disparities
//	Mat imgDisparity16S = Mat( imgLeft.rows, imgLeft.cols, CV_16S );
//	Mat imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );
//
//	if( !imgLeft.data || !imgRight.data )
//	{ std::cout<< " --(!) Error reading images " << std::endl; return; }
//
//	//-- 2. Call the constructor for StereoBM
//	int ndisparities = 16*5;   /**< Range of disparity */
//	int SADWindowSize = 13; /**< Size of the block window. Must be odd */
//
//	StereoBM sbm( StereoBM::BASIC_PRESET,
//		ndisparities, 
//		SADWindowSize );
//
//	sbm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
//	sbm.state->minDisparity = 0;
//	sbm.state->uniquenessRatio = 15;
//	sbm.state->disp12MaxDiff = 1;
//
//	//-- 3. Calculate the disparity image
//	sbm( imgLeft, imgRight, imgDisparity16S, CV_16S );
//
//	//-- Check its extreme values
//	double minVal; double maxVal;
//
//	minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//	
//	cout << "Min disp: "<< minVal << " Max value: " << maxVal << endl;
//
//	//-- 4. Display it as a CV_8UC1 image
//	imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255.0/(maxVal - minVal));
//
//	namedWindow("Disparity", CV_WINDOW_AUTOSIZE);
//	imshow("Disparity", imgDisparity8U);
//
//	//-- 5. Save the image
//	imwrite("SBM_sample.png", imgDisparity16S);
//	waitKey(0);
//
//}

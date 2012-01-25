#include "stereo_calib.h"

// must be initialized in constructor as sysPtr = *System
static void *sysPtr;

inline static void routineWrapper(){ ((StereoCalibration*)sysPtr)->Routine(); }
inline static void specialKeyFuncWrapper(int key, int x, int y){ ((StereoCalibration*)sysPtr)->SpecialKeyFunction(key, x, y); }

int main( int argc, char* argv[])
{

    StereoCalibration stereo_calib;
    stereo_calib.InitPangolin();

    specialKeyBindPangolin(specialKeyFuncWrapper);
    runPangolin(routineWrapper);

    return 0;

}

StereoCalibration::StereoCalibration()
{

    sysPtr = this;

}

void StereoCalibration::InitPangolin()
{


    const int WindowWidth = ImageWidth+PanelWidth-1;
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
    view = &Display("cam").SetBounds(1.0, 0.0, panel, 1.0, -(double)ImageWidth/(double)ImageHeight);
    view->SetHandler(new Handler3D(state));

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

    if(HasResized())
        DisplayBase().ActivateScissorAndClear();

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
    view->ActivateScissorAndClear(state);
    glEnable(GL_DEPTH_TEST);
    glColor3f(1.0,1.0,1.0);

    // Render some stuffvoid
    glutWireTeapot(10.0);

    panel->Render();

}

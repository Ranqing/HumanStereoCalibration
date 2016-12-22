#include "qing_stereo_calibrater.h"

int main(int argc, char * argv[])
{    
    cout << "Usage: " << argv[0] << " ../../HumanDatas/Calib A01 A02\t or "
         << "\n" << argv[0] << "../../HumanDatas/Calib ../../HumanDatas/Calib_Results A01 A02" << endl << endl;

    if(argc < 4)
    {
        cerr << "invalid usage.." << endl;
        return -1;
    }

    string dataFolder, monoFolder, cam0, cam1;
    MODE mode ;

    if(4 == argc)
    {
        dataFolder = argv[1];
        cam0 = argv[2];
        cam1 = argv[3];
        mode = WITHOUT_MONO_RESULT;
        cout << "without mono result. mono-calibration came first, then stereo-calibration." << endl;
        cout << dataFolder << endl;
        cout << cam0 << ' ' << cam1 << endl;
    }
    else if(5 == argc)
    {
        dataFolder = argv[1];
        monoFolder = argv[2];
        cam0 = argv[3];
        cam1 = argv[4];
        mode = WITH_MONO_RESULT;
        cout << "with mono result.stereo-calibration directly." << endl;
        cout << dataFolder << endl;
        cout << monoFolder << endl;
        cout << cam0 << ' ' << cam1 << endl;
    }

    Qing_Stereo_Calibrater calibrater;

    if(WITH_MONO_RESULT == mode)
    {
        calibrater.set(mode, dataFolder, monoFolder, cam0, cam1);
    }
    else if(WITHOUT_MONO_RESULT == mode)
    {
        calibrater.set(mode, dataFolder, cam0, cam1);
    }

    calibrater.init_imagenames();
    calibrater.init_calibinfos();
    calibrater.extract_corners();

    //get camera_matrixs and dist_coeffs
    if(WITH_MONO_RESULT == mode)
    {
        calibrater.init_monoresults();
    }
     else if(WITHOUT_MONO_RESULT == mode)
    {
        calibrater.mono_calib();

    }

    calibrater.bino_calib();
    calibrater.rectify();                          //rectification
#if CHECK_RECONS_ERR
    calibrater.reconstruct();
#endif
    calibrater.save();

    return 1;
}

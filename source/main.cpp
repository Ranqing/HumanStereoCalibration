#include "qing_stereo_calibrater.h"

int main(int argc, char * argv[])
{
    cout << "Usage: " << argv[0] << "../../../../HumanDatas/20160711/Calib ../../../../HumanDatas/20160711/Calib_Results A01 A02" << endl;
    cout << endl;
    if(argc < 5)
    {
        cout << "invalid arguments." << endl;
        return -1;
    }

    string dataFolder =  argv[1];
    string monoFolder = argv[2];
    string cam0 = argv[3];
    string cam1 = argv[4];

    cout << dataFolder << endl;
    cout << monoFolder << endl;
    cout << cam0 << ' ' << cam1 << endl;

    Qing_Stereo_Calibrater calibrater(dataFolder, monoFolder, cam0, cam1);
    calibrater.init();
    calibrater.calib();
    calibrater.rectify();
    calibrater.reconstruct();
    calibrater.save();
    return 1;
}

#ifndef QING_STEREO_CALIBRATER_H
#define QING_STEREO_CALIBRATER_H

#include "common.h"

#define CHECK_RECT_ERR 1
#define CHECK_RECONS_ERR 1

class Qing_Stereo_Calibrater
{
public:
    Qing_Stereo_Calibrater()  {}
    ~Qing_Stereo_Calibrater() {}

    //set calibration mode
    void set(const MODE mode, const string& data_folder, const string& mono_folder, const string& cam0, const string& cam1 );
    void set(const MODE mode, const string& data_folder, const string& cam0, const string& cam1);

    MODE m_mode;
    Pattern m_pattern;
    Size m_boardSize, m_imageSize;
    float m_squareSize;
    bool m_isBouguet;

    string m_data_folder, m_mono_folder, m_cam0, m_cam1, m_cam0_folder, m_cam1_folder, m_out_folder, m_out_file;              //input informations
    vector<string> m_image_names0, m_image_names1;                                              //calibration image names
    Mat m_camera_matrix0, m_dist_coeffs0, m_camera_matrix1, m_dist_coeffs1;
    Mat m_stereo_R, m_stereo_T, m_stereo_E, m_stereo_F, m_R0, m_R1, m_P0, m_P1, m_stereo_Q;     //calibration results
    Mat m_mapx0, m_mapy0, m_mapx1, m_mapy1;                                                     //rectified matrix in x and y
    vector<vector<Point2f> > m_image_points0, m_image_points1;                                  //corners of chessboard
    vector<vector<Point3f> > m_3d_points;                                                       //3d points of corners of left chessboard
    double m_max_recons_err, m_avg_recons_err;                                                  //reconstruction of chessboard error
    double m_rms, m_avg_rect_err, m_max_rect_err;                                               //stereo calibration and rectification error

    void init_imagenames();                                         //initialize calibration image names
    void init_monoresults();                                        //initialize mono calibration results
    void init_calibinfos();                                         //initialize calibration flags

    void extract_corners();

    void mono_calib();                                              //mono calibration
    void bino_calib();                                              //bino calibration
    void rectify();                                                 //rectify calibration images
    void reconstruct();                                             //reconstruct calibration board
    void save();

    void cal_reconstruct_error(vector<Point3f>& object_points, double& total_recons_err, double& max_recons_err);
};

#endif // QING_STEREO_CALIBRATER_H

#include "qing_stereo_calibrater.h"
#include "run_and_save_mono.h"

#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_dir.h"
#include "../../../Qing/qing_io.h"
#include "../../../Qing/qing_converter.h"
#include "../../../Qing/qing_ply.h"
#include "../../../Qing/qing_basic.h"

//calibration paras
#define BOARD_W 14
#define BOARD_H 24
#define SQUARE_SIZE 20.0f               //unit : mm

void Qing_Stereo_Calibrater::set(const MODE mode, const string &data_folder, const string &cam0, const string &cam1)
{
    m_mode = mode;
    m_data_folder = data_folder;
    m_cam0 = cam0;
    m_cam1 = cam1;
}

void Qing_Stereo_Calibrater::set(const MODE mode, const string &data_folder, const string &mono_folder, const string &cam0, const string &cam1)
{
    m_mode = mode;
    m_data_folder = data_folder;
    m_mono_folder = mono_folder;
    m_cam0 = cam0;
    m_cam1 = cam1;
}

void Qing_Stereo_Calibrater::init_imagenames()
{
    string dataFn0 = m_data_folder + "/imagelist_" + m_cam0 + ".txt";
    string dataFn1 = m_data_folder + "/imagelist_" + m_cam1 + ".txt";

    m_image_names0.resize(0);
    m_image_names1.resize(0);
    qing_read_txt(dataFn0, m_image_names0);
    qing_read_txt(dataFn1, m_image_names1);
    cout << "read image names done." << endl;

    m_cam0_folder = m_data_folder + "/" + m_cam0;
    m_cam1_folder = m_data_folder + "/" + m_cam1;
    cout << "cam0's folder:  " << m_cam0_folder << endl;
    cout << "cam1's folder:  " << m_cam1_folder << endl;
}

void Qing_Stereo_Calibrater::init_monoresults()
{
    string monoFn0 = m_mono_folder + "/calib_" + m_cam0 + ".yml";
    string monoFn1 = m_mono_folder + "/calib_" + m_cam1 + ".yml";
    qing_read_intrinsic_yml(monoFn0, m_camera_matrix0, m_dist_coeffs0);
    qing_read_intrinsic_yml(monoFn1, m_camera_matrix1, m_dist_coeffs1);
    cout << monoFn0 << endl << m_camera_matrix0 << endl << m_dist_coeffs0 << endl;
    cout << monoFn1 << ": " << endl << m_camera_matrix1 << endl << m_dist_coeffs1 << endl;
    cout << "read intrinsics file done." << endl;
}

void Qing_Stereo_Calibrater::init_calibinfos()
{
    m_pattern = CHESSBOARD;
    if('A' == m_cam0[0] || 'B' == m_cam0[0] || 'C' == m_cam0[0])
        m_boardSize = Size(BOARD_W, BOARD_H);   //14 * 24
    else
        m_boardSize = Size(BOARD_H, BOARD_W);   //24 * 14
    m_squareSize = SQUARE_SIZE;
    m_isBouguet = true;

    m_out_folder = m_cam0 + m_cam1;
    qing_create_dir(m_out_folder);
    cout << m_out_folder << endl;

    m_image_points0.resize(0);
    m_image_points1.resize(0);
    m_3d_points.resize(0);
    cout << "initialization done..." << endl;
}

void Qing_Stereo_Calibrater::extract_corners()
{
    int width, height;
    int successes = 0;
    int nframes = m_image_names0.size();
    string folder0 = m_cam0_folder + "/";
    string folder1 = m_cam1_folder + "/";

    for(int i = 0; i < nframes; ++i )
    {
        Mat view0 = imread(folder0 + m_image_names0[i], 1);
        if(view0.data == NULL)
        {
            cout << "failed to open " << m_image_names0[i] << endl;
            return ;
        }
        Mat view1 = imread(folder1 + m_image_names1[i], 1);
        if(view1.data == NULL)
        {
            cout << "failed to open " << m_image_names1[i] << endl;
            return ;
        }
        Mat grayView0, grayView1;
        cvtColor(view0, grayView0, CV_BGR2GRAY);
        cvtColor(view1, grayView1, CV_BGR2GRAY);

        m_imageSize = view0.size();

        bool found;
        vector<Point2f> corners0(0), corners1(0);

        switch(m_pattern)
        {
        case CHESSBOARD:
            found = findChessboardCorners(grayView0,  m_boardSize, corners0, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS) &
                    findChessboardCorners(grayView1,  m_boardSize, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(grayView0,  m_boardSize, corners0, CALIB_CB_ASYMMETRIC_GRID) &
                    findCirclesGrid(grayView1,  m_boardSize, corners1, CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            cout << "invalid calibration pattern." << endl;
            return;
        }

        if(found)
        {
            cout << "stereo pairs : " << i << endl;
            cornerSubPix(grayView0, corners0, Size(11, 11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            cornerSubPix(grayView1, corners1, Size(11, 11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners(view0, m_boardSize, corners0, found);
            drawChessboardCorners(view1, m_boardSize, corners1, found);

            width =  m_imageSize.width;
            height = m_imageSize.height;
            Mat canvas(height, 2 * width, CV_8UC3);
            Mat part = canvas(Rect(0,0,width,height));
            view0.copyTo(part);
            part = canvas(Rect(width,0,width,height));
            view1.copyTo(part);

            string savefn = m_out_folder + "/corners_" + int2FormatString(i,4, '0') + ".jpg";
            imwrite(savefn, canvas);
        }
        else
        {
            cerr << "stereo pairs : " << i << " discarded." << endl;
            continue;
        }

        //corners into imagepoints
        m_image_points0.push_back(corners0);
        m_image_points1.push_back(corners1);
        successes ++;
    }
    cout << "corners extraction: " << successes << " pairs. " << endl;
}

//copy from mono-calibration
void Qing_Stereo_Calibrater::mono_calib()
{
    if(m_image_points0.empty() || m_image_points1.empty())
    {
        m_image_points0.clear(); m_image_points0.resize(0);
        m_image_points1.clear(); m_image_points1.resize(0);
        cout << "extract corners first...." << endl;
        extract_corners();
    }

    /********************************calibration params***************************************/
    bool writeExtrinsics = false;
    bool writePoints = false;
    int flags = 0;
    bool calibFixPrincipalPoint = 1;
    bool calibZeroTangentDist = 1;
    float aspectRatio = 1.0f;

    if(calibFixPrincipalPoint) flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flags |= CV_CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flags |= CV_CALIB_FIX_ASPECT_RATIO;
    /****************************************************************************************/

    string out_file_0 = m_out_folder + "/calib_" + m_cam0 + ".yml";

    runAndSave(out_file_0, m_image_points0, m_imageSize, m_boardSize, m_pattern,
               m_squareSize, aspectRatio, flags, m_camera_matrix0, m_dist_coeffs0, writeExtrinsics, writePoints);
    cout << "save " << out_file_0 << endl;

    string out_file_1 = m_out_folder + "/calib_" + m_cam1 + ".yml";
    runAndSave(out_file_1, m_image_points1, m_imageSize, m_boardSize, m_pattern,
               m_squareSize, aspectRatio, flags, m_camera_matrix1, m_dist_coeffs1, writeExtrinsics, writePoints);
    cout << "save " << out_file_1 << endl;
}

void Qing_Stereo_Calibrater::bino_calib()
{
    int successes = m_image_points0.size();
    vector<vector<Point3f> > objectPoints(successes);

    for (int k = 0; k < successes; k ++)
    {
        for (int h = 0; h < m_boardSize.height; h ++)
            for (int w = 0; w < m_boardSize.width; w ++)
                objectPoints[k].push_back(Point3f(float(w * m_squareSize), float(h * m_squareSize), 0.0f));
    }

    cout << "run stereo calibration.. use prepared intrinsics and discoeffs." << endl;

    m_rms = stereoCalibrate(objectPoints, m_image_points0, m_image_points1,
                            m_camera_matrix0, m_dist_coeffs0, m_camera_matrix1, m_dist_coeffs1,
                            m_imageSize, m_stereo_R, m_stereo_T, m_stereo_E, m_stereo_F,
                            TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);
    cout << "done with RMS error = " << m_rms << endl ;

}

void Qing_Stereo_Calibrater::rectify()
{
    //计算行校准矩阵并行校准误差
    m_R0.create(3, 3, CV_64F);
    m_R1.create(3, 3, CV_64F);
    m_P0.create(3, 4, CV_64F);
    m_P1.create(3, 4, CV_64F);
    m_stereo_Q.create(4, 4, CV_64F);

    string folder0 = m_cam0_folder + "/";
    string folder1 = m_cam1_folder + "/";

    if(m_isBouguet)
    {
        m_avg_rect_err = 0.f;
        m_max_rect_err = 0.f;

        stereoRectify(m_camera_matrix0, m_dist_coeffs0, m_camera_matrix1, m_dist_coeffs1,
                      m_imageSize, m_stereo_R, m_stereo_T, m_R0, m_R1, m_P0, m_P1, m_stereo_Q, CALIB_ZERO_DISPARITY, 1, m_imageSize);

        cout << "Rectification Matrix Calculation Done." << endl;

#if CHECK_RECT_ERR

        m_mapx0 = Mat::zeros(m_imageSize, CV_32F);
        m_mapy0 = Mat::zeros(m_imageSize, CV_32F);
        m_mapx1 = Mat::zeros(m_imageSize, CV_32F);
        m_mapy1 = Mat::zeros(m_imageSize, CV_32F);

        bool isVertical =  fabs(m_P1.at<double>(1,3)) > fabs(m_P1.at<double>(0,3));

        initUndistortRectifyMap(m_camera_matrix0, m_dist_coeffs0, m_R0, m_P0, m_imageSize, CV_32FC1, m_mapx0, m_mapy0);
        initUndistortRectifyMap(m_camera_matrix1, m_dist_coeffs1, m_R1, m_P1, m_imageSize, CV_32FC1, m_mapx1, m_mapy1);

        int width = m_imageSize.width;
        int height = m_imageSize.height;
        m_avg_rect_err = 0.0;
        m_image_points0.clear(); m_image_points0.resize(0);
        m_image_points1.clear(); m_image_points1.resize(0);

        int numOfCorners = 0;
        for(int i = 0; i < m_image_names0.size(); ++i)
        {
            Mat view0, gray_view0, view1, gray_view1;
            Mat new_view0, new_gray_view0, new_view1, new_gray_view1;

            view0 = imread( folder0 + m_image_names0[i], 1);
            cvtColor(view0, gray_view0, CV_BGR2GRAY);
            view1 = imread( folder1 + m_image_names1[i], 1);
            cvtColor(view1,gray_view1, CV_BGR2GRAY);

            if(view0.data == NULL || view1.data == NULL)
            {
                cerr << "failed to open stereo images " << endl;
                continue;
            }

            remap(view0, new_view0, m_mapx0, m_mapy0, CV_INTER_LINEAR);
            remap(gray_view0, new_gray_view0, m_mapx0, m_mapy0, CV_INTER_LINEAR);
            remap(view1, new_view1, m_mapx1, m_mapy1, CV_INTER_LINEAR);
            remap(gray_view1, new_gray_view1, m_mapx1, m_mapy1, CV_INTER_LINEAR);

            vector<Point2f> corners0(0), corners1(0);

            bool found = findChessboardCorners(new_gray_view0, m_boardSize, corners0, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS) &
                    findChessboardCorners(new_gray_view1, m_boardSize, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS) ;

            if(found == true)
            {
                cornerSubPix(new_gray_view0, corners0, Size(11, 11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                cornerSubPix(new_gray_view1, corners1, Size(11, 11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                m_image_points0.push_back(corners0);
                m_image_points1.push_back(corners1);
            }
            else
                continue;

            Mat canvas, part;
            if(isVertical == false)
                canvas.create(height, width * 2, CV_8UC3);
            else
                canvas.create(2 * height, width, CV_8UC3);

            part = canvas(Rect(0,0,width,height));
            new_view0.copyTo(part);
            part = isVertical ? canvas(Rect(0, height, width, height)) : canvas(Rect(width,0,width,height));
            new_view1.copyTo(part);

            for(int j = 0; j < corners0.size(); ++j)
            {
                Point2f pt0 = corners0[j];
                Point2f pt1 = corners1[j] + (isVertical ? Point2f(0,height) : Point2f(width,0));
                circle(canvas, pt0, 2, Scalar(255,0,0), 1);
                circle(canvas, pt1, 2, Scalar(255,0,0), 1);
                line(canvas, pt0, pt1, Scalar(255,0,0), 1);

                numOfCorners ++;
                m_avg_rect_err += fabs(pt1.y - pt0.y);

                if(fabs(pt1.y - pt0.y) > m_max_rect_err)
                {
                    m_max_rect_err = fabs(pt0.y - pt1.y);
                }
            }

            string savefn = m_out_folder + "/rectified_" +  int2FormatString(i, 4, '0') + ".jpg";
            imwrite(savefn, canvas);
        }

        if(numOfCorners != 0)
            m_avg_rect_err /= numOfCorners;

#endif
    }
    else
    {
        //HartleyRectify(folder, leftCam, rightCam, lImageList, rImageList, lIntrinsic, lDistCoeffs, rIntrinsic, rDistCoeffs, imageSize, R, T, E, F);
    }
}

void Qing_Stereo_Calibrater::reconstruct()
{
    int numOfFrames  = m_image_points0.size();
    int numOfCorners = m_boardSize.width * m_boardSize.height;

    m_max_recons_err = 0.f;
    m_avg_recons_err = 0.f;

    for(int i = 0; i < numOfFrames; ++i)
    {
        vector<Point3f> object_points(0);
        for(int j = 0; j < numOfCorners; ++j)
        {
            Mat uvd(4, 1, CV_64FC1) ;
            Point2f pt0 = m_image_points0[i][j];
            Point2f pt1 = m_image_points1[i][j];
            uvd.at<double>(0,0) = pt0.x;
            uvd.at<double>(1,0) = pt0.y;
            uvd.at<double>(2,0) = pt0.x - pt1.x;
            uvd.at<double>(3,0) = 1.0;

            Mat xyzw = m_stereo_Q * uvd;
            Point3f pt;
            pt.x = xyzw.at<double>(0,0) / xyzw.at<double>(3, 0);
            pt.y = xyzw.at<double>(1,0) / xyzw.at<double>(3, 0);
            pt.z = xyzw.at<double>(2,0) / xyzw.at<double>(3, 0);
            object_points.push_back(pt);
        }
        m_3d_points.push_back(object_points);
        string savefile = m_out_folder + "/reconstruct_chessboard_" + int2FormatString(i, 4, '0') + ".xyz";
        qing_write_xyz(savefile, object_points);

        cout << "reconstruct frame " << i << ", " << object_points.size() << " points. saving in " << savefile << '\t';

        cal_reconstruct_error(object_points, m_avg_recons_err, m_max_recons_err);
    }
    //  m_avg_recons_err /= ( numOfFrames *  ( m_boardSize.width - 1 ) * m_boardSize.height );
    m_avg_recons_err /= (numOfFrames * ( ( m_boardSize.width - 1 ) * m_boardSize.height +
                                         m_boardSize.width * (m_boardSize.height - 1) ) );

    cout << "average_reconstruct_err = " << m_avg_recons_err << ", max_reconstruct_err = " << m_max_recons_err << endl;
}

void Qing_Stereo_Calibrater::cal_reconstruct_error(vector<Point3f> &object_points, double &total_recons_err, double &max_recons_err)
{
    int w = m_boardSize.width;
    int h = m_boardSize.height;

    //horizon
    for(int i = 0; i < h; ++i)
    {
        for(int j = 0; j < w-1; ++j)
        {
            int idx0 = i * w + j;
            int idx1 = idx0 + 1;

            float dis = qing_euclidean_dis(object_points[idx1] , object_points[idx0]);
            total_recons_err += dis;

            if(dis > max_recons_err)
                max_recons_err = dis;
        }
    }

    // vertical
    for(int i = 0; i < h-1; ++i)
    {
        for(int j = 0; j < w; ++j)
        {
            int idx0 = i * w + j;
            int idx1 = idx0 + w;

            float dis = qing_euclidean_dis(object_points[idx1] , object_points[idx0]);
            total_recons_err += dis;

            if(dis > max_recons_err )
                max_recons_err = dis;
        }
    }

    cout << "max_recons_err = " << max_recons_err << endl;
}

void Qing_Stereo_Calibrater::save()
{
    m_out_file = m_out_folder + "/stereo_" + m_cam0 + m_cam1 + ".yml";
    FileStorage fs(m_out_file, FileStorage::WRITE);

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "Calibration_Time"  << buf;
    fs << "Rectify_Method"    << "Bouguet";
    fs << "Left_Camera_Name"  << m_cam0;
    fs << "Right_Camera_Name" << m_cam1;
    fs << "Image_Width"       << m_imageSize.width;
    fs << "Image_Height"      << m_imageSize.height;
    fs << "RMS"               << m_rms ;
 #if CHECK_RECT_ERR
    fs << "Average_Rectified_Error"   << m_avg_rect_err ;
    fs << "Max_Rectified_Error"       << m_max_rect_err - m_squareSize;
 #endif
 #if CHECK_RECONS_ERR
    fs << "Average_Reconstruct_Error" << m_avg_recons_err - m_squareSize ;
    fs << "Max_Reconstruct_Error"     << m_max_recons_err ;
#endif
    fs << "Rotation_Matrix"    << m_stereo_R;
    fs << "Translation_Vector" << m_stereo_T;
    fs << "Eigen_Matrix"       << m_stereo_E;
    fs << "Fundamental_Matrix" << m_stereo_F;
    fs << "Q" << m_stereo_Q;
#if CHECK_RECT_ERR
    fs << "R1" << m_R0;
    fs << "R2" << m_R1;
    fs << "P1" << m_P0;
    fs << "P2" << m_P1;
#endif
    fs.release();
}

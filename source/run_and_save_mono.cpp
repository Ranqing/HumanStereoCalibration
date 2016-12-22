#include "run_and_save_mono.h"

double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType)
{
    corners.resize(0);

    switch(patternType) {
    case CHESSBOARD:
   // case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

    default:
        cout <<  "Unknown pattern type\n";
    }
}

bool runCalibration(const vector<vector<Point2f> >& imagePoints,
                           const Size& imageSize, const Size& boardSize, Pattern patternType,
                           const float& squareSize, const float& aspectRatio,  int flags,
                           Mat& cameraMatrix, Mat& distCoeffs,
                           vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs,
                           double& totalAvgErr, double &avgErrMe)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                 flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5 );

    cout << "Re-projection error reported by CalibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
void saveCameraParams( const string& outfile,
                              const Size& imageSize, const Size& boardSize,
                              const float& squareSize, const float& aspectRatio, const int flags,
                              const Mat& cameraMatrix, const Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs,
                              const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, double avgErrMe )
{
    FileStorage fs( outfile, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "Calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "Frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "Image_Width" << imageSize.width;
    fs << "Image_Height" << imageSize.height;
    fs << "Board_Width" <<  boardSize.width;
    fs << "Board_Height" << boardSize.height;
    fs << "Square_Size" <<  squareSize;

    if( flags & CALIB_FIX_ASPECT_RATIO )
        fs << "AspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
                 flags & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                 flags & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                 flags & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                 flags & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "Flags_Value" << flags;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_Points" << imagePtMat;
    }
}

void runAndSave(const string& outfile,
                       const vector<vector<Point2f> >& imagePoints,
                       const Size& imageSize,	const Size& boardSize,
                       Pattern pattern,  const float& squareSize,	const float& aspectRatio,  int flags,
                       Mat& cameraMatrix, Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;

    double totalAvgErr = 0;
    double avgErrMe = 0;

    cout << "imagesize "   <<  imageSize << endl;
    cout << "boardsize "   << boardSize << endl;
    cout << "squaresize "  << squareSize << endl;
    cout << "aspectratio " << aspectRatio << endl;


    bool ok = runCalibration(imagePoints, imageSize, boardSize, pattern, squareSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr, avgErrMe);

    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outfile, imageSize, boardSize, squareSize, aspectRatio, flags, cameraMatrix, distCoeffs,
                          writeExtrinsics ? rvecs : vector<Mat>(),
                          writeExtrinsics ? tvecs : vector<Mat>(),
                          writeExtrinsics ? reprojErrs : vector<float>(),
                          writePoints ? imagePoints : vector<vector<Point2f> >(),
                          totalAvgErr , avgErrMe);
}

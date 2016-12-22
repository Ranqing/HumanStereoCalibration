#ifndef RUN_AND_SAVE_MONO_H
#define RUN_AND_SAVE_MONO_H

#include "common.h"

double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors);

void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD);

bool runCalibration(const vector<vector<Point2f> >& imagePoints,
                           const Size& imageSize, const Size& boardSize, Pattern patternType,
                           const float& squareSize, const float& aspectRatio,  int flags,
                           Mat& cameraMatrix, Mat& distCoeffs,
                           vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs,
                           double& totalAvgErr, double &avgErrMe);

// Print camera parameters to the output file
void saveCameraParams( const string& outfile,
                              const Size& imageSize, const Size& boardSize,
                              const float& squareSize, const float& aspectRatio, const int flags,
                              const Mat& cameraMatrix, const Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs,
                              const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, double avgErrMe );

void runAndSave(const string& outfile,
                       const vector<vector<Point2f> >& imagePoints,
                       const Size& imageSize,	const Size& boardSize,
                       Pattern pattern,  const float& squareSize,	const float& aspectRatio,  int flags,
                       Mat& cameraMatrix, Mat& distCoeffs, bool writeExtrinsics, bool writePoints);



#endif // RUN_AND_SAVE_MONO_H

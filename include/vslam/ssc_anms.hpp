#ifndef SSC_ANMS_HPP
#define SSC_ANMS_HPP

// This is an implmentation of SSC-ANMS (Adaptive-Non maximum suppression)
// for fast and homogeneous repartition of the keypoints in the image.

// https://github.com/BAILOOL/ANMS-Codes/tree/master

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

std::vector<cv::KeyPoint> DistributedKeypoint(std::vector<cv::KeyPoint> orbKeypoint, 
                    int numRetPoints, float tolerance, int cols, int rows)
{
    int exp1 = rows + cols + 2 * numRetPoints;
    long long exp2 =
        ((long long)4 * cols + (long long)4 * numRetPoints +
        (long long)4 * rows * numRetPoints + (long long)rows * rows +
        (long long)cols * cols - (long long)2 * rows * cols +
        (long long)4 * rows * cols * numRetPoints);
    double exp3 = sqrt(exp2);
    double exp4 = numRetPoints - 1;

    double sol1 = -round((exp1 + exp3) / exp4); // first solution
    double sol2 = -round((exp1 - exp3) / exp4); // second solution

    // binary search range initialization with positive solution
    int high = (sol1 > sol2) ? sol1 : sol2;
    int low = floor(sqrt((double)orbKeypoint.size() / numRetPoints));
    low = std::max(1, low);

    int width;
    int prevWidth = -1;

    std::vector<int> ResultVec;
    bool complete = false;
    unsigned int K = numRetPoints;
    unsigned int Kmin = round(K - (K * tolerance));
    unsigned int Kmax = round(K + (K * tolerance));

    std::vector<int> result;
    result.reserve(orbKeypoint.size());

    while (!complete)
    {
        width = low + (high - low) / 2;
        if (width == prevWidth || low > high)
        {
            ResultVec = result; 
            break;
        }

        result.clear();
        double c = (double)width / 2.0; // initializing Grid
        int numCellCols = floor(cols / c);
        int numCellRows = floor(rows / c);

        std::vector<std::vector<bool>> coveredVec(numCellRows + 1,
                            std::vector<bool>(numCellCols + 1, false));

        for (unsigned int i = 0; i < orbKeypoint.size(); ++i)
        {
            int row = floor(orbKeypoint[i].pt.y / c);
            int col = floor(orbKeypoint[i].pt.x / c);
            if (coveredVec[row][col] == false)
            {
                result.push_back(i);
                int rowMin = ((row - floor(width / c)) >= 0)
                        ? (row - floor(width / c)) : 0;
                int rowMax = ((row + floor(width / c)) <= numCellRows)
                        ? (row + floor(width / c)) : numCellRows;
                int colMin = ((col - floor(width / c)) >= 0) 
                        ? (col - floor(width / c)) : 0;
                int colMax = ((col + floor(width / c)) <= numCellCols)
                        ? (col + floor(width / c)) : numCellCols;

                for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov)
                {
                    for (int colToCov = colMin; colToCov <= colMax; ++colToCov)
                    {
                        if (!coveredVec[rowToCov][colToCov])
                            coveredVec[rowToCov][colToCov] = true;
                    }
                }
            }
        }

        if (result.size() >= Kmin && result.size() <= Kmax)
        {
            ResultVec = result;
            complete = true;
        }
        else if (result.size() < Kmin)
            high = width - 1;
        else
            low = width + 1;
        prevWidth = width;  
    }

    std::vector<cv::KeyPoint> distKeypoint;
    for (unsigned int i = 0; i < ResultVec.size(); i++)
        distKeypoint.push_back(orbKeypoint[ResultVec[i]]);

    return distKeypoint;
}

#endif // SSC_ANMS_HPP
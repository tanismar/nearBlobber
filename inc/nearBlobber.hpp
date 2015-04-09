/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Tanis Mar, Giulia Pasquale
 * email:  tanis.mar@iit.it, giulia.pasquale@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __NEARBLOBBER_H__
#define __NEARBLOBBER_H__

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

class nearBlobber
{

	int margin;

	int backgroundThresh;
    int frontThresh;

    int minBlobSize;
    int gaussSize;
    
    int imageThreshRatioLow;
    int imageThreshRatioHigh;
    
    cv::Scalar blue, green, red, white;

    cv::Mat aux, fillMask;

public:

    nearBlobber(int imH, int imW,
    		int _margin,
    		int _backgroundThresh, int _frontThresh,
    		int _minBlobSize, int _gaussSize,
    		int _dispThreshRatioLow, int _dispThreshRatioHigh);

    bool setThresh(int low, int high);
    bool setMargin(int mrg);

    double extractBlob(std::vector<cv::Mat> &images, std::vector<int> &roi, std::vector<int> &centroid, cv::Mat &blob);
       
};

#endif

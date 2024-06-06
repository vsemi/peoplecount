/*
 * ToFImage.cpp
 *
 *  Created on: Jun. 12, 2021
 *      Author: vsemi
 */
#include "ToFImage.hpp"

#include <iostream>

ToFImage::ToFImage(int w, int h) : width(w), height(h) {
	n_points            = w * h;
	data_depth          = new float[n_points];
	data_3d_xyz_rgb     = new float[n_points * 8];
	data_2d_bgr         = new uint8_t[n_points * 3];
	saturated_mask      = new uint8_t[n_points];
	data_grayscale      = new uint8_t[n_points];
	data_amplitude      = new float[n_points];
}
ToFImage::~ToFImage()
{
	delete[] data_depth;
	delete[] data_3d_xyz_rgb;
	delete[] data_2d_bgr;
	delete[] saturated_mask;
	delete[] data_grayscale;
	delete[] data_amplitude;
}

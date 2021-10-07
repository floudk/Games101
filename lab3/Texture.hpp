//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v){
        if(u<0) u=0;
        if(u>1) u=1;
        if(v<0) v=0;
        if(v>1) v=1; 
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto  u_1 = std::floor(u_img);
        auto  v_1 = std::floor(v_img);
        auto  u_2 = std::min(std::ceil(u_img),float(width));
        auto  v_2 = std::min(std::ceil(v_img),float(height));
        auto color_11 = image_data.at<cv::Vec3b>(v_1, u_1);
        auto color_12 = image_data.at<cv::Vec3b>(v_1, u_2);
        auto color_21 = image_data.at<cv::Vec3b>(v_2, u_1);
        auto color_22 = image_data.at<cv::Vec3b>(v_2, u_2);
        //first
        float rx = (v_img-v_1)/(v_2-v_1);
        auto color_x1= color_11*rx + color_21*(1-rx);
        auto color_x2= color_12*rx + color_22*(1-rx);
        float ry = (u_img-u_1)/(u_2-u_1);
        auto color = ry*color_x1 + (1-ry)*color_x2;
        return Eigen::Vector3f(color[0], color[1], color[2]);
        // float u_min = std::floor(u_img);
		// float u_max = std::min((float)width, std::ceil(u_img));
		// float v_min = std::floor(v_img);
		// float v_max = std::min((float)height, std::ceil(v_img));
		
		// auto Q11 = image_data.at<cv::Vec3b>(v_max, u_min);
		// auto Q12 = image_data.at<cv::Vec3b>(v_max, u_max);

		// auto Q21 = image_data.at<cv::Vec3b>(v_min, u_min);
		// auto Q22 = image_data.at<cv::Vec3b>(v_min, u_max);

		// float rs = (u_img - u_min) / (u_max - u_min);
		// float rt = (v_img - v_max) / (v_min - v_max);
		// auto cBot = (1 - rs) * Q11 + rs * Q12;
		// auto cTop = (1 - rs) * Q21 + rs * Q22;
		// auto P = (1 - rt) * cBot + rt * cTop;

		// return Eigen::Vector3f(P[0], P[1], P[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H

//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

//本类的作用是从图片生成纹理
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

    //这个接口的主要作用是从生成的纹理中查找纹理颜色的接口
    //因为纹理对应的uv坐标系的范围是0-1，而这里没有对其坐标范围做限制，自己增加限制

    Eigen::Vector3f getColor(float u, float v)
    {
        //对u，v做限制，限制在0-1之间
        u = std::fmin(1,fmax(u,0)); //小于0设置为0，大于1设置为1
        v = std::fmin(1,fmax(v,0));
        
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H

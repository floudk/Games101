#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(std::vector<cv::Point2f> &control_points, float t,int left) 
{
    // TODO: Implement de Casteljau's algorithm
    int right = control_points.size()-1;
    if(left==right)
        return control_points.back();
    while(right>left){
        control_points[right]= control_points[right] *t + control_points[right-1]*(1-t);
        right--;
    }
    return recursive_bezier(control_points,t,left+1);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    auto local_points = control_points;
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {

        auto point = recursive_bezier(local_points,t,0);
        
        cv::Point2f pointup(point.x,point.y+1),pointdown(point.x,point.y-1),pointleft(point.x-1,point.y),pointright(point.x+1,point.y);
        double d1=sqrt(pow(point.x - pointup.x, 2.0) + pow(point.y - pointup.y, 2.0)),
               d2=sqrt(pow(point.x - pointdown.x, 2.0) + pow(point.y - pointdown.y, 2.0)),
               d3=sqrt(pow(point.x - pointleft.x, 2.0) + pow(point.y - pointleft.y, 2.0)),
               d4=sqrt(pow(point.x - pointright.x, 2.0) + pow(point.y - pointright.y, 2.0));


        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        window.at<cv::Vec3b>(pointup.y, pointup.x)[1] = std::max(255/d1,double(window.at<cv::Vec3b>(pointup.y, pointup.x)[1]));
        window.at<cv::Vec3b>(pointdown.y, pointdown.x)[1] = std::max(255/d2,double(window.at<cv::Vec3b>(pointdown.y, pointdown.x)[1]));
        window.at<cv::Vec3b>(pointleft.y, pointleft.x)[1] = std::max(255/d3,double(window.at<cv::Vec3b>(pointleft.y, pointleft.x)[1]));
        window.at<cv::Vec3b>(pointright.y, pointright.x)[1] = std::max(255/d4,double(window.at<cv::Vec3b>(pointright.y, pointright.x)[1] ));
        local_points = control_points;

    }
    //anti-anlising

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}

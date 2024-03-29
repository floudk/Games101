#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) //V，相当于坐标转换，将视角置于原点
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;//做一个平移

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,//绕y轴逆时针旋转angle
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;//x,y,z放大2.5倍
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;//平移，但这里单位矩阵没动过
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    //先scale，再旋转angle,
    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{   //根据eye_fov和长宽比和给定zNear做视角变换和透视投影变换
    // 实际可以在计算阶段简化矩阵从而一步得出最终的矩阵
    //45.0, 1, 0.1, 50
    //先将frustum挤压成长方体---->做透视投影转换为标准立方体
    Eigen::Matrix4f projection;
    eye_fov = eye_fov * MY_PI / 180.f;
    float t=-abs(zNear)*tan(eye_fov/2);
    float r=t*aspect_ratio;
    projection << zNear/r,0,0,0,
                0,zNear/t,0,0,
                0,0,(zNear+zFar)/(zNear-zFar),-(2*zNear*zFar/(zNear-zFar)),
                0,0,1,0;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);
    
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f ambient= 2*ka.cwiseProduct(amb_light_intensity);
    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //diffuse = kd*I/r^2*max(0,n.l)
        float r_square = (light.position-point).squaredNorm();
        Eigen::Vector3f h = (eye_pos-point).normalized()+(light.position-point).normalized();
        diffuse += kd.cwiseProduct(light.intensity/r_square)*std::max(0.0f,normal.dot((light.position-point).normalized()));
        //specular = ks*I/r^s*(max(0,n,h))^p
        specular+= ks.cwiseProduct(light.intensity/r_square)*std::pow(std::max(0.0f,normal.dot(h.normalized())),p);

    }
    result_color=diffuse+specular+ambient;
    return result_color * 255.f;
}


Eigen::Vector3f bio_texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColorBilinear(payload.tex_coords[0],payload.tex_coords[1]);
        //return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f ambient= ka.cwiseProduct(amb_light_intensity);
    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //diffuse = kd*I/r^2*max(0,n.l)
        float r_square = (light.position-point).squaredNorm();
        Eigen::Vector3f h = (eye_pos-point).normalized()+(light.position-point).normalized();
        diffuse += kd.cwiseProduct(light.intensity/r_square)*std::max(0.0f,normal.dot((light.position-point).normalized()));
        //specular = ks*I/r^s*(max(0,n,h))^p
        specular+= ks.cwiseProduct(light.intensity/r_square)*std::pow(std::max(0.0f,normal.dot(h.normalized())),p);

    }
    result_color=diffuse+specular+ambient;
    return result_color * 255.f;
}
Eigen::Vector3f my_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColorBilinear(payload.tex_coords[0],payload.tex_coords[1]);
        //return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);
    
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

        float kh = 0.2, kn = 0.1;
    
    //TODO: Implement bump mapping here
    //Let n = normal = (x, y, z)
    Eigen::Vector3f n=normal;
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t;
    float _tmp = std::sqrt(n.x()*n.x()+n.z()*n.z());
    t<< n.x()*n.y()/_tmp,_tmp,n.z()*n.y()/_tmp;
    // Vector b = n cross product t
    Eigen::Vector3f b = n.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t,b,n;
    float u=payload.tex_coords[0], v=payload.tex_coords[1],w=payload.texture->width,h=payload.texture->height;
    float h_uv_norm = payload.texture->getColor(u,v).norm();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float dU = kh * kn *(payload.texture->getColor(u+1.0f/w,v).norm()-h_uv_norm);
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn *(payload.texture->getColor(u,v+1.0f/h).norm()-h_uv_norm);
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln;
    ln<<(dU*-1),(dV*-1),1;
    // Normal n = normalize(TBN * ln)
    n = (TBN*ln).normalized();
    point+=kn*n*h_uv_norm;
    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f ambient= 2*ka.cwiseProduct(amb_light_intensity);
    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //diffuse = kd*I/r^2*max(0,n.l)
        float r_square = (light.position-point).squaredNorm();
        Eigen::Vector3f h = (eye_pos-point).normalized()+(light.position-point).normalized();
        diffuse += kd.cwiseProduct(light.intensity/r_square)*std::max(0.0f,normal.dot((light.position-point).normalized()));
        //specular = ks*I/r^s*(max(0,n,h))^p
        specular+= ks.cwiseProduct(light.intensity/r_square)*std::pow(std::max(0.0f,normal.dot(h.normalized())),p);

    }
    result_color=diffuse+specular+ambient;
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);//环境光照
    Eigen::Vector3f kd = payload.color;//漫反射
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);//高光

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};//方向，强度
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};//环境光
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
    // components are. Then, accumulate that result on the *result_color* object.
    //环境光ka   ka * Ia
    Vector3f ambient= amb_light_intensity.cwiseProduct(ka);
    Vector3f diffuse{0,0,0};
    Vector3f specular{0,0,0};
    Vector3f v_normal=(eye_pos-point).normalized();
    for (auto light : lights){ //light 有属性 position & intensity
        //散射光kd   kd* Id/r^2 * max(0,n.dot(l)); n:点的法向量normal  l：指向光的向量
        Vector3f l=(light.position-point).normalized();
        float r_square = (light.position-point).squaredNorm();
        diffuse+=kd.cwiseProduct(std::max(0.0f,normal.dot(l))*light.intensity/r_square);
        //高光ks     ks*Is/r^2 * max(0.n.dot(h))^p   h:半程向量，视角和光的中间向量
        Eigen::Vector3f h=(v_normal+l).normalized();
        specular+=ks.cwiseProduct(std::pow(std::max(0.0f,normal.dot(h)),p)*light.intensity/r_square);
    }
    result_color+=ambient;
    result_color+=diffuse;
    result_color+=specular;
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; //颜色
    Eigen::Vector3f point = payload.view_pos;//物体矩阵
    Eigen::Vector3f normal = payload.normal;//深度

    float kh = 0.2, kn = 0.1;
    
    //TODO: Implement bump mapping here
    //Let n = normal = (x, y, z)
    Eigen::Vector3f n=normal;
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t;
    float _tmp = std::sqrt(n.x()*n.x()+n.z()*n.z());
    t<< n.x()*n.y()/_tmp,_tmp,n.z()*n.y()/_tmp;
    // Vector b = n cross product t
    Eigen::Vector3f b = n.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t,b,n;

    float u=payload.tex_coords[0], v=payload.tex_coords[1],w=payload.texture->width,h=payload.texture->height;
    float h_uv_norm = payload.texture->getColor(u,v).norm();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float dU = kh * kn *(payload.texture->getColor(u+1.0f/w,v).norm()-h_uv_norm);
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn *(payload.texture->getColor(u,v+1.0f/h).norm()-h_uv_norm);
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln;
    ln<<(dU*-1),(dV*-1),1;
    // Normal n = normalize(TBN * ln)
    n = (TBN*ln).normalized();

    point+=kn*n*h_uv_norm;

    Eigen::Vector3f result_color = {0,0,0};
    Eigen::Vector3f ambient= ka.cwiseProduct(amb_light_intensity);
    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //diffuse = kd*I/r^2*max(0,n.l)
        float r_square = (light.position-point).squaredNorm();

        Eigen::Vector3f h = (eye_pos-point).normalized()+(light.position-point).normalized();
        diffuse += kd.cwiseProduct(light.intensity/r_square)*std::max(0.0f,normal.dot((light.position-point).normalized()));
        //specular = ks*I/r^s*(max(0,n,h))^p
        specular+= ks.cwiseProduct(light.intensity/r_square)*std::pow(std::max(0.0f,normal.dot(h.normalized())),p);

    }
    result_color += specular+diffuse+ambient;
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    //TODO: Implement bump mapping here
    //Let n = normal = (x, y, z)
    Eigen::Vector3f n=normal;
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t;
    float _tmp = std::sqrt(n.x()*n.x()+n.z()*n.z());
    t<< n.x()*n.y()/_tmp,_tmp,n.z()*n.y()/_tmp;
    // Vector b = n cross product t
    Eigen::Vector3f b = n.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t,b,n;
    float u=payload.tex_coords[0], v=payload.tex_coords[1],w=payload.texture->width,h=payload.texture->height;
    float h_uv_norm = payload.texture->getColor(u,v).norm();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float dU = kh * kn *(payload.texture->getColor(u+1.0f/w,v).norm()-h_uv_norm);
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn *(payload.texture->getColor(u,v+1.0f/h).norm()-h_uv_norm);
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln;
    ln<<(dU*-1),(dV*-1),1;
    // Normal n = normalize(TBN * ln)
    n = (TBN*ln).normalized();


    Eigen::Vector3f result_color = n;
    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.jpg";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);
    // std::cout << "Rasterizing success\n";
    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));
    
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;
    //dy defult, we use phong_shader

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "biotext")
        {
            std::cout << "Rasterizing using the bio_texture_fragment_shader\n";
            active_shader = bio_texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "myshader")
        {
            std::cout << "Rasterizing using myshader\n";
            active_shader = my_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader); //return payload.position,顶点shader
    r.set_fragment_shader(active_shader);//
    

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//set color to 0, set depth to inf
        r.set_model(get_model_matrix(angle));//M，调整模型，并作为model的输入
        r.set_view(get_view_matrix(eye_pos));//V, 调整视角，将其作为view矩阵，并在后续继续调整model
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
        
        r.draw(TriangleList);



        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);
        
        return 0;
    }

    while(key != 27)   //做旋转，如果带有参数，则永远不会触发
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}

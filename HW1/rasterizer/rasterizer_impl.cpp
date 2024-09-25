#include <cstdint>

#include <iostream>
#include "image.hpp"
#include "loader.hpp"
#include "rasterizer.hpp"




bool test_inside(const Triangle &tri, float x, float y){
    auto A1 = glm::vec3(tri.pos[0].x, tri.pos[0].y, 0);
    auto A2 = glm::vec3(tri.pos[1].x, tri.pos[1].y, 0);
    auto A3 = glm::vec3(tri.pos[2].x, tri.pos[2].y, 0);
    auto X = glm::vec3(x, y, 0);

    auto S1 = glm::cross((X - A1), (A2 - A1));
    auto S2 = glm::cross((X - A2), (A3 - A2));
    auto S3 = glm::cross((X - A3), (A1 - A3));

    if(glm::sign(S1.z) == glm::sign(S2.z) && (glm::sign(S2.z) == glm::sign(S3.z))){
        return true;
    }else{
        return false;
    }
}


// TODO

void Rasterizer::DrawPixel(uint32_t x, uint32_t y, Triangle trig, AntiAliasConfig config, uint32_t spp, Image& image, Color color)
{
    if (config == AntiAliasConfig::NONE)            // if anti-aliasing is off
    {
        if (test_inside(trig, (float)x+0.5, (float)y+0.5)){
            image.Set(x, y, color);
        }
    }
    else if (config == AntiAliasConfig::SSAA)       // if anti-aliasing is on
    {   
        uint32_t sp_each = (uint32_t)glm::sqrt(spp);
        float leach = 1.0/sp_each;

        std::vector<glm::vec3> samples;
        for(float k = leach/2; k < 1.0f; k+=leach){
            for(float l = leach/2; l < 1.0f; l+=leach){
                auto sample_x = x+k;
                auto sample_y = y+l;
                samples.push_back(glm::vec3(sample_x, sample_y, 0));
            }
        }

        uint32_t count = 0;
        for (const auto &sample: samples){
            if(test_inside(trig, sample.x, sample.y)){
                count++;
            }
        }
        
        image.Set(x, y, ((float)count/spp) * color);

    }

    // if the pixel is inside the triangle
    //image.Set(x, y, color);

    return;
}


// TODO
void Rasterizer::AddModel(MeshTransform transform, glm::mat4 rotation)
{
    /* model.push_back( model transformation constructed from translation, rotation and scale );*/
    auto Tx = transform.translation[0];
    auto Ty = transform.translation[1];
    auto Tz = transform.translation[2];

    auto Sx = transform.scale[0];
    auto Sy = transform.scale[1];
    auto Sz = transform.scale[2];

    // note that glm mat4, is transpose of real
    glm::mat4 M_trans{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        Tx, Ty, Tz, 1,
    };

    glm::mat4 M_scale{
        Sx, 0, 0, 0,
        0, Sy, 0, 0,
        0, 0, Sz, 0,
        0, 0, 0, 1
    };

    glm::mat4 M_model = M_trans * rotation * M_scale;
    model.push_back(M_model);
    return;
}

// TODO
void Rasterizer::SetView()
{
    const Camera& camera = this->loader.GetCamera();
    glm::vec3 cameraPos = camera.pos;

    // You need both the position of the camera and lookAt vector to compute the camera direction.
    // It would be more intuitive to aim the camera at a certain position, rather than specifying the direction directly.
    // lookat g
    glm::vec3 cameraLookAt = glm::normalize(camera.lookAt - camera.pos);
    // up t
    glm::vec3 cameraLookUp = glm::normalize(camera.up);
    // binormal gxt
    glm::vec3 cameraCross = glm::normalize(glm::cross(cameraLookAt, cameraLookUp));

    glm::mat4 M_rot{ 
        cameraCross.x, cameraLookUp.x, -cameraLookAt.x, 0,
        cameraCross.y, cameraLookUp.y, -cameraLookAt.y, 0,
        cameraCross.z, cameraLookUp.z, -cameraLookAt.z, 0,     
        0, 0, 0, 1
    };


    glm::mat4 M_trans{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        -cameraPos.x, -cameraPos.y, -cameraPos.z, 1
    };

    this->view = M_rot*M_trans;

    return;
}

// TODO
void Rasterizer::SetProjection()
{
    const Camera& camera = this->loader.GetCamera();

    float nearClip = camera.nearClip;                   // near clipping distance, strictly positive
    float farClip = camera.farClip;                     // far clipping distance, strictly positive
    
    float width = camera.width;
    float height = camera.height;
    
    // TODO change this line to the correct projection matrix
    glm::mat4 M_persp2ortho{
        -nearClip, 0, 0, 0,
        0, -nearClip, 0, 0,
        0, 0, -nearClip-farClip,     1,
        0, 0, -(nearClip * farClip), 0,
    };

    glm::mat4 M_ortho_scale{
        2/width, 0, 0, 0,
        0, 2/height, 0, 0,
        0, 0, 2/(farClip - nearClip), 0,
        0, 0, 0, 1
    };

    glm::mat4 M_ortho_trans{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, (nearClip+farClip)/2, 1
    };


    this->projection = M_ortho_scale * M_ortho_trans * M_persp2ortho;

    return;
}

// TODO
void Rasterizer::SetScreenSpace()
{
    float width = this->loader.GetWidth();
    float height = this->loader.GetHeight();

    // TODO change this line to the correct screenspace matrix
    this->screenspace = glm::mat4{
        width/2, 0, 0, 0,
        0, height/2, 0, 0,
        0, 0, 1, 0,
        width/2, height/2, 0, 1
    };


    return;
}

// TODO
glm::vec3 Rasterizer::BarycentricCoordinate(glm::vec2 pos, Triangle trig)
{
    float alpha;
    float beta;
    float gamma;

    glm::vec2 vA = trig.pos[0]; 
    glm::vec2 vB = trig.pos[1];
    glm::vec2 vC = trig.pos[2];

    alpha = (-(pos.x - vB.x ) * (vC.y - vB.y) +
            (pos.y - vB.y ) * (vC.x - vB.x))/(-(vA.x -vB.x) * (vC.y - vB.y) + 
            (vA.y - vB.y) * (vC.x - vB.x));
    beta = (-(pos.x - vC.x) * (vA.y - vC.y) + 
           (pos.y - vC.y) * (vA.x - vC.x)) / (-(vB.x - vC.x) * (vA.y - vC.y) + 
           (vB.y - vC.y) * (vA.x - vC.x));

    gamma = 1.0f - alpha - beta;
    return glm::vec3(alpha, beta, gamma);
}

// TODO
float Rasterizer::zBufferDefault = (float)-2.0;

// TODO
void Rasterizer::UpdateDepthAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, ImageGrey& ZBuffer)
{

    glm::vec3 bc = BarycentricCoordinate(glm::vec2(x, y), transformed);
    float alpha = bc[0];
    float beta = bc[1];
    float gamma = bc[2];

    if(alpha >= 0.0f && beta >= 0.0f && gamma >= 0.0f){
        float inverse_z = 1.0f/(alpha * 1.0f/transformed.pos[0].z + 
                      beta* 1.0f/transformed.pos[1].z + 
                      gamma * 1.0f/transformed.pos[2].z);
       
        if(inverse_z > ZBuffer.Get(x, y).value()){
            float result = inverse_z;
            ZBuffer.Set(x, y, result);
        }
    }
    return;
}

// TODO
void Rasterizer::ShadeAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, Image& image)
{
    Color L_ambt, L_diff, L_spec;
    Color result(0.0f,0.0f,0.0f,255.0f);
    
    auto lights = this->loader.GetLights();
    const glm::vec3 v_pos = this->loader.GetCamera().pos;
    L_ambt = this->loader.GetAmbientColor();
    result = result + L_ambt;
    auto exp = this->loader.GetSpecularExponent();

    glm::vec3 bc = BarycentricCoordinate(glm::vec2(x, y), transformed);

    float alpha = bc[0];
    float beta = bc[1];
    float gamma = bc[2]; 
    
    if(alpha >= 0.0f && beta >= 0.0f && gamma >= 0.0f)
    {
        float inverse_z = 1.0f/(alpha * 1.0f/transformed.pos[0].z + beta* 1.0f/transformed.pos[1].z + gamma * 1.0f/transformed.pos[2].z);
        if(inverse_z == ZBuffer.Get(x, y).value())
        {

            glm::vec3 obj_pos = alpha * original.pos[0] + beta * original.pos[1] + gamma * original.pos[2];
            glm::vec3 un_normal = alpha * original.normal[0] + beta * original.normal[1] + gamma * original.normal[2];
            glm::vec3 V_normal = glm::normalize(un_normal);
            glm::vec3 v_dir = glm::normalize(v_pos-obj_pos); 

            for(const Light& light : lights)
            {   
                glm::vec3 l_dir = glm::normalize(light.pos-obj_pos);
                glm::vec3 h_dir = glm::normalize(l_dir+v_dir);

                float radius = glm::length(light.pos-obj_pos);
                float r_square = radius * radius;
            
                float max_l_n = glm::max(glm::dot(V_normal, l_dir), 0.0f);
                float const_ld = light.intensity * (1.0f/r_square) * max_l_n;
                L_diff = const_ld * light.color;

                float max_n_h = glm::max(glm::dot(V_normal, h_dir), 0.0f);
                float pow = glm::pow(max_n_h, exp);
                float const_ls = light.intensity * (1.0f/r_square) * pow;
                L_spec = const_ls * light.color;

                result = result + L_diff + L_spec;
            }


            image.Set(x, y, result);

        }
    }

    return;
}

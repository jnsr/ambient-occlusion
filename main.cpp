/*
Base Source Used From: https://github.com/ssloy/tinyrenderer
Additions & Modifications by Jamal Nasser
CPSC 4050 Final Project - Ambient Occlusion Renderer
*/


#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model *model     = NULL;
const int width  = 800;
const int height = 800;

Vec3f       eye(0,0,5);
Vec3f    center(0,0,0);
Vec3f        up(0,1,0);

struct ScreenSpace : public IShader {

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = Viewport*Projection*ModelView*gl_Vertex;     // transform it to screen coordinates
        return gl_Vertex;
    };

};


int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/blender_scene_jamal.obj");
    }

    lookat(eye, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection(-1.f/(eye-center).norm());
    inverse_matrix();

    TGAImage image  (width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);
    
    //Fill the Z-Buffer first
    ScreenSpace shader;
    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        fillZbuffer(screen_coords, zbuffer);
    }
    //Shade with Ambient Occlusion Shader
    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        AOShader(screen_coords, i, model, image, zbuffer);
        std::cerr << i << std::endl;
    }

    image.  flip_vertically(); // to place the origin in the bottom left corner of the image
    image.  write_tga_file("output.tga");

    delete model;
    return 0;
}

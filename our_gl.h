#ifndef __OUR_GL_H__
#define __OUR_GL_H__

#include "tgaimage.h"
#include "geometry.h"
#include "model.h"

extern Matrix ModelView;
extern Matrix Viewport;
extern Matrix Projection;
extern Matrix Inverse;

void viewport(int x, int y, int w, int h);
void projection(float coeff=0.f); // coeff = -1/c
void lookat(Vec3f eye, Vec3f center, Vec3f up);
void inverse_matrix();

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert) = 0;
};

void fillZbuffer(Vec4f *pts, TGAImage &zbuffer);
void AOShader(Vec4f *pts, int faceNum, Model *m, TGAImage &image, TGAImage &zbuffer);

#endif //__OUR_GL_H__


#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix ModelView;
Matrix Viewport;
Matrix Projection;
Matrix Inverse;


IShader::~IShader() {}

void viewport(int x, int y, int w, int h) {
    Viewport = Matrix::identity();
    Viewport[0][3] = x+w/2.f;
    Viewport[1][3] = y+h/2.f;
    Viewport[2][3] = 255.f/2.f;
    Viewport[0][0] = w/2.f;
    Viewport[1][1] = h/2.f;
    Viewport[2][2] = 255.f/2.f;
}

void projection(float coeff) {
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = cross(up,z).normalize();
    Vec3f y = cross(z,x).normalize();
    ModelView = Matrix::identity();
    for (int i=0; i<3; i++) {
        ModelView[0][i] = x[i];
        ModelView[1][i] = y[i];
        ModelView[2][i] = z[i];
        ModelView[i][3] = -center[i];
    }
}

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

unsigned my_rand(void)
{
  static unsigned next1=1151752134u, next2=2070363486u;
  next1 = next1 * 1701532575u + 571550083u;
  next2 = next2 * 3145804233u + 4178903934u;
  return (next1<<16)^next2;
}
float U_m1_p1(){
  return float(my_rand())*(1.0f/2147483648.0f) - 1.0f;
}
Vec3f pick_random_point_in_sphere(){
  float x0,x1,x2,x3,d2;
  do{
    x0=U_m1_p1();
    x1=U_m1_p1();
    x2=U_m1_p1();
    x3=U_m1_p1();
    d2=x0*x0+x1*x1+x2*x2+x3*x3;
  }while(d2>1.0f);
  float scale = 1.0f/d2;
  return Vec3f(2*(x1*x3+x0*x2)*scale,
                  2*(x2*x3+x0*x1)*scale,
                  (x0*x0+x3*x3-x1*x1-x2*x2)*scale);
}
//Get random vector within hemisphere oriented by vector
Vec3f getRandHemi(Vec3f const &v){
  Vec3f result=pick_random_point_in_sphere();
  if(result*v<0){
    result.x=-result.x;
    result.y=-result.y;
    result.z=-result.z;
  }
  return result;
}

/*=====================================================================*/
// Modified or added code for ambient occlusion below                  //
/*=====================================================================*/

void inverse_matrix(){
    //Matrix used to go BACK from screen space to world space

    Matrix cumulative = (Viewport*Projection*ModelView).invert_transpose();
    Inverse = cumulative;

    //Transpose the transpose to get back to original...
    //Not sure why the invert function also transposes the result
    Inverse[0][3] = cumulative[3][0];
    Inverse[1][3] = cumulative[3][1];
    Inverse[3][0] = Inverse[3][1] = 0.0;
    Inverse[3][2] = cumulative[2][3];
    Inverse[2][3] = cumulative[3][2];
}

// Implementation of Möller–Trumbore intersection algorithm to check for ray hits
bool rayIntersection(Vec3f rayOrigin, Vec3f rayDir, Vec3f * tri){
    const float EPSILON = 0.0000001;
    Vec3f e1, e2, h, s, q;
    float a,f,u,v;
    e1 = tri[1] - tri[0];
    e2 = tri[2] - tri[0];
    h = cross(rayDir, e2);
    a = e1*h;
    if (a > -EPSILON && a < EPSILON) return false;
    f = 1.0/a;
    s = rayOrigin - tri[0];
    u = f * (s * h);
    if (u < 0.0 || u > 1.0) return false;
    q = cross(s, e1);
    v = f * (rayDir*q);
    if (v < 0.0 || u + v > 1.0) return false;
    float t = f * (e2*q);
    if (t > EPSILON && t < .5) return true;

    else return false;
}

bool castRay(Vec3f start, Vec3f dir, int currFace, Model *m){
    for (int i=0; i<m->nfaces(); i++) {
        if(currFace == i) continue; //Don't intersect with self
        std::vector<int> face = m->face(i); 
        Vec3f verts[3]; 
        for (int j=0; j<3; j++) verts[j] = m->vert(face[j]); 
        if(rayIntersection(start, dir, verts))
            return true;
    }
    return false;
}

const int N_RAYS = 50;
int occlusionSampler(Vec3f origin_pt, Vec3f norm, int face, Model *m){
    float c = 255.0;
    for(int i = 0; i < N_RAYS; i++){
        Vec3f randir = getRandHemi(norm).normalize();
        bool hit = castRay(origin_pt, randir, face, m);
        if(hit) c *= 0.90; //For every 'hit' reduce intensity by 10%
    }
    return c;
}

//Populate the Z-Buffer with final scene values, smaller values are closer to the image plane
void fillZbuffer(Vec4f *pts, TGAImage &zbuffer) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
        }
    }
    Vec2i P;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
            if (c.x<0 || c.y<0 || c.z<0 || zbuffer.get(P.x, P.y)[0]>frag_depth) continue;
            zbuffer.set(P.x, P.y, TGAColor(frag_depth));
        }
    }
}
//Shade the given triangle with the ambient occlusion algorithm, triangle vertices are in 'pts'
void AOShader(Vec4f *pts, int faceNum, Model *m, TGAImage &image, TGAImage &zbuffer) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
        }
    }
    Vec2i P;
    Vec3f n = m->normal(faceNum, 1);
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
            if (c.x<0 || c.y<0 || c.z<0 || zbuffer.get(P.x, P.y)[0]>frag_depth) continue;

                Vec4f v;
                v[0] = P.x;
                v[1] = P.y;
                v[2] = (float)frag_depth;
                v[3] = w;
                v = Inverse*v;

                Vec3f world;
                world[0] = v[0];
                world[1] = v[1];
                world[2] = v[2];


                int clr = occlusionSampler(world, n, faceNum, m);
                image.set(P.x, P.y, TGAColor(clr, clr, clr));

        }
    }
}

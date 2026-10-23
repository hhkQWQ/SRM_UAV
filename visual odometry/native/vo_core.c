/* Original implementation for RM, 2026. Algorithmic references are documented
 * in SOURCES.md; no upstream implementation is copied into this file. */
#include "vo_core.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>

#define P(c, name) ((c)->cfg[V_##name])
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
static float clampf(float x, float lo, float hi) { return MAX(lo, MIN(hi, x)); }
static int inside(float u, float v, float margin) {
    return isfinite(u) && isfinite(v) && u >= margin && v >= margin &&
           u < VO_WIDTH-1-margin && v < VO_HEIGHT-1-margin;
}
static float sample(const uint8_t *im, size_t stride, float u, float v) {
    int x = (int)u, y = (int)v;
    float a = u-x, b = v-y;
    const uint8_t *p = im+(size_t)y*stride+(size_t)x;
    return (1-b)*((1-a)*p[0]+a*p[1])+b*((1-a)*p[stride]+a*p[stride+1]);
}
static void rotate(const float *r, float x, float y, float z, float *a) {
    for (int i=0; i<3; ++i) a[i]=r[3*i]*x+r[3*i+1]*y+r[3*i+2]*z;
}
static int ground(const vo_context_t *c, const float *m, float u, float v, float *g) {
    float xd=(u-P(c,CX))/P(c,FX), yd=(v-P(c,CY))/P(c,FY), x=xd, y=yd;
    for (int i=0; i<6; ++i) {
        float r=x*x+y*y, k=1+P(c,K1)*r+P(c,K2)*r*r+P(c,K3)*r*r*r;
        if (!isfinite(k) || fabsf(k)<1e-8f) return 0;
        float tx=2*P(c,P1)*x*y+P(c,P2)*(r+2*x*x);
        float ty=P(c,P1)*(r+2*y*y)+2*P(c,P2)*x*y;
        x=(xd-tx)/k; y=(yd-ty)/k;
    }
    float ray[3]; rotate(m,x,y,1,ray);
    if (!isfinite(ray[2]) || ray[2]<=P(c,RAY)) return 0;
    g[0]=m[9]+m[12]*ray[0]/ray[2]; g[1]=m[10]+m[12]*ray[1]/ray[2];
    return isfinite(g[0]) && isfinite(g[1]);
}
static int pixel(const vo_context_t *c, const float *m, float gx, float gy, float *uv) {
    gx-=m[9]; gy-=m[10];
    float z=m[2]*gx+m[5]*gy+m[8]*m[12];
    if (!isfinite(z) || z<1e-5f) return 0;
    float x=(m[0]*gx+m[3]*gy+m[6]*m[12])/z;
    float y=(m[1]*gx+m[4]*gy+m[7]*m[12])/z;
    float r=x*x+y*y, k=1+P(c,K1)*r+P(c,K2)*r*r+P(c,K3)*r*r*r;
    uv[0]=P(c,FX)*(x*k+2*P(c,P1)*x*y+P(c,P2)*(r+2*x*x))+P(c,CX);
    uv[1]=P(c,FY)*(y*k+P(c,P1)*(r+2*y*y)+2*P(c,P2)*x*y)+P(c,CY);
    return isfinite(uv[0]) && isfinite(uv[1]);
}
static int predicted_pixel(const vo_context_t *c, const vo_frame_t *ref,
                           const float *m, const float *pred, float u, float v, float *uv) {
    float g[2];
    return ground(c,ref->meta,u,v,g) && pixel(c,m,
        g[0]+ref->position[0]-pred[0],g[1]+ref->position[1]-pred[1],uv);
}
static int warp(const vo_context_t *c, const vo_frame_t *ref, const float *m,
                const float *pred, float u, float v, float *uv, float *inverse) {
    float a[2],b[2];
    if (!predicted_pixel(c,ref,m,pred,u,v,uv) ||
        !predicted_pixel(c,ref,m,pred,u+1,v,a) ||
        !predicted_pixel(c,ref,m,pred,u,v+1,b)) return 0;
    float j0=a[0]-uv[0],j1=b[0]-uv[0],j2=a[1]-uv[1],j3=b[1]-uv[1];
    float det=j0*j3-j1*j2;
    if (!isfinite(det) || fabsf(det)<.04f || fabsf(det)>25) return 0;
    inverse[0]=j3/det; inverse[1]=-j1/det;
    inverse[2]=-j2/det; inverse[3]=j0/det;
    return 1;
}
/* Warp only a local template, retaining original QVGA coordinates. */
static int template_patch(uint8_t *patch, const uint8_t *im, size_t stride,
                          float u, float v, const float *inverse) {
    for (int y=0; y<VO_PATCH; ++y) for (int x=0; x<VO_PATCH; ++x) {
        float dx=(float)(x-4),dy=(float)(y-4);
        float a=u+inverse[0]*dx+inverse[1]*dy,b=v+inverse[2]*dx+inverse[3]*dy;
        if (!inside(a,b,0)) return 0;
        patch[y*VO_PATCH+x]=(uint8_t)(sample(im,stride,a,b)+.5f);
    }
    return 1;
}
/* 8x8 integer SAD. M7 DSP variant uses unaligned-safe memcpy word loads. */
static uint32_t sad8(const uint8_t *patch,const uint8_t *im,size_t stride,int u,int v) {
    uint32_t cost=0;
    for (int y=0;y<8;++y) {
        const uint8_t *a=patch+8*y,*b=im+(size_t)(v-4+y)*stride+(size_t)(u-4);
#if defined(__ARM_FEATURE_DSP) && defined(__GNUC__) && !defined(VO_SCALAR_SAD)
        for(int x=0;x<8;x+=4) {
            uint32_t s,t; memcpy(&s,a+x,4); memcpy(&t,b+x,4);
            __asm__ volatile ("usada8 %0, %1, %2, %0" : "+r"(cost) : "r"(s), "r"(t));
        }
#else
        for(int x=0;x<8;++x) cost+=(uint32_t)abs((int)a[x]-(int)b[x]);
#endif
    }
    return cost;
}
/* Translation KLT on the affine-warped original template. Gradients only in
 * the current 8x8 window, with per-window brightness offset eliminated. */
static int klt(const vo_context_t *c,const uint8_t *patch,const uint8_t *im,
               size_t stride,float *u,float *v) {
    for (int it=0;it<(int)P(c,KLT_ITERS);++it) {
        if(!inside(*u,*v,6)) return 0;
        float xx=0,xy=0,yy=0,xe=0,ye=0,sx=0,sy=0,se=0,ee=0;
        for(int y=0;y<8;++y) for(int x=0;x<8;++x) {
            float a=*u+x-4,b=*v+y-4;
            float e=sample(im,stride,a,b)-patch[y*8+x];
            float gx=.5f*(sample(im,stride,a+1,b)-sample(im,stride,a-1,b));
            float gy=.5f*(sample(im,stride,a,b+1)-sample(im,stride,a,b-1));
            xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;xe+=gx*e;ye+=gy*e;
            sx+=gx;sy+=gy;se+=e;ee+=e*e;
        }
        xx-=sx*sx/64;xy-=sx*sy/64;yy-=sy*sy/64;
        xe-=sx*se/64;ye-=sy*se/64;
        float det=xx*yy-xy*xy,tr=xx+yy;
        float eig=.5f*(tr-sqrtf(MAX(0,(xx-yy)*(xx-yy)+4*xy*xy)))/64;
        if(det<1e-6f || eig<P(c,MIN_EIG)) return 0;
        float du=(yy*xe-xy*ye)/det,dv=(xx*ye-xy*xe)/det;
        if(!isfinite(du) || !isfinite(dv) || fabsf(du)>4 || fabsf(dv)>4) return 0;
        *u-=du;*v-=dv;
        if(du*du+dv*dv<.0009f) {
            return sqrtf(MAX(0,ee-se*se/64)/64)<=P(c,KLT_ERROR) && inside(*u,*v,6);
        }
    }
    return 0; /* unconverged tracks are not measurements */
}
static int backward(const vo_context_t *c,const uint8_t *current,size_t stride,
                    const uint8_t *ref,float u,float v,float ru,float rv,const float *inverse) {
    float det=inverse[0]*inverse[3]-inverse[1]*inverse[2];
    if(fabsf(det)<1e-6f) return 0;
    float j[4]={inverse[3]/det,-inverse[1]/det,-inverse[2]/det,inverse[0]/det};
    uint8_t patch[64];
    if(!template_patch(patch,current,stride,u,v,j)) return 0;
    float bu=ru,bv=rv;
    return klt(c,patch,ref,VO_WIDTH,&bu,&bv) &&
        (bu-ru)*(bu-ru)+(bv-rv)*(bv-rv)<=P(c,FB)*P(c,FB);
}
static int track_point(vo_context_t *c,const uint8_t *ref,const uint8_t *im,size_t stride,
                       float ru,float rv,float *uv,const float *inverse,float radius) {
    if(!template_patch(c->patch,ref,VO_WIDTH,ru,rv,inverse)) return 0;
    float u=uv[0],v=uv[1];
    if(klt(c,c->patch,im,stride,&u,&v) && backward(c,im,stride,ref,u,v,ru,rv,inverse)) {
        uv[0]=u;uv[1]=v;return 1;
    }
    int cx=(int)lroundf(uv[0]),cy=(int)lroundf(uv[1]),r=(int)radius;
    int lo_x=MAX(6,cx-r),hi_x=MIN(VO_WIDTH-8,cx+r);
    int lo_y=MAX(6,cy-r),hi_y=MIN(VO_HEIGHT-8,cy+r);
    int step=MAX(1,MIN(2,r/8)),bx=0,by=0;
    uint32_t best=UINT_MAX;
    for(int y=lo_y;y<=hi_y;y+=step) for(int x=lo_x;x<=hi_x;x+=step) {
        uint32_t cost=sad8(c->patch,im,stride,x,y);
        if(cost<best) {best=cost;bx=x;by=y;}
    }
    int x0=MAX(lo_x,bx-step),x1=MIN(hi_x,bx+step);
    int y0=MAX(lo_y,by-step),y1=MIN(hi_y,by+step);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
        uint32_t cost=sad8(c->patch,im,stride,x,y);
        if(cost<best) {best=cost;bx=x;by=y;}
    }
    if(best==UINT_MAX || best>64*P(c,KLT_ERROR)*1.5f) return 0;
    u=(float)bx;v=(float)by;
    if(!klt(c,c->patch,im,stride,&u,&v) || !backward(c,im,stride,ref,u,v,ru,rv,inverse)) return 0;
    uv[0]=u;uv[1]=v;return 2;
}
static int cell(float u,float v) {
    int x=MAX(0,MIN(2,(int)(u*3/VO_WIDTH))),y=MAX(0,MIN(2,(int)(v*3/VO_HEIGHT)));
    return y*3+x;
}
static int fast_score(const uint8_t *im,size_t stride,int x,int y,int threshold) {
    static const int8_t dx[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
    static const int8_t dy[16]={-3,-3,-2,-1,0,1,2,3,3,3,2,1,0,-1,-2,-3};
    int center=im[(size_t)y*stride+(size_t)x],diff[16];
    for(int k=0;k<16;++k) diff[k]=(int)im[(size_t)(y+dy[k])*stride+(size_t)(x+dx[k])]-center;
    int score=0;
    for(int start=0;start<16;++start) {
        int bright=255,dark=255;
        for(int k=0;k<9;++k) {
            int d=diff[(start+k)&15];bright=MIN(bright,d);dark=MIN(dark,-d);
            if(bright<=threshold && dark<=threshold) break;
        }
        score=MAX(score,MAX(bright,dark));
    }
    return score>threshold?score:0;
}
/* 6x4 grid; refill under-populated cells, preserving accepted tracked points.
 * Scan QVGA samples directly (no reduced image or pyramid). */
static int replenish(vo_context_t *c,const uint8_t *im,size_t stride,int count) {
    int limit=(int)P(c,FEATURES),quota=(limit+23)/24;
    int counts[24]={0};
    for(int i=0;i<count;++i) {
        int x=MIN(5,(int)(c->current[i].x*6/VO_WIDTH)),y=MIN(3,(int)(c->current[i].y*4/VO_HEIGHT));
        counts[6*y+x]++;
    }
    for(int grid=0;grid<24 && count<limit;++grid) {
        if(counts[grid]>=quota) continue;
        vo_point_t candidates[3]={{0}};
        int x0=MAX(12,(grid%6)*VO_WIDTH/6),x1=MIN(VO_WIDTH-12,(grid%6+1)*VO_WIDTH/6);
        int y0=MAX(12,(grid/6)*VO_HEIGHT/4),y1=MIN(VO_HEIGHT-12,(grid/6+1)*VO_HEIGHT/4);
        for(int y=y0;y<y1;y+=2) for(int x=x0;x<x1;x+=2) {
            int s=fast_score(im,stride,x,y,(int)P(c,FAST));
            if(s<=0) continue;
            int close=0;
            for(int i=0;i<count;++i) if(fabsf(c->current[i].x-x)<10 && fabsf(c->current[i].y-y)<10) {close=1;break;}
            if(close) continue;
            for(int k=0;k<3;++k) {
                if(candidates[k].score && fabsf(candidates[k].x-x)<10 && fabsf(candidates[k].y-y)<10) {
                    if(s>candidates[k].score) candidates[k]=(vo_point_t){(float)x,(float)y,(float)s};
                    close=1;break;
                }
            }
            if(close) continue;
            int weakest=0;
            for(int k=1;k<3;++k) if(candidates[k].score<candidates[weakest].score) weakest=k;
            if(s>candidates[weakest].score) candidates[weakest]=(vo_point_t){(float)x,(float)y,(float)s};
        }
        while(counts[grid]<quota && count<limit) {
            int best=0;
            for(int k=1;k<3;++k) if(candidates[k].score>candidates[best].score) best=k;
            if(!candidates[best].score) break;
            c->current[count++]=candidates[best];candidates[best].score=0;counts[grid]++;
        }
    }
    return count;
}
static int add_pair(vo_context_t *c,int n,const float *rm,const float *m,float au,float av,float bu,float bv) {
    float a[2],b[2];
    if(!inside(au,av,0) || !inside(bu,bv,0) || !ground(c,rm,au,av,a) || !ground(c,m,bu,bv,b)) return n;
    c->pairs[n]=(vo_pair_t){au,av,bu,bv,a[0],a[1],b[0],b[1]};return n+1;
}
static float residual(const vo_context_t *c,const float *m,const vo_pair_t *p,float tx,float ty,float co,float si) {
    float x=p->ax-tx,y=p->ay-ty,uv[2];
    if(!pixel(c,m,co*x+si*y,-si*x+co*y,uv)) return 1e20f;
    return (uv[0]-p->bu)*(uv[0]-p->bu)+(uv[1]-p->bv)*(uv[1]-p->bv);
}
static int classify(vo_context_t *c,int n,const float *m,float tx,float ty,float yaw,float gate,float *cost) {
    int count=0;*cost=0;float co=cosf(yaw),si=sinf(yaw);
    for(int i=0;i<n;++i) {
        float e=residual(c,m,&c->pairs[i],tx,ty,co,si);
        c->inlier[i]=(uint8_t)(e<=gate*gate);
        if(c->inlier[i]) {++count;*cost+=e;}
    }
    return count;
}
static int least_squares(vo_context_t *c,int n,float *tx,float *ty,float *yaw) {
    float ax=0,ay=0,bx=0,by=0;int count=0;
    for(int i=0;i<n;++i) if(c->inlier[i]) {vo_pair_t *p=&c->pairs[i];ax+=p->ax;ay+=p->ay;bx+=p->bx;by+=p->by;++count;}
    if(count<2) return 0;
    ax/=count;ay/=count;bx/=count;by/=count;
    float dot=0,cross=0,spread=0;
    for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *p=&c->pairs[i];float x=p->bx-bx,y=p->by-by,u=p->ax-ax,v=p->ay-ay;
        dot+=x*u+y*v;cross+=x*v-y*u;spread+=x*x+y*y;
    }
    if(spread<1e-8f) return 0;
    *yaw=atan2f(cross,dot);
    if(fabsf(*yaw)>P(c,YAW)) return 0;
    float co=cosf(*yaw),si=sinf(*yaw);
    *tx=ax-co*bx+si*by;*ty=ay-si*bx-co*by;
    return isfinite(*tx) && isfinite(*ty);
}
/* Displacement consensus -> SE(2) fit -> residual reclassification -> refit.
 * A bounded two-pair RANSAC is used only if this first pass is insufficient. */
static int fit(vo_context_t *c,int n,const float *m) {
    c->output[O_MATCHES]=(float)n;
    if(n<(int)P(c,MATCHES)) return 0;
    float gate=P(c,GATE),best_cost=1e30f,best_tx=0,best_ty=0,best_yaw=0;
    int best_count=0,iterations=0;
    float coarse=4*gate*MIN(m[12],c->key.meta[12])/MAX(P(c,FX),P(c,FY));
    int cluster=0,cluster_size=0;
    for(int i=0;i<n;++i) {
        int hits=0;vo_pair_t *p=&c->pairs[i];
        for(int j=0;j<n;++j) {
            vo_pair_t *q=&c->pairs[j];float x=(p->ax-p->bx)-(q->ax-q->bx),y=(p->ay-p->by)-(q->ay-q->by);
            hits+=(x*x+y*y<=coarse*coarse);
        }
        if(hits>cluster_size) {cluster=i;cluster_size=hits;}
    }
    vo_pair_t *p=&c->pairs[cluster];float dx=p->ax-p->bx,dy=p->ay-p->by;
    for(int i=0;i<n;++i) {
        vo_pair_t *q=&c->pairs[i];float x=q->ax-q->bx-dx,y=q->ay-q->by-dy;
        c->inlier[i]=(uint8_t)(x*x+y*y<=coarse*coarse);
    }
    float tx=0,ty=0,yaw=0,cost=0;
    if(least_squares(c,n,&tx,&ty,&yaw)) {
        best_count=classify(c,n,m,tx,ty,yaw,gate,&cost);
        best_cost=cost;best_tx=tx;best_ty=ty;best_yaw=yaw;
    }
    if(best_count<(int)P(c,INLIERS) || (float)best_count/n<P(c,RATIO)) {
        for(int it=0;it<(int)P(c,RANSAC_ITERS);++it) {
            c->random=1664525u*c->random+1013904223u;int i=(int)(c->random%(unsigned)n);
            c->random=1664525u*c->random+1013904223u;int j=(int)(c->random%(unsigned)n);
            if(i==j) j=(j+1)%n;
            memset(c->inlier,0,(size_t)n);c->inlier[i]=c->inlier[j]=1;++iterations;
            if(!least_squares(c,n,&tx,&ty,&yaw)) continue;
            int count=classify(c,n,m,tx,ty,yaw,gate,&cost);
            if(count>best_count || (count==best_count && cost<best_cost)) {
                best_count=count;best_cost=cost;best_tx=tx;best_ty=ty;best_yaw=yaw;
            }
        }
    }
    tx=best_tx;ty=best_ty;yaw=best_yaw;
    for(int pass=0;pass<2;++pass) {
        classify(c,n,m,tx,ty,yaw,gate,&cost);
        if(!least_squares(c,n,&tx,&ty,&yaw)) return 0;
    }
    int count=classify(c,n,m,tx,ty,yaw,gate,&cost),cells[9]={0},occupied=0;
    c->output[O_INLIERS]=(float)count;
    if(count<(int)P(c,INLIERS) || (float)count/n<P(c,RATIO)) return 0;
    float x=0,y=0,xx=0,yy=0,xy=0,vx=0,vy=0,co=cosf(yaw),si=sinf(yaw);
    for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *q=&c->pairs[i];cells[cell(q->bu,q->bv)]=1;
        x+=q->bu;y+=q->bv;xx+=q->bu*q->bu;yy+=q->bv*q->bv;xy+=q->bu*q->bv;
        float ex=q->ax-co*q->bx+si*q->by-tx,ey=q->ay-si*q->bx-co*q->by-ty;
        vx+=ex*ex;vy+=ey*ey;
    }
    for(int i=0;i<9;++i) occupied+=cells[i];
    xx-=x*x/count;yy-=y*y/count;xy-=x*y/count;
    float tr=xx+yy,root=sqrtf(MAX(0,(xx-yy)*(xx-yy)+4*xy*xy));
    float spread=MAX(0,tr-root)/MAX(1e-9f,tr+root);
    c->output[O_CELLS]=(float)occupied;c->output[O_SPREAD]=spread;
    if(occupied<(int)P(c,CELLS) || spread<.01f) return 0;
    c->output[O_DN]=tx;c->output[O_DE]=ty;c->output[O_YAW]=yaw;
    c->output[O_RATIO]=(float)count/n;c->output[O_RESIDUAL]=sqrtf(cost/count);
    c->output[O_VAR_N]=vx/MAX(1,count-1);c->output[O_VAR_E]=vy/MAX(1,count-1);
    c->output[O_ITERATIONS]=(float)iterations;
    return 1;
}
static int track(vo_context_t *c,const vo_frame_t *ref,const uint8_t *rim,
                 const uint8_t *im,size_t stride,const float *m,const float *pred,float radius) {
    int n=0,klt_count=0,sad_count=0;
    for(int i=0;i<ref->count;++i) {
        vo_point_t p=ref->points[i];float uv[2],inverse[4];
        if(!warp(c,ref,m,pred,p.x,p.y,uv,inverse)) continue;
        int kind=track_point(c,rim,im,stride,p.x,p.y,uv,inverse,radius);
        if(!kind) continue;
        int added=add_pair(c,n,ref->meta,m,p.x,p.y,uv[0],uv[1]);
        if(added!=n) {klt_count+=(kind==1);sad_count+=(kind==2);n=added;}
    }
    c->output[O_KLT]=(float)klt_count;c->output[O_SAD]=(float)sad_count;
    return fit(c,n,m);
}
static void save_frame(vo_frame_t *f,uint8_t *dst,const uint8_t *im,size_t stride,
                       const float *m,int64_t stamp,const vo_point_t *points,int count,const float *position) {
    memcpy(f->meta,m,VO_META_FLOATS*sizeof(float));f->stamp=stamp;f->count=count;
    memcpy(f->position,position,2*sizeof(float));memcpy(f->points,points,(size_t)count*sizeof(vo_point_t));
    for(size_t y=0;y<VO_HEIGHT;++y) memcpy(dst+y*VO_WIDTH,im+y*stride,VO_WIDTH);
}
static void commit(vo_context_t *c,const vo_frame_t *ref,int base,const uint8_t *im,
                   size_t stride,const float *m,int64_t stamp) {
    c->output[O_BASE]=(float)base;c->output[O_REF_HEIGHT]=ref->meta[12];
    c->output[O_REF_DT]=(float)((stamp-ref->stamp)*1e-6);
    float position[2]={ref->position[0]+c->output[O_DN],ref->position[1]+c->output[O_DE]};
    c->output[O_N]=position[0];c->output[O_E]=position[1];
    int count=0;
    for(int i=0;i<(int)c->output[O_MATCHES];++i) if(c->inlier[i]) {
        c->current[count++]=(vo_point_t){c->pairs[i].bu,c->pairs[i].bv,1};
    }
    if(count<P(c,FEATURES)*P(c,TOPUP)) count=replenish(c,im,stride,count);
    float dx=position[0]-c->key.position[0],dy=position[1]-c->key.position[1];
    float rotation=0;
    for(int i=0;i<9;++i) rotation+=m[i]*c->key.meta[i];
    rotation=acosf(clampf((rotation-1)*.5f,-1,1));
    int refresh=hypotf(dx,dy)>P(c,KEY_DISTANCE)*MIN(m[12],c->key.meta[12]) ||
        MAX(m[12]/c->key.meta[12],c->key.meta[12]/m[12])>P(c,KEY_HEIGHT) ||
        rotation>P(c,KEY_ROTATION) || c->output[O_RATIO]<P(c,KEY_RATIO);
    if(refresh) {
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,count,position);
        ++c->key_id;c->output[O_KEY_CHANGED]=1;
    }
    save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,count,position);
    c->output[O_FEATURES]=(float)count;c->output[O_KEY_ID]=(float)c->key_id;c->output[O_STATUS]=2;
}
static int valid_input(const uint8_t *image,size_t stride,const float *m) {
    if(!image || !m || stride<VO_WIDTH || m[12]<=.001f) return 0;
    for(int i=0;i<VO_META_FLOATS;++i) if(!isfinite(m[i])) return 0;
    return 1;
}
size_t vo_context_bytes(void) { return sizeof(vo_context_t); }
const float *vo_get_result(const vo_context_t *c) { return c->output; }
int vo_init(vo_context_t *c,const float *cfg) {
    memset(c,0,sizeof(*c));
    for(int i=0;i<VO_CONFIG_FLOATS;++i) if(!isfinite(cfg[i])) return 0;
    memcpy(c->cfg,cfg,VO_CONFIG_FLOATS*sizeof(float));
    if(P(c,FX)<=0 || P(c,FY)<=0 || P(c,FEATURES)<P(c,MATCHES) || P(c,FEATURES)>VO_MAX_FEATURES ||
       P(c,MATCHES)<3 || P(c,INLIERS)<3 || P(c,INLIERS)>P(c,MATCHES) ||
       P(c,RATIO)<=0 || P(c,RATIO)>1 || P(c,CELLS)<1 || P(c,CELLS)>9 ||
       P(c,GATE)<=0 || P(c,YAW)<=0 || P(c,YAW)>.3f || P(c,SEARCH_MIN)<0 ||
       P(c,SEARCH_MAX)<P(c,SEARCH_MIN) || P(c,SEARCH_MAX)>96 ||
       P(c,KLT_ITERS)<1 || P(c,KLT_ITERS)>40 || P(c,KEY_HEIGHT)<=1 ||
       P(c,KLT_ERROR)<=0 || P(c,FB)<=0 || P(c,MIN_EIG)<=0 ||
       P(c,FAST)<1 || P(c,FAST)>255 || P(c,TOPUP)<=0 || P(c,TOPUP)>1 ||
       P(c,KEY_DISTANCE)<=0 || P(c,KEY_ROTATION)<=0 || P(c,NEAR_FLOW)<=0 ||
       P(c,RAY)<=0 || P(c,RAY)>=1 || P(c,RANSAC_ITERS)<1 || P(c,RANSAC_ITERS)>128 ||
       P(c,KEY_RATIO)<=0 || P(c,KEY_RATIO)>1) return 0;
    c->random=1;return 1;
}
void vo_reset(vo_context_t *c) {
    c->ready=0;c->frame_id=0;c->key_id=0;
    memset(c->output,0,sizeof(c->output));
}
void vo_step(vo_context_t *c,const uint8_t *im,size_t stride,const float *m,
             int64_t stamp,const float *pred,float radius) {
    memset(c->output,0,sizeof(c->output));c->output[O_KEY_ID]=(float)c->key_id;
    if(!valid_input(im,stride,m) || !isfinite(pred[0]) || !isfinite(pred[1]) || !isfinite(radius) ||
       (c->ready && stamp<=c->previous.stamp)) {c->output[O_STATUS]=-2;return;}
    if(!c->ready) {
        int count=replenish(c,im,stride,0);float origin[2]={0,0};
        c->output[O_FEATURES]=(float)count;
        if(count<(int)P(c,MATCHES)) {c->output[O_STATUS]=-1;return;}
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,count,origin);
        save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,count,origin);
        c->ready=1;c->output[O_STATUS]=1;c->output[O_KEY_CHANGED]=1;return;
    }
    ++c->frame_id;radius=clampf(radius,P(c,SEARCH_MIN),P(c,SEARCH_MAX));
    float flow=hypotf(pred[0]-c->previous.position[0],pred[1]-c->previous.position[1])*MAX(P(c,FX),P(c,FY))/m[12];
    int near=flow>P(c,NEAR_FLOW) && c->frame_id%3!=0;
    for(int expansion=0;expansion<2;++expansion) {
        if(expansion && radius>=P(c,SEARCH_MAX)) break;
        float search=expansion?P(c,SEARCH_MAX):radius;
        for(int attempt=0;attempt<2;++attempt) {
            if(attempt && c->key.stamp==c->previous.stamp) continue;
            int base=attempt?1-near:near;
            const vo_frame_t *ref=base?&c->previous:&c->key;
            const uint8_t *rim=base?c->previous_image:c->key_image;
            if(track(c,ref,rim,im,stride,m,pred,search)) {commit(c,ref,base,im,stride,m,stamp);return;}
        }
    }
}
void vo_recover(vo_context_t *c,const uint8_t *im,size_t stride,const float *m,
                int64_t stamp,const float *pairs,size_t count) {
    memset(c->output,0,sizeof(c->output));c->output[O_KEY_ID]=(float)c->key_id;
    if(!c->ready || !valid_input(im,stride,m) || stamp<=c->previous.stamp) {c->output[O_STATUS]=-2;return;}
    int n=0;
    for(size_t i=0;i<count && n<VO_MAX_FEATURES;++i) {
        const float *p=pairs+4*i;n=add_pair(c,n,c->key.meta,m,p[0],p[1],p[2],p[3]);
    }
    if(fit(c,n,m)) commit(c,&c->key,0,im,stride,m,stamp);
}

/* RM downfacing translation VO. Detector provenance: SOURCES.md / vo_fast.c. */
#include "vo_core.h"
#include "vo_fast.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>

#define P(c, name) ((c)->cfg[V_##name])
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
static float clampf(float x, float lo, float hi) { return MAX(lo, MIN(hi, x)); }
static uint32_t tick(const vo_context_t *c) { return c->clock_us?c->clock_us():0; }
static void phase(vo_context_t *c,int item,uint32_t start) {
    if(c->clock_us) c->timing[item]+=c->clock_us()-start;
}
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
static int warp(vo_context_t *c, const vo_frame_t *ref, const float *m,
                const float *pred, float u, float v, float *uv, float *inverse,int affine) {
    float a[2],b[2];
    if(!predicted_pixel(c,ref,m,pred,u,v,uv)) return 0;
    if(!affine) {
        inverse[0]=inverse[3]=1;inverse[1]=inverse[2]=0;return 1;
    }
    c->output[O_WARPS]+=1;
    if (
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
static int template_patch(vo_context_t *c,uint8_t *patch, const uint8_t *im, size_t stride,
                          float u, float v, const float *inverse) {
    uint32_t start=tick(c);int ok=1;
    for (int y=0; y<VO_TEMPLATE && ok; ++y) for (int x=0; x<VO_TEMPLATE; ++x) {
        float dx=(float)(x-5),dy=(float)(y-5);
        float a=u+inverse[0]*dx+inverse[1]*dy,b=v+inverse[2]*dx+inverse[3]*dy;
        if (!inside(a,b,0)) {ok=0;break;}
        patch[y*VO_TEMPLATE+x]=(uint8_t)(sample(im,stride,a,b)+.5f);
    }
    phase(c,T_WARP,start);return ok;
}
/* 8x8 integer SAD. M7 DSP variant uses unaligned-safe memcpy word loads. */
static uint32_t sad8(const uint8_t *patch,const uint8_t *im,size_t stride,int u,int v) {
    uint32_t cost=0;
    for (int y=0;y<8;++y) {
        const uint8_t *a=patch+10*(y+1)+1,*b=im+(size_t)(v-4+y)*stride+(size_t)(u-4);
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
/* Inverse-compositional translation KLT: template gradients/Hessian once per
 * call; one current-image sample per pixel/iteration. One shared workspace. */
static int klt_impl(vo_context_t *c,const uint8_t *patch,const uint8_t *im,
                    size_t stride,float *u,float *v,float *error) {
    float sx=0,sy=0,xx=0,xy=0,yy=0;
    for(int y=0;y<8;++y) for(int x=0;x<8;++x) {
        int i=y*8+x,j=(y+1)*10+x+1;
        float gx=.5f*((float)patch[j+1]-patch[j-1]);
        float gy=.5f*((float)patch[j+10]-patch[j-10]);
        c->gradient[0][i]=gx;c->gradient[1][i]=gy;sx+=gx;sy+=gy;
    }
    sx/=64;sy/=64;
    for(int i=0;i<64;++i) {
        float gx=c->gradient[0][i]-sx,gy=c->gradient[1][i]-sy;
        c->gradient[0][i]=gx;c->gradient[1][i]=gy;
        xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;
    }
    float det=xx*yy-xy*xy,tr=xx+yy;
    float eig=.5f*(tr-sqrtf(MAX(0,(xx-yy)*(xx-yy)+4*xy*xy)))/64;
    if(det<1e-6f || eig<P(c,MIN_EIG)) return -1;
    float hxx=yy/det,hxy=-xy/det,hyy=xx/det;
    for(int it=0;it<(int)P(c,KLT_ITERS);++it) {
        if(!inside(*u,*v,6)) return 0;
        float xe=0,ye=0;
        c->output[O_ITERATIONS]+=1;
        for(int y=0;y<8;++y) for(int x=0;x<8;++x) {
            int i=y*8+x;
            float e=sample(im,stride,*u+x-4,*v+y-4)-patch[(y+1)*10+x+1];
            xe+=c->gradient[0][i]*e;ye+=c->gradient[1][i]*e;
        }
        float du=hxx*xe+hxy*ye,dv=hxy*xe+hyy*ye;
        if(!isfinite(du) || !isfinite(dv) || fabsf(du)>4 || fabsf(dv)>4) return 0;
        *u-=du;*v-=dv;
        if(du*du+dv*dv<.0009f && inside(*u,*v,6)) {
            float se=0,ee=0;
            for(int y=0;y<8;++y) for(int x=0;x<8;++x) {
                float e=sample(im,stride,*u+x-4,*v+y-4)-patch[(y+1)*10+x+1];
                se+=e;ee+=e*e;
            }
            *error=sqrtf(MAX(0,ee-se*se/64)/64);
            return *error<=P(c,KLT_ERROR);
        }
    }
    return 0;
}
static int klt(vo_context_t *c,const uint8_t *patch,const uint8_t *im,
               size_t stride,float *u,float *v,float *error) {
    uint32_t start=tick(c);int result=klt_impl(c,patch,im,stride,u,v,error);
    phase(c,T_KLT,start);return result;
}
static int backward(vo_context_t *c,const uint8_t *current,size_t stride,
                    const uint8_t *ref,float u,float v,float ru,float rv,
                    const float *inverse,float *fb) {
    float det=inverse[0]*inverse[3]-inverse[1]*inverse[2];
    if(fabsf(det)<1e-6f) return 0;
    float j[4]={inverse[3]/det,-inverse[1]/det,-inverse[2]/det,inverse[0]/det};
    uint8_t patch[VO_TEMPLATE*VO_TEMPLATE];
    if(!template_patch(c,patch,current,stride,u,v,j)) return 0;
    float bu=ru,bv=rv,error=0;
    if(klt(c,patch,ref,VO_WIDTH,&bu,&bv,&error)!=1) return 0;
    *fb=hypotf(bu-ru,bv-rv);return *fb<=P(c,FB);
}
/* Radius is bounded to eight pixels, hence at most 289 candidates. The
 * second-best basin must be distinct from the +/-2 pixel best basin. */
static int sad_search(vo_context_t *c,const uint8_t *im,size_t stride,
                      float u,float v,float radius,int *bx,int *by,int normalize) {
    uint32_t start=tick(c),best=UINT_MAX,second=UINT_MAX;
    if(!isfinite(u) || !isfinite(v) || u < -8 || v < -8 ||
       u > VO_WIDTH+8 || v > VO_HEIGHT+8) {
        phase(c,T_SAD,start);return 0;
    }
    uint16_t costs[17*17];
    int cx=(int)lroundf(u),cy=(int)lroundf(v),r=(int)radius;
    int lx=MAX(6,cx-r),hx=MIN(VO_WIDTH-8,cx+r);
    int ly=MAX(6,cy-r),hy=MIN(VO_HEIGHT-8,cy+r),n=0;
    *bx=*by=0;
    int template_sum=0;
    if(normalize) for(int y=0;y<8;++y) for(int x=0;x<8;++x) template_sum+=c->patch[(y+1)*10+x+1];
    for(int y=ly;y<=hy;++y) for(int x=lx;x<=hx;++x) {
        uint32_t cost=0;
        if(normalize) {
            int sum=0;
            for(int py=0;py<8;++py) for(int px=0;px<8;++px)
                sum+=im[(size_t)(y-4+py)*stride+(size_t)(x-4+px)];
            int offset=(sum-template_sum)/64;
            for(int py=0;py<8;++py) for(int px=0;px<8;++px)
                cost+=(uint32_t)abs((int)im[(size_t)(y-4+py)*stride+(size_t)(x-4+px)]-
                                   c->patch[(py+1)*10+px+1]-offset);
        } else cost=sad8(c->patch,im,stride,x,y);
        costs[n++]=(uint16_t)cost;
        if(cost<best) {best=cost;*bx=x;*by=y;}
    }
    c->output[O_SAD_TESTS]+=(float)n;n=0;
    for(int y=ly;y<=hy;++y) for(int x=lx;x<=hx;++x) {
        uint32_t cost=costs[n++];
        if((abs(x-*bx)>2 || abs(y-*by)>2) && cost<second) second=cost;
    }
    phase(c,T_SAD,start);
    return best!=UINT_MAX && second!=UINT_MAX &&
        best<=64*P(c,KLT_ERROR)*1.5f && (float)best<P(c,UNIQUENESS)*(float)second;
}
static int track_point(vo_context_t *c,const uint8_t *ref,const uint8_t *im,size_t stride,
                       float ru,float rv,float *uv,const float *inverse,float radius,float *weight) {
    if(!template_patch(c,c->patch,ref,VO_WIDTH,ru,rv,inverse)) return 0;
    float u=uv[0],v=uv[1],error=0,fb=0;
    int success=klt(c,c->patch,im,stride,&u,&v,&error),kind=1;
    if(success<0) return 0; /* aperture/flat template: searching cannot fix it */
    if(success!=1 || !backward(c,im,stride,ref,u,v,ru,rv,inverse,&fb)) {
        int bx,by;kind=2;
        if(!sad_search(c,im,stride,uv[0],uv[1],radius,&bx,&by,0)) return 0;
        u=(float)bx;v=(float)by;
        if(klt(c,c->patch,im,stride,&u,&v,&error)!=1 ||
           !backward(c,im,stride,ref,u,v,ru,rv,inverse,&fb)) return 0;
    }
    *weight=1.0f/(1.0f+(error/P(c,KLT_ERROR))*(error/P(c,KLT_ERROR))+
                  (fb/P(c,FB))*(fb/P(c,FB)));
    uv[0]=u;uv[1]=v;return kind;
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
/* Bounded 6x4 grid refill. Full scan only when no reference exists; low
 * texture never causes a repeated unbounded scan during measured tracking. */
static int replenish(vo_context_t *c,const uint8_t *im,size_t stride,int count,int full) {
    uint32_t start=tick(c);
    int limit=(int)P(c,FEATURES),quota=(limit+23)/24,initial=count;
    int cells=full?24:(int)P(c,TOPUP_CELLS);
    int additions=full?limit:(int)P(c,TOPUP_POINTS),counts[24]={0};
    int offset[16];
    static const int8_t dx[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
    static const int8_t dy[16]={3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3};
    for(int i=0;i<16;++i) offset[i]=dy[i]*(int)stride+dx[i];
    for(int i=0;i<count;++i) {
        int x=MIN(5,(int)(c->current[i].x*6/VO_WIDTH)),y=MIN(3,(int)(c->current[i].y*4/VO_HEIGHT));
        counts[6*y+x]++;
    }
    int scanned=0,begin=c->grid_cursor;
    for(int visited=0;visited<24 && scanned<cells && count<limit && count-initial<additions;++visited) {
        int grid=(begin+visited)%24;c->grid_cursor=(grid+1)%24;
        if(counts[grid]>=quota) continue;
        ++scanned;
        vo_point_t candidates[3]={{0}};
        int x0=MAX(12,(grid%6)*VO_WIDTH/6),x1=MIN(VO_WIDTH-12,(grid%6+1)*VO_WIDTH/6);
        int y0=MAX(12,(grid/6)*VO_HEIGHT/4),y1=MIN(VO_HEIGHT-12,(grid/6+1)*VO_HEIGHT/4);
        for(int y=y0;y<y1;++y) for(int x=x0;x<x1;++x) {
            if(!vo_fast_test(im+(size_t)y*stride+(size_t)x,offset,(int)P(c,FAST))) continue;
            int score=fast_score(im,stride,x,y,(int)P(c,FAST));
            if(score<=0) continue;
            int close=0;
            for(int i=0;i<count;++i) if(fabsf(c->current[i].x-x)<10 && fabsf(c->current[i].y-y)<10) {close=1;break;}
            if(close) continue;
            for(int k=0;k<3;++k) {
                if(candidates[k].score && fabsf(candidates[k].x-x)<10 && fabsf(candidates[k].y-y)<10) {
                    if(score>candidates[k].score) candidates[k]=(vo_point_t){(float)x,(float)y,(float)score};
                    close=1;break;
                }
            }
            if(close) continue;
            int weakest=0;
            for(int k=1;k<3;++k) if(candidates[k].score<candidates[weakest].score) weakest=k;
            if(score>candidates[weakest].score) candidates[weakest]=(vo_point_t){(float)x,(float)y,(float)score};
        }
        while(counts[grid]<quota && count<limit && count-initial<additions) {
            int best=0;
            for(int k=1;k<3;++k) if(candidates[k].score>candidates[best].score) best=k;
            if(!candidates[best].score) break;
            c->current[count++]=candidates[best];candidates[best].score=0;counts[grid]++;
        }
    }
    c->output[O_DETECT_CELLS]+=(float)scanned;c->output[O_ADDED]+=(float)(count-initial);
    phase(c,T_DETECT,start);return count;
}
static int add_pair(vo_context_t *c,int n,const float *rm,const float *m,
                    float au,float av,float bu,float bv,float weight) {
    float a[2],b[2];uint32_t start=tick(c);
    int valid=inside(au,av,0) && inside(bu,bv,0) &&
        ground(c,rm,au,av,a) && ground(c,m,bu,bv,b);
    phase(c,T_WARP,start);
    if(!valid) return n;
    for(int i=0;i<n;++i) {
        vo_pair_t *p=&c->pairs[i];
        if(hypotf(p->bu-bu,p->bv-bv)<3 || hypotf(p->au-au,p->av-av)<3) return n;
    }
    c->pairs[n]=(vo_pair_t){au,av,bu,bv,a[0],a[1],b[0],b[1],weight};return n+1;
}
static float residual(const vo_context_t *c,const float *m,const vo_pair_t *p,float tx,float ty) {
    float uv[2];
    if(!pixel(c,m,p->ax-tx,p->ay-ty,uv)) return 1e20f;
    return (uv[0]-p->bu)*(uv[0]-p->bu)+(uv[1]-p->bv)*(uv[1]-p->bv);
}
static int classify(vo_context_t *c,int n,const float *m,float tx,float ty,float *cost) {
    int count=0;*cost=0;
    for(int i=0;i<n;++i) {
        float e=residual(c,m,&c->pairs[i],tx,ty);
        c->inlier[i]=(uint8_t)(e<=P(c,GATE)*P(c,GATE));
        if(c->inlier[i]) {++count;*cost+=e;}
    }
    return count;
}
/* Attitude/height fix rotation and scale: only two translation unknowns.
 * Deterministic displacement consensus, a strict majority, and bounded refits.
 * No random hypotheses and no rank-two feature-distribution requirement. */
static int fit_impl(vo_context_t *c,int n,const float *m) {
    c->output[O_MATCHES]=(float)n;c->output[O_REASON]=VO_REASON_COUNT;
    if(n<(int)P(c,MATCHES)) return 0;
    float best_cost=1e30f,best_tx=0,best_ty=0;int best_count=0;
    for(int i=0;i<n;++i) {
        vo_pair_t *p=&c->pairs[i];float cost,tx=p->ax-p->bx,ty=p->ay-p->by;
        int count=classify(c,n,m,tx,ty,&cost);
        if(count>best_count || (count==best_count && cost<best_cost)) {
            best_count=count;best_cost=cost;best_tx=tx;best_ty=ty;
        }
    }
    c->output[O_INLIERS]=(float)best_count;c->output[O_REASON]=VO_REASON_RATIO;
    if(best_count<(int)P(c,INLIERS) || best_count<=n/2 || (float)best_count/n<P(c,RATIO)) return 0;
    float tx=best_tx,ty=best_ty,cost=0;
    for(int pass=0;pass<2;++pass) {
        classify(c,n,m,tx,ty,&cost);
        float x=0,y=0,w=0;
        for(int i=0;i<n;++i) if(c->inlier[i]) {
            vo_pair_t *p=&c->pairs[i];x+=p->weight*(p->ax-p->bx);
            y+=p->weight*(p->ay-p->by);w+=p->weight;
        }
        if(w<=0) return 0;
        tx=x/w;ty=y/w;
    }
    int count=classify(c,n,m,tx,ty,&cost),cells[9]={0},occupied=0;
    c->output[O_INLIERS]=(float)count;
    if(count<(int)P(c,INLIERS) || count<=n/2 || (float)count/n<P(c,RATIO)) return 0;
    float x=0,y=0,xx=0,yy=0,vx=0,vy=0,w=0;
    for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *q=&c->pairs[i];cells[cell(q->bu,q->bv)]=1;
        x+=q->bu;y+=q->bv;xx+=q->bu*q->bu;yy+=q->bv*q->bv;
        float ex=q->ax-q->bx-tx,ey=q->ay-q->by-ty;
        vx+=q->weight*ex*ex;vy+=q->weight*ey*ey;w+=q->weight;
    }
    for(int i=0;i<9;++i) occupied+=cells[i];
    float spread=sqrtf(MAX(0,xx+yy-(x*x+y*y)/count)/(count*(float)(VO_WIDTH*VO_WIDTH+VO_HEIGHT*VO_HEIGHT)));
    c->output[O_DN]=tx;c->output[O_DE]=ty;c->output[O_YAW]=0;
    c->output[O_CELLS]=(float)occupied;c->output[O_SPREAD]=spread;
    c->output[O_RATIO]=(float)count/n;c->output[O_RESIDUAL]=sqrtf(cost/count);
    c->output[O_VAR_N]=vx/w;c->output[O_VAR_E]=vy/w;c->output[O_EFFECTIVE]=w;
    c->output[O_SPARSE]=(float)(count<=3);c->output[O_REASON]=VO_REASON_OK;
    return isfinite(tx) && isfinite(ty);
}
static int fit(vo_context_t *c,int n,const float *m) {
    uint32_t start=tick(c);int result=fit_impl(c,n,m);phase(c,T_FIT,start);return result;
}
static float frame_angle(const float *a,const float *b) {
    float tr=0;for(int i=0;i<9;++i) tr+=a[i]*b[i];
    return acosf(clampf((tr-1)*.5f,-1,1));
}
static int needs_affine(const vo_context_t *c,const vo_frame_t *ref,const float *m) {
    return frame_angle(m,ref->meta)>P(c,WARP_ANGLE) ||
        fabsf(m[12]/ref->meta[12]-1)>P(c,WARP_HEIGHT);
}
static int track(vo_context_t *c,const vo_frame_t *ref,const uint8_t *rim,
                 const uint8_t *im,size_t stride,const float *m,const float *pred,float radius) {
    int n=0,klt_count=0,sad_count=0,affine=needs_affine(c,ref,m);
    c->output[O_ATTEMPTS]+=1;
    for(int i=0;i<ref->count;++i) {
        vo_point_t p=ref->points[i];float uv[2],inverse[4],weight=0;
        uint32_t start=tick(c);
        int valid=warp(c,ref,m,pred,p.x,p.y,uv,inverse,affine);
        phase(c,T_WARP,start);
        if(!valid) continue;
        int kind=track_point(c,rim,im,stride,p.x,p.y,uv,inverse,radius,&weight);
        if(!kind) continue;
        int added=add_pair(c,n,ref->meta,m,p.x,p.y,uv[0],uv[1],weight);
        if(added!=n) {klt_count+=(kind==1);sad_count+=(kind==2);n=added;}
    }
    c->output[O_KLT]=(float)klt_count;c->output[O_SAD]=(float)sad_count;
    if(!fit(c,n,m)) return 0;
    /* Sparse tracking needs a uniqueness check even when KLT converges. */
    if(c->output[O_SPARSE]) for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *p=&c->pairs[i];float uv[2],inverse[4];int bx,by;
        uint32_t start=tick(c);
        int valid=warp(c,ref,m,pred,p->au,p->av,uv,inverse,affine);
        phase(c,T_WARP,start);
        if(!valid || !template_patch(c,c->patch,rim,VO_WIDTH,p->au,p->av,inverse) ||
           !sad_search(c,im,stride,p->bu,p->bv,P(c,SEARCH_MIN),&bx,&by,1) ||
           hypotf((float)bx-p->bu,(float)by-p->bv)>P(c,GATE)) {
            c->output[O_REASON]=VO_REASON_AMBIGUOUS;return 0;
        }
    }
    return 1;
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
    int measured_count=count;
    if(count<P(c,FEATURES)*P(c,TOPUP)) count=replenish(c,im,stride,count,0);
    float dx=position[0]-c->key.position[0],dy=position[1]-c->key.position[1];
    float rotation=0;
    for(int i=0;i<9;++i) rotation+=m[i]*c->key.meta[i];
    rotation=acosf(clampf((rotation-1)*.5f,-1,1));
    int refresh=c->refresh_key || measured_count<ref->count*P(c,KEY_RATIO) ||
        hypotf(dx,dy)>P(c,KEY_DISTANCE)*MIN(m[12],c->key.meta[12]) ||
        MAX(m[12]/c->key.meta[12],c->key.meta[12]/m[12])>P(c,KEY_HEIGHT) ||
        rotation>P(c,KEY_ROTATION) || c->output[O_RATIO]<P(c,KEY_RATIO);
    uint32_t copy_start=tick(c);
    if(refresh) {
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,count,position);
        c->refresh_key=0;
        ++c->key_id;c->output[O_KEY_CHANGED]=1;
    }
    save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,count,position);
    phase(c,T_COPY,copy_start);
    c->output[O_FEATURES]=(float)count;c->output[O_KEY_ID]=(float)c->key_id;c->output[O_STATUS]=2;
}
static int valid_input(const uint8_t *image,size_t stride,const float *m) {
    if(!image || !m || stride<VO_WIDTH || m[12]<=.001f) return 0;
    for(int i=0;i<VO_META_FLOATS;++i) if(!isfinite(m[i])) return 0;
    return 1;
}
size_t vo_context_bytes(void) { return sizeof(vo_context_t); }
const float *vo_get_result(const vo_context_t *c) { return c->output; }
const uint32_t *vo_get_timings(const vo_context_t *c) { return c->timing; }
int vo_init(vo_context_t *c,const float *cfg) {
    memset(c,0,sizeof(*c));
    for(int i=0;i<VO_CONFIG_FLOATS;++i) if(!isfinite(cfg[i])) return 0;
    memcpy(c->cfg,cfg,VO_CONFIG_FLOATS*sizeof(float));
    if(P(c,FX)<=0 || P(c,FY)<=0 || P(c,FEATURES)<P(c,MATCHES) || P(c,FEATURES)>VO_MAX_FEATURES ||
       P(c,MATCHES)<2 || P(c,INLIERS)<2 || P(c,INLIERS)>P(c,MATCHES) ||
       P(c,RATIO)<=.5f || P(c,RATIO)>1 || P(c,GATE)<=0 || P(c,SEARCH_MIN)<1 ||
       P(c,SEARCH_MAX)<P(c,SEARCH_MIN) || P(c,SEARCH_MAX)>8 ||
       P(c,KLT_ITERS)<1 || P(c,KLT_ITERS)>40 || P(c,KEY_HEIGHT)<=1 ||
       P(c,KLT_ERROR)<=0 || P(c,FB)<=0 || P(c,MIN_EIG)<=0 ||
       P(c,FAST)<1 || P(c,FAST)>255 || P(c,TOPUP)<=0 || P(c,TOPUP)>1 ||
       P(c,KEY_DISTANCE)<=0 || P(c,KEY_ROTATION)<=0 ||
       P(c,RAY)<=0 || P(c,RAY)>=1 || P(c,KEY_RATIO)<=0 || P(c,KEY_RATIO)>1 ||
       P(c,TOPUP_CELLS)<1 || P(c,TOPUP_CELLS)>24 || P(c,TOPUP_POINTS)<0 ||
       P(c,TOPUP_POINTS)>VO_MAX_FEATURES || P(c,KEY_INTERVAL)<1 || P(c,KEY_INTERVAL)>1000 ||
       P(c,WARP_ANGLE)<=0 || P(c,WARP_HEIGHT)<=0 ||
       P(c,UNIQUENESS)<=0 || P(c,UNIQUENESS)>=1) return 0;
    return 1;
}
void vo_reset(vo_context_t *c) {
    c->ready=0;c->frame_id=0;c->key_id=0;c->grid_cursor=0;c->initializing=0;c->refresh_key=0;
    memset(c->output,0,sizeof(c->output));memset(c->timing,0,sizeof(c->timing));
}
void vo_profile(vo_context_t *c,uint32_t (*clock_us)(void)) { c->clock_us=clock_us; }
void vo_step(vo_context_t *c,const uint8_t *im,size_t stride,const float *m,
             int64_t stamp,const float *pred,float radius) {
    memset(c->output,0,sizeof(c->output));memset(c->timing,0,sizeof(c->timing));
    c->output[O_KEY_ID]=(float)c->key_id;
    if(!valid_input(im,stride,m) || !isfinite(pred[0]) || !isfinite(pred[1]) || !isfinite(radius) ||
       (c->ready && stamp<=c->previous.stamp)) {
        c->output[O_STATUS]=-2;c->output[O_REASON]=VO_REASON_INPUT;return;
    }
    if(!c->ready) {
        int count=replenish(c,im,stride,0,!c->initializing);float origin[2]={0,0};
        c->initializing=1;c->output[O_FEATURES]=(float)count;
        if(count<(int)P(c,MATCHES)) {c->output[O_STATUS]=-1;c->output[O_REASON]=VO_REASON_COUNT;return;}
        uint32_t start=tick(c);
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,count,origin);
        save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,count,origin);
        phase(c,T_COPY,start);
        c->ready=1;c->output[O_STATUS]=1;c->output[O_KEY_CHANGED]=1;c->output[O_REASON]=VO_REASON_INIT;return;
    }
    ++c->frame_id;radius=clampf(radius,P(c,SEARCH_MIN),P(c,SEARCH_MAX));
    /* One complete pass. Anchor frames are scheduled, never a second full
     * retry in the same image. Motion prediction remains search-only. */
    int base=c->frame_id%(uint32_t)P(c,KEY_INTERVAL)!=0;
    if(c->refresh_key) base=1;
    if(c->key.stamp==c->previous.stamp) base=1;
    const vo_frame_t *ref=base?&c->previous:&c->key;
    const uint8_t *rim=base?c->previous_image:c->key_image;
    if(track(c,ref,rim,im,stride,m,pred,radius)) commit(c,ref,base,im,stride,m,stamp);
    else if(!base) c->refresh_key=1; /* retire an unusable anchor after next real fix */
}
#ifdef VO_TESTING
/* Host-only geometry seam: accepts observations, never fabricates images or
 * alters production reference/position state. Not linked into firmware. */
void vo_test_pairs(vo_context_t *c,const float *rm,const float *m,const float *pairs,int count) {
    memset(c->output,0,sizeof(c->output));int n=0;
    for(int i=0;i<count && n<VO_MAX_FEATURES;++i) {
        const float *p=pairs+4*i;n=add_pair(c,n,rm,m,p[0],p[1],p[2],p[3],1);
    }
    c->output[O_STATUS]=fit(c,n,m)?2:0;
}
void vo_test_limit(vo_context_t *c,int count,int topup) {
    c->key.count=MIN(c->key.count,count);c->previous.count=MIN(c->previous.count,count);
    P(c,TOPUP_POINTS)=(float)topup;
}
void vo_test_spoil_key(vo_context_t *c) { memset(c->key_image,0,VO_PIXELS); }
#endif

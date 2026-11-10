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
static int camera_pixel(const vo_context_t *c,float x,float y,float z,float *uv) {
    if (!isfinite(z) || z<1e-5f) return 0;
    x/=z;y/=z;
    float r=x*x+y*y, k=1+P(c,K1)*r+P(c,K2)*r*r+P(c,K3)*r*r*r;
    uv[0]=P(c,FX)*(x*k+2*P(c,P1)*x*y+P(c,P2)*(r+2*x*x))+P(c,CX);
    uv[1]=P(c,FY)*(y*k+P(c,P1)*(r+2*y*y)+2*P(c,P2)*x*y)+P(c,CY);
    return isfinite(uv[0]) && isfinite(uv[1]);
}
static int pixel(const vo_context_t *c, const float *m, float gx, float gy, float *uv) {
    gx-=m[9];gy-=m[10];
    return camera_pixel(c,m[0]*gx+m[3]*gy+m[6]*m[12],
        m[1]*gx+m[4]*gy+m[7]*m[12],m[2]*gx+m[5]*gy+m[8]*m[12],uv);
}
static int predicted_pixel(const vo_context_t *c, const vo_frame_t *ref,
                           const float *m, const float *pred, float u, float v, float *uv) {
    float g[2];
    return ground(c,ref->meta,u,v,g) && pixel(c,m,
        g[0]+ref->position[0]-pred[0],g[1]+ref->position[1]-pred[1],uv);
}
static int warp(vo_context_t *c, const vo_frame_t *ref, const float *m,
                 const float *pred, float u, float v, float *uv, float *inverse,int affine,float *ground_ref) {
    float a[2],b[2];
    if(!ground(c,ref->meta,u,v,ground_ref) || !pixel(c,m,
        ground_ref[0]+ref->position[0]-pred[0],ground_ref[1]+ref->position[1]-pred[1],uv)) return 0;
    if(!affine) {
        inverse[0]=inverse[3]=1;inverse[1]=inverse[2]=0;return 1;
    }
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
                      float u,float v,float radius,int *bx,int *by) {
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
    if(lx>hx || ly>hy) {phase(c,T_SAD,start);return 0;}
    int template_sum=0,columns[24]={0},width=hx-lx+8;
    for(int y=0;y<8;++y) for(int x=0;x<8;++x) template_sum+=c->patch[(y+1)*10+x+1];
    for(int x=0;x<width;++x) for(int y=0;y<8;++y)
        columns[x]+=im[(size_t)(ly-4+y)*stride+(size_t)(lx-4+x)];
    for(int y=ly;y<=hy;++y) {
        int sum=0;for(int x=0;x<8;++x) sum+=columns[x];
        for(int x=lx;x<=hx;++x) {
            if(x>lx) sum+=columns[x-lx+7]-columns[x-lx-1];
            int offset_sum=sum-template_sum;
            uint32_t scaled=0;
            for(int py=0;py<8;++py) for(int px=0;px<8;++px) {
                int difference=(int)im[(size_t)(y-4+py)*stride+(size_t)(x-4+px)]-
                               c->patch[(py+1)*10+px+1];
                scaled+=(uint32_t)abs(64*difference-offset_sum);
            }
            /* Exact fractional mean, one rounding after the sum; no signed
             * residual is clipped into a uint8_t DSP SAD lane. */
            uint32_t cost=(scaled+32)>>6;
            costs[n++]=(uint16_t)cost;
            if(cost<best) {best=cost;*bx=x;*by=y;}
        }
        if(y<hy) for(int x=0;x<width;++x)
            columns[x]+=(int)im[(size_t)(y+4)*stride+(size_t)(lx-4+x)]-
                            im[(size_t)(y-4)*stride+(size_t)(lx-4+x)];
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
    if(!template_patch(c,c->patch,ref,VO_WIDTH,ru,rv,inverse)) {
        c->output[O_KLT_FAIL]+=1;return 0;
    }
    float u=uv[0],v=uv[1],error=0,fb=0;
    int success=klt(c,c->patch,im,stride,&u,&v,&error);
    if(success<0) {
        c->output[O_KLT_EIG_REJECT]+=1;return 0;
    } /* aperture/flat template: searching cannot fix it */
    if(success!=1 || !backward(c,im,stride,ref,u,v,ru,rv,inverse,&fb)) {
        int bx,by;
        if(!sad_search(c,im,stride,uv[0],uv[1],radius,&bx,&by)) {
            c->output[O_SAD_FAIL]+=1;return 0;
        }
        u=(float)bx;v=(float)by;
        if(klt(c,c->patch,im,stride,&u,&v,&error)!=1) {
            c->output[O_KLT_FAIL]+=1;return 0;
        }
        if(!backward(c,im,stride,ref,u,v,ru,rv,inverse,&fb)) {
            c->output[O_FB_REJECT]+=1;return 0;
        }
    }
    *weight=1.0f/(1.0f+(error/P(c,KLT_ERROR))*(error/P(c,KLT_ERROR))+
                  (fb/P(c,FB))*(fb/P(c,FB)));
    uv[0]=u;uv[1]=v;return 1;
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
/* The same demeaned 8x8 structure tensor that the KLT uses. Testing the
 * positive-semidefinite threshold first avoids a square root at most grid
 * samples on a featureless floor. */
static float corner_eigen(const uint8_t *im,size_t stride,int x,int y,float floor_value) {
    float sx=0,sy=0,xx=0,xy=0,yy=0;
    for(int py=-4;py<4;++py) for(int px=-4;px<4;++px) {
        const uint8_t *p=im+(size_t)(y+py)*stride+(size_t)(x+px);
        float gx=.5f*((float)p[1]-(float)p[-1]);
        float gy=.5f*((float)p[stride]-(float)p[-(ptrdiff_t)stride]);
        sx+=gx;sy+=gy;xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;
    }
    xx-=sx*sx/64;xy-=sx*sy/64;yy-=sy*sy/64;
    float limit=64*floor_value;
    if(xx<limit || yy<limit || (xx-limit)*(yy-limit)<xy*xy) return 0;
    return .5f*(xx+yy-sqrtf(MAX(0,(xx-yy)*(xx-yy)+4*xy*xy)))/64;
}
/* MODE2-only sampled diagnostics. Each 6x4 cell is sampled on a four-pixel
 * lattice; the camera frame and the normal VO step remain untouched. */
void vo_image_metrics(const uint8_t *im,size_t stride,float min_eigenvalue,
                      float out[VO_IMAGE_METRICS]) {
    for(int i=0;i<VO_IMAGE_METRICS;++i) out[i]=0;
    if(!im || stride<VO_WIDTH || !isfinite(min_eigenvalue) || min_eigenvalue<=0) return;
    float sx[24]={0},sy[24]={0},xx[24]={0},xy[24]={0},yy[24]={0};
    int samples[24]={0},saturated[24]={0},total=0,sat_total=0;
    float gsx=0,gsy=0,gxx=0,gxy=0,gyy=0;
    for(int y=4;y<VO_HEIGHT-4;y+=4) for(int x=4;x<VO_WIDTH-4;x+=4) {
        int cell=(y*4/VO_HEIGHT)*6+x*6/VO_WIDTH;
        const uint8_t *p=im+(size_t)y*stride+(size_t)x;
        int sat=*p>=250;float gx=.5f*((float)p[1]-(float)p[-1]);
        float gy=.5f*((float)p[stride]-(float)p[-(ptrdiff_t)stride]);
        ++samples[cell];saturated[cell]+=sat;++total;sat_total+=sat;
        sx[cell]+=gx;sy[cell]+=gy;xx[cell]+=gx*gx;
        xy[cell]+=gx*gy;yy[cell]+=gy*gy;
        gsx+=gx;gsy+=gy;gxx+=gx*gx;gxy+=gx*gy;gyy+=gy*gy;
    }
    uint32_t sat_mask=0,good_mask=0;int good=0;
    for(int i=0;i<24;++i) {
        if(!samples[i]) continue;
        if(saturated[i]*20>=samples[i]) sat_mask|=(uint32_t)1<<i;
        float inv=1.0f/samples[i];
        float a=xx[i]-sx[i]*sx[i]*inv,b=xy[i]-sx[i]*sy[i]*inv;
        float d=yy[i]-sy[i]*sy[i]*inv;
        float eigen=.5f*(a+d-sqrtf(MAX(0,(a-d)*(a-d)+4*b*b)))*inv;
        if(eigen>=min_eigenvalue) {good_mask|=(uint32_t)1<<i;++good;}
    }
    float inv=1.0f/MAX(total,1);
    float a=gxx-gsx*gsx*inv,b=gxy-gsx*gsy*inv,d=gyy-gsy*gsy*inv;
    out[0]=(float)sat_total*inv;out[1]=(float)sat_mask;
    out[2]=.5f*(a+d-sqrtf(MAX(0,(a-d)*(a-d)+4*b*b)))*inv;
    out[3]=(float)good;out[4]=(float)good_mask;
}
static float distance2(float ax,float ay,float bx,float by) {
    float dx=ax-bx,dy=ay-by;return dx*dx+dy*dy;
}
static int near_current(const vo_context_t *c,int count,float x,float y,float radius2) {
    for(int i=0;i<count;++i)
        if(distance2(c->current[i].x,c->current[i].y,x,y)<radius2) return 1;
    return 0;
}
static void offer_point(vo_point_t *pool,int *used,int capacity,vo_point_t point,float radius2) {
    int close=-1;
    for(int i=0;i<*used;++i) if(distance2(pool[i].x,pool[i].y,point.x,point.y)<radius2) {
        close=i;break;
    }
    if(close>=0) {
        if(point.score<=pool[close].score) return;
        for(int i=0;i<*used;++i) if(i!=close &&
                distance2(pool[i].x,pool[i].y,point.x,point.y)<radius2) return;
        pool[close]=point;return;
    }
    if(*used<capacity) {pool[(*used)++]=point;return;}
    int weakest=0;
    for(int i=1;i<*used;++i) if(pool[i].score<pool[weakest].score) weakest=i;
    if(point.score>pool[weakest].score) pool[weakest]=point;
}
static int take_best(vo_context_t *c,vo_point_t *pool,int *used,int count,float radius2) {
    while(*used) {
        int best=0;
        for(int i=1;i<*used;++i) if(pool[i].score>pool[best].score) best=i;
        vo_point_t p=pool[best];pool[best]=pool[--*used];
        if(near_current(c,count,p.x,p.y,radius2)) continue;
        c->current[count]=p;c->current_fresh[count]=1;
        return count+1;
    }
    return count;
}
/* A bounded 6x4 scan, with coverage first and at most two extra points per
 * textured cell. FAST shortlists are checked against the KLT tensor; weak
 * two-dimensional texture is sampled only when the shortlist is insufficient. */
static int replenish(vo_context_t *c,const uint8_t *im,size_t stride,int count,int full) {
    uint32_t start=tick(c);
    int limit=(int)P(c,FEATURES),initial=count,cap=(int)P(c,CELL_MAX_POINTS);
    int cells=full?24:(int)P(c,TOPUP_CELLS);
    int additions=full?limit:(int)P(c,TOPUP_POINTS),counts[24]={0};
    int offset[16],selected[24],selected_count=0;
    vo_point_t candidates[24][3];int available[24]={0};
    float spacing2=P(c,MIN_DISTANCE)*P(c,MIN_DISTANCE);
    static const int8_t dx[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
    static const int8_t dy[16]={3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3};
    for(int i=0;i<16;++i) offset[i]=dy[i]*(int)stride+dx[i];
    for(int i=0;i<count;++i) {
        int x=MIN(5,(int)(c->current[i].x*6/VO_WIDTH));
        int y=MIN(3,(int)(c->current[i].y*4/VO_HEIGHT));
        counts[6*y+x]++;
    }
    int begin=c->grid_cursor;
    for(int visited=0;visited<24 && selected_count<cells && count<limit;++visited) {
        int grid=(begin+visited)%24;c->grid_cursor=(grid+1)%24;
        if(counts[grid]>=cap) continue;
        selected[selected_count++]=grid;
        vo_point_t fast[8];int fast_count=0;
        int x0=MAX(12,(grid%6)*VO_WIDTH/6),x1=MIN(VO_WIDTH-12,(grid%6+1)*VO_WIDTH/6);
        int y0=MAX(12,(grid/6)*VO_HEIGHT/4),y1=MIN(VO_HEIGHT-12,(grid/6+1)*VO_HEIGHT/4);
        for(int y=y0;y<y1;++y) for(int x=x0;x<x1;++x) {
            if(!vo_fast_test(im+(size_t)y*stride+(size_t)x,offset,(int)P(c,FAST))) continue;
            int score=fast_score(im,stride,x,y,(int)P(c,FAST));
            if(score<=0) continue;
            c->output[O_FAST_CANDIDATES]+=1;
            if(near_current(c,count,(float)x,(float)y,spacing2)) continue;
            offer_point(fast,&fast_count,8,(vo_point_t){(float)x,(float)y,(float)score},spacing2);
        }
        for(int i=0;i<fast_count;++i) {
            int x=(int)fast[i].x,y=(int)fast[i].y;
            float eigen=corner_eigen(im,stride,x,y,P(c,MIN_EIG));
            if(eigen>=P(c,MIN_EIG))
                offer_point(candidates[grid],&available[grid],3,
                    (vo_point_t){(float)x,(float)y,eigen},spacing2);
        }
        if(available[grid]<MIN(3,cap-counts[grid])) {
            int step=(int)P(c,WEAK_STRIDE);
            for(int y=y0;y<y1;y+=step) for(int x=x0;x<x1;x+=step) {
                if(near_current(c,count,(float)x,(float)y,spacing2)) continue;
                float eigen=corner_eigen(im,stride,x,y,P(c,MIN_EIG));
                if(eigen<P(c,MIN_EIG)) continue;
                c->output[O_WEAK_CANDIDATES]+=1;
                offer_point(candidates[grid],&available[grid],3,
                    (vo_point_t){(float)x,(float)y,eigen},spacing2);
            }
        }
    }
    /* The first pass cannot spend the four-point top-up budget in one cell. */
    for(int pass=0;pass<2 && count<limit && count-initial<additions;++pass)
        for(int i=0;i<selected_count && count<limit && count-initial<additions;++i) {
            int grid=selected[i];
            if(pass==0) {
                if(counts[grid]) continue;
                int next=take_best(c,candidates[grid],&available[grid],count,spacing2);
                if(next>count) {count=next;counts[grid]++;}
            } else {
                while(counts[grid]<cap && count<limit && count-initial<additions) {
                    int next=take_best(c,candidates[grid],&available[grid],count,spacing2);
                    if(next==count) break;
                    count=next;counts[grid]++;
                }
            }
        }
    c->output[O_DETECT_CELLS]+=(float)selected_count;
    c->output[O_ADDED]+=(float)(count-initial);
    phase(c,T_DETECT,start);return count;
}
static int add_pair(vo_context_t *c,int n,const float *rm,const float *m,
                    float au,float av,float bu,float bv,float weight,
                    const float *ground_ref,const float *inverse) {
    float a[2],b[2];uint32_t start=tick(c);
    int valid=inside(au,av,0) && inside(bu,bv,0) &&
        (ground_ref || ground(c,rm,au,av,a)) && ground(c,m,bu,bv,b);
    if(ground_ref) {a[0]=ground_ref[0];a[1]=ground_ref[1];}
    phase(c,T_WARP,start);
    if(!valid) return n;
    for(int i=0;i<n;++i) {
        vo_pair_t *p=&c->pairs[i];
        float dx=p->bu-bu,dy=p->bv-bv,rx=p->au-au,ry=p->av-av;
        if(dx*dx+dy*dy<9 || rx*rx+ry*ry<9) return n;
    }
    vo_pair_t *p=&c->pairs[n];
    *p=(vo_pair_t){au,av,bu,bv,a[0],a[1],b[0],b[1],weight,{1,0,0,1}};
    if(inverse) memcpy(p->inverse,inverse,4*sizeof(float));
    return n+1;
}
static float residual(const vo_context_t *c,int i,const float *shift) {
    float uv[2];
    const vo_point_t *a=&c->current[i];const vo_pair_t *p=&c->pairs[i];
    if(!camera_pixel(c,a->x+shift[0],a->y+shift[1],a->score+shift[2],uv)) return 1e20f;
    return (uv[0]-p->bu)*(uv[0]-p->bu)+(uv[1]-p->bv)*(uv[1]-p->bv);
}
static int classify(vo_context_t *c,int n,const float *m,float tx,float ty,float *cost,int required) {
    int count=0;*cost=0;
    float shift[3]={-m[0]*tx-m[3]*ty,-m[1]*tx-m[4]*ty,-m[2]*tx-m[5]*ty};
    for(int i=0;i<n;++i) {
        float e=residual(c,i,shift);
        c->inlier[i]=(uint8_t)(e<=P(c,GATE)*P(c,GATE));
        if(c->inlier[i]) {++count;*cost+=e;}
        if(count+n-i-1<required) return -1;
    }
    return count;
}
/* Attitude/height fix rotation and scale: only two translation unknowns.
 * Deterministic displacement consensus, a strict majority, and bounded refits.
 * No random hypotheses and no rank-two feature-distribution requirement. */
static int fit_impl(vo_context_t *c,int n,const float *m) {
    c->output[O_MATCHES]=(float)n;c->output[O_REASON]=VO_REASON_COUNT;
    if(n<(int)P(c,MATCHES)) return 0;
    /* current[] is idle until commit: reuse it for homogeneous projection
     * coefficients, rather than allocate another per-point work array. */
    for(int i=0;i<n;++i) {
        float gx=c->pairs[i].ax-m[9],gy=c->pairs[i].ay-m[10];
        c->current[i]=(vo_point_t){m[0]*gx+m[3]*gy+m[6]*m[12],
            m[1]*gx+m[4]*gy+m[7]*m[12],m[2]*gx+m[5]*gy+m[8]*m[12]};
    }
    float best_cost=1e30f,best_tx=0,best_ty=0;int best_count=0;
    for(int i=0;i<n;++i) {
        vo_pair_t *p=&c->pairs[i];float cost,tx=p->ax-p->bx,ty=p->ay-p->by;
        int count=classify(c,n,m,tx,ty,&cost,best_count);
        if(count>best_count || (count==best_count && cost<best_cost)) {
            best_count=count;best_cost=cost;best_tx=tx;best_ty=ty;
        }
    }
    c->output[O_INLIERS]=(float)best_count;c->output[O_REASON]=VO_REASON_RATIO;
    if(best_count<(int)P(c,INLIERS) || best_count<=n/2 || (float)best_count/n<P(c,RATIO)) return 0;
    float tx=best_tx,ty=best_ty,cost=0;
    for(int pass=0;pass<2;++pass) {
        classify(c,n,m,tx,ty,&cost,0);
        float x=0,y=0,w=0;
        for(int i=0;i<n;++i) if(c->inlier[i]) {
            vo_pair_t *p=&c->pairs[i];x+=p->weight*(p->ax-p->bx);
            y+=p->weight*(p->ay-p->by);w+=p->weight;
        }
        if(w<=0) return 0;
        tx=x/w;ty=y/w;
    }
    int count=classify(c,n,m,tx,ty,&cost,0),cells[9]={0},occupied=0;
    c->output[O_INLIERS]=(float)count;
    if(count<(int)P(c,INLIERS) || count<=n/2 || (float)count/n<P(c,RATIO)) return 0;
    float vx=0,vy=0,w=0,effective=0;
    int spatial_count[24]={0};
    for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *q=&c->pairs[i];
        int cx=MAX(0,MIN(5,(int)(q->bu*6/VO_WIDTH)));
        int cy=MAX(0,MIN(3,(int)(q->bv*4/VO_HEIGHT)));
        ++spatial_count[6*cy+cx];
    }
    for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *q=&c->pairs[i];cells[cell(q->bu,q->bv)]=1;
        float ex=q->ax-q->bx-tx,ey=q->ay-q->by-ty;
        vx+=q->weight*ex*ex;vy+=q->weight*ey*ey;w+=q->weight;
        int cx=MAX(0,MIN(5,(int)(q->bu*6/VO_WIDTH)));
        int cy=MAX(0,MIN(3,(int)(q->bv*4/VO_HEIGHT)));
        int occupancy=spatial_count[6*cy+cx];
        effective+=q->weight*MIN(1.0f,3.0f/occupancy);
    }
    for(int i=0;i<9;++i) occupied+=cells[i];
    c->output[O_DN]=tx;c->output[O_DE]=ty;
    c->output[O_CELLS]=(float)occupied;
    c->output[O_RATIO]=(float)count/n;c->output[O_RESIDUAL]=sqrtf(cost/count);
    c->output[O_VAR_N]=vx/w;c->output[O_VAR_E]=vy/w;
    c->output[O_EFFECTIVE]=effective;c->output[O_MATCH_WEIGHT_SUM]=w;
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
    int n=0,affine=needs_affine(c,ref,m);
    c->output[O_ATTEMPTS]+=1;
    c->output[O_REF_COUNT]=(float)ref->count;
    if(ref==&c->previous)
        for(int i=0;i<ref->count;++i) c->output[O_NEW_PREV_COUNT]+=ref->fresh[i]!=0;
    for(int i=0;i<ref->count;++i) {
        vo_point_t p=ref->points[i];float uv[2],inverse[4],ground_ref[2],weight=0;
        uint32_t start=tick(c);
        int valid=warp(c,ref,m,pred,p.x,p.y,uv,inverse,affine,ground_ref);
        phase(c,T_WARP,start);
        if(!valid) continue;
        c->output[O_PROJECTED]+=1;
        if(!track_point(c,rim,im,stride,p.x,p.y,uv,inverse,radius,&weight)) continue;
        int next=add_pair(c,n,ref->meta,m,p.x,p.y,uv[0],uv[1],weight,ground_ref,inverse);
        if(next>n && ref==&c->previous) c->output[O_NEW_PREV_SURVIVED]+=ref->fresh[i]!=0;
        n=next;
    }
    if(!fit(c,n,m)) return 0;
    /* Sparse tracking needs a uniqueness check even when KLT converges. */
    if(c->output[O_SPARSE]) for(int i=0;i<n;++i) if(c->inlier[i]) {
        vo_pair_t *p=&c->pairs[i];int bx,by;
        if(!template_patch(c,c->patch,rim,VO_WIDTH,p->au,p->av,p->inverse) ||
           !sad_search(c,im,stride,p->bu,p->bv,P(c,SEARCH_MIN),&bx,&by) ||
           hypotf((float)bx-p->bu,(float)by-p->bv)>P(c,GATE)) {
            c->output[O_REASON]=VO_REASON_AMBIGUOUS;return 0;
        }
    }
    return 1;
}
static void save_frame(vo_frame_t *f,uint8_t *dst,const uint8_t *im,size_t stride,
                       const float *m,int64_t stamp,const vo_point_t *points,
                       const uint8_t *fresh,int count,const float *position) {
    memcpy(f->meta,m,VO_META_FLOATS*sizeof(float));f->stamp=stamp;f->count=count;
    memcpy(f->position,position,2*sizeof(float));memcpy(f->points,points,(size_t)count*sizeof(vo_point_t));
    memcpy(f->fresh,fresh,(size_t)count);
    for(size_t y=0;y<VO_HEIGHT;++y) memcpy(dst+y*VO_WIDTH,im+y*stride,VO_WIDTH);
}
static void commit(vo_context_t *c,const vo_frame_t *ref,int base,const uint8_t *im,
                   size_t stride,const float *m,int64_t stamp) {
    c->output[O_BASE]=(float)base;c->output[O_REF_HEIGHT]=ref->meta[12];
    float position[2]={ref->position[0]+c->output[O_DN],ref->position[1]+c->output[O_DE]};
    c->output[O_N]=position[0];c->output[O_E]=position[1];
    int count=0;
    for(int i=0;i<(int)c->output[O_MATCHES];++i) if(c->inlier[i]) {
        c->current[count++]=(vo_point_t){c->pairs[i].bu,c->pairs[i].bv,1};
        c->current_fresh[count-1]=0;
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
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,c->current_fresh,count,position);
        c->refresh_key=0;
        ++c->key_id;c->output[O_KEY_CHANGED]=1;
    }
    save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,c->current_fresh,count,position);
    phase(c,T_COPY,copy_start);
    c->output[O_KEY_ID]=(float)c->key_id;c->output[O_STATUS]=2;
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
       P(c,UNIQUENESS)<=0 || P(c,UNIQUENESS)>=1 ||
       P(c,CELL_MAX_POINTS)<1 || P(c,CELL_MAX_POINTS)>3 ||
       P(c,MIN_DISTANCE)<1 || P(c,MIN_DISTANCE)>16 ||
       P(c,WEAK_STRIDE)<1 || P(c,WEAK_STRIDE)>8 ||
       P(c,INIT_FULL_INTERVAL)<1 || P(c,INIT_FULL_INTERVAL)>100) return 0;
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
        int full=(c->initializing%(int)P(c,INIT_FULL_INTERVAL))==0;
        int count=replenish(c,im,stride,0,full);float origin[2]={0,0};
        ++c->initializing;
        if(count<(int)P(c,MATCHES)) {c->output[O_STATUS]=-1;c->output[O_REASON]=VO_REASON_COUNT;return;}
        uint32_t start=tick(c);
        save_frame(&c->key,c->key_image,im,stride,m,stamp,c->current,c->current_fresh,count,origin);
        save_frame(&c->previous,c->previous_image,im,stride,m,stamp,c->current,c->current_fresh,count,origin);
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
    else {
        if(!base) c->refresh_key=1; /* retire an unusable anchor after next real fix */
        /* The failed image cannot become a reference. Refill the last known
         * image so a later image has more viable points to try. */
        int old=c->previous.count;
        memcpy(c->current,c->previous.points,(size_t)old*sizeof(vo_point_t));
        memcpy(c->current_fresh,c->previous.fresh,(size_t)old);
        int count=replenish(c,c->previous_image,VO_WIDTH,old,0);
        if(count>old) {
            memcpy(c->previous.points,c->current,(size_t)count*sizeof(vo_point_t));
            memcpy(c->previous.fresh,c->current_fresh,(size_t)count);
            c->previous.count=count;
        }
    }
}
#ifdef VO_TESTING
/* Host-only geometry seam: accepts observations, never fabricates images or
 * alters production reference/position state. Not linked into firmware. */
void vo_test_pairs(vo_context_t *c,const float *rm,const float *m,const float *pairs,int count) {
    memset(c->output,0,sizeof(c->output));int n=0;
    for(int i=0;i<count && n<VO_MAX_FEATURES;++i) {
        const float *p=pairs+4*i;n=add_pair(c,n,rm,m,p[0],p[1],p[2],p[3],1,NULL,NULL);
    }
    c->output[O_STATUS]=fit(c,n,m)?2:0;
}
void vo_test_limit(vo_context_t *c,int count,int topup) {
    c->key.count=MIN(c->key.count,count);c->previous.count=MIN(c->previous.count,count);
    P(c,TOPUP_POINTS)=(float)topup;
}
void vo_test_spoil_key(vo_context_t *c) { memset(c->key_image,0,VO_PIXELS); }
int vo_test_search(vo_context_t *c,const uint8_t *ref,const uint8_t *im,
                   float ru,float rv,float u,float v,float *out) {
    const float inverse[4]={1,0,0,1};int bx,by;
    if(!template_patch(c,c->patch,ref,VO_WIDTH,ru,rv,inverse) ||
       !sad_search(c,im,VO_WIDTH,u,v,P(c,SEARCH_MAX),&bx,&by)) return 0;
    out[0]=(float)bx;out[1]=(float)by;return 1;
}
#endif

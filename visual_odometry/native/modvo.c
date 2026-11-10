/* MicroPython binding: generic buffer protocol, no OpenMV private headers. */
#include "py/runtime.h"
#include "py/mphal.h"
#include "vo_core.h"
#include <string.h>

typedef struct { mp_obj_base_t base; vo_context_t core; } vo_obj_t;
static const mp_obj_type_t vo_type;
static int64_t timestamp(mp_obj_t obj) {
    mp_buffer_info_t info;
    mp_get_buffer_raise(obj,&info,MP_BUFFER_READ);
    if(info.typecode!='I' || info.len!=8)
        mp_raise_ValueError(MP_ERROR_TEXT("timestamp requires two uint32 words"));
    uint32_t words[2];memcpy(words,info.buf,8);
    return (int64_t)((uint64_t)words[0]|((uint64_t)words[1]<<32));
}
static void *buffer(mp_obj_t obj,size_t bytes,mp_uint_t flags,int floating) {
    mp_buffer_info_t info;
    mp_get_buffer_raise(obj,&info,flags);
    if(info.len<bytes || (floating && (info.typecode!='f' || info.len%sizeof(float))))
        mp_raise_ValueError(MP_ERROR_TEXT("VO buffer size/type mismatch (float32 required)"));
    return info.buf;
}
static mp_obj_t vo_make_new(const mp_obj_type_t *type,size_t n_args,size_t n_kw,const mp_obj_t *args) {
    mp_arg_check_num(n_args,n_kw,1,1,false);
    const float *cfg=buffer(args[0],VO_CONFIG_FLOATS*sizeof(float),MP_BUFFER_READ,1);
    vo_obj_t *self=mp_obj_malloc(vo_obj_t,type);
    if(!vo_init(&self->core,cfg)) mp_raise_ValueError(MP_ERROR_TEXT("invalid VO settings"));
    return MP_OBJ_FROM_PTR(self);
}
static mp_obj_t reset(mp_obj_t self_in) {
    vo_reset(&((vo_obj_t *)MP_OBJ_TO_PTR(self_in))->core);return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(reset_obj,reset);
/* step(image_buffer, meta_f32, stamp_u32[lo,hi], prediction_f32[NE,radius], out_f32) */
static mp_obj_t step(size_t n_args,const mp_obj_t *args) {
    (void)n_args;
    vo_obj_t *self=MP_OBJ_TO_PTR(args[0]);
    const uint8_t *image=buffer(args[1],VO_PIXELS,MP_BUFFER_READ,0);
    const float *meta=buffer(args[2],VO_META_FLOATS*sizeof(float),MP_BUFFER_READ,1);
    int64_t stamp=timestamp(args[3]);
    const float *pred=buffer(args[4],3*sizeof(float),MP_BUFFER_READ,1);
    float *out=buffer(args[5],VO_RESULT_FLOATS*sizeof(float),MP_BUFFER_WRITE,1);
    vo_step(&self->core,image,VO_WIDTH,meta,stamp,pred,pred[2]);
    memcpy(out,self->core.output,VO_RESULT_FLOATS*sizeof(float));return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(step_obj,6,6,step);
static uint32_t profile_clock(void) { return (uint32_t)mp_hal_ticks_us(); }
static mp_obj_t profile(mp_obj_t self_in,mp_obj_t enabled) {
    vo_profile(&((vo_obj_t *)MP_OBJ_TO_PTR(self_in))->core,
               mp_obj_is_true(enabled)?profile_clock:NULL);return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(profile_obj,profile);
static mp_obj_t timings(mp_obj_t self_in,mp_obj_t output) {
    mp_buffer_info_t info;mp_get_buffer_raise(output,&info,MP_BUFFER_WRITE);
    if(info.typecode!='I' || info.len!=VO_PHASES*sizeof(uint32_t))
        mp_raise_ValueError(MP_ERROR_TEXT("timings requires six uint32 words"));
    memcpy(info.buf,((vo_obj_t *)MP_OBJ_TO_PTR(self_in))->core.timing,info.len);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(timings_obj,timings);
static mp_obj_t image_metrics(mp_obj_t self_in,mp_obj_t image_in,mp_obj_t output) {
    vo_obj_t *self=MP_OBJ_TO_PTR(self_in);
    const uint8_t *image=buffer(image_in,VO_PIXELS,MP_BUFFER_READ,0);
    float *out=buffer(output,VO_IMAGE_METRICS*sizeof(float),MP_BUFFER_WRITE,1);
    vo_image_metrics(image,VO_WIDTH,self->core.cfg[V_MIN_EIG],out);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(image_metrics_obj,image_metrics);
static const mp_rom_map_elem_t locals_table[]={
    {MP_ROM_QSTR(MP_QSTR_reset),MP_ROM_PTR(&reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_step),MP_ROM_PTR(&step_obj)},
    {MP_ROM_QSTR(MP_QSTR_profile),MP_ROM_PTR(&profile_obj)},
    {MP_ROM_QSTR(MP_QSTR_timings),MP_ROM_PTR(&timings_obj)},
    {MP_ROM_QSTR(MP_QSTR_image_metrics),MP_ROM_PTR(&image_metrics_obj)},
};
static MP_DEFINE_CONST_DICT(locals_dict,locals_table);
static MP_DEFINE_CONST_OBJ_TYPE(vo_type,MP_QSTR_Core,MP_TYPE_FLAG_NONE,
    make_new,vo_make_new,locals_dict,&locals_dict);
static const mp_rom_map_elem_t globals_table[]={
    {MP_ROM_QSTR(MP_QSTR___name__),MP_ROM_QSTR(MP_QSTR_rm_vo)},
    {MP_ROM_QSTR(MP_QSTR_Core),MP_ROM_PTR(&vo_type)},
    {MP_ROM_QSTR(MP_QSTR_ABI_VERSION),MP_ROM_INT(VO_ABI_VERSION)},
    {MP_ROM_QSTR(MP_QSTR_RESULT_FLOATS),MP_ROM_INT(VO_RESULT_FLOATS)},
    {MP_ROM_QSTR(MP_QSTR_CONTEXT_BYTES),MP_ROM_INT(sizeof(vo_context_t))},
};
static MP_DEFINE_CONST_DICT(globals_dict,globals_table);
const mp_obj_module_t rm_vo_module={.base={&mp_type_module},.globals=(mp_obj_dict_t *)&globals_dict};
MP_REGISTER_MODULE(MP_QSTR_rm_vo,rm_vo_module);

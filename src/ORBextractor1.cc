#include <fstream>
#include <iostream>
#include "ORBextractor.h"

//#define USE_SSE
#ifdef USE_SSE
                                                                                                                        #include <pmmintrin.h>  //SSE3
#include <emmintrin.h>  //SSE2
#include <xmmintrin.h>  //SSE
#endif // USE_SSE

//#undef __ARM_NEON__
#ifdef __ARM_NEON__
                                                                                                                        #include <arm_neon.h>
#include <android/log.h>
#define LOG_TAG "brodra"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#endif


namespace ORB_SLAM2 {

//static ofstream f4("/storage/emulated/0/rgb/f4_ORBextractor_myResize.txt");
//static ofstream f3("/storage/emulated/0/rgb/f3_ORBextractor_ComputeKeyPointsOctTree.txt");
//static ofstream f2("/storage/emulated/0/rgb/f2_ORBextractor.txt");

    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 20;

#ifdef __ARM_NEON__

                                                                                                                            int32x2x4_t off_x = { 0, 1, 2, 3, 3, 3, 2, 1};
int32x2x4_t off_y = {+3,+3,+2,+1, 0,-1,-2,-3};
uint8x8_t zero8x8 = vdup_n_u8 ( 0x00 );
uint8x8_t one8x8 = vdup_n_u8 ( 0x01 );
uint8x16_t zero8x16 = vdupq_n_u8 ( 0x00 );
uint8x16_t one8x16 = vdupq_n_u8 ( 0x01 );
uint8x16_t Ipx[16],bx[16],dx[16];
float32x2_t half = {0.5f,0.5f};
int32x2_t sgn = vdup_n_s32 ( 0x80000000 );
int32x2_t nsgn = vdup_n_s32 ( 0x7fffffff );
int32x4_t k0 = vdupq_n_s32 ( 442 );
int32x4_t k1 = vdupq_n_s32 ( 391 );
int32x4_t k2 = vdupq_n_s32 ( 268 );
int32x4_t k3 = vdupq_n_s32 ( 144 );

#else

    int32_t off_x[8] = {0, 1, 2, 3, 3, 3, 2, 1};
    int32_t off_y[8] = {+3, +3, +2, +1, 0, -1, -2, -3};

#endif

    void makeOffsets(int pixel[16], int rowStride) {
#ifdef __ARM_NEON__

                                                                                                                                int32x2_t step = vdup_n_s32 ( rowStride );
    int32x2_t off;
    for ( int i = 0,j = 0; i<4; i++,j+=2 ) {
        off = vmla_s32 ( off_x.val[i],off_y.val[i],step );
        vst1_s32 ( pixel+j,off );
        vst1_s32 ( pixel+j+8,vneg_s32 ( off ) );
        vst1_s32 ( pixel+j+16,off );
    }
    pixel[24] = pixel[8];

#else
        for (int j = 0; j < 8; j++) {
            pixel[j] = off_x[j] + off_y[j] * rowStride;
            pixel[j + 8] = -pixel[j];
            pixel[j + 16] = pixel[j];
        }
        pixel[24] = pixel[8];

#endif
    }

    int16_t cornerScore_b(const uint8_t *ptr, const int pixel[], const int threshold, const uint16_t bt) {
        const int N = 25;
        int k;
        int16_t v = ptr[0];
        int16_t d[N];
        for (k = 0; k < N; k++)
            d[k] = ptr[pixel[k]] - v;

        int16_t th = threshold;
        for (k = 0; k < 16; k++) {
            if (bt & (0x0001 << k)) {
                int16_t m = min(min(min(d[k + 1], d[k + 2]), min(d[k + 3], d[k + 4])),
                                min(min(d[k + 5], d[k + 6]), min(d[k + 7], d[k + 8])));
                th = max(th, min(m, d[k]));
            }
        }
        th--;
        return th;
    }

    int16_t cornerScore_d(const uchar *ptr, const int pixel[], const int threshold, const uint16_t dt) {
        const int N = 25;
        int k;
        int16_t v = ptr[0];
        int16_t d[N];
        for (k = 0; k < N; k++)
            d[k] = v - ptr[pixel[k]];

        int16_t th = threshold;
        for (k = 0; k < 16; k++) {
            if (dt & (0x0001 << k)) {
                int16_t m = min(min(min(d[k + 1], d[k + 2]), min(d[k + 3], d[k + 4])),
                                min(min(d[k + 5], d[k + 6]), min(d[k + 7], d[k + 8])));
                th = max(th, min(m, d[k]));
            }
        }
        th--;
        return th;
    }

    __inline__ __attribute__((always_inline)) uint16_t shift_and(const uint16_t x, const int i) {
        return x & ((x << (16 - i)) | (x >> i));
    }

#ifdef USE_SSE
                                                                                                                            __inline__ __attribute__((always_inline)) __m128i shand(const __m128i x,const int i)
{
    return _mm_and_si128(x,_mm_or_si128(_mm_slli_epi16(x,16-i),_mm_srli_epi16(x,i)));//x&((x<<(16-i))|(x>>i));
}
__inline__ __attribute__((always_inline)) __m128i load_uint8x8_int16x8(const uint8_t* ptr)
{
    return _mm_set_epi16( short(ptr[7]) , short(ptr[6]) , short(ptr[5]) , short(ptr[4]) , short(ptr[3]) , short(ptr[2]) , short(ptr[1]) , short(ptr[0]));
}
#endif // USE_SSE

    const int patternSize = 16;
    const int K = patternSize / 2, N = patternSize + K + 1;

    void ORBextractor::myFast(cv::Mat &img, std::vector<KeyPoint> &keypoints, int threshold) {
        int i, j, k, pixel[25];
        makeOffsets(pixel, img.step);

        keypoints.clear();

#ifdef USE_SSE
                                                                                                                                __m128i th = _mm_set1_epi8(uint8_t(threshold));
    __m128i zeros = _mm_setzero_si128();

    const int quarterPatternSize = patternSize/4;
    (void)quarterPatternSize;
    __m128i delta = _mm_set1_epi8(-128), t = _mm_set1_epi8((char)threshold), K16 = _mm_set1_epi8((char)K);
    (void)K16;
    (void)delta;
    (void)t;
#endif // USE_SSE
#ifdef __ARM_NEON__
                                                                                                                                uint8x16_t th = vdupq_n_u8 ( ( uchar ) threshold );
    uint8x16_t half = vdupq_n_u8 ( K );
#endif
        uchar threshold_tab[512];
        for (i = -255; i <= 255; i++)
            threshold_tab[i + 255] = (uchar) (i < -threshold ? 1 : i > threshold ? 2 : 0);

        AutoBuffer<uchar> _buf((img.cols + 16) * 3 * (sizeof(int) + sizeof(uchar)) + 128);
        uchar *buf[3];
        buf[0] = _buf;
        buf[1] = buf[0] + img.cols;
        buf[2] = buf[1] + img.cols;
        int *cpbuf[3];
        cpbuf[0] = (int *) alignPtr(buf[2] + img.cols, sizeof(int)) + 1;
        cpbuf[1] = cpbuf[0] + img.cols + 1;
        cpbuf[2] = cpbuf[1] + img.cols + 1;
        memset(buf[0], 0, img.cols * 3);

        for (i = 3; i < img.rows - 2; i++) {
            const uchar *ptr = img.ptr<uchar>(i) + 3;
            uchar *curr = buf[(i - 3) % 3];
            int *cornerpos = cpbuf[(i - 3) % 3];
            memset(curr, 0, img.cols);
            int ncorners = 0;

            if (i < img.rows - 3) {
                j = 3;
#ifdef USE_SSE
                                                                                                                                        /*for(; j < img.cols - 16 - 3; j += 16, ptr += 16)
                {
                    __m128i m0, m1;
                    __m128i v0 = _mm_loadu_si128((const __m128i*)ptr);
                    __m128i v1 = _mm_xor_si128(_mm_subs_epu8(v0, t), delta);
                    v0 = _mm_xor_si128(_mm_adds_epu8(v0, t), delta);

                    __m128i x0 = _mm_sub_epi8(_mm_loadu_si128((const __m128i*)(ptr + pixel[0])), delta);
                    __m128i x1 = _mm_sub_epi8(_mm_loadu_si128((const __m128i*)(ptr + pixel[quarterPatternSize])), delta);
                    __m128i x2 = _mm_sub_epi8(_mm_loadu_si128((const __m128i*)(ptr + pixel[2*quarterPatternSize])), delta);
                    __m128i x3 = _mm_sub_epi8(_mm_loadu_si128((const __m128i*)(ptr + pixel[3*quarterPatternSize])), delta);
                    m0 = _mm_and_si128(_mm_cmpgt_epi8(x0, v0), _mm_cmpgt_epi8(x1, v0));
                    m1 = _mm_and_si128(_mm_cmpgt_epi8(v1, x0), _mm_cmpgt_epi8(v1, x1));
                    m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmpgt_epi8(x1, v0), _mm_cmpgt_epi8(x2, v0)));
                    m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmpgt_epi8(v1, x1), _mm_cmpgt_epi8(v1, x2)));
                    m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmpgt_epi8(x2, v0), _mm_cmpgt_epi8(x3, v0)));
                    m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmpgt_epi8(v1, x2), _mm_cmpgt_epi8(v1, x3)));
                    m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmpgt_epi8(x3, v0), _mm_cmpgt_epi8(x0, v0)));
                    m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmpgt_epi8(v1, x3), _mm_cmpgt_epi8(v1, x0)));
                    m0 = _mm_or_si128(m0, m1);
                    int mask = _mm_movemask_epi8(m0);
                    if( mask == 0 )
                        continue;
                    if( (mask & 255) == 0 )
                    {
                        j -= 8;
                        ptr -= 8;
                        continue;
                    }

                    __m128i c0 = _mm_setzero_si128(), c1 = c0, max0 = c0, max1 = c0;
                    for( k = 0; k < N; k++ )
                    {
                        __m128i x = _mm_xor_si128(_mm_loadu_si128((const __m128i*)(ptr + pixel[k])), delta);
                        m0 = _mm_cmpgt_epi8(x, v0);//x>v+t      b
                        m1 = _mm_cmpgt_epi8(v1, x);//v-t>x     d

                        c0 = _mm_and_si128(_mm_sub_epi8(c0, m0), m0);
                        c1 = _mm_and_si128(_mm_sub_epi8(c1, m1), m1);

                        max0 = _mm_max_epu8(max0, c0);
                        max1 = _mm_max_epu8(max1, c1);
                    }

                    //max0 = _mm_max_epu8(max0, max1);
                    int mb = _mm_movemask_epi8(_mm_cmpgt_epi8(max0, K16));
                    int md = _mm_movemask_epi8(_mm_cmpgt_epi8(max1, K16));

                    for( k = 0; (mb > 0||md > 0) && k < 16; k++, mb >>= 1,md >>= 1 )
                        if(mb & 1)
                        {
                            cornerpos[ncorners++] = j+k;
                                curr[j+k] = (uchar)cornerScore_b(ptr+k, pixel, threshold,0xffff);
                        }
                        if(md & 1)
                        {
                            cornerpos[ncorners++] = j+k;
                                curr[j+k] = (uchar)cornerScore_d(ptr+k, pixel, threshold,0xffff);
                        }
                }*/
            for ( ; j < img.cols -16 - 3; j+=16, ptr+=16 ) {
                __m128i v,vi;
                __m128i bh,bl,dh,dl,bh_sa,bl_sa,dh_sa,dl_sa;
                __m128i mask;
                uint16_t is_sim;
                v = _mm_loadu_si128((__m128i*)ptr);//uint8x16

                mask = _mm_set1_epi8(0x01);//mask = 0x01
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[0]));
                bl = _mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask);//b = ((vi-v)>th)&mask
                dl = _mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask);//d = ((v-vi)>th)&mask
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[8]));
                bh = _mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask);//b |= ((vi-v)>th)&0x0100
                dh = _mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask);

                bh_sa = _mm_unpackhi_epi8(bl,bh);
                bl_sa = _mm_unpacklo_epi8(bl,bh);
                dh_sa = _mm_unpackhi_epi8(dl,dh);
                dl_sa = _mm_unpacklo_epi8(dl,dh);

                is_sim = _mm_movemask_epi8(_mm_cmpeq_epi16(_mm_or_si128(_mm_or_si128(bh_sa,bl_sa),_mm_or_si128(dh_sa,dl_sa)),zeros));
                if (is_sim == 0xffff)
                     continue;//(b|d)==0

                mask = _mm_set1_epi8(0x10);//mask = 0x10
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[4]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[12]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                bh_sa = _mm_unpackhi_epi8(bl,bh);
                bl_sa = _mm_unpacklo_epi8(bl,bh);
                dh_sa = _mm_unpackhi_epi8(dl,dh);
                dl_sa = _mm_unpacklo_epi8(dl,dh);

                bh_sa = shand(bh_sa,4);
                bl_sa = shand(bl_sa,4);
                dh_sa = shand(dh_sa,4);
                dl_sa = shand(dl_sa,4);

                is_sim = _mm_movemask_epi8(_mm_cmpeq_epi16(_mm_or_si128(_mm_or_si128(bh_sa,bl_sa),_mm_or_si128(dh_sa,dl_sa)),zeros));
                if (is_sim == 0xffff)
                     continue;//(b|d)==0

                mask = _mm_set1_epi8(0x04);//mask = 0x04
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[2]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[10]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                mask = _mm_set1_epi8(0x40);//mask = 0x40
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[6]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[14]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                bh_sa = _mm_unpackhi_epi8(bl,bh);
                bl_sa = _mm_unpacklo_epi8(bl,bh);
                dh_sa = _mm_unpackhi_epi8(dl,dh);
                dl_sa = _mm_unpacklo_epi8(dl,dh);

                bh_sa = shand(bh_sa,2);
                bl_sa = shand(bl_sa,2);
                dh_sa = shand(dh_sa,2);
                dl_sa = shand(dl_sa,2);

                bh_sa = shand(bh_sa,4);
                bl_sa = shand(bl_sa,4);
                dh_sa = shand(dh_sa,4);
                dl_sa = shand(dl_sa,4);

                is_sim = _mm_movemask_epi8(_mm_cmpeq_epi16(_mm_or_si128(_mm_or_si128(bh_sa,bl_sa),_mm_or_si128(dh_sa,dl_sa)),zeros));
                if (is_sim == 0xffff)
                     continue;//(b|d)==0

                mask = _mm_set1_epi8(0x02);//mask = 0x02
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[1]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[9]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                mask = _mm_set1_epi8(0x08);//mask = 0x08
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[3]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[11]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                mask = _mm_set1_epi8(0x20);//mask = 0x20
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[5]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[13]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                mask = _mm_set1_epi8(0x80);//mask = 0x80
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[7]));
                bl = _mm_or_si128(bl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dl = _mm_or_si128(dl,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));
                vi = _mm_loadu_si128((__m128i*)(ptr+pixel[15]));
                bh = _mm_or_si128(bh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(vi,v),th),zeros),mask));//b |= ((vi-v)>th)&mask
                dh = _mm_or_si128(dh,_mm_andnot_si128(_mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(v,vi),th),zeros),mask));

                bh_sa = _mm_unpackhi_epi8(bl,bh);
                bl_sa = _mm_unpacklo_epi8(bl,bh);
                dh_sa = _mm_unpackhi_epi8(dl,dh);
                dl_sa = _mm_unpacklo_epi8(dl,dh);

                bh_sa = shand(bh_sa,1);
                bl_sa = shand(bl_sa,1);
                dh_sa = shand(dh_sa,1);
                dl_sa = shand(dl_sa,1);

                bh_sa = shand(bh_sa,2);
                bl_sa = shand(bl_sa,2);
                dh_sa = shand(dh_sa,2);
                dl_sa = shand(dl_sa,2);

                bh_sa = shand(bh_sa,4);
                bl_sa = shand(bl_sa,4);
                dh_sa = shand(dh_sa,4);
                dl_sa = shand(dl_sa,4);

                bh_sa = shand(bh_sa,1);
                bl_sa = shand(bl_sa,1);
                dh_sa = shand(dh_sa,1);
                dl_sa = shand(dl_sa,1);
                is_sim = _mm_movemask_epi8(_mm_cmpeq_epi16(_mm_or_si128(_mm_or_si128(bh_sa,bl_sa),_mm_or_si128(dh_sa,dl_sa)),zeros));
                if (is_sim == 0xffff)
                     continue;//(b|d)==0

                uint16_t b[16],d[16];
                _mm_storeu_si128((__m128i*)b,bl_sa);
                _mm_storeu_si128((__m128i*)(b+8),bh_sa);
                _mm_storeu_si128((__m128i*)d,dl_sa);
                _mm_storeu_si128((__m128i*)(d+8),dh_sa);

                for (int tmp = 0;tmp<16;tmp++)
                {
                    if (b[tmp])
                    {
                        cornerpos[ncorners++] = j+tmp;
                        curr[j+tmp] = ( uchar ) cornerScore_b ( ptr+tmp, pixel, threshold ,b[tmp]);
                        //cout<<i<<"\t"<<j+tmp<<"\t"<<b[tmp]<<"\t"<<uint(curr[j+tmp])<<endl;
                    }
                    if (d[tmp])
                    {
                        cornerpos[ncorners++] = j+tmp;
                        curr[j+tmp] = ( uchar ) cornerScore_d ( ptr+tmp, pixel, threshold ,d[tmp]);
                        //cout<<i<<"\t"<<j+tmp<<"\t"<<d[tmp]<<"\t"<<uint(curr[j+tmp])<<endl;
                    }
                }
            }
#endif // USE_SSE
#ifdef __ARM_NEON__
                                                                                                                                        for ( ; j < img.cols - 16 - 3; j += 16, ptr += 16 ) {
                uint8x16_t omax = vld1q_u8 ( ptr ); //中心p像素
                uint64_t mask;
                uint64x2_t cmp;

                cmp = vreinterpretq_s64_u8 ( vorrq_u8 (
                                                 vcgtq_u8 ( vabdq_u8 ( omax,vld1q_u8 ( ptr+pixel[0] ) ),th ),
                                                 vcgtq_u8 ( vabdq_u8 ( omax,vld1q_u8 ( ptr+pixel[8] ) ),th )
                                             ) );

                vst1_u64 ( &mask,vorr_u64 ( vget_high_u64 ( cmp ),vget_low_u64 ( cmp ) ) );

                if ( !mask )
                    continue;

                uint8x16_t omin = vqsubq_u8 ( omax,th );
                omax = vqaddq_u8 ( omax,th );

                //cmp = vreinterpret_s64_u8(vand_u8(cmp,vorr_u8(
                //    vcgt_u8(vabd_u8(o,vld1_u8(ptr+pixel[4])),th),
                //    vcgt_u8(vabd_u8(o,vld1_u8(ptr+pixel[12])),th)
                //)));

                //vst1_u64(&mask,cmp);

                //if( !mask )
                //    continue;

                for ( k = 0; k<16; k+=4 ) {
                    Ipx[k] = vld1q_u8 ( ptr+pixel[k] ); //p0像素
                    bx[k] = vcgtq_u8 ( Ipx[k],omax );
                    dx[k] = vcltq_u8 ( Ipx[k],omin );
                }

                uint8x16_t b = vandq_u8 ( bx[0],bx[4] );
                uint8x16_t d = vandq_u8 ( dx[0],dx[4] );
                b = vorrq_u8 ( b,vandq_u8 ( bx[4],bx[8] ) );
                d = vorrq_u8 ( d,vandq_u8 ( dx[4],dx[8] ) );
                b = vorrq_u8 ( b,vandq_u8 ( bx[8],bx[12] ) );
                d = vorrq_u8 ( d,vandq_u8 ( dx[8],dx[12] ) );
                b = vorrq_u8 ( b,vandq_u8 ( bx[12],bx[0] ) );
                d = vorrq_u8 ( d,vandq_u8 ( dx[12],dx[0] ) );
                cmp = vreinterpretq_s64_u8 ( vorrq_u8 ( b,d ) );
                vst1_u64 ( &mask,vorr_u64 ( vget_high_u64 ( cmp ),vget_low_u64 ( cmp ) ) );
                if ( !mask )
                    continue;

                uint8x16_t bn = zero8x16, dn = zero8x16;
                uint8x16_t bm = zero8x16, dm = zero8x16;
                for ( k = 0; k < 16; ) {
                    bn = vandq_u8 ( vaddq_u8 ( bn, one8x16 ), bx[k] );
                    dn = vandq_u8 ( vaddq_u8 ( dn, one8x16 ), dx[k] );

                    bm = vmaxq_u8 ( bn, bm );
                    dm = vmaxq_u8 ( dn, dm );
                    k++;
                    for ( int t = 0; t<3; t++,k++ ) {
                        Ipx[k] = vld1q_u8 ( ptr+pixel[k] ); //pk像素
                        bx[k] = vcgtq_u8 ( Ipx[k],omax );
                        dx[k] = vcltq_u8 ( Ipx[k],omin );

                        bn = vandq_u8 ( vaddq_u8 ( bn, one8x16 ), bx[k] );
                        dn = vandq_u8 ( vaddq_u8 ( dn, one8x16 ), dx[k] );

                        bm = vmaxq_u8 ( bn, bm );
                        dm = vmaxq_u8 ( dn, dm );
                    }
                }

                for ( k = 0; k<9; k++ ) {
                    bn = vandq_u8 ( vaddq_u8 ( bn, one8x16 ), bx[k] );
                    dn = vandq_u8 ( vaddq_u8 ( dn, one8x16 ), dx[k] );

                    bm = vmaxq_u8 ( bn, bm );
                    dm = vmaxq_u8 ( dn, dm );
                }

                uint8x16_t m = vcgtq_u8 ( vmaxq_u8 ( bm,dm ), half );

                for ( k = 0; k < 16; k++ ) {
                    if ( m[k] ) {
                        cornerpos[ncorners++] = j+k;
                        curr[j+k] = ( uchar ) cornerScore ( ptr+k, pixel, threshold );
                    }
                }
            }
#endif
                /*for( ; j < img.cols - 3; j++, ptr++ )
            {
                int v = ptr[0];
                const uchar* tab = &threshold_tab[0] - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if( d & 1 )
                {
                    int vt = v - threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x < vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                curr[j] = (uchar)cornerScore_d(ptr, pixel, threshold,0xffff);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }

                if( d & 2 )
                {
                    int vt = v + threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x > vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                curr[j] = (uchar)cornerScore_b(ptr, pixel, threshold,0xffff);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }
            }*/
                for (; j < img.cols - 3; j++, ptr++) {
                    int v, vi;
                    uint16_t b, d, bt, dt;
                    v = ptr[0];

                    vi = ptr[pixel[0]];
                    b = (vi - v) > threshold;
                    d = (v - vi) > threshold;
                    vi = ptr[pixel[8]];
                    b |= ((vi - v) > threshold) << 8;
                    d |= ((v - vi) > threshold) << 8;
                    if (!(b | d)) continue;

                    vi = ptr[pixel[4]];
                    b |= ((vi - v) > threshold) << 4;
                    d |= ((v - vi) > threshold) << 4;
                    vi = ptr[pixel[12]];
                    b |= ((vi - v) > threshold) << 12;
                    d |= ((v - vi) > threshold) << 12;
                    bt = shift_and(b, 4);
                    dt = shift_and(d, 4);
                    if (!(bt | dt)) continue;

                    vi = ptr[pixel[2]];
                    b |= ((vi - v) > threshold) << 2;
                    d |= ((v - vi) > threshold) << 2;
                    vi = ptr[pixel[6]];
                    b |= ((vi - v) > threshold) << 6;
                    d |= ((v - vi) > threshold) << 6;
                    vi = ptr[pixel[10]];
                    b |= ((vi - v) > threshold) << 10;
                    d |= ((v - vi) > threshold) << 10;
                    vi = ptr[pixel[14]];
                    b |= ((vi - v) > threshold) << 14;
                    d |= ((v - vi) > threshold) << 14;
                    bt = shift_and(b, 2);
                    dt = shift_and(d, 2);
                    bt = shift_and(bt, 4);
                    dt = shift_and(dt, 4);
                    if (!(bt | dt)) continue;

                    vi = ptr[pixel[1]];
                    b |= ((vi - v) > threshold) << 1;
                    d |= ((v - vi) > threshold) << 1;
                    vi = ptr[pixel[3]];
                    b |= ((vi - v) > threshold) << 3;
                    d |= ((v - vi) > threshold) << 3;
                    vi = ptr[pixel[5]];
                    b |= ((vi - v) > threshold) << 5;
                    d |= ((v - vi) > threshold) << 5;
                    vi = ptr[pixel[7]];
                    b |= ((vi - v) > threshold) << 7;
                    d |= ((v - vi) > threshold) << 7;
                    vi = ptr[pixel[9]];
                    b |= ((vi - v) > threshold) << 9;
                    d |= ((v - vi) > threshold) << 9;
                    vi = ptr[pixel[11]];
                    b |= ((vi - v) > threshold) << 11;
                    d |= ((v - vi) > threshold) << 11;
                    vi = ptr[pixel[13]];
                    b |= ((vi - v) > threshold) << 13;
                    d |= ((v - vi) > threshold) << 13;
                    vi = ptr[pixel[15]];
                    b |= ((vi - v) > threshold) << 15;
                    d |= ((v - vi) > threshold) << 15;
                    bt = shift_and(b, 1);
                    dt = shift_and(d, 1);
                    bt = shift_and(bt, 2);
                    dt = shift_and(dt, 2);
                    bt = shift_and(bt, 4);
                    dt = shift_and(dt, 4);
                    bt = shift_and(bt, 1);
                    dt = shift_and(dt, 1);
                    if (!(bt | dt)) continue;

                    if (bt) {
                        cornerpos[ncorners++] = j;
                        curr[j] = (uchar) cornerScore_b(ptr, pixel, threshold, bt);
                        //cout<<i<<"\t"<<j<<"\t"<<bt<<"\t"<<uint(curr[j])<<endl;
                    }
                    if (dt) {
                        cornerpos[ncorners++] = j;
                        curr[j] = (uchar) cornerScore_d(ptr, pixel, threshold, dt);
                        //cout<<i<<"\t"<<j<<"\t"<<dt<<"\t"<<uint(curr[j])<<endl;
                    }
                }

            }

            cornerpos[-1] = ncorners;

            if (i == 3)
                continue;

            const uchar *prev = buf[(i - 4 + 3) % 3];
            const uchar *pprev = buf[(i - 5 + 3) % 3];
            cornerpos = cpbuf[(i - 4 + 3) % 3];
            ncorners = cornerpos[-1];

            for (k = 0; k < ncorners; k++) {
#ifdef __ARM_NEON__
                                                                                                                                        j = cornerpos[k];
            uint8_t score = prev[j];
            uint8x8_t neib = {
                pprev[j-1], pprev[j],   pprev[j+1],
                prev[j-1],              prev[j+1],
                curr[j-1],  curr[j],    curr[j+1]
            };
            uint64_t nm;
            vst1_u64 ( &nm,vreinterpret_u64_u8 ( vcle_u8 ( vdup_n_u8 ( score ),neib ) ) );
            if ( !nm ) {
                keypoints.push_back ( KeyPoint ( j, ( i-1 ), 7, -1, score ) );
            }
#else

                j = cornerpos[k];
                int score = prev[j];
                if (
                        (score > prev[j + 1] && score > prev[j - 1] &&
                         score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
                         score > curr[j - 1] && score > curr[j] && score > curr[j + 1])) {
                    keypoints.push_back(KeyPoint((float) j, (float) (i - 1), 7.f, -1, (float) score));
                }

#endif
            }
        }
    }


    static float IC_Angle(const Mat &image, Point2i pt, const vector<int> &u_max) {
#ifdef __ARM_NEON__
                                                                                                                                int m_01 = 0, m_10 = 0;
    int step = image.step;

    const uchar* center_top = image.ptr<uchar> ( pt.y-HALF_PATCH_SIZE, pt.x );

    int32x2_t m= {0,0};

    for ( int r = -HALF_PATCH_SIZE; r<=HALF_PATCH_SIZE; r++,center_top+=step ) {
        int32x2_t x= {0,0},y= {0,0};
        int32x2_t pixel;
        int32x2_t kx,ky;
        ky = vdup_n_s32 ( r );
        int d = u_max[abs ( r )];

        for ( int i = 1; i<=d; i++ ) {
            pixel[0]=center_top[i];
            pixel[1]=center_top[-i];
            kx[0] = i;
            kx[1] = -i;
            x = vadd_s32 ( x,vmul_s32 ( pixel,kx ) );
            y = vadd_s32 ( y,vmul_s32 ( pixel,ky ) );
        }
        pixel[0] = 0;
        pixel[1] = center_top[0];
        m = vadd_s32 ( m,vadd_s32 ( vpadd_s32 ( x,y ),vmul_s32 ( pixel,ky ) ) );
    }
    m_10 = m[0];
    m_01 = m[1];

    return fastAtan2 ( m_01, m_10 );
#else

        int m_01 = 0, m_10 = 0;

        const uchar *center = image.ptr<uchar>(pt.y, pt.x);

        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int) image.step1();
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u) {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return fastAtan2((float) m_01, (float) m_10);


#endif
    }

    __inline__ __attribute__((always_inline)) int myRound(float f) //android端uneon版和PC端数据在51帧有一位描述子不同
    {
        void *pf = &f;
        int s = *((int *) pf);
        int i = s & 0x7fffffff;
        s = s & 0x80000000;
        void *pi = &i;
        f = *((float *) pi) + 0.5f;
        i = *((int *) pf) | s;
        return *((float *) pi);
    }

    const float factorPI = (float) (CV_PI / 180.f);

    static void computeOrbDescriptor(const KeyPoint &kpt,
                                     const Mat &img, const Point *pattern,
                                     uchar *desc) {
#ifdef __ARM_NEON__
                                                                                                                                float angle = ( float ) kpt.angle*factorPI;
    float a = ( float ) cos ( angle ), b = ( float ) sin ( angle );

    const uchar* center = img.ptr<uchar> ( kpt.pt.y, kpt.pt.x );
    const int step = img.step;

    float32x2_t ba = {b,a};
    float32x2_t a_b = {a,-b};
    int32x2_t step1 = {step,1};
    for ( int i = 0; i < 4; ++i, pattern += 128 ) {
        uint8x8_t v = zero8x8;
        for ( int j = 0,tmp1 = 0; j<8; j++,tmp1+=2 ) {
            uchar t0[8], t1[8];
            for ( int k = 0,tmp2 = 0; k<8; k++,tmp2+=16 ) {
                int t = tmp1+tmp2;
                int32x2_t xy0 = {pattern[t].x,pattern[t].y};
                float32x2_t f0 = vcvt_f32_s32 ( xy0 );//int32转float32
                t++;
                int32x2_t xy1 = {pattern[t].x,pattern[t].y};
                float32x2_t f1 = vcvt_f32_s32 ( xy1 );//int32转float32

                f0 = vpadd_f32 ( vmul_f32 ( f0,ba ),vmul_f32 ( f0,a_b ) );//[ y' , x' ] = [ x*b+y*a , x*a-y*b ]
                xy0 = vcvt_s32_f32 ( vorr_s32 ( vadd_f32 ( vand_s32 ( f0,nsgn ),half ),vand_s32 ( f0,sgn ) ) );

                f1 = vpadd_f32 ( vmul_f32 ( f1,ba ),vmul_f32 ( f1,a_b ) );//[ y' , x' ] = [ x*b+y*a , x*a-y*b ]
                xy1 = vcvt_s32_f32 ( vorr_s32 ( vadd_f32 ( vand_s32 ( f1,nsgn ),half ),vand_s32 ( f1,sgn ) ) );

                int32x2_t xy = vpadd_s32 ( vmul_s32 ( xy0,step1 ),vmul_s32 ( xy1,step1 ) );//[ y'*step+x' ]
                t0[k] = center[xy[0]];
                t1[k] = center[xy[1]];
            }
            uint8x8_t v0 = vld1_u8 ( t0 );
            uint8x8_t v1 = vld1_u8 ( t1 );
            uint8x8_t vcmp = vand_u8 ( vclt_u8 ( v0,v1 ),vdup_n_u8 ( 0x01<<j ) );
            v = vorr_u8 ( vcmp,v );
        }
        vst1_u8 ( desc+ ( i*8 ),v );
    }
#endif // __ARM_NEON__
#ifdef USE_SSE
                                                                                                                                float angle = kpt.angle*factorPI;
    float a = cos(angle), b = sin(angle);
    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    float step = img.step;
    __m128 ba = {b,a,b,a};//float32x4
    __m128 a_b = {a,-b,a,-b};//float32x4
    __m128 step1 = _mm_set1_ps(step);//int32x4
    //分两次做，第一次先计算前256个点，即128对点；得到128位描述子
    for (int i = 0; i < 2; ++i, pattern += 256) {
        __m128i v = _mm_setzero_si128();
        for(int j = 0,tmp1 = 0; j<8; j++,tmp1+=4) {
            uint16_t t0[8], t1[8],t2[8],t3[8];
            for(int k = 0,tmp2 = 0; k<8; k++,tmp2+=32) {
                int t = tmp1+tmp2;
                __m128 f0 = _mm_cvtepi32_ps(_mm_loadu_si128((__m128i*)(pattern+t)));//int32x4转float32x4
                t+=2;
                __m128 f1 = _mm_cvtepi32_ps(_mm_loadu_si128((__m128i*)(pattern+t)));//int32x4转float32x4
                __m128 f01_ba =_mm_cvtepi32_ps(_mm_cvtps_epi32(_mm_hadd_ps(_mm_mul_ps(f0,ba),_mm_mul_ps(f1,ba))));//[ y' ] = [ x*b+y*a ]
                __m128 f01_a_b = _mm_cvtepi32_ps(_mm_cvtps_epi32(_mm_hadd_ps(_mm_mul_ps(f0,a_b),_mm_mul_ps(f1,a_b))));//[ x' ] = [ x*a-y*b ]
                __m128 xy = _mm_add_ps(_mm_mul_ps(f01_ba,step1),f01_a_b);//[ y'*step + x' ]
                float index[4];
                _mm_storeu_ps(index,xy);

                t0[k] = center[(int)index[0]];
                t1[k] = center[(int)index[1]];
                t2[k] = center[(int)index[2]];
                t3[k] = center[(int)index[3]];
            }
            __m128i v0 = _mm_loadu_si128((__m128i*)t0);
            __m128i v1 = _mm_loadu_si128((__m128i*)t1);
            __m128i v2 = _mm_loadu_si128((__m128i*)t2);
            __m128i v3 = _mm_loadu_si128((__m128i*)t3);

            __m128i vcmp1 = _mm_and_si128(_mm_cmplt_epi16(v0,v1),_mm_set1_epi16(1<<(2*j)));
            __m128i vcmp2 = _mm_and_si128(_mm_cmplt_epi16(v2,v3),_mm_set1_epi16(2<<(2*j)));
            v = _mm_or_si128(vcmp1,v);
            v = _mm_or_si128(vcmp2,v);

        }
        _mm_storeu_si128((__m128i*)desc+i,v);
    }
#endif // USE_SSE
        /*#ifndef USE_SSE|__ARM_NEON__
        float angle = ( float ) kpt.angle*factorPI;
        float a = ( float ) cos ( angle ), b = ( float ) sin ( angle );

        const uchar* center = &img.at<uchar> ( cvRound ( kpt.pt.y ), cvRound ( kpt.pt.x ) );
        const int step = ( int ) img.step;

    //    #define GET_VALUE(idx) \
    //            center[myRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
    //                   myRound(pattern[idx].x*a - pattern[idx].y*b)]

    #define GET_VALUE(idx) \
            center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
                   cvRound(pattern[idx].x*a - pattern[idx].y*b)]


        for ( int i = 0; i < 32; ++i, pattern += 16 ) {
            int t0, t1, val;
            t0 = GET_VALUE ( 0 );
            t1 = GET_VALUE ( 1 );
            val = t0 < t1;
            t0 = GET_VALUE ( 2 );
            t1 = GET_VALUE ( 3 );
            val |= ( t0 < t1 ) << 1;
            t0 = GET_VALUE ( 4 );
            t1 = GET_VALUE ( 5 );
            val |= ( t0 < t1 ) << 2;
            t0 = GET_VALUE ( 6 );
            t1 = GET_VALUE ( 7 );
            val |= ( t0 < t1 ) << 3;
            t0 = GET_VALUE ( 8 );
            t1 = GET_VALUE ( 9 );
            val |= ( t0 < t1 ) << 4;
            t0 = GET_VALUE ( 10 );
            t1 = GET_VALUE ( 11 );
            val |= ( t0 < t1 ) << 5;
            t0 = GET_VALUE ( 12 );
            t1 = GET_VALUE ( 13 );
            val |= ( t0 < t1 ) << 6;
            t0 = GET_VALUE ( 14 );
            t1 = GET_VALUE ( 15 );
            val |= ( t0 < t1 ) << 7;

            desc[i] = ( uchar ) val;
        }

    #undef GET_VALUE
    #endif // USE_SSE*/
    }

    static int bit_pattern_31_[256 * 4] = {
            8, -3, 9, 5/*mean (0), correlation (0)*/,
            4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
            -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
            7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
            2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
            1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
            -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
            -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
            -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
            10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
            -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
            -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
            7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
            -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
            -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
            -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
            12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
            -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
            -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
            11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
            4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
            5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
            3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
            -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
            -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
            -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
            -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
            -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
            -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
            5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
            5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
            1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
            9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
            4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
            2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
            -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
            -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
            4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
            0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
            -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
            -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
            -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
            8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
            0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
            7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
            -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
            10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
            -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
            10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
            -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
            -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
            3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
            5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
            -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
            3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
            2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
            -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
            -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
            -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
            -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
            6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
            -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
            -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
            -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
            3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
            -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
            -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
            2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
            -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
            -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
            5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
            -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
            -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
            -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
            10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
            7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
            -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
            -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
            7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
            -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
            -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
            -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
            7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
            -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
            1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
            2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
            -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
            -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
            7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
            1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
            9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
            -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
            -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
            7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
            12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
            6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
            5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
            2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
            3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
            2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
            9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
            -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
            -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
            1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
            6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
            2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
            6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
            3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
            7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
            -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
            -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
            -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
            -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
            8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
            4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
            -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
            4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
            -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
            -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
            7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
            -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
            -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
            8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
            -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
            1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
            7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
            -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
            11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
            -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
            3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
            5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
            0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
            -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
            0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
            -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
            5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
            3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
            -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
            -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
            -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
            6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
            -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
            -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
            1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
            4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
            -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
            2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
            -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
            4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
            -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
            -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
            7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
            4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
            -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
            7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
            7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
            -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
            -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
            -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
            2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
            10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
            -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
            8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
            2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
            -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
            -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
            -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
            5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
            -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
            -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
            -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
            -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
            -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
            2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
            -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
            -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
            -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
            -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
            6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
            -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
            11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
            7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
            -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
            -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
            -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
            -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
            -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
            -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
            -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
            -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
            1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
            1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
            9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
            5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
            -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
            -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
            -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
            -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
            8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
            2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
            7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
            -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
            -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
            4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
            3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
            -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
            5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
            4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
            -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
            0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
            -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
            3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
            -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
            8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
            -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
            2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
            10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
            6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
            -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
            -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
            -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
            -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
            -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
            4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
            2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
            6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
            3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
            11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
            -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
            4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
            2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
            -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
            -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
            -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
            6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
            0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
            -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
            -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
            -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
            5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
            2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
            -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
            9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
            11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
            3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
            -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
            3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
            -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
            5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
            8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
            7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
            -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
            7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
            9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
            7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
            -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
    };

    ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST) :
            nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
            iniThFAST(_iniThFAST), minThFAST(_minThFAST) {
        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < nlevels; i++) {
            mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
            mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for (int i = 0; i < nlevels; i++) {
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        mvImagePyramid.resize(nlevels);

        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale =
                nfeatures * (1 - factor) / (1 - (float) pow((double) factor, (double) nlevels));

        int sumFeatures = 0;
        for (int level = 0; level < nlevels - 1; level++) {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

        const int npoints = 512;
        const Point *pattern0 = (const Point *) bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        //This is for orientation
        // pre-compute the end of a row in a circular patch
        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }

        first = true;
        iscale = scaleFactor * 2048;
    }

    static void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax) {
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint) {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4) {
        const int halfX = ceil(static_cast<float> ( UR.x - UL.x ) / 2);
        const int halfY = ceil(static_cast<float> ( BR.y - UL.y ) / 2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for (size_t i = 0; i < vKeys.size(); i++) {
            const cv::KeyPoint &kp = vKeys[i];
            if (kp.pt.x < n1.UR.x) {
                if (kp.pt.y < n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            } else if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;

    }

    bool less(const pair<int, ExtractorNode *> &m1, const pair<int, ExtractorNode *> &m2) {
        int dn = m1.first - m2.first;
        if (dn < 0) {
            return true;
        } else if (dn > 0) {
            return false;
        } else {
            int du = m1.second->UL.y - m2.second->UL.y;
            if (du < 0) {
                return true;
            } else if (du > 0) {
                return false;
            } else {
                int dl = m1.second->UL.x - m2.second->UL.x;
                if (dl < 0) {
                    return true;
                } else if (dl > 0) {
                    return false;
                } else {
                    int db = m1.second->BR.y - m2.second->BR.y;
                    if (db < 0) {
                        return true;
                    } else if (db > 0) {
                        return false;
                    } else {
                        return m1.second->BR.x - m2.second->BR.x < 0;
                    }
                }
            }
        }
    }

    vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                         const int &maxX, const int &minY, const int &maxY,
                                                         const int &N, const int &level) {
        // Compute how many initial nodes
        const int nIni = round(static_cast<float> ( maxX - minX ) / (maxY - minY));

        const float hX = static_cast<float> ( maxX - minX ) / nIni;

        list<ExtractorNode> lNodes;

        vector<ExtractorNode *> vpIniNodes;
        vpIniNodes.resize(nIni);

        for (int i = 0; i < nIni; i++) {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX * static_cast<float> ( i ), 0);
            ni.UR = cv::Point2i(hX * static_cast<float> ( i + 1 ), 0);
            ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
            ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for (size_t i = 0; i < vToDistributeKeys.size(); i++) {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
        }

        list<ExtractorNode>::iterator lit = lNodes.begin();

        while (lit != lNodes.end()) {
            if (lit->vKeys.size() == 1) {
                lit->bNoMore = true;
                lit++;
            } else if (lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int, ExtractorNode *> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        while (!bFinish) {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while (lit != lNodes.end()) {
                if (lit->bNoMore) {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                } else {
                    // If more than one point, subdivide
                    ExtractorNode n1, n2, n3, n4;
                    lit->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0) {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0) {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0) {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0) {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit = lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize) {
                bFinish = true;
            } else if (((int) lNodes.size() + nToExpand * 3) > N) {

                while (!bFinish) {

                    prevSize = lNodes.size();

                    vector<pair<int, ExtractorNode *> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end(), *less);
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--) {
                        ExtractorNode n1, n2, n3, n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        if (n1.vKeys.size() > 0) {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0) {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0) {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0) {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if ((int) lNodes.size() >= N)
                            break;
                    }

                    if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++) {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint *pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            for (size_t k = 1; k < vNodeKeys.size(); k++) {
                if (vNodeKeys[k].response > maxResponse) {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }


    void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> > &allKeypoints) {
        //struct timeval T0,T1,T2,T3;
        //int t0,t1,t2,t3;
        allKeypoints.resize(nlevels);

        const float W = 40;

        for (int level = 0; level < nlevels; ++level) {
            //gettimeofday(&T0,NULL);

            const int minBorderX = EDGE_THRESHOLD;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD;

            vector<cv::KeyPoint> vToDistributeKeys;
            vToDistributeKeys.reserve(nfeatures * 10);

            //const float width = ( maxBorderX-minBorderX );
            //const float height = ( maxBorderY-minBorderY );

            //const int nCols = width/W;
            //const int nRows = height/W;
            //const int wCell = ceil ( width/nCols );
            //const int hCell = ceil ( height/nRows );

            for (int i = minBorderY; i < maxBorderY; i += W) {
                const float iniY = i;
                float maxY = iniY + W;

                if (maxY > maxBorderY)
                    maxY = maxBorderY;
                if (maxY - iniY <= 8)
                    continue;

                for (int j = minBorderX; j < maxBorderX; j += W) {
                    const float iniX = j;
                    float maxX = iniX + W;
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;
                    if (maxX - iniX <= 8)
                        continue;

                    vector<cv::KeyPoint> vKeysCell;
                    cv::Mat roi = mvImagePyramid[level].rowRange(iniY, maxY - 2).colRange(iniX, maxX - 2);
                    //FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),vKeysCell,iniThFAST,true);
                    myFast(roi, vKeysCell, iniThFAST);


                    if (vKeysCell.empty()) {
                        //FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),vKeysCell,minThFAST,true);
                        myFast(roi, vKeysCell, minThFAST);
                    }

                    if (!vKeysCell.empty()) {
                        for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++) {
                            (*vit).pt.x += iniX - minBorderX;
                            (*vit).pt.y += iniY - minBorderY;
                            vToDistributeKeys.push_back(*vit);
                        }
                    }

                }
            }
            //gettimeofday(&T1,NULL);

            vector<KeyPoint> &keypoints = allKeypoints[level];
            keypoints.reserve(nfeatures);

            keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY, mnFeaturesPerLevel[level], level);

            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++) {
                keypoints[i].pt.x += minBorderX;
                keypoints[i].pt.y += minBorderY;
                keypoints[i].octave = level;
                keypoints[i].size = scaledPatchSize;
            }

            //gettimeofday(&T2,NULL);
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
            //cv::computeOrientation(mvImagePyramid[level], allKeypoints[level],HALF_PATCH_SIZE, umax);
            /*gettimeofday(&T3,NULL);
        t0 = T0.tv_sec*1000000+T0.tv_usec;
        t1 = T1.tv_sec*1000000+T1.tv_usec;
        t2 = T2.tv_sec*1000000+T2.tv_usec;
        t3 = T3.tv_sec*1000000+T3.tv_usec;
        if(!level)
        f3<<nkps<<"\t"<<level<<"\t"<<t3-t2<<"\t"<<t2-t1<<"\t"<<t1-t0<<std::endl;*/
        }

        // compute orientations
        //for (int level = 0; level < nlevels; ++level)
    }

/*void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> > &allKeypoints)
{
    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;

    for (int level = 0; level < nlevels; ++level)
    {
        const int nDesiredFeatures = mnFeaturesPerLevel[level];

        const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
        const int levelRows = imageRatio*levelCols;

        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;

        const int W = maxBorderX - minBorderX;
        const int H = maxBorderY - minBorderY;
        const int cellW = ceil((float)W/levelCols);
        const int cellH = ceil((float)H/levelRows);

        const int nCells = levelRows*levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);

        vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

        vector<vector<int> > nToRetain(levelRows,vector<int>(levelCols,0));
        vector<vector<int> > nTotal(levelRows,vector<int>(levelCols,0));
        vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));
        vector<int> iniXCol(levelCols);
        vector<int> iniYRow(levelRows);
        int nNoMore = 0;
        int nToDistribute = 0;


        float hY = cellH + 6;

        for(int i=0; i<levelRows; i++)
        {
            const float iniY = minBorderY + i*cellH - 3;
            iniYRow[i] = iniY;

            if(i == levelRows-1)
            {
                hY = maxBorderY+3-iniY;
                if(hY<=0)
                    continue;
            }

            float hX = cellW + 6;

            for(int j=0; j<levelCols; j++)
            {
                float iniX;

                if(i==0)
                {
                    iniX = minBorderX + j*cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];
                }


                if(j == levelCols-1)
                {
                    hX = maxBorderX+3-iniX;
                    if(hX<=0)
                        continue;
                }


                Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);

                cellKeyPoints[i][j].reserve(nfeaturesCell*5);

                FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);

                if(cellKeyPoints[i][j].size()<=3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);
                }


                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j] = nKeys;

                if(nKeys>nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;
                    bNoMore[i][j] = false;
                }
                else
                {
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell-nKeys;
                    bNoMore[i][j] = true;
                    nNoMore++;
                }

            }
        }


        // Retain by score

        while(nToDistribute>0 && nNoMore<nCells)
        {
            int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
            nToDistribute = 0;

            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
                    if(!bNoMore[i][j])
                    {
                        if(nTotal[i][j]>nNewFeaturesCell)
                        {
                            nToRetain[i][j] = nNewFeaturesCell;
                            bNoMore[i][j] = false;
                        }
                        else
                        {
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell-nTotal[i][j];
                            bNoMore[i][j] = true;
                            nNoMore++;
                        }
                    }
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nDesiredFeatures*2);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Retain by score and transform coordinates
        for(int i=0; i<levelRows; i++)
        {
            for(int j=0; j<levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);
                if((int)keysCell.size()>nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);


                for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                {
                    keysCell[k].pt.x+=iniXCol[j];
                    keysCell[k].pt.y+=iniYRow[i];
                    keysCell[k].octave=level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        if((int)keypoints.size()>nDesiredFeatures)
        {
            KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }

        //cv::ORB::computeOrientation(mvImagePyramid[level], allKeypoints[level],HALF_PATCH_SIZE, umax);

    }

    // and compute orientations
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}*/

    static void computeDescriptors(const Mat &image, vector<KeyPoint> &keypoints, Mat &descriptors,
                                   const vector<Point> &pattern) {
        descriptors = Mat::zeros((int) keypoints.size(), 32, CV_8UC1);

        for (size_t i = 0; i < keypoints.size(); i++)
            computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int) i));
    }

//int kernel[] = { 144,268,391,442,391,268,144 };


    void ORBextractor::myGaussian(cv::Mat &img) {
        int H = img.rows, W = img.cols;
        Mat &tmp = mvTmp[0];
        int *pTmp = tmp.ptr<int>(0, 3);
        uchar *pImg = img.ptr<uchar>(0, 3);
        //int* pK = kernel + 3;
        int i, j;
        for (i = 0; i < H; ++i, pTmp += 6, pImg += 6) {
            for (j = 3; j < W - 3; ++j, pTmp++, pImg++) {
                pTmp[0] = pImg[0] * 442;
                pTmp[0] += ((int) pImg[1] + pImg[-1]) * 391;
                pTmp[0] += ((int) pImg[2] + pImg[-2]) * 268;
                pTmp[0] += ((int) pImg[3] + pImg[-3]) * 144;
            }
        }

        pTmp = tmp.ptr<int>(3, 3);
        pImg = img.ptr<uchar>(3, 3);
        int t;
        int W2 = W * 2, W3 = W * 3;

        for (i = 3; i < H - 3; i++, pTmp += 6, pImg += 6) {
            j = 3;

#ifdef __ARM_NEON__

                                                                                                                                    for ( ; j < W - 3 - 16; j+=16, pTmp+=16, pImg+=16 ) {
            int32x4_t v0 = vmulq_s32 ( k0,vld1q_s32 ( pTmp ) );
            v0 = vaddq_s32 ( v0,vmulq_s32 ( k1,vaddq_s32 ( vld1q_s32 ( pTmp+W ),vld1q_s32 ( pTmp-W ) ) ) );
            v0 = vaddq_s32 ( v0,vmulq_s32 ( k2,vaddq_s32 ( vld1q_s32 ( pTmp+W2 ),vld1q_s32 ( pTmp-W2 ) ) ) );
            v0 = vaddq_s32 ( v0,vmulq_s32 ( k3,vaddq_s32 ( vld1q_s32 ( pTmp+W3 ),vld1q_s32 ( pTmp-W3 ) ) ) );
            v0 = vshrq_n_s32 ( v0,22 );

            int32x4_t v1 = vmulq_s32 ( k0,vld1q_s32 ( pTmp+4 ) );
            v1 = vaddq_s32 ( v1,vmulq_s32 ( k1,vaddq_s32 ( vld1q_s32 ( pTmp+W+4 ),vld1q_s32 ( pTmp-W+4 ) ) ) );
            v1 = vaddq_s32 ( v1,vmulq_s32 ( k2,vaddq_s32 ( vld1q_s32 ( pTmp+W2+4 ),vld1q_s32 ( pTmp-W2+4 ) ) ) );
            v1 = vaddq_s32 ( v1,vmulq_s32 ( k3,vaddq_s32 ( vld1q_s32 ( pTmp+W3+4 ),vld1q_s32 ( pTmp-W3+4 ) ) ) );
            v1 = vshrq_n_s32 ( v1,22 );

            int32x4_t v2 = vmulq_s32 ( k0,vld1q_s32 ( pTmp+8 ) );
            v2 = vaddq_s32 ( v2,vmulq_s32 ( k1,vaddq_s32 ( vld1q_s32 ( pTmp+W+8 ),vld1q_s32 ( pTmp-W+8 ) ) ) );
            v2 = vaddq_s32 ( v2,vmulq_s32 ( k2,vaddq_s32 ( vld1q_s32 ( pTmp+W2+8 ),vld1q_s32 ( pTmp-W2+8 ) ) ) );
            v2 = vaddq_s32 ( v2,vmulq_s32 ( k3,vaddq_s32 ( vld1q_s32 ( pTmp+W3+8 ),vld1q_s32 ( pTmp-W3+8 ) ) ) );
            v2 = vshrq_n_s32 ( v2,22 );

            int32x4_t v3 = vmulq_s32 ( k0,vld1q_s32 ( pTmp+12 ) );
            v3 = vaddq_s32 ( v3,vmulq_s32 ( k1,vaddq_s32 ( vld1q_s32 ( pTmp+W+12 ),vld1q_s32 ( pTmp-W+12 ) ) ) );
            v3 = vaddq_s32 ( v3,vmulq_s32 ( k2,vaddq_s32 ( vld1q_s32 ( pTmp+W2+12 ),vld1q_s32 ( pTmp-W2+12 ) ) ) );
            v3 = vaddq_s32 ( v3,vmulq_s32 ( k3,vaddq_s32 ( vld1q_s32 ( pTmp+W3+12 ),vld1q_s32 ( pTmp-W3+12 ) ) ) );
            v3 = vshrq_n_s32 ( v3,22 );

            vst1q_u8 ( pImg,vcombine_u8 (
                           vqmovun_s16 ( vcombine_s16 ( vmovn_s32 ( v0 ), vmovn_s32 ( v1 ) ) ),
                           vqmovun_s16 ( vcombine_s16 ( vmovn_s32 ( v2 ), vmovn_s32 ( v3 ) ) )
                       ) );
        }

#endif

            for (; j < W - 3; ++j, pTmp++, pImg++) {
                t = pTmp[0] * 442;
                t += (pTmp[W] + pTmp[-W]) * 391;
                t += (pTmp[W2] + pTmp[-W2]) * 268;
                t += (pTmp[W3] + pTmp[-W3]) * 144;
                pImg[0] = t >> 22;
            }
        }

    }


    void ORBextractor::init(cv::Mat &img) {
        first = false;
        mvImagePyramid.resize(nlevels);
        mvTmp.resize(nlevels);
        int type = img.type();
        cv::Size size = img.size();
        mvImagePyramid[0] = img;
        mvTmp[0].create(size, CV_32SC1);

        for (int lv = 1; lv < nlevels; lv++) {
            //float scale = mvInvScaleFactor[lv];
            size.width = cvRound(size.width / scaleFactor);
            mvTmp[lv].create(size, CV_32SC1);
            size.height = cvRound(size.height / scaleFactor);
            mvImagePyramid[lv].create(size, type);
        }
    }


    void ORBextractor::operator()(InputArray _image, InputArray _mask, vector<KeyPoint> &_keypoints,
                                  OutputArray _descriptors) {
        if (_image.empty())
            return;

        Mat image = _image.getMat();
        if (first) {
            init(image);
        }
        GaussianBlur(_image.getMat(), image, Size(5, 5), 2, 2, BORDER_REFLECT_101);
        //myGaussian ( image );
        //assert ( image.type() == CV_8UC1 );

        // Pre-compute the scale pyramid
        ComputePyramid(image);

        vector<vector<KeyPoint> > allKeypoints;
        ComputeKeyPointsOctTree(allKeypoints);
        //ComputeKeyPointsOld(allKeypoints);

        Mat descriptors;

        int nkeypoints = 0;
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int) allKeypoints[level].size();
        if (nkeypoints == 0)
            _descriptors.release();
        else {
            _descriptors.create(nkeypoints, 32, CV_8U);
            descriptors = _descriptors.getMat();
        }

        _keypoints.clear();
        _keypoints.reserve(nkeypoints);

        int offset = 0;
        for (int level = 0; level < nlevels; ++level) {
            vector<KeyPoint> &keypoints = allKeypoints[level];
            int nkeypointsLevel = (int) keypoints.size();

            if (nkeypointsLevel == 0)
                continue;

            // preprocess the resized image
            //Mat workingMat = mvImagePyramid[level].clone();
            //GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors
            Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            computeDescriptors( /*workingMat*/mvImagePyramid[level], keypoints, desc, pattern);
            //cv::computeDescriptors(workingMat, keypoints, desc, pattern,32,2);

            offset += nkeypointsLevel;

            // Scale keypoint coordinates
            if (level != 0) {
                float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
                for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                             keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                    keypoint->pt *= scale;
            }
            // And add the keypoints to the output
            _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());

        }

    }

    void ORBextractor::myResize(cv::Mat &src, cv::Mat &tmp, cv::Mat &dst) {
        //struct timeval T0,T1;
        //int t0,t1;
        //gettimeofday(&T0,NULL);
        uchar *pSrc = src.data;
        uchar *pDst = dst.data;
        int srcW = src.cols, srcH = src.rows;
        int dstW = dst.cols, dstH = dst.rows;
        int *pTmp = tmp.ptr<int>();
        //float scale_x = (float)srcW / dstW;
        //float scale_y = (float)srcH / dstH;
        //int iscale_x = scale_x*2048;
        //int iscale_y = scale_y*2048;
        //int iscale_x = (srcW<<11)/dstW;
        //int iscale_y = (srcH<<11)/dstH;
        short a0, a1, b0, b1;
        int i, j;

        for (j = 0; j < srcH; ++j) {
            for (i = 0; i < dstW; ++i) {
                /*float fx = (i + 0.5f) * scale_x - 0.5f;
            int sx = int(fx);
            fx -= sx;

            if (sx >= srcW - 1) {
                fx = 0, sx = srcW - 2;
            }

            a0 = cv::saturate_cast<short>((1.f - fx) * 2048);
            a1 = 2048 - a0;*/
                int sx = i * iscale;
                a1 = sx & 0x07ff;
                a0 = 2048 - a1;
                sx = sx >> 11;
                if (sx > srcW - 2) {
                    a0 = 2048;
                    a1 = 0;
                    sx = srcW - 2;
                }


                *(pTmp) = *(pSrc + j * srcW + sx) * a0 + *(pSrc + j * srcW + sx + 1) * a1;
                pTmp++;
            }
        }

        pTmp = tmp.ptr<int>();

        for (j = 0; j < dstH; ++j) {
            /*float fy = (j + 0.5f) * scale_y - 0.5f;
        int sy = int(fy);
        fy -= sy;
        sy = std::min(sy, srcH - 2);

        b0 = cv::saturate_cast<short>((1.f - fy) * 2048);
        b1 = 2048 - b0;*/
            int sy = j * iscale;
            b1 = sy & 0x07ff;
            b0 = 2048 - b1;
            sy = sy >> 11;

            i = 0;

#ifdef __ARM_NEON__

                                                                                                                                    int*S0 = pTmp + sy*dstW + i,*S1 = pTmp + ( sy + 1 ) *dstW + i;
        int16x8_t v_b0 = vdupq_n_s16 ( b0 ), v_b1 = vdupq_n_s16 ( b1 ), v_delta = vdupq_n_s16 ( 2 );

        for ( ; i < dstW-16; i+=16 ) {
            int32x4_t v_src00 = vshrq_n_s32 ( vld1q_s32 ( S0 + i ), 4 );
            int32x4_t v_src10 = vshrq_n_s32 ( vld1q_s32 ( S1 + i ), 4 );
            int32x4_t v_src01 = vshrq_n_s32 ( vld1q_s32 ( S0 + i + 4 ), 4 );
            int32x4_t v_src11 = vshrq_n_s32 ( vld1q_s32 ( S1 + i + 4 ), 4 );

            int16x8_t v_src0 = vcombine_s16 ( vmovn_s32 ( v_src00 ), vmovn_s32 ( v_src01 ) );
            int16x8_t v_src1 = vcombine_s16 ( vmovn_s32 ( v_src10 ), vmovn_s32 ( v_src11 ) );

            int16x8_t v_dst0 = vaddq_s16 ( vshrq_n_s16 ( vqdmulhq_s16 ( v_src0, v_b0 ), 1 ),
                                           vshrq_n_s16 ( vqdmulhq_s16 ( v_src1, v_b1 ), 1 ) );
            v_dst0 = vshrq_n_s16 ( vaddq_s16 ( v_dst0, v_delta ), 2 );

            v_src00 = vshrq_n_s32 ( vld1q_s32 ( S0 + i + 8 ), 4 );
            v_src10 = vshrq_n_s32 ( vld1q_s32 ( S1 + i + 8 ), 4 );
            v_src01 = vshrq_n_s32 ( vld1q_s32 ( S0 + i + 12 ), 4 );
            v_src11 = vshrq_n_s32 ( vld1q_s32 ( S1 + i + 12 ), 4 );

            v_src0 = vcombine_s16 ( vmovn_s32 ( v_src00 ), vmovn_s32 ( v_src01 ) );
            v_src1 = vcombine_s16 ( vmovn_s32 ( v_src10 ), vmovn_s32 ( v_src11 ) );

            int16x8_t v_dst1 = vaddq_s16 ( vshrq_n_s16 ( vqdmulhq_s16 ( v_src0, v_b0 ), 1 ),
                                           vshrq_n_s16 ( vqdmulhq_s16 ( v_src1, v_b1 ), 1 ) );
            v_dst1 = vshrq_n_s16 ( vaddq_s16 ( v_dst1, v_delta ), 2 );

            vst1q_u8 ( pDst, vcombine_u8 ( vqmovun_s16 ( v_dst0 ), vqmovun_s16 ( v_dst1 ) ) );
            pDst+=16;
        }
#endif

            for (; i < dstW; ++i) {
                int S0 = *(pTmp + sy * dstW + i);
                int S1 = *(pTmp + (sy + 1) * dstW + i);
                *(pDst) = uchar((((b0 * (S0 >> 4)) >> 16) + ((b1 * (S1 >> 4)) >> 16) + 2) >> 2);
                pDst++;
            }
        }
        //gettimeofday(&T1,NULL);
        //t0 = T0.tv_sec*1000000+T0.tv_usec;
        //t1 = T1.tv_sec*1000000+T1.tv_usec;
        //f4<<srcW<<"\t"<<srcH<<"\t"<<dstW<<"\t"<<dstH<<"\t"<<t1-t0<<std::endl;
    }

    void ORBextractor::ComputePyramid(cv::Mat image) {
        mvImagePyramid[0] = image;
        for (int level = 1; level < nlevels; ++level) {
            Mat &dst = mvImagePyramid[level];
            //myResize ( mvImagePyramid[level-1],mvTmp[level], dst );
            resize(mvImagePyramid[level - 1], dst, dst.size());
        }
    }

} //namespace ORB_SLAM

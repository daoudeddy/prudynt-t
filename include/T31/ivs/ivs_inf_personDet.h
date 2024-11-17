#ifndef __IVS_INF_PERSONDET_H__
#define __IVS_INF_PERSONDET_H__

#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif /* __cplusplus */

#include "ivs_common.h"

#define NUM_OF_PERSONS 20
#define IVS_PERSONDET_PERM_MAX_ROI  4
#define IVS_PERSONDET_PERM_MAX_RECT 4

/*
 * 人形信息结构体
 */
typedef struct {
    IVSRect box; /**reserved*/
    IVSRect show_box; /**< Humanoid area coordinates */
    float confidence; /**< Confidence of humanoid detection results */
}person_info;
/*
 * 
 */
typedef struct {
  IVSPoint *p;    /**< Information about each vertex of the perimeter cannot be crossed by lines. */
  int pcnt;      /**<  Number of perimeter vertices, up to 8 points,*/
  uint64_t alarm_last_time;	/**<  reserved*/
}persondet_perm_t;


/*
 * Humanoid detection input structure
 */
typedef struct {
    bool ptime; /**< Whether to print detection time */
    int skip_num; /**< Number of skipped frames */
    IVSFrameInfo frameInfo; /**< frame information */
    int sense; /**< Detection sensitivity 0~5 0: least sensitive 5: most sensitive default:4*/
    int detdist; /**< Detection distance 0~4  0:6m max(img_w, img_h) >= 320 ;  1:8m  max(img_w, img_h) >= 416 \
                    2:10m max(img_w, img_h) >= 512   3:11m max(img_w, img_h) >= 640 \
                    4:13m  max(img_w, img_h) >= 800 default:2 */
    bool enable_move; /**<true: Turn on motion detection to trigger humanoid detection false:Turn off motion detection to trigger humanoid detection default: false>*/
    bool enable_perm; /**<true: Enable perimeter function false: Turn off perimeter functionality default: false>*/
    persondet_perm_t perms[IVS_PERSONDET_PERM_MAX_ROI];  /**< perimeter information */
    int permcnt;                             /**<  Number of perimeters, up to 4*/


}persondet_param_input_t;

/*
 * Humanoid detection output structure
 */
typedef struct {
    int count; /**< Number of human figures identified */
    int count_move;/**< Number of moving targets identified */
    person_info person[NUM_OF_PERSONS]; /**< Recognized humanoid information */
    int64_t timeStamp; /**< Timestamp */
}persondet_param_output_t;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
#endif /* __IVS_INF_PERSONDET_H__ */

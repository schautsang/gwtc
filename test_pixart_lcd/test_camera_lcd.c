/* PMSIS includes. */
#include "pmsis.h"

/* PMSIS BSP includes. */
#include "bsp/bsp.h"
#include "bsp/fs.h"

/* Demo includes. */
#include "setup.h"

#include "ImgIO.h"


static pi_task_t task;
static uint8_t *imgBuff0;
static struct pi_device cam;

static void cam_handler(void *arg);

static void cam_handler(void *arg)
{
    pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);
}

#if defined(HIMAX)
static int32_t open_camera_himax(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;
    pi_himax_conf_init(&cam_conf);
    #if defined(QVGA)
    cam_conf.format = PI_CAMERA_QVGA;
    #endif  /* QVGA */
    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
    {
        return -1;
    }
    return 0;
}
#elif defined(MT9V034)
#define MT9V034_BLACK_LEVEL_CTRL (0x47)
#define MT9V034_BLACK_LEVEL_AUTO (0 << 0)
#define MT9V034_AEC_AGC_ENABLE   (0xAF)
#define MT9V034_AEC_ENABLE_A     (1 << 0)
#define MT9V034_AGC_ENABLE_A     (1 << 1)
#define MT9V034_AEC_ENABLE_B     (1 << 8)
#define MT9V034_AGC_ENABLE_B     (1 << 9)

static int32_t open_camera_mt9v034(struct pi_device *device)
{
    struct pi_mt9v034_conf cam_conf;
    pi_mt9v034_conf_init(&cam_conf);
    #if defined(QVGA)
    cam_conf.format = PI_CAMERA_QVGA;
    #else
    cam_conf.format = PI_CAMERA_QQVGA;
    #endif  /* QVGA */
    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
    {
        return -1;
    }
    uint16_t val = MT9V034_BLACK_LEVEL_AUTO;
    pi_camera_reg_set(device, MT9V034_BLACK_LEVEL_CTRL, (uint8_t *) &val);
    val = MT9V034_AEC_ENABLE_A|MT9V034_AGC_ENABLE_A;
    pi_camera_reg_set(device, MT9V034_AEC_AGC_ENABLE, (uint8_t *) &val);
    return 0;
}

#else  // pixart
static int32_t open_camera_pixart(struct pi_device *device)
{
    struct pi_pixart_conf cam_conf;
    pi_pixart_conf_init(&cam_conf);
    #if defined(QVGA)
    cam_conf.format = PI_CAMERA_QVGA;
    #endif  /* QVGA */
    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
    {
        return -1;
    }
    return 0;
}
#endif  /* HIMAX */

static int32_t open_camera(struct pi_device *device)
{
    #if defined(HIMAX)
    return open_camera_himax(device);
    #elif defined(MT9V034)
    return open_camera_mt9v034(device);
    #else
    return open_camera_pixart(device);
    #endif  /* HIMAX */
    return -1;
}

void test_camera_with_lcd(void)
{
    printf("Entering main controller...\n");

    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);

    imgBuff0 = (uint8_t *) pmsis_l2_malloc((320 * 240) * sizeof(uint8_t));
    if (imgBuff0 == NULL)
    {
        printf("Failed to allocate Memory for Image \n");
        pmsis_exit(-1);
    }

    if (open_camera(&cam))
    {
        printf("Failed to open camera\n");
        pmsis_exit(-2);
    }

    uint8_t i = 0;
    printf("Main loop start\n");
    while (1)
    {
        #if defined(ASYNC)
        printf("ASYNC...\n");
        printf("Camera stop.\n");
        pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);
        pi_task_callback(&task, cam_handler, NULL);
        printf("Image capture.\n");
        pi_camera_capture_async(&cam, imgBuff0, CAMERA_WIDTH * CAMERA_HEIGHT, &task);
        printf("Camera start.\n");
        pi_camera_control(&cam, PI_CAMERA_CMD_START, 0);
        pi_task_wait_on(&task);
        #else
        printf("SYNC...\n");
        printf("Camera start.\n");
        pi_camera_control(&cam, PI_CAMERA_CMD_START, 0);
        printf("Image captured.\n");
        pi_camera_capture(&cam, imgBuff0, CAMERA_WIDTH * CAMERA_HEIGHT);
        printf("Camera stop.\n");
        pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);
        #endif

        char filename[30];
        sprintf(filename, "../../../pgmfile%d.pgm", i);
        WriteImageToFile(filename, CAMERA_WIDTH, CAMERA_HEIGHT, imgBuff0, 1);

        i++;
    }

    pmsis_exit(0);
}

int main(void)
{
    printf("\n\t*** PMSIS Camera with LCD Test ***\n\n");
    return pmsis_kickoff((void *) test_camera_with_lcd);
}

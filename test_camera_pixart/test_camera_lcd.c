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
float L_Avg;
#define L_Target 127
float L_Ratio;
float Exposure_Present = 0x2710 + 3642.0;
float Exposure_Target;
float Gain_Present = 1;
float Gain_Target;
float GEProduct_Present;
float GEProduct_Target;
#define R_AE_Start_X 0
#define R_AE_Start_Y 0
#define R_AE_End_X 319
#define R_AE_End_Y 239
#define R_AE_BlockSize_X 320
#define R_AE_BlockSIze_Y 240
#define R_AE_LockRange_In 8
#define R_AE_LockRange_Out 16
uint8_t R_AE_Converged_Flag = 0;
#define R_AE_MaxExpo 114839
#define R_AE_MinExpo 120
#define R_AE_MaxGain 8
#define R_AE_MinGain 1

static void camera_pixart_AutoExpoAutoGain(struct pi_device *camera, uint8_t *ImgIn)
{
  L_Avg = 0;
  for (uint32_t i = R_AE_Start_X * R_AE_Start_Y; i < (R_AE_End_X + 1) * (R_AE_End_Y + 1); i++)
  {
    L_Avg += ImgIn[i];
  }
  L_Avg /= (R_AE_End_X + 1) * (R_AE_End_Y + 1);
printf("L_Avg = %f...\n", L_Avg);
  if (L_Target - R_AE_LockRange_In <= L_Avg && L_Avg <= L_Target + R_AE_LockRange_In)
  {
    R_AE_Converged_Flag = 1;
  }
  else
  {
    if (L_Target - R_AE_LockRange_Out <= L_Avg && L_Avg <= L_Target + R_AE_LockRange_Out && R_AE_Converged_Flag == 1)
    {
      R_AE_Converged_Flag = 1;
    }
    else
    {
      R_AE_Converged_Flag = 0;
    }
  }
  if (R_AE_Converged_Flag == 0)
  {
    L_Ratio = L_Target / L_Avg;
    //GEProduct_Present = Exposure_Present * Gain_Present;
    //GEProduct_Target = GEProduct_Present * L_Ratio;
    GEProduct_Target = Exposure_Present * Gain_Present * L_Target / L_Avg;

    Exposure_Target = GEProduct_Target / R_AE_MinGain;
    if (Exposure_Target > R_AE_MaxExpo)
      Exposure_Target = R_AE_MaxExpo;
    else if (Exposure_Target < R_AE_MinExpo)
      Exposure_Target = R_AE_MinExpo;
    else
      Exposure_Target = Exposure_Target;

    Gain_Target = GEProduct_Target / Exposure_Target;
    if (Gain_Target > R_AE_MaxGain)
      Gain_Target = R_AE_MaxGain;
    else if (Gain_Target < R_AE_MinGain)
      Gain_Target = R_AE_MinGain;
    else
      Gain_Target = Gain_Target;

printf("L_Ratio = %f, Exposure_Target = %f, Gain_Target = %f...\n", L_Ratio, Exposure_Target, Gain_Target);
    uint8_t addr, value;

    addr = 0x7F;
    value = 0x00;
    pi_camera_reg_set(camera, addr, &value);
    if (Exposure_Target >= 3642)
    {
      addr = 0x18;
      value = 0x00;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x19;
      value = 0x00;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x48;
      value = ((uint32_t)(Exposure_Target + 0.5 - 3642)) & 0x0FF;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x49;
      value = ((uint32_t)(Exposure_Target + 0.5 - 3642) >> 8) & 0x0FF;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x4A;
      value = ((uint32_t)(Exposure_Target + 0.5 - 3642) >> 16) & 0x0FF;
      pi_camera_reg_set(camera, addr, &value);
    }
    else
    {
      addr = 0x18;
      value = ((uint16_t)(3642 + 0.5 - Exposure_Target)) & 0x0FF;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x19;
      value = ((uint16_t)(3642 + 0.5 - Exposure_Target) >> 8) & 0x0FF;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x48;
      value = 0x00;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x49;
      value = 0x00;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x4A;
      value = 0x00;
      pi_camera_reg_set(camera, addr, &value);
    }
    addr = 0x7F;
    value = 0x01;
    pi_camera_reg_set(camera, addr, &value);
    addr = 0x00;
    value = 0x01;
    pi_camera_reg_set(camera, addr, &value);

    addr = 0x7F;
    value = 0x00;
    pi_camera_reg_set(camera, addr, &value);
    if (Gain_Target >= 1.0 && Gain_Target < 1.5)
    {
      addr = 0x11;
      value = 0x41;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x12;
      value = 0x3D;
      pi_camera_reg_set(camera, addr, &value);
    }
    else if (Gain_Target >= 1.5 && Gain_Target < 3.0)
    {
      addr = 0x11;
      value = 0x41;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x12;
      value = 0xBD;
      pi_camera_reg_set(camera, addr, &value);
    }
    else if (Gain_Target >= 3.0 && Gain_Target < 6)
    {
      addr = 0x11;
      value = 0x61;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x12;
      value = 0xBD;
      pi_camera_reg_set(camera, addr, &value);
    }
    else
    {
      addr = 0x11;
      value = 0x71;
      pi_camera_reg_set(camera, addr, &value);
      addr = 0x12;
      value = 0xBD;
      pi_camera_reg_set(camera, addr, &value);
    }
    addr = 0x7F;
    value = 0x01;
    pi_camera_reg_set(camera, addr, &value);
    addr = 0x00;
    value = 0x01;
    pi_camera_reg_set(camera, addr, &value);

    Exposure_Present = Exposure_Target;
    Gain_Present = Gain_Target;
  }
}

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

        camera_pixart_AutoExpoAutoGain(&cam, imgBuff0);

        if (R_AE_Converged_Flag == 1)
        {
          char filename[30];
          sprintf(filename, "../../../pgmfile%d.pgm", i);
          WriteImageToFile(filename, CAMERA_WIDTH, CAMERA_HEIGHT, imgBuff0, 1);
        }

        i++;
    }

    pmsis_exit(0);
}

int main(void)
{
    printf("\n\t*** PMSIS Camera with LCD Test ***\n\n");
    return pmsis_kickoff((void *) test_camera_with_lcd);
}

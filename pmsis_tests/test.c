#include <stdio.h>
#include <stdint.h>
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera.h"

#include "gaplib/ImgIO.h"

#define CAMERA_WIDTH              (324)
#define CAMERA_HEIGHT             (244)
#define BUFF_SIZE                 (CAMERA_WIDTH * CAMERA_HEIGHT)
#define SPIS_PAGE_SIZE            (256)
#define SPIM_RCV_UCODE_SIZE       (36)
#define SPIM_PPQ_UCODE_SIZE       (7*sizeof(uint32_t))

#define DUMMY_CYCLES              (32)

enum{
    // bit 0: enable/disable qspi
    WR_REG0 = 0x1,
    RD_REG0 = 0x5,
    // number of dummy cycles
    WR_REG1 = 0x11,
    RD_REG1 = 0x7,
    // wrap length, low
    WR_REG2 = 0x20,
    RD_REG2 = 0x21,
    // wrap length, high
    WR_REG3 = 0x30,
    RD_REG3 = 0x31,

    WR_MEM  = 0x2,
    RD_MEM  = 0xB
};

typedef struct {
    uint32_t addr;
    uint32_t value;
}spis_reg_t;

PI_L2 spis_reg_t spis_cmd;
PI_L2 spis_reg_t spis_read;

PI_L2 uint8_t *spim_tx_buffer;
PI_L2 uint8_t *spim_rx_buffer;
PI_L2 uint8_t *spis_buffer;

static struct pi_device camera;

static char imgName[50];

static void pad_init()
{
    // Loopback connection: A4 SPIM1_MISO -> A16 SPIS_MOSI
    // Loopback connection: B3 SPIM1_MOSI -> B15 SPIS_MISO
    // Loopback connection: A5 SPIM1_CSN  -> A9 SPIS_CSN
    // Loopback connection: B4 SPIM1_SCK  -> B9 SPIS_SCK

    /* Init pad for SPIM1 */
    pi_pad_set_function(PI_PAD_8_A4_RF_SPIM1_MISO, PI_PAD_8_A4_RF_SPIM1_MISO_FUNC0 );
    pi_pad_set_function(PI_PAD_9_B3_RF_SPIM1_MOSI, PI_PAD_9_B3_RF_SPIM1_MOSI_FUNC0 );
    pi_pad_set_function(PI_PAD_10_A5_RF_SPIM1_CSN, PI_PAD_10_A5_RF_SPIM1_CSN_FUNC0 );
    pi_pad_set_function(PI_PAD_11_B4_RF_SPIM1_SCK, PI_PAD_11_B4_RF_SPIM1_SCK_FUNC0 );

    /* Init pad for SPIS */
    pi_pad_set_function(PI_PAD_47_A9_SPIS0_CSN,   PI_PAD_47_A9_SPIS0_CSN_FUNC0   );
    pi_pad_set_function(PI_PAD_48_B15_SPIS0_MISO, PI_PAD_48_B15_SPIS0_MISO_FUNC0 );
    pi_pad_set_function(PI_PAD_49_A16_SPIS0_MOSI, PI_PAD_49_A16_SPIS0_MOSI_FUNC0 );
    pi_pad_set_function(PI_PAD_50_B9_SPIS0_SCK,   PI_PAD_50_B9_SPIS0_SCK_FUNC0   );
}

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

static uint32_t spis_reg_get(struct pi_device *spi, uint32_t reg_cmd)
{
    spis_read.addr = reg_cmd;
    pi_spi_send(spi, (uint8_t *)&spis_read.addr, 8, PI_SPI_CS_KEEP);
    pi_spi_receive(spi, (uint8_t *)&spis_read.value, 32, PI_SPI_CS_AUTO);
    return spis_read.value;
}


static void spis_reg_set(struct pi_device *spi, uint32_t reg_cmd, uint32_t value)
{
    spis_cmd.addr = reg_cmd;
    spis_cmd.value = value;
    pi_spi_send(spi, (uint8_t *)&spis_cmd.addr, 8, PI_SPI_CS_KEEP);
    pi_spi_send(spi, (uint8_t *)&spis_cmd.value, 8, PI_SPI_CS_AUTO);
}

static void spis_wr_mem(struct pi_device *spi, uint32_t wr_cmd, uint32_t addr)
{
    spis_cmd.addr = wr_cmd;
    spis_cmd.value = addr;
    pi_spi_send(spi, (uint8_t *)&spis_cmd.addr, 8, PI_SPI_CS_KEEP);
    pi_spi_send(spi, (uint8_t *)&spis_cmd.value, 32, PI_SPI_CS_KEEP);
}

static void spis_rd_mem(struct pi_device *spi, uint32_t wr_cmd, uint32_t addr)
{
    spis_cmd.addr = wr_cmd;
    spis_cmd.value = addr;
    // set dummy cycles in spi slave.
    spis_reg_set(spi, WR_REG1, (DUMMY_CYCLES - 1));

    pi_spi_send(spi, (uint8_t *)&spis_cmd.addr, 8, PI_SPI_CS_KEEP);
    pi_spi_send(spi, (uint8_t *)&spis_cmd.value, 32, PI_SPI_CS_KEEP);
}

static uint32_t addr_invert(uint32_t addr)
{
    uint32_t out_addr = 0;
    out_addr |= (addr&0xFF)<<24;
    out_addr |= (addr&0xFF00)<<8;
    out_addr |= (addr&0xFF0000)>>8;
    out_addr |= (addr&0xFF000000)>>24;

    return out_addr;
}

static int spim2spis_wr_mem(struct pi_device *spi, uint32_t flash_addr,
      const void *data, uint32_t size)
{
    uint8_t *l2_buff = pi_l2_malloc(SPIS_PAGE_SIZE);
    if(!l2_buff)
    {
        printf("Malloc failed!\n");
        return -1;
    }
    uint8_t *ucode = pi_l2_malloc(SPIM_PPQ_UCODE_SIZE + 2*4);
    if(!ucode)
    {
        printf("Malloc failed!\n");
        return -1;
    }
    uint32_t size_left = size;
    uint32_t curr_size = 0;
    uint32_t curr_pos = 0;

    volatile uint32_t *ucode_u32 = (uint32_t*)ucode;

    if((flash_addr & 0xFF) && (((flash_addr & 0xFF)+size_left) > 0x100))
    {
        curr_pos  = 0;
        curr_size = 0x100 - (flash_addr & 0xFF);
        size_left -= curr_size;
        // any write/erase op must be preceeded by a WRITE ENABLE op,
        // with full CS cycling
        //pi_spi_send(qspi_dev, (void*)g_write_enable, 8,
        //        SPI_LINES_FLAG | PI_SPI_CS_AUTO);
        memcpy(l2_buff, data+curr_pos, curr_size);
        ucode_u32[0] = pi_spi_get_config(spi);
        ucode_u32[1] = SPI_CMD_SOT(0);
        ucode_u32[2] = SPI_CMD_TX_DATA(8*1, 0, 0);
        ucode[12] = WR_MEM;//0x2;
        ucode_u32[4] = SPI_CMD_TX_DATA(8*4, 0, 0);
        ucode[20] = ((flash_addr+curr_pos) & 0xFF000000)>>24;
        ucode[21] = ((flash_addr+curr_pos) & 0x00FF0000)>>16;
        ucode[22] = ((flash_addr+curr_pos) & 0x0000FF00)>>8;
        ucode[23] = ((flash_addr+curr_pos) & 0x000000FF)>>0;
        ucode_u32[6] = SPI_CMD_TX_DATA(curr_size*8, 0, 0);
        pi_spi_send_with_ucode(spi, (void*)l2_buff, (curr_size)*8,
                PI_SPI_LINES_SINGLE | PI_SPI_CS_AUTO, SPIM_PPQ_UCODE_SIZE, ucode);
        //wait_wip(flash_dev);
        curr_pos += curr_size;
    }

    while(size_left)
    {
        if(size_left >= SPIS_PAGE_SIZE)
        {
            curr_size = SPIS_PAGE_SIZE;
            size_left -= SPIS_PAGE_SIZE;
        }
        else
        {
            curr_size = size_left;
            size_left = 0;
        }
        memcpy(l2_buff, data+curr_pos, curr_size);
        // any write/erase op must be preceeded by a WRITE ENABLE op,
        // with full CS cycling
        //pi_spi_send(qspi_dev, (void*)g_write_enable, 8,
        //        SPI_LINES_FLAG | PI_SPI_CS_AUTO);
        ucode_u32[0] = pi_spi_get_config(spi);
        ucode_u32[1] = SPI_CMD_SOT(0);
        ucode_u32[2] = SPI_CMD_TX_DATA(8*1, 0, 0);
        ucode[12] = WR_MEM;
        ucode_u32[4] = SPI_CMD_TX_DATA(8*4, 0, 0);
        ucode[20] = ((flash_addr+curr_pos) & 0xFF000000)>>24;
        ucode[21] = ((flash_addr+curr_pos) & 0x00FF0000)>>16;
        ucode[22] = ((flash_addr+curr_pos) & 0x0000FF00)>>8;
        ucode[23] = ((flash_addr+curr_pos) & 0x000000FF)>>0;
        ucode_u32[6] = SPI_CMD_TX_DATA(curr_size*8, 0, 0);
        pi_spi_send_with_ucode(spi, (void*)l2_buff, (curr_size)*8,
                PI_SPI_LINES_SINGLE | PI_SPI_CS_AUTO, SPIM_PPQ_UCODE_SIZE, ucode);
        //wait_wip(flash_dev);
        curr_pos += curr_size;
    }
    pi_l2_free(l2_buff, SPIS_PAGE_SIZE);
    pi_l2_free(ucode, SPIM_PPQ_UCODE_SIZE);

    return 0;
}

static int spim2spis_rd_mem(struct pi_device *spi, uint32_t addr, void *data,
        uint32_t size)
{
    // set dummy cycles in spi slave.
    spis_reg_set(spi, WR_REG1, (DUMMY_CYCLES - 1));

    uint32_t flash_addr = addr;
    uint32_t sector_size = SPIS_PAGE_SIZE;

    uint8_t *l2_buff = pi_l2_malloc(SPIS_PAGE_SIZE);
    if(!l2_buff)
    {
        printf("MALLOC FAILED\n");
        return -1;
    }
    uint8_t *ucode = pi_l2_malloc(SPIM_RCV_UCODE_SIZE);
    if(!ucode)
    {
        printf("Malloc failed!\n");
        return -1;
    }

    uint32_t size_left = size;
    uint32_t curr_size = 0;
    uint32_t curr_pos = 0;

    volatile uint32_t *ucode_u32 = (uint32_t*)ucode;

    while(size_left)
    {
        if(size_left >= SPIS_PAGE_SIZE)
        {
            curr_size = SPIS_PAGE_SIZE;
            size_left -= SPIS_PAGE_SIZE;
        }
        else
        {
            curr_size = size_left;
            size_left = 0;
        }
        uint32_t curr_addr = flash_addr+curr_pos;
        ucode_u32[0] = pi_spi_get_config(spi);
        ucode_u32[1] = SPI_CMD_SOT(0);
        ucode_u32[2] = SPI_CMD_TX_DATA(8*1, 0, 0);
        ucode[12] = RD_MEM; //0xB
        ucode_u32[4] = SPI_CMD_TX_DATA(8*4, 0, 0);
        ucode[20] = (curr_addr >> 24 )& 0xFFUL;
        ucode[21] = (curr_addr >> 16 )& 0xFFUL;
        ucode[22] = (curr_addr >> 8) & 0xFFUL;
        ucode[23] = curr_addr & 0xFFUL;
        ucode_u32[6] = SPI_CMD_DUMMY(DUMMY_CYCLES);
        ucode_u32[7] = SPI_CMD_RX_DATA(curr_size*8, 0, 0);// use 4 lines to recv
        ucode_u32[8] = SPI_CMD_EOT(1);
        // any write/erase op must be preceeded by a WRITE ENABLE op,
        // with full CS cycling
        pi_spi_receive_with_ucode(spi, (void*)l2_buff, (curr_size)*8,
                PI_SPI_LINES_SINGLE | PI_SPI_CS_AUTO, SPIM_RCV_UCODE_SIZE,
                ucode);

        memcpy(data+curr_pos, l2_buff, curr_size);
        curr_pos += curr_size;
    }
    pi_l2_free(l2_buff, SPIS_PAGE_SIZE);
    pi_l2_free(ucode, SPIM_RCV_UCODE_SIZE);

    return 0;
}

static int test_entry()
{
    if (pi_pmu_set_voltage(1200, 1))
    {
      printf("Error changing voltage !\nTest failed...\n");
      pmsis_exit(-1);
    }
    if (pi_fll_set_frequency(FLL_SOC, 250000000, 1) == -1)
    {
      printf("Error changing frequency !\nTest failed...\n");
      pmsis_exit(-1);
    }

#ifdef HIMAX
    if (open_camera_himax(&camera))
    {
        printf("Failed to open camera...\n");
        pmsis_exit(-1);
    }
#endif

    pad_init();

    struct pi_spi_conf conf;
    struct pi_device spim;

    pi_spi_conf_init(&conf);

    conf.wordsize = PI_SPI_WORDSIZE_32;
    conf.big_endian = 0;
    conf.max_baudrate = 40000000;
    conf.polarity = 0;
    conf.phase = 0;
    conf.itf = 1; //SPIM1
    conf.cs = 0;  //cs0
    //conf.dummy_cycle=31;

    pi_open_from_conf(&spim, &conf);

    if (pi_spi_open(&spim))
        return -1;

    spim_tx_buffer = pmsis_l2_malloc(BUFF_SIZE);
    if (spim_tx_buffer == NULL) return -1;

    spim_rx_buffer = pmsis_l2_malloc(BUFF_SIZE);
    if (spim_rx_buffer == NULL) return -1;

    spis_buffer = pmsis_l2_malloc(BUFF_SIZE);
    if (spis_buffer == NULL) return -1;

    for (int i = 0; i < BUFF_SIZE; i++)
    {
        spim_tx_buffer[i] = i & 0x0FF;
        spim_rx_buffer[i] = 0x00;
        spis_buffer[i] = 0xFF;
    }

    // Set to NoQSPI mode
    spis_reg_set(&spim, WR_REG0, 0x00);
    printf("%x\n", spis_reg_get(&spim, RD_REG0));

    // the addressing in GAP8 is big endien, so need to invert the address
    //uint32_t addr_test = addr_invert((uint32_t) &spis_buffer[0]);

    // send write mem command
    //spis_wr_mem(&spim, WR_MEM, addr_test);

    // send data
    //pi_spi_send(&spim, spim_tx_buffer, SPIS_PAGE_SIZE, PI_SPI_CS_AUTO);

#ifndef HIMAX
    spim2spis_wr_mem(&spim, (uint32_t)&spis_buffer[0], spim_tx_buffer, BUFF_SIZE);
    spim2spis_rd_mem(&spim, (uint32_t)&spis_buffer[0], spim_rx_buffer, BUFF_SIZE);
#endif

    // send read mem command
    //spis_rd_mem(&spim, RD_MEM, addr_test);

    // read data from spi slave
    // TODO: the read need to set dummy cycles, however, the actual api in master doesn't support this.
    //pi_spi_receive(&spim, spim_rx_buffer, SPIS_PAGE_SIZE, PI_SPI_CS_AUTO);

#ifdef HIMAX
    int idx = 0;
    while (1)
    {
        PRINTF("Camera start.\n");
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_camera_capture(&camera, spis_buffer, CAMERA_WIDTH * CAMERA_HEIGHT);
        PRINTF("Image captured.\n");
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        PRINTF("Camera stop.\n");

        // original image
        sprintf(imgName, "../../../img_OUT_%d.ppm", idx);
        printf("Dumping image %s\n", imgName);
        WriteImageToFile(imgName, CAMERA_WIDTH, CAMERA_HEIGHT, sizeof(uint8_t), spis_buffer, GRAY_SCALE_IO);
        idx++;

        // big endian to small endian
        for (int i = 0; i < BUFF_SIZE; i=i+4)
        {
          *((uint32_t *)&spis_buffer[i]) = addr_invert(*((uint32_t *)&spis_buffer[i]));
        }

        printf("SPIM read out data from SPIS...\n");
        spim2spis_rd_mem(&spim, (uint32_t)&spis_buffer[0], spim_rx_buffer, BUFF_SIZE);

        // image read from spi slave
        sprintf(imgName, "../../../img_OUT_%d.ppm", idx);
        printf("Dumping image %s\n", imgName);
        WriteImageToFile(imgName, CAMERA_WIDTH, CAMERA_HEIGHT, sizeof(uint8_t), spim_rx_buffer, GRAY_SCALE_IO);

        idx++;
    }
#endif

    int error = 0;

    for (int i = 0; i < BUFF_SIZE; i++)
    {
        printf("id@%05d: rx@%p: %02X tx@%p: %02X re@%p: %02X \n", i, &spim_rx_buffer[i], spim_rx_buffer[i], &spim_tx_buffer[i], spim_tx_buffer[i], &spis_buffer[i], spis_buffer[i]);

        if (spim_rx_buffer[i] != spim_tx_buffer[i])
        {
            if (error == 0)
                printf("First error at index %d, expected 0x%x, got 0x%x at %p\n", i, spim_tx_buffer[i], spim_rx_buffer[i], &spim_rx_buffer[i]);
            error++;
            //return -1;
        }
    }

    if (error)
    {
        printf("Got %d errors\n", error);
    }
    else
    {
        printf("Test success\n");
    }
    pi_spi_close(&spim);
    return error;
}

static void test_kickoff(void *arg)
{
    int ret = test_entry();
    pmsis_exit(ret);
}

int main()
{
    return pmsis_kickoff((void *)test_kickoff);
}

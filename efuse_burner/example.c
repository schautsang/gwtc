// This example shows how to program efuses.
//
// This is a permanent operation. Once and efuse is programmed, the same value is
// read, even after power-up. This can be used to remember some settings.
// Be careful that there are some efuses reserved for the ROM to specify the
// boot mode, check the specifications to know which efuses can be used.

#include <pmsis.h>


static int entry()
{
    printf("Entering main controller\n");

#ifdef EFUSE_WRITE
    printf("Writing efuse 50 with value 0x12\n");

    // Before writing the efuse, we must activate the program operation
    // Once activated, we can wrote as many efuses as we want
    plp_efuse_startProgram();

    plp_efuse_writeByte(80, 0x12);

    // Close the current operation once done
    plp_efuse_sleep();
#else
    printf("Efuse has not been written, recompile with make clean all run EFUSE_WRITE=1, be careful that this is a permanent operation !!!\n");
#endif


#ifdef EFUSE_READ_FULL

    // write test
    plp_efuse_startProgram();
    plp_efuse_writeByte(1, 0x42);
    plp_efuse_sleep();

    // read test
    plp_efuse_startRead();
    for (uint8_t i = 0; i < 128; i++)
    {
      printf("REG[%03d] = 0x%x...\n", i, plp_efuse_readWord(i));
    }
    plp_efuse_sleep();

#else
    // Before reading the efuse, we must activate the read operation
    // Once activated, we can wrote as many efuses as we want
    plp_efuse_startRead();

    int value = plp_efuse_readWord(80);

    // Close the current operation once done
    plp_efuse_sleep();

    printf("Read efuse 50: 0x%x\n", value);
#endif

    return 0;
}


static void pmsis_wrapper(void)
{
    int retval = entry();
    pmsis_exit(retval);
}


int main(void)
{
    return pmsis_kickoff((void *)pmsis_wrapper);
}


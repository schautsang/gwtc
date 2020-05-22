APP              = test
APP_SRCS        += example.c
APP_INC	        +=

ifdef EFUSE_WRITE
APP_CFLAGS      += -DEFUSE_WRITE=1
endif

ifdef EFUSE_READ
APP_CFLAGS      += -DEFUSE_READ_FULL=1
endif

include $(RULES_DIR)/pmsis_rules.mk

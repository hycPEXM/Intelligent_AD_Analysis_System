include ./mk.param

TARGET := ShowADDemoNew

SMP_SRCS += $(NNIE_SAMPLE_DIR)/sample/ShowAD_File/src/ad_show.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_isp.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_sys.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_venc.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_vi.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_vo.c
SMP_SRCS += $(PLATRORM_COMMON_DIR)/sample_comm_vpss.c
SMP_SRCS += $(SVP_COMMON_DIR)/sample_comm_ive_old.c
SMP_SRCS += $(SVP_COMMON_DIR)/sample_comm_svp.c

CPP_SRCS += $(NNIE_SAMPLE_DIR)/sample/ShowAD_File/src/display.cpp
CPP_SRCS += $(NNIE_SAMPLE_DIR)/sample/ShowAD_File/src/UDPSocket.cpp
CPP_SRCS += $(NNIE_SAMPLE_DIR)/sample/ShowAD_File/src/main.cpp
# SMP_SRCS += $(AI_SAMPLE_DIR)/mpp_help/src/ive_img.c

CFLAGS += -I$(NNIE_SAMPLE_DIR)/sample/ShowAD_File/src/
CFLAGS += -I$(COLOR_SAMPLE_DIR)/smp
CFLAGS += -I$(AI_SAMPLE_DIR)/mpp_help/include
CFLAGS += -I$(AI_SAMPLE_DIR)/ai_infer_process
# 根据实际的类型设置，可以用set_sensor脚本设置
SENSOR0_TYPE = -DSENSOR0_TYPE=SONY_IMX335_MIPI_4M_30FPS_12BIT
SENSOR1_TYPE = -DSENSOR1_TYPE=SONY_IMX335_MIPI_4M_30FPS_12BIT
CFLAGS += $(SENSOR0_TYPE)
CFLAGS += $(SENSOR1_TYPE)
# 编译工程所需的mk文件
include ./sdk_linux.mk
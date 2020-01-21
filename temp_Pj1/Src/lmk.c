#include "main.h"

//---------------------------------------------------------------------------------

reg_lmk04828 lmk;

void INIT_REG_LMK (void)
{

lmk.RESET=0;
lmk.SPI_3WIRE_DIS=1;
lmk.POWERDOWN=0;
lmk.ID_DEVICE_TYPE=0;//read only
lmk.ID_PROD=0;//read only
lmk.ID_MASKREV=0;//read only
lmk.ID_VNDR=0;//read only

lmk.CLKout0_1_ODL=0;
lmk.CLKout2_3_ODL=0;
lmk.CLKout4_5_ODL=0;
lmk.CLKout6_7_ODL=0;

lmk.CLKout0_1_IDL=0;
lmk.CLKout2_3_IDL=0;
lmk.CLKout4_5_IDL=0;
lmk.CLKout6_7_IDL=0;

lmk.DCLKout0_DIV=10;//не больше 32!!!!
lmk.DCLKout2_DIV=25;//25 - 96 МГц
lmk.DCLKout4_DIV=25;
lmk.DCLKout6_DIV=10;//240 МГц АЦП2
lmk.DCLKout8_DIV=10;//240 МГц АЦП1
lmk.DCLKout10_DIV=25;
lmk.DCLKout12_DIV=25;

lmk.DCLKout0_DDLY_CNTH =5;
lmk.DCLKout2_DDLY_CNTH =5;
lmk.DCLKout4_DDLY_CNTH =5;
lmk.DCLKout6_DDLY_CNTH =5;
lmk.DCLKout8_DDLY_CNTH =5;
lmk.DCLKout10_DDLY_CNTH=5;
lmk.DCLKout12_DDLY_CNTH=5;

lmk.DCLKout0_DDLY_CNTL=5;
lmk.DCLKout2_DDLY_CNTL=5;
lmk.DCLKout4_DDLY_CNTL=5;
lmk.DCLKout6_DDLY_CNTL=5;
lmk.DCLKout8_DDLY_CNTL=5;
lmk.DCLKout10_DDLY_CNTL=5;
lmk.DCLKout12_DDLY_CNTL=5;

lmk.DCLKout0_ADLY=0;
lmk.DCLKout2_ADLY=0;
lmk.DCLKout4_ADLY=0;
lmk.DCLKout6_ADLY=0;
lmk.DCLKout8_ADLY=0;
lmk.DCLKout10_ADLY=0;
lmk.DCLKout12_ADLY=0;

lmk.DCLKout0_ADLY_PD=1;
lmk.DCLKout2_ADLY_PD=1;
lmk.DCLKout4_ADLY_PD=1;
lmk.DCLKout6_ADLY_PD=1;
lmk.DCLKout8_ADLY_PD=1;
lmk.DCLKout10_ADLY_PD=1;
lmk.DCLKout12_ADLY_PD=1;

lmk.DCLKout0_ADLYg_PD=1;
lmk.DCLKout2_ADLYg_PD=1;
lmk.DCLKout4_ADLYg_PD=1;
lmk.DCLKout6_ADLYg_PD=1;
lmk.DCLKout8_ADLYg_PD=1;
lmk.DCLKout10_ADLYg_PD=1;
lmk.DCLKout12_ADLYg_PD=1;

lmk.DCLKout0_ADLY_MUX=0;
lmk.DCLKout2_ADLY_MUX=0;
lmk.DCLKout4_ADLY_MUX=0;
lmk.DCLKout6_ADLY_MUX=0;
lmk.DCLKout8_ADLY_MUX=0;
lmk.DCLKout10_ADLY_MUX=0;
lmk.DCLKout12_ADLY_MUX=0;

lmk.DCLKout0_MUX=0;
lmk.DCLKout2_MUX=0;
lmk.DCLKout4_MUX=0;
lmk.DCLKout6_MUX=0;
lmk.DCLKout8_MUX=0;
lmk.DCLKout10_MUX=0;
lmk.DCLKout12_MUX=0;


lmk.DCLKout0_HS=0;
lmk.DCLKout2_HS=0;
lmk.DCLKout4_HS=0;
lmk.DCLKout6_HS=0;
lmk.DCLKout8_HS=0;
lmk.DCLKout10_HS=0;
lmk.DCLKout12_HS=0;

lmk.SDCLKout1_MUX=1;
lmk.SDCLKout3_MUX=1;
lmk.SDCLKout5_MUX=1;
lmk.SDCLKout7_MUX=1;
lmk.SDCLKout9_MUX=1;
lmk.SDCLKout11_MUX=1;
lmk.SDCLKout13_MUX=1;

lmk.SDCLKout1_DDLY=0;
lmk.SDCLKout3_DDLY=0;
lmk.SDCLKout5_DDLY=0;
lmk.SDCLKout7_DDLY=0;
lmk.SDCLKout9_DDLY=0;
lmk.SDCLKout11_DDLY=0;
lmk.SDCLKout13_DDLY=0;

lmk.SDCLKout1_HS=0;
lmk.SDCLKout3_HS=0;
lmk.SDCLKout5_HS=0;
lmk.SDCLKout7_HS=0;
lmk.SDCLKout9_HS=0;
lmk.SDCLKout11_HS=0;
lmk.SDCLKout13_HS=0;

lmk.SDCLKout1_ADLY_EN=0;
lmk.SDCLKout3_ADLY_EN=0;
lmk.SDCLKout5_ADLY_EN=0;
lmk.SDCLKout7_ADLY_EN=0;
lmk.SDCLKout9_ADLY_EN=0;
lmk.SDCLKout11_ADLY_EN=0;
lmk.SDCLKout13_ADLY_EN=0;

lmk.SDCLKout1_ADLY=0;
lmk.SDCLKout3_ADLY=0;
lmk.SDCLKout5_ADLY=0;
lmk.SDCLKout7_ADLY=0;
lmk.SDCLKout9_ADLY=0;
lmk.SDCLKout11_ADLY=0;
lmk.SDCLKout13_ADLY=0;

lmk.DCLKout0_DDLY_PD=0;
lmk.DCLKout2_DDLY_PD=0;
lmk.DCLKout4_DDLY_PD=0;
lmk.DCLKout6_DDLY_PD=0;
lmk.DCLKout8_DDLY_PD=0;
lmk.DCLKout10_DDLY_PD=0;
lmk.DCLKout12_DDLY_PD=0;

lmk.DCLKout0_HSg_PD=0;
lmk.DCLKout2_HSg_PD=0;
lmk.DCLKout4_HSg_PD=0;
lmk.DCLKout6_HSg_PD=0;
lmk.DCLKout8_HSg_PD=0;
lmk.DCLKout10_HSg_PD=0;
lmk.DCLKout12_HSg_PD=0;

lmk.DCLKout_ADLYg_PD=1;
lmk.DCLKout_ADLY_PD=1;


lmk.CLKout0_1_PD  =0;///повердаун из-за ошибки в схеме!!! сигнал заведён напрямую на ПЛИС!!
lmk.CLKout2_3_PD  =0;//ЦАП DD28
lmk.CLKout4_5_PD  =0;//ЦАП DD29
lmk.CLKout6_7_PD  =0;//АЦП2
lmk.CLKout8_9_PD  =0;//АЦП1
lmk.CLKout10_11_PD=0;//clk 96MHz for TX jesd
lmk.CLKout12_13_PD=1;//повердаун из-за ошибки в схеме!!! сигнал заведён напрямую на ПЛИС!!

lmk.SDCLKout1_DIS_MODE=0;
lmk.SDCLKout3_DIS_MODE=0;
lmk.SDCLKout5_DIS_MODE=0;
lmk.SDCLKout7_DIS_MODE=0;
lmk.SDCLKout9_DIS_MODE=0;
lmk.SDCLKout11_DIS_MODE=0;
lmk.SDCLKout13_DIS_MODE=0;

lmk.SDCLKout1_PD=0;
lmk.SDCLKout3_PD=0;
lmk.SDCLKout5_PD=0;
lmk.SDCLKout7_PD=0;
lmk.SDCLKout9_PD=0;
lmk.SDCLKout11_PD=1;
lmk.SDCLKout13_PD=1;

lmk.SDCLKout1_POL=0;
lmk.SDCLKout3_POL=1;
lmk.SDCLKout5_POL=0;
lmk.SDCLKout7_POL=1;
lmk.SDCLKout9_POL=1;
lmk.SDCLKout11_POL=0;
lmk.SDCLKout13_POL=0;

lmk.SDCLKout1_FMT=1;//LVDS
lmk.SDCLKout3_FMT=5;//LVPECL 1600 mV
lmk.SDCLKout5_FMT=5;
lmk.SDCLKout7_FMT=5;
lmk.SDCLKout9_FMT=5;
lmk.SDCLKout11_FMT=0;
lmk.SDCLKout13_FMT=0;

lmk.DCLKout0_POL=0;
lmk.DCLKout2_POL=0;
lmk.DCLKout4_POL=0;
lmk.DCLKout6_POL=0;
lmk.DCLKout8_POL=0;
lmk.DCLKout10_POL=0;
lmk.DCLKout12_POL=0;

lmk.CLKout0_FMT=0;
lmk.CLKout2_FMT=5;//1600 mV
lmk.CLKout4_FMT=5;
lmk.CLKout6_FMT=5;
lmk.CLKout8_FMT=5;
lmk.CLKout10_FMT=5;
lmk.CLKout12_FMT=0;

lmk.VCO_MUX=0;// 0 - VCO0, 1 - VCO1

lmk.OSCout_MUX=0;
lmk.OSCout_FMT=0;

lmk.SYSREF_CLKin0_MUX=0;
lmk.SYSREF_MUX=3;//SYSREF Continuous , SYSREF Pulser
lmk.SYSREF_DIV=1600;//частота sysref 2400 МГц/1600 = 1.5 Мгц (96/1.5=64 кратно 32 что ОК)
lmk.SYSREF_DDLY=8;
lmk.SYSREF_PULSE_CNT=2;

lmk.PLL2_NCLK_MUX=0;
lmk.PLL1_NCLK_MUX=0;
lmk.FB_MUX=0;
lmk.FB_MUX_EN=0;

lmk.PLL1_PD=0;
lmk.VCO_LDO_PD=0;
lmk.VCO_PD=0;
lmk.OSCin_PD=0;

lmk.SYSREF_GBL_PD=1;
lmk.SYSREF_PD=0;
lmk.SYSREF_DDLY_PD=0;
lmk.SYSREF_PLSR_PD=1;

lmk.DDLYd_SYSREF_EN=0;
lmk.DDLYd12_EN=0;
lmk.DDLYd10_EN=0;
lmk.DDLYd8_EN=0;
lmk.DDLYd6_EN=0;
lmk.DDLYd4_EN=0;
lmk.DDLYd2_EN=0;
lmk.DDLYd0_EN=0;
lmk.DDLYd_STEP_CNT=0;

lmk.SYSREF_CLR=0;
lmk.SYNC_1SHOT_EN=1;
lmk.SYNC_POL=0;
lmk.SYNC_EN=1;

lmk.SYNC_PLL2_DLD=0;
lmk.SYNC_PLL1_DLD=0;

lmk.SYNC_MODE=1;

lmk.SYNC_DISSYSREF=1;
lmk.SYNC_DIS12=1;
lmk.SYNC_DIS10=1;
lmk.SYNC_DIS8=1;
lmk.SYNC_DIS6=1;
lmk.SYNC_DIS4=1;
lmk.SYNC_DIS2=1;
lmk.SYNC_DIS0=1;

lmk.Fixed_Register=127;
lmk.CLKin2_EN=0;
lmk.CLKin1_EN=0;
lmk.CLKin0_EN=0;
lmk.CLKin2_TYPE=1;
lmk.CLKin1_TYPE=1;
lmk.CLKin0_TYPE=1;
lmk.CLKin_SEL_POL=0;
lmk.CLKin_SEL_MODE=4;
lmk.CLKin1_OUT_MUX=2;
lmk.CLKin0_OUT_MUX=2;
lmk.CLKin_SEL0_MUX=0;
lmk.CLKin_SEL0_TYPE=2;
lmk.SDIO_RDBK_TYPE=1;
lmk.CLKin_SEL1_MUX=0;
lmk.CLKin_SEL1_TYPE=2;
lmk.RESET_MUX=6;//SPI Readback
lmk.RESET_TYPE=3;
lmk.LOS_TIMEOUT=0;
lmk.LOS_EN=0;
lmk.TRACK_EN=0;
lmk.HOLDOVER_FORCE=0;
lmk.MAN_DAC_EN=1;
lmk.MAN_DAC=512;
lmk.DAC_TRIP_LOW=0;
lmk.DAC_CLK_MULT=0;
lmk.DAC_TRIP_HIGH=0;
lmk.DAC_CLK_CNTR=127;
lmk.HOLDOVER_PLL1_DET=0;
lmk.HOLDOVER_LOS_DET=0;
lmk.HOLDOVER_VTUNE_DET=0;
lmk.HOLDOVER_HITLESS_SWITCH=1;
lmk.HOLDOVER_EN=0;
lmk.HOLDOVER_DLD_CNT=2000;
lmk.CLKin0_R=120;
lmk.CLKin1_R=121;
lmk.CLKin2_R=122;
lmk.PLL1_N=130;
lmk.PLL1_WND_SIZE=3;
lmk.PLL1_CP_TRI=0;
lmk.PLL1_CP_POL=1;
lmk.PLL1_CP_GAIN=4;
lmk.PLL1_DLD_CNT=3200;
lmk.PLL1_R_DLY=0;
lmk.PLL1_N_DLY=0;
lmk.PLL1_LD_MUX=2;//Status_LD1 pin. PLL2_N/2
lmk.PLL1_LD_TYPE=3;//Output (push-pull)
lmk.PLL2_R=3;
lmk.PLL2_P=2;
lmk.OSCin_FREQ=4;//360 МГц РЕФ
lmk.PLL2_XTAL_EN=0;
lmk.PLL2_REF_2X_EN=0;
lmk.PLL2_N_CAL=10;
lmk.PLL2_N=10;
lmk.PLL2_FCAL_DIS=0;
lmk.PLL2_WND_SIZE=2;

lmk.PLL2_CP_GAIN=0;//3

lmk.PLL2_CP_POL=0;
lmk.PLL2_CP_TRI=0;
lmk.Fixed_Value_0x169=1;
lmk.PLL2_DLD_CNT=1000;
lmk.SYSREF_REQ_EN=0;
lmk.PLL2_LF_R4=0;
lmk.PLL2_LF_R3=0;
lmk.PLL2_LF_C4=0;
lmk.PLL2_LF_C3=0;
lmk.PLL2_LD_MUX=2;// 2 Status_LD2 pin  PLL2 DLD (Digital lock Detect)
lmk.PLL2_LD_TYPE=3;//Output inverted (push-pull)
lmk.PLL2_PRE_PD=0;
lmk.PLL2_PD=0;
lmk.OPT_REG_1=21; 
lmk.OPT_REG_2=51;
lmk.RB_PLL1_LD_LOST=0;//не писать Read-Back
lmk.RB_PLL1_LD=0;//не писать Read-Back
lmk.CLR_PLL1_LD_LOST=0;
lmk.RB_PLL2_LD_LOST=0;//не писать Read-Back
lmk.RB_PLL2_LD=0;//не писать Read-Back
lmk.CLR_PLL2_LD_LOST=0;
//lmk.RB_DAC_VALUE;//не писать Read-Back
//lmk.RB_CLKin2_SEL;//не писать Read-Back
//lmk.RB_CLKin1_SEL;//не писать Read-Back
//lmk.RB_CLKin0_SEL;//не писать Read-Back
//lmk.RB_CLKin1_LOS;//не писать Read-Back
//lmk.RB_CLKin0_LOS;//не писать Read-Back
//lmk.RB_DAC_VALUE;//не писать Read-Back
//lmk.RB_HOLDOVER;//не писать Read-Back
lmk.SPI_LOCK=83;//83 - можно писать регистры, если любое другое число то нельзя.

	lmk.R[  0] = 0x00000000;
	lmk.R[  0] = (lmk.RESET<<7)+(lmk.SPI_3WIRE_DIS<<4);
	lmk.R[  1] = (0x002<<8)+lmk.POWERDOWN;
	lmk.R[  2] = (0x003<<8)+lmk.ID_DEVICE_TYPE;
	lmk.R[  3] = (0x004<<8)+((lmk.ID_PROD>>8)			&0x00ff);
	lmk.R[  4] = (0x005<<8)+( lmk.ID_PROD				&0x00ff);
	lmk.R[  5] = (0x006<<8)+( lmk.ID_MASKREV			&0x00ff);
	lmk.R[  6] = (0x00c<<8)+((lmk.ID_VNDR>>8)			&0x00ff);
	lmk.R[  7] = (0x00d<<8)+((lmk.ID_VNDR>>0)			&0x00ff);
	lmk.R[  8] = (0x100<<8)+((lmk.CLKout0_1_ODL     <<6) + (lmk.CLKout0_1_IDL     <<5) + (lmk.DCLKout0_DIV)	    				&0xff);
	lmk.R[  9] = (0x101<<8)+((lmk.DCLKout0_DDLY_CNTH<<4) + (lmk.DCLKout0_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 10] = (0x103<<8)+((lmk.DCLKout0_ADLY     <<3) + (lmk.DCLKout0_ADLY_MUX <<2) + (lmk.DCLKout0_MUX<<0)					&0xff);
	lmk.R[ 11] = (0x104<<8)+((lmk.DCLKout0_HS<<6) + (lmk.SDCLKout1_MUX<<5) + (lmk.SDCLKout1_DDLY<<1) + (lmk.SDCLKout1_HS<<0)	&0xff);
	lmk.R[ 12] = (0x105<<8)+((lmk.SDCLKout1_ADLY_EN<<4) + (lmk.SDCLKout1_ADLY<<0)												&0xff);
	lmk.R[ 13] = (0x106<<8)+((lmk.DCLKout0_DDLY_PD<<7) + (lmk.DCLKout0_HSg_PD<<6)  + (lmk.DCLKout_ADLYg_PD<<5)
						   + (lmk.DCLKout0_ADLY_PD<<4)+ (lmk.CLKout0_1_PD<<3) + (lmk.SDCLKout1_DIS_MODE<<1) + (lmk.SDCLKout1_PD<<0)&0xff);

	lmk.R[ 14] = (0x107<<8)+((lmk.SDCLKout1_POL<<7) + (lmk.SDCLKout1_FMT<<4) + (lmk.DCLKout0_POL<<3) + (lmk.CLKout0_FMT<<0)		&0xff);
	lmk.R[ 15] = (0x108<<8)+((lmk.CLKout2_3_ODL<<6) + (lmk.CLKout2_3_IDL<<5)+(lmk.DCLKout2_DIV<<0)								&0xff); 
	lmk.R[ 16] = (0x109<<8)+((lmk.DCLKout2_DDLY_CNTH<<4)+(lmk.DCLKout2_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 17] = (0x10b<<8)+((lmk.DCLKout2_ADLY<<3) + (lmk.DCLKout2_ADLY_MUX<<2) + (lmk.DCLKout2_MUX<<0)						&0xff);
	lmk.R[ 18] = (0x10c<<8)+((lmk.DCLKout2_HS<<6) + (lmk.SDCLKout3_MUX<<5) + (lmk.SDCLKout3_DDLY<<1)+(lmk.SDCLKout3_HS)			&0xff);

	lmk.R[ 19] = (0x10d<<8)+((lmk.SDCLKout3_ADLY_EN<<4) + (lmk.SDCLKout3_ADLY_EN<<0)										&0xff);

	lmk.R[ 20] = (0x10e<<8)+((lmk.DCLKout2_DDLY_PD<<7)+(lmk.DCLKout2_HSg_PD<<6)+(lmk.DCLKout2_ADLYg_PD<<5)+(lmk.DCLKout2_ADLY_PD<<4)+
							 (lmk.CLKout2_3_PD<<3)+(lmk.SDCLKout3_DIS_MODE<<1)+(lmk.SDCLKout3_PD)								&0xff);

	lmk.R[ 21] = (0x10f<<8)+((lmk.SDCLKout3_POL<<7)+(lmk.SDCLKout3_FMT<<4)+(lmk.DCLKout2_POL<<3)+(lmk.CLKout2_FMT<<0)				&0xff);

	lmk.R[ 22] = (0x110<<8)+((lmk.CLKout4_5_ODL<<6)+(lmk.CLKout4_5_IDL<<5)+(lmk.DCLKout4_DIV<<0)								&0xff);
	lmk.R[ 23] = (0x111<<8)+((lmk.DCLKout4_DDLY_CNTH<<4)+(lmk.DCLKout4_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 24] = (0x113<<8)+((lmk.DCLKout4_ADLY<<3)+(lmk.DCLKout4_ADLY_MUX<<2)+(lmk.DCLKout4_MUX<<0)							&0xff);
	lmk.R[ 25] = (0x114<<8)+((lmk.DCLKout4_HS<<6)+(lmk.SDCLKout5_MUX<<5)+(lmk.SDCLKout5_DDLY<<1)+(lmk.SDCLKout5_HS<<0)			&0xff);
	lmk.R[ 26] = (0x115<<8)+((lmk.SDCLKout5_ADLY_EN<<4)+(lmk.SDCLKout5_ADLY<<0)													&0xff);
	lmk.R[ 27] = (0x116<<8)+((lmk.DCLKout4_DDLY_PD<<7)+(lmk.DCLKout4_HSg_PD<<6)+(lmk.DCLKout4_ADLYg_PD<<5)+(lmk.DCLKout4_ADLY_PD<<4)+
							 (lmk.CLKout4_5_PD<<3)+(lmk.SDCLKout5_DIS_MODE<<1)+(lmk.SDCLKout5_PD<<0)							&0xff);
	lmk.R[ 28] = (0x117<<8)+((lmk.SDCLKout5_POL<<7)+(lmk.SDCLKout5_FMT<<4)+(lmk.DCLKout4_POL<<3)+(lmk.CLKout4_FMT<<0)				&0xff);
	lmk.R[ 29] = (0x118<<8)+((lmk.CLKout6_7_ODL<<6)+(lmk.CLKout6_7_IDL<<5)+(lmk.DCLKout6_DIV<<0)								&0xff);
	lmk.R[ 30] = (0x119<<8)+((lmk.DCLKout6_DDLY_CNTH<<4)+(lmk.DCLKout6_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 31] = (0x11b<<8)+((lmk.DCLKout6_ADLY<<3)+(lmk.DCLKout6_ADLY_MUX<<2)+(lmk.DCLKout6_MUX<<0)							&0xff);
	lmk.R[ 32] = (0x11c<<8)+((lmk.DCLKout6_HS<<6)+(lmk.SDCLKout7_MUX<<5)+(lmk.SDCLKout7_DDLY<<1)+(lmk.SDCLKout7_HS<<0)			&0xff);
	lmk.R[ 33] = (0x11d<<8)+((lmk.SDCLKout7_ADLY_EN<<4)+(lmk.SDCLKout7_ADLY<<0)													&0xff);
	lmk.R[ 34] = (0x11e<<8)+((lmk.DCLKout6_DDLY_PD<<7)+(lmk.DCLKout6_HSg_PD<<6)+(lmk.DCLKout6_ADLYg_PD<<5)+(lmk.DCLKout6_ADLY_PD<<4)+
							 (lmk.CLKout6_7_PD<<3)+(lmk.SDCLKout7_DIS_MODE<<1)+(lmk.SDCLKout7_PD<<0)							&0xff);
	lmk.R[ 35] = (0x11f<<8)+((lmk.SDCLKout7_POL<<7)+(lmk.SDCLKout7_FMT<<4)+(lmk.DCLKout6_POL<<3)+(lmk.CLKout6_FMT<<0)				&0xff);
	lmk.R[ 36] = (0x120<<8)+((lmk.CLKout8_9_ODL<<6)+(lmk.CLKout8_9_IDL<<5)+(lmk.DCLKout8_DIV<<0)								&0xff);
	lmk.R[ 37] = (0x121<<8)+((lmk.DCLKout8_DDLY_CNTH<<4)+(lmk.DCLKout8_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 38] = (0x123<<8)+((lmk.DCLKout8_ADLY<<3)+(lmk.DCLKout8_ADLY_MUX<<2)+(lmk.DCLKout8_MUX<<0)							&0xff);
	lmk.R[ 39] = (0x124<<8)+((lmk.DCLKout8_HS<<6)+(lmk.SDCLKout9_MUX<<5)+(lmk.SDCLKout9_DDLY<<1)+(lmk.SDCLKout9_HS<<0)			&0xff);
	lmk.R[ 40] = (0x125<<8)+((lmk.SDCLKout9_ADLY_EN<<4)+(lmk.SDCLKout9_ADLY<<0)													&0xff);
	lmk.R[ 41] = (0x126<<8)+((lmk.DCLKout8_DDLY_PD<<7)+(lmk.DCLKout8_HSg_PD<<6)+(lmk.DCLKout8_ADLYg_PD<<5)+
							 (lmk.DCLKout8_ADLY_PD<<4)+(lmk.CLKout8_9_PD<<3)+(lmk.SDCLKout9_DIS_MODE<<1)+(lmk.SDCLKout9_PD<<0)	&0xff);
	lmk.R[ 42] = (0x127<<8)+((lmk.SDCLKout9_POL<<7)+(lmk.SDCLKout9_FMT<<4)+(lmk.DCLKout8_POL<<3)+(lmk.CLKout8_FMT<<0)				&0xff);
	lmk.R[ 43] = (0x128<<8)+((lmk.CLKout10_11_ODL<<6)+(lmk.CLKout10_11_IDL<<5)+(lmk.DCLKout10_DIV<<0)							&0xff);
	lmk.R[ 44] = (0x129<<8)+((lmk.DCLKout10_DDLY_CNTH<<4)+(lmk.DCLKout10_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 45] = (0x12b<<8)+((lmk.DCLKout10_ADLY<<3)+(lmk.DCLKout10_ADLY_MUX<<2)+(lmk.DCLKout10_MUX)							&0xff);
	lmk.R[ 46] = (0x12c<<8)+((lmk.DCLKout10_HS<<6)+(lmk.SDCLKout11_MUX<<5)+(lmk.SDCLKout11_DDLY<<1)+(lmk.SDCLKout11_HS<<0)		&0xff);
	lmk.R[ 47] = (0x12d<<8)+((lmk.SDCLKout11_ADLY_EN<<4)+(lmk.SDCLKout11_ADLY<<0)												&0xff);
	lmk.R[ 48] = (0x12e<<8)+((lmk.DCLKout10_DDLY_PD<<7)+(lmk.DCLKout10_HSg_PD<<6)+(lmk.DCLKout10_ADLYg_PD<<5)+
							 (lmk.DCLKout10_ADLY_PD<<4)+(lmk.CLKout10_11_PD<<3)+(lmk.SDCLKout11_DIS_MODE<<1)+(lmk.SDCLKout11_PD<<0)&0xff);
	lmk.R[ 49] = (0x12f<<8)+((lmk.SDCLKout11_POL<<7)+(lmk.SDCLKout11_FMT<<4)+(lmk.DCLKout10_POL<<3)+(lmk.CLKout10_FMT<<0)			&0xff);
	lmk.R[ 50] = (0x130<<8)+((lmk.CLKout12_13_ODL<<6)+(lmk.CLKout12_13_IDL<<5)+(lmk.DCLKout12_DIV<<0)							&0xff);
	lmk.R[ 51] = (0x131<<8)+((lmk.DCLKout12_DDLY_CNTH<<4)+(lmk.DCLKout12_DDLY_CNTL<<0)											&0xff);
	lmk.R[ 52] = (0x133<<8)+((lmk.DCLKout12_ADLY<<3)+(lmk.DCLKout12_ADLY_MUX<<2)+(lmk.DCLKout12_MUX<<0)							&0xff);
	lmk.R[ 53] = (0x134<<8)+((lmk.DCLKout12_HS<<6)+(lmk.SDCLKout13_MUX<<5)+(lmk.SDCLKout13_DDLY<<1)+(lmk.SDCLKout13_HS<<0)		&0xff);
	lmk.R[ 54] = (0x135<<8)+((lmk.SDCLKout13_ADLY_EN<<4)+(lmk.SDCLKout13_ADLY<<0)												&0xff);
	lmk.R[ 55] = (0x136<<8)+((lmk.DCLKout12_DDLY_PD<<7)+(lmk.DCLKout12_HSg_PD<<6)+(lmk.DCLKout12_ADLYg_PD<<5)+
							 (lmk.DCLKout12_ADLY_PD<<4)+(lmk.CLKout12_13_PD<<3)+(lmk.SDCLKout13_DIS_MODE<<1)+(lmk.SDCLKout13_PD)&0xff);
	lmk.R[ 56] = (0x137<<8)+((lmk.SDCLKout13_POL<<7)+(lmk.SDCLKout13_FMT<<4)+(lmk.DCLKout12_POL<<3)+(lmk.CLKout12_FMT<<0)		&0xff);
	lmk.R[ 57] = (0x138<<8)+((lmk.VCO_MUX<<5)+(lmk.OSCout_MUX<<4)+(lmk.OSCout_FMT<<0)											&0xff);
	lmk.R[ 58] = (0x139<<8)+((lmk.SYSREF_CLKin0_MUX<<2)+(lmk.SYSREF_MUX<<0)														&0xff);
    lmk.R[ 59] = (0x13a<<8)+((lmk.SYSREF_DIV >>8)																				&0xff);
    lmk.R[ 60] = (0x13b<<8)+((lmk.SYSREF_DIV <<0)																				&0xff);
    lmk.R[ 61] = (0x13c<<8)+((lmk.SYSREF_DDLY>>8)																				&0xff);
    lmk.R[ 62] = (0x13d<<8)+((lmk.SYSREF_DDLY<<0)																				&0xff);
    lmk.R[ 63] = (0x13e<<8)+((lmk.SYSREF_PULSE_CNT<<0)																			&0xff);
    lmk.R[ 64] = (0x13f<<8)+((lmk.PLL2_NCLK_MUX<<4)+(lmk.PLL1_NCLK_MUX<<3)+(lmk.FB_MUX<<1)+(lmk.FB_MUX_EN<<0)					&0xff);
    lmk.R[ 65] = (0x140<<8)+((lmk.PLL1_PD<<7)+(lmk.VCO_LDO_PD<<6)+(lmk.VCO_PD<<5)+(lmk.OSCin_PD<<4)+(lmk.SYSREF_GBL_PD<<3)+
    						 (lmk.SYSREF_PD<<2)+(lmk.SYSREF_DDLY_PD<<1)+(lmk.SYSREF_PLSR_PD<<0)								    &0xff);	
    lmk.R[ 66] = (0x141<<8)+((lmk.DDLYd_SYSREF_EN<<7)+(lmk.DDLYd12_EN<<6)+(lmk.DDLYd10_EN<<5)+(lmk.DDLYd7_EN<<4)+
    						 (lmk.DDLYd6_EN<<3)+(lmk.DDLYd4_EN<<2)+(lmk.DDLYd2_EN<<1)+(lmk.DDLYd0_EN<<0)						&0xff);	
    lmk.R[ 67] = (0x142<<8)+((lmk.DDLYd_STEP_CNT<<0)																			&0xff);
    lmk.R[ 68] = (0x143<<8)+((lmk.SYSREF_DDLY_CLR<<7)+(lmk.SYNC_1SHOT_EN<<6)+(lmk.SYNC_POL  <<5)+(lmk.SYNC_EN<<4)+
    						 (lmk.SYNC_PLL2_DLD  <<3)+(lmk.SYNC_PLL1_DLD<<2)+(lmk.SYNC_MODE <<0)								&0xff);
    lmk.R[ 69] = (0x144<<8)+((lmk.SYNC_DISSYSREF <<7)+(lmk.SYNC_DIS12   <<6)+(lmk.SYNC_DIS10<<5)+(lmk.SYNC_DIS8<<4)+
    						 (lmk.SYNC_DIS6<<3)+(lmk.SYNC_DIS4<<2)+(lmk.SYNC_DIS2<<1)+(lmk.SYNC_DIS0<<0)						&0xff);
    lmk.R[ 70] = (0x145<<8)+((0x7f)																								&0xff);
    lmk.R[ 71] = (0x146<<8)+((lmk.CLKin2_EN<<5)+(lmk.CLKin1_EN<<4)+(lmk.CLKin0_EN<<3)+(lmk.CLKin2_TYPE<<2)+(lmk.CLKin1_TYPE)+
    						 (lmk.CLKin0_TYPE<<0)																				&0xff);
    lmk.R[ 72] = (0x147<<8)+((lmk.CLKin_SEL_POL <<7)+(lmk.CLKin_SEL_MODE<<4)+(lmk.CLKin1_OUT_MUX<<2)+(lmk.CLKin0_OUT_MUX<<0)	&0xff);
    lmk.R[ 73] = (0x148<<8)+((lmk.CLKin_SEL0_MUX<<3)+(lmk.CLKin_SEL0_TYPE<<0)													&0xff);
    lmk.R[ 74] = (0x149<<8)+((lmk.SDIO_RDBK_TYPE<<6)+(lmk.CLKin_SEL1_MUX<<3)+(lmk.CLKin_SEL1_TYPE<<0)							&0xff);
    lmk.R[ 75] = (0x14a<<8)+((lmk.RESET_MUX<<3)+(lmk.RESET_TYPE<<0)																&0xff);
    lmk.R[ 76] = (0x14b<<8)+((lmk.LOS_TIMEOUT<<6)+(lmk.LOS_EN<<5)+(lmk.TRACK_EN<<4)+(lmk.HOLDOVER_FORCE<<3)+(lmk.MAN_DAC_EN<<2)+
    						 (lmk.MAN_DAC>>8)																					&0xff);
    lmk.R[ 77] = (0x14c<<8)+((lmk.MAN_DAC)																						&0xff);
    lmk.R[ 78] = (0x14d<<8)+((lmk.DAC_TRIP_LOW)																					&0xff);
    lmk.R[ 79] = (0x14e<<8)+((lmk.DAC_CLK_MULT<<6)+(lmk.DAC_TRIP_HIGH<<0)														&0xff);
    lmk.R[ 80] = (0x14f<<8)+((lmk.DAC_CLK_CNTR<<0)																				&0xff);
    lmk.R[ 81] = (0x150<<8)+((lmk.HOLDOVER_PLL1_DET<<4)+(lmk.HOLDOVER_LOS_DET<<3)+(lmk.HOLDOVER_VTUNE_DET<<2)+
    						 (lmk.HOLDOVER_HITLESS_SWITCH<<1)+(lmk.HOLDOVER_EN<<0)												&0xff);
    lmk.R[ 82] = (0x151<<8)+((lmk.HOLDOVER_DLD_CNT>>8)																			&0xff);
    lmk.R[ 83] = (0x152<<8)+((lmk.HOLDOVER_DLD_CNT)																				&0xff);
    lmk.R[ 84] = (0x153<<8)+((lmk.CLKin0_R>>8)																					&0xff);
    lmk.R[ 85] = (0x154<<8)+((lmk.CLKin0_R)																						&0xff);
    lmk.R[ 86] = (0x155<<8)+((lmk.CLKin1_R>>8)																					&0xff);
    lmk.R[ 87] = (0x156<<8)+((lmk.CLKin1_R)																						&0xff);
    lmk.R[ 88] = (0x157<<8)+((lmk.CLKin2_R>>8)																					&0xff);
    lmk.R[ 89] = (0x158<<8)+((lmk.CLKin2_R)																						&0xff);
    lmk.R[ 90] = (0x159<<8)+((lmk.PLL1_N>>8)																					&0xff);
    lmk.R[ 91] = (0x15a<<8)+((lmk.PLL1_N)																						&0xff);
    lmk.R[ 92] = (0x15b<<8)+((lmk.PLL1_WND_SIZE<<6)+(lmk.PLL1_CP_TRI<<5)+(lmk.PLL1_CP_POL>>4)+(lmk.PLL1_CP_GAIN<<0)				&0xff);
    lmk.R[ 93] = (0x15c<<8)+((lmk.PLL1_DLD_CNT>>8)																				&0xff);
    lmk.R[ 94] = (0x15d<<8)+((lmk.PLL1_DLD_CNT)																					&0xff);
    lmk.R[ 95] = (0x15e<<8)+((lmk.PLL1_R_DLY <<3)+(lmk.PLL1_N_DLY  <<0)															&0xff);
    lmk.R[ 96] = (0x15f<<8)+((lmk.PLL1_LD_MUX<<3)+(lmk.PLL1_LD_TYPE<<0)															&0xff);
    lmk.R[ 97] = (0x160<<8)+((lmk.PLL2_R>>8)																					&0xff);
    lmk.R[ 98] = (0x161<<8)+((lmk.PLL2_R)																						&0xff);
    lmk.R[ 99] = (0x162<<8)+((lmk.PLL2_P<<5)+(lmk.OSCin_FREQ<<2)+(lmk.PLL2_XTAL_EN<<1)+(lmk.PLL2_REF_2X_EN<<0)					&0xff);
    lmk.R[100] = (0x163<<8)+((lmk.PLL2_N_CAL>>16)																				&0xff);
    lmk.R[101] = (0x164<<8)+((lmk.PLL2_N_CAL>>8)																				&0xff);
    lmk.R[102] = (0x165<<8)+((lmk.PLL2_N_CAL)																					&0xff);
    lmk.R[103] = (0x166<<8)+((lmk.PLL2_FCAL_DIS<<2)+(lmk.PLL2_N>>16)															&0xff);
    lmk.R[104] = (0x167<<8)+((lmk.PLL2_N>>8)																					&0xff);
    lmk.R[105] = (0x168<<8)+((lmk.PLL2_N)																						&0xff);
    lmk.R[106] = (0x169<<8)+((lmk.PLL2_WND_SIZE<<5)+(lmk.PLL2_CP_GAIN<<3)+(lmk.PLL2_CP_POL<<2)+(lmk.PLL2_CP_TRI<<1)+1 			&0xff);
    lmk.R[107] = (0x16a<<8)+((lmk.SYSREF_REQ_EN<<6)+(lmk.PLL2_DLD_CNT>>8)														&0xff);
    lmk.R[108] = (0x16b<<8)+((lmk.PLL2_DLD_CNT)																					&0xff);
    lmk.R[109] = (0x16c<<8)+((lmk.PLL2_LF_R4 <<3)+(lmk.PLL2_LF_R3) 																&0xff);
    lmk.R[110] = (0x16d<<8)+((lmk.PLL2_LF_C4 <<4)+(lmk.PLL2_LF_C3)																&0xff);
    lmk.R[111] = (0x16e<<8)+((lmk.PLL2_LD_MUX<<4)+(lmk.PLL2_LD_TYPE)															&0xff);
    lmk.R[112] = (0x171<<8)+(0xaa                                                                                               &0xff);
    lmk.R[113] = (0x172<<8)+(0x02                                                                                               &0xff);
    lmk.R[114] = (0x173<<8)+((lmk.PLL2_PRE_PD<<6)+(lmk.PLL2_PD<<5)																&0xff);
    lmk.R[115] = (0x17c<<8)+((lmk.OPT_REG_1)																					&0xff);
    lmk.R[116] = (0x17d<<8)+((lmk.OPT_REG_2)																					&0xff);
    lmk.R[117] = (0x182<<8)+((lmk.RB_PLL1_LD_LOST<<2)+(lmk.RB_PLL1_LD<<1)+(lmk.CLR_PLL1_LD_LOST<<0)								&0xff);
    lmk.R[118] = (0x183<<8)+((lmk.RB_PLL2_LD_LOST<<2)+(lmk.RB_PLL2_LD<<1)+(lmk.CLR_PLL2_LD_LOST<<0)	 							&0xff);
    lmk.R[119] = (0x184<<8)+(((lmk.RB_DAC_VALUE>>8)&0xff)+(lmk.RB_CLKin2_SEL<<5)+(lmk.RB_CLKin1_SEL<<4)+(lmk.RB_CLKin0_SEL<<3)+
    						  (lmk.RB_CLKin1_LOS<<1)+(lmk.RB_CLKin0_LOS<<0)													    &0xff);
    lmk.R[120] = (0x185<<8)+((lmk.RB_DAC_VALUE)																					&0xff);	
    lmk.R[121] = (0x188<<8)+((lmk.RB_HOLDOVER)																					&0xff);
    lmk.R[122] = (0x1ffd<<8)+((lmk.SPI_LOCK>>16)																				&0xff);
    lmk.R[123] = (0x1ffe<<8)+((lmk.SPI_LOCK>>8)																					&0xff);
    lmk.R[124] = (0x1fff<<8)+((lmk.SPI_LOCK)																					&0xff);

}


void spisend_lmk (unsigned int d) //24бита
{  
   CS_LMK_0;
 //  Delay(1);
   
  // Transf("." );
   
   spisend8(((d >> 16)&0xff));//устанавливаем бит записи 
   spisend8( (d >>  8)&0xff);
   spisend8( (d)      &0xff);
      
 //  Delay(1);  
   CS_LMK_1;
   Delay(1);
 
}

char spiread_lmk (unsigned short adr) //24бита
{
   char data; 
   CS_LMK_0;
 //  Delay(1);
 //  Transf("." );
   spisend8(((adr >>  8)&0xff)|0x80);// устанавливаем бит чтения
   spisend8( (adr >>  0)&0xff);
   data=spisend8(0);
  
 //  Delay(1);  
   CS_LMK_1;
   Delay(1);

   return data; 
}


void lmk_write (char r)
{
	int i=0;
	

  spisend_lmk(0x90);//шлём RESET

  INIT_REG_LMK ();
  
  for (i=0;i<103;i++)
  {
	 if ((i!=2)&&
         (i!=3)&&
         (i!=4)&&
         (i!=5)&&
         (i!=6)&&
         (i!=7)
         )     {
                    spisend_lmk(lmk.R[i]); //Programming registers in numeric order from 0x000 to 0x165
                   // Transf("\r\n");
                   // x_out("lmk.R:",lmk.R[i]);
                   // un_out("adr:",(lmk.R[i]>>8));  Transf("\t"); x_out("",lmk.R[i]);
                 }
       }
   spisend_lmk(lmk.R[112]);// 0xAA
   spisend_lmk(lmk.R[113]);// 0x02
   spisend_lmk(lmk.R[115]);//Programming registers 0x17C 
   spisend_lmk(lmk.R[116]);//Programming registers 0x17D
   
   for (i=103;i<125;i++)
  {
	 if ((i!=112)&&
         (i!=113)&&
         (i!=115)&&
         (i!=116)&&
         (i!=117)&&
         (i!=118)&&
         (i!=119)&&
         (i!=120)&&
         (i!=121)
         ) 
     {
        spisend_lmk(lmk.R[i]); // Programming registers 0x166 to 0x1FFF
        //un_out("adr:",(lmk.R[i]>>8));  Transf("\t"); x_out("",lmk.R[i]);
        }
  } 
  
}

void init_lmk_temp (void)
{
    spisend_lmk(0x80);//шлём RESET
    spisend_lmk(0x10);//4wire
    spisend_lmk((0x15f<<8)+(0x7<<3)+(0x3));//SPI Readback + Output (push-pull)
    spisend_lmk((0x16e<<8)+(0x0<<3)+(0x4));//test out
}

void init_lmk (u8 a)

{
	if (a==1)
	{
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Пришла команда активации LMK!\r\n" );
	 Transf("Программирую ФАПЧ:\r" );
     lmk_write(0);
	 Transf("\r\n" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
	}
}

#include "main.h"

//---------------------------------------------------------------------------------
//--------------------------


 reg_ADC42jb49 adc1;
 reg_ADC42jb49 adc2;

void INIT_REG_ADC (reg_ADC42jb49 *adc)
{
 adc->CLK_DIV	  =0;
 adc->SYSREF_DELAY=0;
 adc->PDN_CHA =0;
 adc->PDN_CHB =0;
 adc->STDBY	=0;
 adc->DATA_FORMAT=0;//0 : Twos complement //
 adc->RESET=0;
 adc->CHA_GAIN=0;//19 максимальный уровень 1 вольт
 adc->CHA_GAIN_EN=1;//1
 adc->CHB_GAIN=0;//19 максимальный уровень 1 вольт
 adc->CHB_GAIN_EN=1;//1
 adc->HIGH_FREQ_1=0;
 adc->FAST_OVR_EN=0;
 adc->HIGH_FREQ_2=0;
 adc->CHA_TEST_PATTERNS=0x0;//0010 : All '1's
 adc->CHB_TEST_PATTERNS=0x0;
 adc->CUSTOM_PATTERN1=0xabcd;//0xabcd;
 adc->CUSTOM_PATTERN2=0x9876;//0x1234;
 adc->FAST_OVR_THRESHOLD=0;
 adc->SERDES_TEST_PATTERN=0;
 adc->IDLE_SYNC=0;
 adc->TESTMODE_EN=0;
 adc->FLIP_ADC_DATA=0;
 adc->LANE_ALIGN=0;
 adc->FRAME_ALIGN=0;
 adc->TX_LINK_CONFIG_DATA0=0;//1 : ILA disabled
 adc->CTRLK=1;//1 : Frames per multiframe can be set in register 2Dh
 adc->CTRLF=1;//1 : Octets per frame can be specified in register 2Ch
 adc->SCRAMBLE_EN=0;
 adc->OCTETS_PER_FRAME=0;
 adc->FRAMES_PER_MULTIFRAME=15;
 adc->SUBCLASS=0;//010 : Subclass 2. Deterministic latency using SYNC~ detection
 adc->SYNC_REQ=0;
 adc->LMFC_RESET_MASK=0;
 adc->OUTPUT_CURRENT_SEL=11;//11 - 5 ma
 adc->LINK_LAYER_TESTMODE=0;
 adc->LINK_LAYER_RPAT=0;
 adc->PULSE_DET_MODES=2;
 adc->FORCE_LMFC_COUNT=0;
 adc->LMFC_COUNT_INIT=0;
 adc->RELEASE_ILANE_SEQ=0;
 
}


void defrag_REG_ADC (reg_ADC42jb49 *adc)
{
	int i=0;
	
	for (i=0;i<64;i++) adc->R[i]=0;
	
	 adc->R[0x06] = adc->CLK_DIV&0x3;
	 adc->R[0x07] = adc->SYSREF_DELAY&0x7;
 	 adc->R[0x08] = (adc->PDN_CHA    <<7)+
					(adc->PDN_CHB    <<6)+
					(adc->STDBY      <<5)+
					(adc->DATA_FORMAT<<4)+
					(               1<<3)+
					(adc->RESET      <<0);
				 
	 adc->R[0x0B] =	(adc->CHA_GAIN   <<3)+
					(adc->CHA_GAIN_EN<<2);
					
	 adc->R[0x0C] =	(adc->CHB_GAIN   <<3)+
					(adc->CHB_GAIN_EN<<2);

	 adc->R[0x0D] =	(adc->HIGH_FREQ_1<<7)+
					(adc->HIGH_FREQ_1<<4)+
					(adc->FAST_OVR_EN<<0);
	
	 adc->R[0x0E] =	(adc->HIGH_FREQ_2<<7)+
					(adc->HIGH_FREQ_2<<4);

	 adc->R[0x0F] =	((adc->CHA_TEST_PATTERNS&0x0f)<<4)+
					((adc->CHB_TEST_PATTERNS<<0)&0x0f);
					
	 adc->R[0x10] =  (adc->CUSTOM_PATTERN1>>8)&0xff;
	 adc->R[0x11] =  (adc->CUSTOM_PATTERN1)   &0xff;
	 adc->R[0x12] =  (adc->CUSTOM_PATTERN2>>8)&0xff;
	 adc->R[0x13] =  (adc->CUSTOM_PATTERN2)   &0xff;
	 
	 adc->R[0x1F] = (adc->FAST_OVR_THRESHOLD)&0x7f;
	 
	 adc->R[0x26] = (adc->SERDES_TEST_PATTERN<<6)+
					(adc->IDLE_SYNC    <<5)+
					(adc->TESTMODE_EN  <<4)+
					(adc->FLIP_ADC_DATA<<3)+
					(adc->LANE_ALIGN   <<2)+
					(adc->FRAME_ALIGN  <<1)+
				   ((adc->TX_LINK_CONFIG_DATA0)&0x01);
					
	 adc->R[0x27] = (adc->CTRLK<<1)+
					(adc->CTRLF<<0);
					
	 adc->R[0x2B] = (adc->SCRAMBLE_EN<<7);
	 
	 adc->R[0x2C] =	(adc->OCTETS_PER_FRAME)&0x01;
	 
	 adc->R[0x2D] = (adc->FRAMES_PER_MULTIFRAME)&0x1f;
	 
	 
	 adc->R[0x30] = (adc->SUBCLASS<<5);
	 
	 adc->R[0x36] = (adc->SYNC_REQ<<7)+
					(adc->LMFC_RESET_MASK<<6)+
				   ((adc->OUTPUT_CURRENT_SEL<<0)&0x0f);
					
	 adc->R[0x37] = (adc->LINK_LAYER_TESTMODE<<5)+
					(adc->LINK_LAYER_RPAT    <<4)+
					
					(adc->PULSE_DET_MODES    <<0);
					
	 adc->R[0x38] = (adc->FORCE_LMFC_COUNT <<7)+
					(adc->LMFC_COUNT_INIT  <<2)+
				   ((adc->RELEASE_ILANE_SEQ<<0)&0x3);
					
					
	 
					
}
//---------------ADC2-----------------------------------
	
void write_reg_adc2 (u8 adr,u8 d) //8бита
{  
   CS_adc2_0; 
   delay_us(10);
   spisend8(((adr)    &0x3f));//устанавливаем бит записи 
   spisend8( (d)      &0xff);
   delay_us(10);
   CS_adc2_1;
  // nx_out("R[",adr);
  // x_out("]:",d);
}

u8 spiread_adc2 (u8 adr) //8бит
{
   char data;
   
   CS_adc2_0;
   delay_us(10);
   spisend8(((adr)&0x3f)|0x80);// устанавливаем бит чтения
   data=spisend8(0);
   delay_us(10);
   CS_adc2_1;
  
   return data; 
}

void adc2_write (char r)
{
	int i=0;

	for (i=0;i<64;i++) adc2.R[i]=0;
	
	IO("~0 ADC2_RST:0;");
	delay_us(100);
	IO("~0 ADC2_RST:1;");
	delay_us(100);
	IO("~0 ADC2_RST:0;");
	
	//adc2_read_reg ();//чтение обресеченых регистров
	
	INIT_REG_ADC  (&adc2);
	defrag_REG_ADC (&adc2);
	
	write_reg_adc2(0x06,adc2.R[0x06]);
	write_reg_adc2(0x07,adc2.R[0x07]);
	write_reg_adc2(0x08,adc2.R[0x08]);
	write_reg_adc2(0x0B,adc2.R[0x0B]);
	write_reg_adc2(0x0C,adc2.R[0x0C]);
	write_reg_adc2(0x0D,adc2.R[0x0D]);
	write_reg_adc2(0x0E,adc2.R[0x0E]);
	write_reg_adc2(0x0F,adc2.R[0x0F]);
	write_reg_adc2(0x10,adc2.R[0x10]);
	write_reg_adc2(0x11,adc2.R[0x11]);
	write_reg_adc2(0x12,adc2.R[0x12]);
	write_reg_adc2(0x13,adc2.R[0x13]);
	write_reg_adc2(0x1F,adc2.R[0x1F]);
	write_reg_adc2(0x26,adc2.R[0x26]);
	write_reg_adc2(0x27,adc2.R[0x27]);
	write_reg_adc2(0x2B,adc2.R[0x2B]);
	write_reg_adc2(0x2C,adc2.R[0x2C]);
	write_reg_adc2(0x2D,adc2.R[0x2D]);
	write_reg_adc2(0x30,adc2.R[0x30]);
	write_reg_adc2(0x36,adc2.R[0x36]);
	write_reg_adc2(0x37,adc2.R[0x37]);
	write_reg_adc2(0x38,adc2.R[0x38]);   
}

void adc2_test (u8 a)
{
	adc2.CHA_TEST_PATTERNS=a;//: All '1's
    adc2.CHB_TEST_PATTERNS=a;
	defrag_REG_ADC (&adc2);
	write_reg_adc2(0x0F,adc2.R[0x0F]);
}

void adc2_read_reg (void)
{
	u8 data;
		
	data=spiread_adc2(0x06);
    x_out("reg[0x06]:",data&0xff);
	data=spiread_adc2(0x07);
    x_out("reg[0x07]:",data&0xff);
	data=spiread_adc2(0x08);
    x_out("reg[0x08]:",data&0xff);
	data=spiread_adc2(0x0b);
    x_out("reg[0x0b]:",data&0xff);
	data=spiread_adc2(0x0c);
    x_out("reg[0x0c]:",data&0xff);
	data=spiread_adc2(0x0d);
    x_out("reg[0x0d]:",data&0xff);
	data=spiread_adc2(0x0e);
    x_out("reg[0x0e]:",data&0xff);
	data=spiread_adc2(0x0f);
    x_out("reg[0x0f]:",data&0xff);
	data=spiread_adc2(0x10);
    x_out("reg[0x10]:",data&0xff);
	data=spiread_adc2(0x11);
    x_out("reg[0x11]:",data&0xff);
	data=spiread_adc2(0x12);
    x_out("reg[0x12]:",data&0xff);
	data=spiread_adc2(0x13);
    x_out("reg[0x13]:",data&0xff);
	data=spiread_adc2(0x1f);
    x_out("reg[0x1f]:",data&0xff);
	data=spiread_adc2(0x26);
    x_out("reg[0x26]:",data&0xff);
	data=spiread_adc2(0x27);
    x_out("reg[0x27]:",data&0xff);
	data=spiread_adc2(0x2b);
    x_out("reg[0x2b]:",data&0xff);
	data=spiread_adc2(0x2c);
    x_out("reg[0x2c]:",data&0xff);
	data=spiread_adc2(0x2d);
    x_out("reg[0x2d]:",data&0xff);
	data=spiread_adc2(0x30);
    x_out("reg[0x30]:",data&0xff);
	data=spiread_adc2(0x36);
    x_out("reg[0x36]:",data&0xff);
	data=spiread_adc2(0x37);
    x_out("reg[0x37]:",data&0xff);
	data=spiread_adc2(0x38);
    x_out("reg[0x38]:",data&0xff);
	
	
}

void adc2_fifo_read (u8 adr,u8 buf)
{
	u16 i=0;
	u16 data_fifo;
	u8 adr_adc=0;
	
	if (adr==0) adr_adc=57; else if (adr==1) adr_adc=58;

	while (i<2048)
	{
			i++;
			data_fifo=FPGA_rSPI (16,adr_adc);
			//data_fifo=i;
			if (buf==1) u_out("",data_fifo);
	}
}

//---------------ADC1----------------------------
	
void write_reg_adc1 (u8 adr,u8 d) //8бита
{  
   CS_adc1_0; 
   delay_us(10);
   spisend8(((adr)    &0x3f));//устанавливаем бит записи 
   spisend8( (d)      &0xff);
   delay_us(10);
   CS_adc1_1;
  // nx_out("R[",adr);
  // x_out("]:",d);
}

u8 spiread_adc1 (u8 adr) //8бит
{
   char data;
   
   CS_adc1_0;
   delay_us(10);
   spisend8(((adr)&0x3f)|0x80);// устанавливаем бит чтения
   data=spisend8(0);
   delay_us(10);
   CS_adc1_1;
  
   return data; 
}

void adc1_write (char r)
{
	int i=0;
	
	for (i=0;i<64;i++) adc1.R[i]=0;
	
	IO("~0 ADC1_RST:0;");
	delay_us(100);
	IO("~0 ADC1_RST:1;");
	delay_us(100);
	IO("~0 ADC1_RST:0;");
	
	//adc2_read_reg ();//чтение обресеченых регистров
	
	write_reg_adc1(0x08,0x01);//soft reset
	
	INIT_REG_ADC  (&adc1);
	defrag_REG_ADC (&adc1);
	
	write_reg_adc1(0x06,adc1.R[0x06]);
	write_reg_adc1(0x07,adc1.R[0x07]);
	write_reg_adc1(0x08,adc1.R[0x08]);
	write_reg_adc1(0x0B,adc1.R[0x0B]);
	write_reg_adc1(0x0C,adc1.R[0x0C]);
	write_reg_adc1(0x0D,adc1.R[0x0D]);
	write_reg_adc1(0x0E,adc1.R[0x0E]);
	write_reg_adc1(0x0F,adc1.R[0x0F]);
	write_reg_adc1(0x10,adc1.R[0x10]);
	write_reg_adc1(0x11,adc1.R[0x11]);
	write_reg_adc1(0x12,adc1.R[0x12]);
	write_reg_adc1(0x13,adc1.R[0x13]);
	write_reg_adc1(0x1F,adc1.R[0x1F]);
	write_reg_adc1(0x26,adc1.R[0x26]);
	write_reg_adc1(0x27,adc1.R[0x27]);
	write_reg_adc1(0x2B,adc1.R[0x2B]);
	write_reg_adc1(0x2C,adc1.R[0x2C]);
	write_reg_adc1(0x2D,adc1.R[0x2D]);
	write_reg_adc1(0x30,adc1.R[0x30]);
	write_reg_adc1(0x36,adc1.R[0x36]);
	write_reg_adc1(0x37,adc1.R[0x37]);
	write_reg_adc1(0x38,adc1.R[0x38]);   
}

void adc1_test (u8 a)
{
	adc1.CHA_TEST_PATTERNS=a;//: All '1's
    adc1.CHB_TEST_PATTERNS=a;
	defrag_REG_ADC (&adc1);
	write_reg_adc1(0x0F,adc1.R[0x0F]);
}

void adc1_read_reg (void)
{
	u8 data;
		
	data=spiread_adc1(0x06);
    x_out("reg[0x06]:",data&0xff);
	data=spiread_adc1(0x07);
    x_out("reg[0x07]:",data&0xff);
	data=spiread_adc1(0x08);
    x_out("reg[0x08]:",data&0xff);
	data=spiread_adc1(0x0b);
    x_out("reg[0x0b]:",data&0xff);
	data=spiread_adc1(0x0c);
    x_out("reg[0x0c]:",data&0xff);
	data=spiread_adc1(0x0d);
    x_out("reg[0x0d]:",data&0xff);
	data=spiread_adc1(0x0e);
    x_out("reg[0x0e]:",data&0xff);
	data=spiread_adc1(0x0f);
    x_out("reg[0x0f]:",data&0xff);
	data=spiread_adc1(0x10);
    x_out("reg[0x10]:",data&0xff);
	data=spiread_adc1(0x11);
    x_out("reg[0x11]:",data&0xff);
	data=spiread_adc1(0x12);
    x_out("reg[0x12]:",data&0xff);
	data=spiread_adc1(0x13);
    x_out("reg[0x13]:",data&0xff);
	data=spiread_adc1(0x1f);
    x_out("reg[0x1f]:",data&0xff);
	data=spiread_adc1(0x26);
    x_out("reg[0x26]:",data&0xff);
	data=spiread_adc1(0x27);
    x_out("reg[0x27]:",data&0xff);
	data=spiread_adc1(0x2b);
    x_out("reg[0x2b]:",data&0xff);
	data=spiread_adc1(0x2c);
    x_out("reg[0x2c]:",data&0xff);
	data=spiread_adc1(0x2d);
    x_out("reg[0x2d]:",data&0xff);
	data=spiread_adc1(0x30);
    x_out("reg[0x30]:",data&0xff);
	data=spiread_adc1(0x36);
    x_out("reg[0x36]:",data&0xff);
	data=spiread_adc1(0x37);
    x_out("reg[0x37]:",data&0xff);
	data=spiread_adc1(0x38);
    x_out("reg[0x38]:",data&0xff);	
}

void adc1_fifo_read (u8 adr,u8 buf)
{
	u16 i=0;
	u16 data_fifo;
	u8 adr_adc=0;
	
	if (adr==0) adr_adc=107; else if (adr==1) adr_adc=108;

	while (i<2048)
	{
			i++;
			data_fifo=FPGA_rSPI (16,adr_adc);
			//data_fifo=i;
			if (buf==1) u_out("",data_fifo);
	}

}

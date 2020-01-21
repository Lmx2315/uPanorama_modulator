#include "main.h"

//---------------------------------------------------------------------------------
 reg_DAC37j82 dac1;
 reg_DAC37j82 dac2;


void INIT_REG_DAC1 (reg_DAC37j82 *dac)
{
 dac->output_delayab			  =0;
 dac->output_delaycd 			  =0;
 dac->qmc_offsetab_ena            =0;
 dac->qmc_offsetcd_ena            =0;
 dac->qmc_corrab_ena              =1;//QMC коррекция включена
 dac->qmc_corrcd_ena              =1;
 dac->interp                      =8;//Determines the interpolation amount
 dac->alarm_zeros_txenable_ena    =0;
 dac->outsum_ena                  =0;//Turns on the summing of the A+C and B+D data paths
 dac->alarm_zeros_jesddata_ena    =1;
 dac->alarm_out_ena               =1;
 dac->alarm_out_pol               =1;
 dac->pap_ena                     =0;//Power Amp Protection (PAP) logic
 dac->inv_sinc_ab_ena             =0;
 dac->inv_sinc_cd_ena             =0;

 dac->sfrac_ena_ab                =0;
 dac->sfrac_ena_cd                =0;
 dac->lfrac_ena_ab                =0;
 dac->lfrac_ena_cd                =0;
 dac->sfrac_sel_ab                =0;
 dac->sfrac_sel_cd                =1;//1 : Data path C goes through filter
 dac->daca_compliment             =0;
 dac->dacb_compliment             =0;
 dac->dacc_compliment             =0;
 dac->dacd_compliment             =0;

 dac->dac_bitwidth                =0;//Determines the bit width of the DAC
 dac->zer_invalid_data            =1;
 dac->shorttest_ena               =0;//Turns on the short test pattern of the JESD interface
 dac->sif4_ena                    =1;                             
 dac->mixer_ena                   =0;      
 dac->mixer_gain                  =0;                   
 dac->nco_ena                     =0;
 dac->twos                        =1;
 dac->sif_reset                   =0;

 dac->coarse_dac                  =15;//Scales the output current in 16 equal steps.
 dac->fifo_error_zeros_data_ena   =0;
 dac->sif_txenable                =1;//TXENABLE

 dac->alarms_mask                 =0xffffffffffff;//Each bit is used to mask an alarm. mask DAC PLL lock alarm

 dac->memin_tempdata              =0x00;//the chip temperature sensor (config7)
 dac->memin_lane_skew             =0x0000;// READ_ONLY !

 dac->qmc_offseta                 =0x0000;
 dac->qmc_offsetb                 =0x0000;
 dac->qmc_offsetc                 =0x0000;
 dac->qmc_offsetd                 =0x0000;
 dac->qmc_gaina                   =0x400;

 dac->fs8                         =0;
 dac->fs4                         =0;
 dac->fs2                         =0;
 dac->fsm4                        =0;
 dac->qmc_gainb                   =0x400;
 dac->qmc_gainc                   =0x400;

 dac->output_delayab_reserved     =0;      
 dac->output_delaycd_reserved     =0;  
 dac->qmc_gaind                   =0x400;

 dac->qmc_phaseab                 =0x000;
 dac->qmc_phasecd                 =0x000;

 dac->phaseoffsetab               =0x0000;
 dac->phaseoffsetcd               =0x0000;

 dac->phaseaddab                  =0x000000000000;
 dac->phaseaddcd                  =0x000000000000;

 dac->vbgr_sleep                  =0;
 dac->biasopamp_sleep             =0;
 dac->tsense_sleep                =0;
 dac->pll_sleep                   =0;//Puts the DAC PLL into sleep mode when asserted.
 dac->clkrecv_sleep               =0;
 dac->daca_sleep                  =0;
 dac->dacb_sleep                  =1;
 dac->dacc_sleep                  =1;
 dac->dacd_sleep                  =0;

 dac->extref_ena                  =0;
 dac->dtest_lane                  =0x00;//0=lane0
 dac->dtest                       =0x00;//SYNC (lane selected by dtest_lane)
 dac->atest                       =0x00;// 001010 : DACA VDDCLK09 (0.9) ~001000(0x8) :8- DAC PLL loop filter voltage (0 to 1V, ~0.5V when locked)

 dac->syncsel_qmoffsetab          =9;
 dac->syncsel_qmoffsetcd          =9;
 dac->syncsel_qmcorrab            =9;
 dac->syncsel_qmcorrcd            =9;  
                        
 dac->syncsel_mixerab             =9; 
 dac->syncsel_mixercd             =9; 
 dac->syncsel_nco                 =0x8; 
 dac->sif_sync                    =0;

 dac->syncsel_dither              =8;
 dac->syncsel_pap                 =0;
 dac->syncsel_fir5a               =8;

 dac->patha_in_sel                =0;
 dac->pathb_in_sel                =1;
 dac->pathc_in_sel                =2;
 dac->pathd_in_sel                =3;
 dac->patha_out_sel               =0;
 dac->pathb_out_sel               =1;
 dac->pathc_out_sel               =2;
 dac->pathd_out_sel               =1;//0 и 1 - I и Q составляющие первого канала
 dac->sleep_cntl                  =0x1ff;

 dac->cdrvser_sysref_mode         =0x000;
 dac->clkjesd_div				  =0x4;//0x4 - 16 ...JESD clock divider = Interpolation * L / M
 dac->dither_ena                  =0x0;//turn on
 dac->dither_mixer_ena            =0x0;
 dac->dither_sra_sel              =0xf;
 dac->dither_zero                 =0;

 dac->pap_dlylen_sel              =0;
 dac->pap_gain                    =1;
 dac->pap_vth                     =0xffff;
 dac->titest_dieid_read_ena       =0;
//---------------TEST DDS-------------------------------------------- 
 dac->sifdac_ena                  =0;//0
 dac->sifdac                      =0x0000;//0x7fff  0x0000
//-------------------------------------------------------------------
 dac->lockdet_adj                 =0x3;
 dac->pll_reset                   =0;
 dac->pll_ndivsync_ena            =0;
 dac->pll_ena                     =1;
 dac->pll_cp                      =0x00;
 dac->pll_n                       =0;//N+1  (REF 96 MHz)
 dac->memin_pll_lfvolt            =0;//READ_ONLY Indicates the loop filter voltage  config49

 dac->pll_m                       =15;//M+1
 dac->pll_p                       =1;//1 - div by 3 prescaler 4608 = 1536 x 3

 dac->pll_vcosel                  =0;//0 - 5GHz VCO ; 1 - 4GHz
 dac->pll_vco                     =13;//(4.6GHz) VCO frequency range control 60 - 4ггц
 dac->pll_vcoitune                =2;//9.8mA
 dac->pll_cp_adj                  =20;//1.55mA  charge pump current (было 1) 0 - 31

 dac->syncb_lvds_lopwrb           =0;
 dac->syncb_lvds_lopwra           =0;
 dac->syncb_lvds_lpsel            =0;
 dac->syncb_lvds_effuse_sel       =0;
 dac->lvds_sleep                  =0;
 dac->lvds_sub_ena                =0;

 dac->serdes_clk_sel              =1;
 dac->serdes_refclk_div           =3;//serdes PLL reference clock divider + 1

 dac->rw_cfgpll                   =0x0000
 								  |(   1<<15)//ENDIVCLK, enables output of a divide-by-5 of PLL clock.
 								  |(   2<<11)//LB, specify loop bandwidth settings.Low loop bandwidth
 								  |(   0<<10)//SLEEPPLL, puts the PLL into sleep state when high.
 								  |(   1<< 9)//VRANGE, select between high and low VCO.
 								  |(0x14<< 1)//(0x14 - 5x) MPY, select PLL multiply factor between 4 and 25  
 								  |(   0<< 0);	

 
 dac->rw_cfgrx0_L                 =0x0000
 								  |(   0<<13)// LOS,  0b000 corresponds to LOS compensation disabled and 0b100 means LOS compensation is enabled.
 								  |(   0<<11)// reserved.
 								  |(   1<< 8)//TERM, select input termination options for serial lanes.AC coupling
 								  |(   0<< 7)// reserved
 								  |(   2<< 5)//RATE, operating rate, select full, half, quarter or eighth rate operation. 
								  |(   2<< 2)//BUSWIDTH, select the parallel interface width 20 bit 
								  |(   0<< 1)//SLEEPRX, powers the receiver down into sleep (fast power up) state when high.
 								  |(   0<< 0);
								  
 dac->rw_cfgrx0_H				  =0x0000
								  |(   0<< 12)//TESTPATT, Enables and selects verification
 								  |(   0<< 8)// reserved
 								  |(   1<< 7)// ENOC, enable samplers offset compensation 
								  |(   0<< 6)// EQHLD, hold the equalizer in its current status. 
								  |(   1<< 3)// EQ, enable and configure the equalizer to compensate the loss
 								  |(   0<< 0);;	 								  
 
 dac->w_cfgrx0                    =0;
 dac->INVPAIR                     =0;
 
 dac->errorcnt_link0              =0;//READ_ONLY config65
 dac->errorcnt_link1              =0;//READ_ONLY config66
 dac->errorcnt_link2              =0;//READ_ONLY
 dac->errorcnt_link3              =0;//READ_ONLY

 dac->lid0                        =0;
 dac->lid1                        =1;
 dac->lid2                        =2;
 dac->lid3                        =3;
 dac->lid4                        =4;
 dac->lid5                        =5;
 dac->lid6                        =6;
 dac->lid7                        =7;
 dac->subclassv                   =1;
 dac->jesdv                       =1;

 dac->link_assign                 =0x0000;
//-----------------TEST-----------------------------
 dac->lane_ena                    =0x3;// 3 |SerDes lane0 enable SerDes lane1 enable
//--------------------------------------------------- 
 dac->jesd_test_seq               =0x00;// verify repeating K.28.5 (188)mixed frequency pattern 
 dac->dual                        =1;//1-Turn on “DUAL DAC” mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 dac->init_state                  =0xf;//было 0х1111
 dac->jesd_reset_n                =1;

 dac->rbd_m1                      =15;// controls the amount of elastic buffers being used in the JESD
 dac->f_m1                        =1;// 1+1 number of octets in the frame
 dac->k_m1                        =15;//0-31This is the number of frames in a multi-frame. The range is 0-31 (LMFC=96/k_m1)
 dac->l_m1                        =1;//1+1 = 2 the number of lanes used by the JESD
 dac->m_m1                        =1;//1+1 the number of converters per link
 dac->s_m1                        =0;

 dac->nprime_m1                   =15;
 dac->hd                          =0;
 dac->scr                         =0;
 dac->n_m1                        =15;
//-----------------------------------------------------
 dac->match_data                  =0x1c;//0x1c
 dac->match_specific              =1;//будет ждать символ  /R/=/K28.0/=0x1C чтобы писать в буфер
 dac->match_ctrl                  =1;
 dac->no_lane_sync                =1;
 dac->jesd_commaalign_ena         =1;
//-----------------------------------------------------
 dac->adjcnt_link0                =0;
 dac->adjdir_link0                =0;
 dac->bid_link0                   =0;
 dac->cf_link0                    =0;
 dac->cs_link0                    =0;
 dac->did_link0                   =0;
 dac->sync_request_ena_link0      =0xff;//bit5 = link configuration error

 dac->disable_err_report_link0    =0;//Assertion means that errors will not be reported on the sync_n output.
 dac->phadj_link0                 =0;
 dac->error_ena_link0             =0xff;//bit5 = link configuration error

 dac->adjcnt_link1                =0;
 dac->adjdir_link1                =0;
 dac->bid_link1                   =0;
 dac->cf_link1                    =0;
 dac->cs_link1                    =0;

 dac->did_link1                   =0;
 dac->sync_request_ena_link1      =0x7;

 dac->disable_err_report_link1    =0;
 dac->phadj_link1                 =0;
 dac->error_ena_link1             =0xff;

 dac->adjcnt_link2                =0;
 dac->adjdir_link2                =0;
 dac->bid_link2                   =0;
 dac->cf_link2                    =0;
 dac->cs_link2                    =0;

 dac->did_link2                   =0;
 dac->sync_request_ena_link2      =0x00;

 dac->disable_err_report_link2    =1;
 dac->phadj_link2                 =0;
 dac->error_ena_link2             =0x00;

 dac->adjcnt_link3                =0;
 dac->adjdir_link3                =0;
 dac->bid_link3                   =0;
 dac->cf_link3                    =0;
 dac->cs_link3                    =0;
 
 dac->did_link3                   =0;
 dac->sync_request_ena_link3      =0x00;

 dac->disable_err_report_link3    =1;   
 dac->phadj_link3                 =0;
 dac->error_ena_link3             =0x00;

 dac->err_cnt_clr_link3           =1;
 
 dac->sysref_mode_link3           =0;
 dac->err_cnt_clr_link2           =1; 
 dac->sysref_mode_link2           =0;
 
 dac->err_cnt_clr_link1           =1; 
 dac->sysref_mode_link1           =0; 
 dac->err_cnt_clr_link0           =1;
 dac->sysref_mode_link0           =0;

 dac->res1                        =0;
 dac->res2                        =0;

 dac->octetpath_sel0            =0;
 dac->octetpath_sel1            =1;
 dac->octetpath_sel2            =2;   
 dac->octetpath_sel3            =3;
 dac->octetpath_sel4            =4;
 dac->octetpath_sel5            =5;
 dac->octetpath_sel6            =6;
 dac->octetpath_sel7            =7;

 dac->syncn_pol                  =0;
 dac->syncncd_sel                =1;
 dac->syncnab_sel                =1;
 //-----------------TEST!!!----------------------------------
 dac->syncn_sel                  =1;//bit0=link0  | 0 - выключен JESD as SYNC_N - always zero!!!
//---------------------------------------------------------
 dac->alarm_l_error0            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags0         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error1            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags1         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error2            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags2         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error3            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags3         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error4            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags4         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error5            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags5         =0x00;//WRITE TO CLEAR

 dac->alarm_l_error6            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags6         =0x00;//WRITE TO CLEAR
 
 dac->alarm_l_error7            =0x00;//WRITE TO CLEAR
 dac->alarm_fifo_flags7         =0x00;//WRITE TO CLEAR

 dac->alarm_sysref_err          =0x00;//WRITE TO CLEAR
 dac->alarm_pap                 =0x00;//WRITE TO CLEAR
 dac->alarm_rw0_pll             =0x00;//WRITE TO CLEAR
 dac->alarm_rw1_pll             =0x00;//WRITE TO CLEAR
 dac->alarm_from_pll            =0x00;//WRITE TO CLEAR

 dac->alarm_from_shorttest      =0x00;
 dac->memin_rw_losdct           =0x00;

 dac->sfrac_coef0_ab            =0;
 dac->sfrac_coef1_ab            =0;
 dac->sfrac_coef2_ab            =0;
 dac->sfrac_coef3_ab            =0;
 dac->sfrac_coef4_ab            =0;
 dac->sfrac_coef5_ab            =0;
 dac->sfrac_coef6_ab            =0;
 dac->sfrac_coef7_ab            =0;
 dac->sfrac_coef8_ab            =0;
 dac->sfrac_coef9_ab            =0;

 dac->sfrac_invgain_ab          =0;  

 dac->lfras_coefsel_a           =0;
 dac->lfras_coefsel_b           =0;

 dac->sfrac_coef0_cd            =0;
 dac->sfrac_coef1_cd            =0;
 dac->sfrac_coef2_cd            =0;
 dac->sfrac_coef3_cd            =0;
 dac->sfrac_coef4_cd            =0; 
 dac->sfrac_coef5_cd            =0;
 dac->sfrac_coef6_cd            =0;
 dac->sfrac_coef7_cd            =0;
 dac->sfrac_coef8_cd            =0;
 dac->sfrac_coef9_cd            =0;
 dac->sfrac_invgain_cd          =0;
 dac->lfras_coefsel_c           =0;
 dac->lfras_coefsel_d           =0;

 dac->memin_efc_autoload_done   =0;// READ_ONLY
 dac->memin_efc_error           =0;// READ_ONLY
 dac->vendorid                  =1;// READ_ONLY
 dac->versionid                 =1;// READ_ONLY
 
}

void defrag_REG_DAC1 (reg_DAC37j82 *dac)
{
	 dac->R[0] = (0x0<<16)+
 			(dac->qmc_offsetab_ena<<15)+
			(dac->qmc_offsetcd_ena<<14)+
			(dac->qmc_corrab_ena  <<13)+
			(dac->qmc_corrcd_ena  <<12)+
			(dac->interp          << 8)+
			(dac->alarm_zeros_txenable_ena<<7)+
			(dac->outsum_ena<<6)+
			(dac->alarm_zeros_jesddata_ena<<5)+
			(dac->alarm_out_ena<<4)+
			(dac->alarm_out_pol<<3)+
			(dac->pap_ena<<2)+
			(dac->inv_sinc_ab_ena<<1)+(dac->inv_sinc_cd_ena<<0);
			
 dac->R[1] = (0x1<<16)+
 			(dac->sfrac_ena_ab<<15)+
			(dac->sfrac_ena_cd<<14)+
			(dac->lfrac_ena_ab<<13)+
			(dac->lfrac_ena_cd<<12)+
			(dac->sfrac_sel_ab<<11)+
			(dac->sfrac_sel_cd<<10)+
			(dac->daca_compliment<<7)+
			(dac->dacb_compliment<<6)+
			(dac->dacc_compliment<<5)+
			(dac->dacd_compliment<<4)+0x3;
			
 dac->R[2] = (0x2<<16)+
 			(dac->dac_bitwidth<<14)+
			(dac->zer_invalid_data<<13)+
			(dac->shorttest_ena<<12)+
			(dac->sif4_ena<<7)+
			(dac->mixer_ena<<6)+
			(dac->mixer_gain<<5)+
			(dac->nco_ena<<4)+
			(dac->twos<<1)+
			(dac->sif_reset<<0);
			
 dac->R[3]  =(0x3<<16)+
			 (dac->coarse_dac<<12)+
			 (0x3<<8)+
			 (dac->fifo_error_zeros_data_ena<<7)+
			 (dac->sif_txenable);
			
 dac->R[4]  = (0x4<<16)+(dac->alarms_mask&0xffff);
 
 dac->R[5]  = (0x5<<16)+(dac->alarms_mask>>16);
 
 dac->R[6]  = (0x6<<16)+(dac->alarms_mask>>32);
 
 dac->R[7]  = (0x7<<16)+(dac->memin_tempdata <<8);
			
 dac->R[8]  = (0x8<<16)+(dac->qmc_offseta);
 
 dac->R[9]  = (0x9<<16)+(dac->qmc_offsetb);
 
 dac->R[10] = (0xa<<16)+(dac->qmc_offsetc);
 
 dac->R[11] = (0xb<<16)+(dac->qmc_offsetd);
 
 dac->R[12] = (0xc<<16)+(dac->qmc_gaina);
 
 dac->R[13] = (0xd<<16)+
 			 (dac->fs8<<15)+
			 (dac->fs4<<14)+
			 (dac->fs2<<13)+
			 (dac->fsm4<<12)+
			 (dac->qmc_gainb);
 
 dac->R[14] = (0xe<<16)+(dac->qmc_gainc);

 dac->R[15] = (0xf<<16)+
 			 (dac->output_delayab<<14)+
 			 (dac->output_delaycd<<12)+
 			 (dac->qmc_gaind);

 dac->R[16] = (0x10<<16)+(dac->qmc_phaseab);

 dac->R[17] = (0x11<<16)+(dac->qmc_phasecd);

 dac->R[18] = (0x12<<16)+(dac->phaseoffsetab);

 dac->R[19] = (0x13<<16)+(dac->phaseoffsetcd);

 dac->R[20] = (0x14<<16)+((dac->phaseaddab>> 0)&0xffff);

 dac->R[21] = (0x15<<16)+((dac->phaseaddab>>16)&0xffff);

 dac->R[22] = (0x16<<16)+((dac->phaseaddab>>32)&0xffff);

 dac->R[23] = (0x17<<16)+((dac->phaseaddcd>> 0)&0xffff);

 dac->R[24] = (0x18<<16)+((dac->phaseaddcd>>16)&0xffff);

 dac->R[25] = (0x19<<16)+((dac->phaseaddcd>>32)&0xffff);

 dac->R[26] = (0x1a<<16)+
 			 (dac->vbgr_sleep<<8)+
 			 (dac->biasopamp_sleep<<7)+
 			 (dac->tsense_sleep<<6)+
 			 (dac->pll_sleep<<5)+
 			 (dac->clkrecv_sleep<<4)+
 			 (dac->daca_sleep<<3)+
 			 (dac->dacb_sleep<<2)+
 			 (dac->dacc_sleep<<1)+
 			 (dac->dacd_sleep<<0);

 dac->R[27] = (0x1b<<16)+
 			 (dac->extref_ena<<15)+
 			 (dac->dtest_lane<<12)+
 			 (dac->dtest<<8)+
 			 (dac->atest<<0);

 dac->R[28] = (0x1c<<16)+(0);//rezerved

 dac->R[29] = (0x1d<<16)+(0);//rezerved

 dac->R[30] = (0x1e<<16)+
 			 (dac->syncsel_qmoffsetab<<12)+
 			 (dac->syncsel_qmoffsetcd<<8)+
 			 (dac->syncsel_qmcorrab  <<4)+
 			 (dac->syncsel_qmcorrcd  <<0);

 dac->R[31] = (0x1f<<16)+
 			 (dac->syncsel_mixerab<<12)+
 			 (dac->syncsel_mixercd<<8)+
 			 (dac->syncsel_nco<<4)+
 			 (dac->sif_sync<<1);

 dac->R[32] = (0x20<<16)+
 			 (dac->syncsel_dither<<12)+
 			 (dac->syncsel_pap<<4)+
 			 (dac->syncsel_fir5a<<0);

 dac->R[33] = (0x21<<16)+(0);//rezerved

 dac->R[34] = (0x22<<16)+
 			 (dac->patha_in_sel<<14)+
 			 (dac->pathb_in_sel<<12)+
 			 (dac->pathc_in_sel<<10)+
 			 (dac->pathd_in_sel<< 8)+
 			 (dac->patha_out_sel<<6)+
 			 (dac->pathb_out_sel<<4)+
 			 (dac->pathc_out_sel<<2)+
 			 (dac->pathd_out_sel<<0);

 dac->R[35] = (0x23<<16)+(dac->sleep_cntl);

 dac->R[36] = (0x24<<16)+(dac->cdrvser_sysref_mode<<4);

 dac->R[37] = (0x25<<16)+(dac->clkjesd_div<<13);
 			 
 dac->R[38] = (0x26<<16)+
 			 (dac->dither_ena<<12)+
 			 (dac->dither_mixer_ena<<8)+
 			 (dac->dither_sra_sel<<4);
			// (dac->dither_zero)&0x01;

 dac->R[39] = (0x27<<16)+(0); //rezerved

 dac->R[40] = (0x28<<16)+(0); //rezerved

 dac->R[41] = (0x29<<16)+(0xFFFF); //rezerved

 dac->R[42] = (0x2a<<16)+(0); //rezerved

 dac->R[43] = (0x2b<<16)+(0); //rezerved

 dac->R[44] = (0x2c<<16)+(0); //rezerved 

 dac->R[45] = (0x2d<<16)+
 			 (dac->pap_dlylen_sel<<3)+
 			 (dac->pap_gain);

 dac->R[46] = (0x2e<<16)+(dac->pap_vth);

 dac->R[47] =(0x2f<<16)+
 			 (dac->titest_dieid_read_ena<<14)+
			 (1<<2)+
 			 (dac->sifdac_ena);

 dac->R[48] = (0x30<<16)+(dac->sifdac);

 dac->R[49] = (0x31<<16)+
 			 (dac->lockdet_adj<<13)+
 			 (dac->pll_reset  <<12)+
 			 (dac->pll_ndivsync_ena<<11)+
 			 (dac->pll_ena<<10)+
 			 (dac->pll_cp<<8)+
 			 (dac->pll_n<<3)+
 			 (dac->memin_pll_lfvolt);

 dac->R[50] =(0x32<<16)+
 			 (dac->pll_m<<8)+
 			 (dac->pll_p<<4);

 dac->R[51] =(0x33<<16)+
 			 (dac->pll_vcosel<<15)+
 			 (dac->pll_vco<<9)+
 			 (dac->pll_vcoitune<<7)+
 			 (dac->pll_cp_adj<<2);

 dac->R[52] = (0x34<<16)+
 			 (dac->syncb_lvds_lopwrb<<15)+
 			 (dac->syncb_lvds_lopwra<<14)+
 			 (dac->syncb_lvds_lpsel <<13)+
 			 (dac->syncb_lvds_effuse_sel<<12)+
 			 (dac->syncb_lvds_sleep<<8)+
 			 (dac->syncb_lvds_sub_ena<<7);

 dac->R[53] = (0x35<<16)+(0);//rezerved

 dac->R[54] = (0x36<<16)+(0);//rezerved

 dac->R[55] = (0x37<<16)+(0);//rezerved
 
 dac->R[56] = (0x38<<16)+(0);//rezerved
 
 dac->R[57] = (0x39<<16)+(0);//rezerved
 
 dac->R[58] = (0x3A<<16)+(0);//rezerved

 dac->R[59] = (0x3b<<16)+
 			 (dac->serdes_clk_sel<<15)+
 			 (dac->serdes_refclk_div<<11);

 dac->R[60] = (0x3c<<16)+(dac->rw_cfgpll);

 dac->R[61] = (0x3d<<16)+(dac->rw_cfgrx0_H)&0x7fff;

 dac->R[62] = (0x3e<<16)+(dac->rw_cfgrx0_L);

 dac->R[63] = (0x3f<<16)+(dac->INVPAIR&0xff);

 dac->R[64] = (0x40<<16)+(0);//rezerv

 dac->R[65] = (65<<16)+(dac->errorcnt_link0);//READ_ONLY

 dac->R[66] = (66<<16)+(dac->errorcnt_link1);//READ_ONLY

 dac->R[67] = (67<<16)+(dac->errorcnt_link2);//READ_ONLY

 dac->R[68] = (68<<16)+(dac->errorcnt_link3);//READ_ONLY

 dac->R[69] = (69<<16)+(0);//rezerv

 dac->R[70] = (70<<16)+
 			 (dac->lid0<<11)+
 			 (dac->lid1<< 6)+
 			 (dac->lid2<< 1);
 
 dac->R[71] = (71<<16)+
 			 (dac->lid3<<11)+
 			 (dac->lid4<< 6)+
 			 (dac->lid5<< 1);

 dac->R[72] = (72<<16)+
 			 (dac->lid6<<11)+
 			 (dac->lid7<< 6)+
 			 (dac->subclassv<<1)+
 			 (dac->jesdv<<0);

 dac->R[73] = (73<<16)+(dac->link_assign);

 dac->R[74] = (74<<16)+
 			 (dac->lane_ena<<8)+
 			 (dac->jesd_test_seq<<6)+
 			 (dac->dual<<5)+
 			 (dac->init_state<<1)+
 			 (dac->jesd_reset_n);

 dac->R[75] = (75<<16)+
 			 (dac->rbd_m1<<8)+
 			 (dac->f_m1);

 dac->R[76] = (76<<16)+
 			 (dac->k_m1<<8)+
 			 (dac->l_m1);

 dac->R[77] = (77<<16)+
 			 (dac->m_m1<<8)+
 			 (dac->s_m1);

 dac->R[78] = (78<<16)+
 			 (dac->nprime_m1<<8)+
 			 (dac->hd<<6)+
 			 (dac->scr<<5)+
 			 (dac->n_m1);

 dac->R[79] = (79<<16)+
 			 (dac->match_data<<8)+
 			 (dac->match_specific<<7)+
 			 (dac->match_ctrl<<6)+
 			 (dac->no_lane_sync<<5)+
 			 (dac->jesd_commaalign_ena);

 dac->R[80] = (80<<16)+
 			 (dac->adjcnt_link0<<12)+
 			 (dac->adjdir_link0<<11)+
 			 (dac->bid_link0   << 7)+
 			 (dac->cf_link0    << 2)+
 			 (dac->cs_link0    << 0);

 dac->R[81] = (81<<16)+
 			 (dac->did_link0<<8)+
 			 (dac->sync_request_ena_link0);

 dac->R[82] = (82<<16)+
 			 (dac->disable_err_report_link0<<9)+
 			 (dac->phadj_link0<<8)+
 			 (dac->error_ena_link0);

 dac->R[83] = (83<<16)+
 			 (dac->adjcnt_link1<<12)+
 			 (dac->adjdir_link1<<11)+
 			 (dac->bid_link1   << 7)+
 			 (dac->cf_link1    << 2)+
 			 (dac->cs_link1    << 0);

 dac->R[84] = (84<<16)+
 			 (dac->did_link1<<8)+
 			 (dac->sync_request_ena_link1);

 dac->R[85] = (85<<16)+
 			 (dac->disable_err_report_link1<<9)+
 			 (dac->phadj_link1<<8)+
 			 (dac->error_ena_link1);

 dac->R[86] = (86<<16)+
 			 (dac->adjcnt_link2<<12)+
 			 (dac->adjdir_link2<<11)+
 			 (dac->bid_link2   << 7)+
 			 (dac->cf_link2    << 2)+
 			 (dac->cs_link2    << 0);

 dac->R[87] = (87<<16)+
 			 (dac->did_link2<<8)+
 			 (dac->sync_request_ena_link2);

 dac->R[88] = (88<<16)+
 			 (dac->disable_err_report_link2<<9)+
 			 (dac->phadj_link2<<8)+
 			 (dac->error_ena_link2);

 dac->R[89] = (89<<16)+
 			 (dac->adjcnt_link3<<12)+
 			 (dac->adjdir_link3<<11)+
 			 (dac->bid_link3   << 7)+
 			 (dac->cf_link3    << 2)+
 			 (dac->cs_link3    << 0);

 dac->R[90] = (90<<16)+
 			 (dac->did_link3<<8)+
 			 (dac->sync_request_ena_link3);

 dac->R[91] = (91<<16)+
 			 (dac->disable_err_report_link3<<9)+
 			 (dac->phadj_link3<<8)+
 			 (dac->error_ena_link3);

 dac->R[92] = (92<<16)+
 			 (dac->err_cnt_clr_link3<<15)+
 			 (dac->sysref_mode_link3<<12)+
 			 (dac->err_cnt_clr_link2<<11)+
 			 (dac->sysref_mode_link2<< 8)+
 			 (dac->err_cnt_clr_link1<< 7)+
 			 (dac->sysref_mode_link1<< 4)+
 			 (dac->err_cnt_clr_link0<< 3)+
 			 (dac->sysref_mode_link0<< 0);

 dac->R[93] = (0x5d<<16)+(0);//rezerved

			 
 dac->R[94] = (94<<16)+
 			 (dac->res1<<8)+
 			 (dac->res2<<0);
			 

 dac->R[95] = (95<<16)+
 			 (dac->octetpath_sel0<<12)+
 			 (dac->octetpath_sel1<< 8)+
 			 (dac->octetpath_sel2<< 4)+
 			 (dac->octetpath_sel3<< 0);

 dac->R[96] = (96<<16)+
 			 (dac->octetpath_sel4<<12)+
 			 (dac->octetpath_sel5<< 8)+
 			 (dac->octetpath_sel6<< 4)+
 			 (dac->octetpath_sel7<< 0);

 dac->R[97] = (97<<16)+
 			 (dac->syncn_pol<<15)+
 			 (dac->syncncd_sel<<8)+
 			 (dac->syncnab_sel<<4)+
 			 (dac->syncn_sel<<0);

 dac->R[98] = (98<<16)+(0);//rezerv

 dac->R[99] = (99<<16)+(0);//rezerv
//----------------------------?????????????----------------
 dac->R[100] = (100<<16)+
 			  (dac->alarm_i_error0<<8)+
 			  (dac->alarm_fifo_flags0);//write to clear

 dac->R[101] = (101<<16)+
 			  (dac->alarm_i_error1<<8)+
 			  (dac->alarm_fifo_flags1);//write to clear

 dac->R[102] = (102<<16)+
 			  (dac->alarm_i_error2<<8)+
 			  (dac->alarm_fifo_flags2);//write to clear

 dac->R[103] = (103<<16)+
 			  (dac->alarm_i_error3<<8)+
 			  (dac->alarm_fifo_flags3);//write to clear

 dac->R[104] = (104<<16)+
 			  (dac->alarm_i_error4<<8)+
 			  (dac->alarm_fifo_flags4);//write to clear

 dac->R[105] = (105<<16)+
 			  (dac->alarm_i_error5<<8)+
 			  (dac->alarm_fifo_flags5);//write to clear

 dac->R[106] = (106<<16)+
 			  (dac->alarm_i_error6<<8)+
 			  (dac->alarm_fifo_flags6);//write to clear

 dac->R[107] = (107<<16)+
 			  (dac->alarm_i_error7<<8)+
 			  (dac->alarm_fifo_flags7);//write to clear ?

//----------------------------?????????????----------------

 dac->R[108] = (108<<16)+
 			  (dac->alarm_sysref_err<<12)+
 			  (dac->alarm_pap<<8)+
 			  (dac->alarm_rw0_pll<<3)+
 			  (dac->alarm_rw1_pll<<2)+
 			  (dac->alarm_from_pll);

 dac->R[109] =(109<<16)+
 			  (dac->alarm_from_shorttest<<8)+
 			  (dac->memin_rw_losdct);

 dac->R[110] =(110<<16)+
 			  (dac->sfrac_coef0_ab<<14)+
 			  (dac->sfrac_coef1_ab<< 9)+
 			  (dac->sfrac_coef2_ab<< 1);

 dac->R[111] =(111<<16)+(dac->sfrac_coef3_ab);

 dac->R[112] =(112<<16)+(dac->sfrac_coef4_ab);

 dac->R[113] =(113<<16)+
 			  (dac->sfrac_coef4_ab>>16)&0xffff+
 			  (dac->sfrac_coef5_ab);

 dac->R[114] = (114<<16)+(dac->sfrac_coef6_ab);

 dac->R[115] = (115<<16)+
 			  (dac->sfrac_coef7_ab<<9)+
 			  (dac->sfrac_coef8_ab<<4)+
 			  (dac->sfrac_coef9_ab<<2);

 dac->R[116] = (116<<16)+(dac->sfrac_invgain_ab);

 dac->R[117] = (117<<16)+
 			  (dac->sfrac_invgain_ab>>16)+
 			  (dac->lfras_coefsel_a<<3)+
 			  (dac->lfras_coefsel_b);

 dac->R[118] = (118<<16)+
 			  (dac->sfrac_coef0_cd<<14)+
 			  (dac->sfrac_coef1_cd<< 9)+
 			  (dac->sfrac_coef2_cd<< 1);

 dac->R[119] = (119<<16)+(dac->sfrac_coef3_cd);

 dac->R[120] = (120<<16)+(dac->sfrac_coef4_cd);

 dac->R[121] = (121<<16)+
 			  ((dac->sfrac_coef4_cd>>16)<<13)+
 			   (dac->sfrac_coef5_cd);

 dac->R[122] = (122<<16)+(dac->sfrac_coef6_cd);

 dac->R[123] = (123<<16)+
 			  (dac->sfrac_coef7_cd<<9)+
 			  (dac->sfrac_coef8_cd<<4)+
 			  (dac->sfrac_coef9_cd<<2);

 dac->R[124] = (124<<16)+(dac->sfrac_invgain_cd);

 dac->R[125] =  (125<<16)+
 			  ((dac->sfrac_invgain_cd>>16)<<12)+
 			   (dac->lfrac_coefsel_c<<3)+
 			   (dac->lfrac_coefsel_d);

 dac->R[126] = (126<<16)+(0);//rezerved
		 
 		 
}

//----------------DAC1-------------------------------

void spisend_dac1 (u32 d) //24бита
{  
   CS_DAC1_0;
   spisend8(((d >> 16)&0x7f));//устанавливаем бит записи 
   spisend8( (d >>  8)&0xff);
   spisend8( (d)      &0xff);

   CS_DAC1_1;
/*   
   Transf("config[");
   nu_out("",(d >> 16)&0x7f);
   x32_out("]:",d&0xffff);  
 */  
}

	
void write_reg_dac1 (u8 adr,u16 d) //24бита
{  
   CS_DAC1_0; 
   spisend8(((adr)    &0x7f));//устанавливаем бит записи 
   spisend8( (d >>  8)&0xff);
   spisend8( (d)      &0xff);
 
   CS_DAC1_1;
   Delay(1);
}

u16 spiread_dac1 (unsigned short adr) //24бита
{
   char dataH; 
   char dataL;
   
   CS_DAC1_0;
   spisend8(((adr)&0xff)|0x80);// устанавливаем бит чтения
   dataH=spisend8(0);
   dataL=spisend8(0);
  
 //  Delay(1);  
   CS_DAC1_1;
   Delay(1);

   return ((dataH<<8)+(dataL)); 
}

u32 alarm_dac1_serdes_pll (u8 clr)
{
	u16 data;

	if (clr==1) spisend_dac1((108<<16)&0xfffff3);

	data=spiread_dac1(108);
	data=(data>>2)&0x3;
	return data;
}

void dac1_info (u32 a)
{
	u16 data;
	u16 data_bool;

Transf ("-------\r\n");	

Delay (100);

  data=spiread_dac1(109);
  data_bool=(data>>0)&0xff;
  x32_out("loss of signal detect:",data_bool);
  x32_out("alarms from lanes    :",data>>8);

  data=spiread_dac1(49);
  data_bool=(data>>0)&0x7; //
  x32_out("memin_pll_lfvolt(3-4):",data_bool);
  
  data=spiread_dac1(108);
  data_bool=(data>>0)&0x01;
  x32_out("alarm_from_pll       :",data_bool);
  data_bool=(data>>2)&0x01;
  x32_out("alarm_rw1_pll        :",data_bool);
  data_bool=(data>>3)&0x01;
  x32_out("alarm_rw0_pll        :",data_bool);
  data_bool=(data>>12)&0x03;
  x32_out("alarm_sysref_ err    :",data_bool);
  
  data=spiread_dac1(100);
  data_bool=(data>>8)&0xff; //
  x32_out("alarm_l_error(0)     :",data_bool);
  data_bool=(data>>0)&0x0f; //
  x32_out("alarm_fifo_flags(0)  :",data_bool);
  
  data=spiread_dac1(101);
  data_bool=(data>>8)&0xff; //
  x32_out("alarm_l_error(1)     :",data_bool);
  data_bool=(data>>0)&0x0f; //
  x32_out("alarm_fifo_flags(1)  :",data_bool);
  
  data=spiread_dac1(65);
  data_bool=(data>>0); //
  x32_out("error count for link0:",data_bool);
 
 
  data=spiread_dac1(66);
  data_bool=(data>>0); //
  x32_out("error count for link1:",data_bool);
 
  data=spiread_dac1(70);
  data_bool=(data>>0)&0x7; //
  x32_out("JESD ID for lain0:",(data_bool>>11)&0x1f);
 /* 
  data=spiread_dac1(70);
  data=(data>>0)&0x7; //
  x32_out("JESD ID for lain1:",(data>>6)&0x1f);
 */ 
  data=spiread_dac1(127);
  data_bool=(data>>0)&0x7; //
  x32_out("versionid        :",(data_bool>>0)&0x7);
  x32_out("vendorid         :",(data_bool>>3)&0x7);
  x32_out("memin_efc_ error :",(data_bool>>10)&0x1f);
  
 data=spiread_dac1(100);
 x32_out("alarm(100):",data);  
 data=spiread_dac1(101);
 x32_out("alarm(101):",data); 
 data=spiread_dac1(102);
 x32_out("alarm(102):",data); 
 data=spiread_dac1(103);
 x32_out("alarm(103):",data); 
 data=spiread_dac1(104);
 x32_out("alarm(104):",data); 
 data=spiread_dac1(105);
 x32_out("alarm(105):",data); 
 data=spiread_dac1(106);
 x32_out("alarm(106):",data); 
 data=spiread_dac1(107);
 x32_out("alarm(107):",data); 
 data=spiread_dac1(108);
 x32_out("alarm(108):",data); 
 data=spiread_dac1(109);
 x32_out("alarm(109):",data); 
  
  	//clear the alarms
write_reg_dac1 (109,0x0000);//clear reg_DAC37j82
write_reg_dac1 (108,0x0000);//clear reg_DAC37j82
write_reg_dac1 (107,0x0000);//clear reg_DAC37j82
write_reg_dac1 (106,0x0000);//clear reg_DAC37j82
write_reg_dac1 (105,0x0000);//clear reg_DAC37j82
write_reg_dac1 (104,0x0000);//clear reg_DAC37j82
write_reg_dac1 (103,0x0000);//clear reg_DAC37j82
write_reg_dac1 (102,0x0000);//clear reg_DAC37j82
write_reg_dac1 (101,0x0000);//clear reg_DAC37j82
write_reg_dac1 (100,0x0000);//clear reg_DAC37j82

spisend_dac1(dac1.R[92]);//clear reg_DAC37j82 err_cnt_ clr_link0 
    
}

void dac1_write (char r)
{
	int i=0;
	u16 data;
	u16 data_bool;
	
	IO("~0 DAC1_PWRDN:0;");
	
	for (i=0;i<128;i++) dac1.R[i]=0;
	
  //	RESETB_DAC_0;
  IO("~0 DAC1_RST:0;");
    data=spiread_dac1(49);
  //	RESETB_DAC_1;	
  IO("~0 DAC1_RST:1;");

  INIT_REG_DAC1(&dac1);
  defrag_REG_DAC1 (&dac1);
 
/* 
  	dac1.sif_txenable=0;//TXdesABLE
    defrag_REG_DAC1 (&dac1);
    spisend_dac1(dac1.R[3]);
*/ 

  x32_out("dac1.R[74]:",dac1.R[74]);
 
  dac1.cdrvser_sysref_mode  =0x0000;//3
  defrag_REG_DAC1 (&dac1);  
  spisend_dac1(dac1.R[36]);
  
  dac1.sysref_mode_link0 = 0x0000;//5
  defrag_REG_DAC1 (&dac1);
  spisend_dac1(dac1.R[92]);
  
  dac1.jesd_reset_n  =0;
  dac1.init_state    =0x000f;
  defrag_REG_DAC1 (&dac1);
  spisend_dac1(dac1.R[74]);
  
Transf ("--------\r\n"); 

  spisend_dac1(dac1.R[26]);
  spisend_dac1(dac1.R[49]);
  spisend_dac1(dac1.R[50]);
  spisend_dac1(dac1.R[51]);
  spisend_dac1(dac1.R[2]);//spi 4 enb
//*** delay. 
  for (i=0;i<20;i++)   data=spiread_dac1(49);
//***   
  data=spiread_dac1(49);
  data=(data>>0)&0x7; //
  x32_out("memin_pll_lfvolt(3-4):",data);
  if ((data<3)||(data>4)) Transf("PLL:fail\r\n"); else Transf("PLL:Ok\r\n");
  write_reg_dac1 (108,0x0000);//clear reg_DAC37j82
  data=spiread_dac1(108);
  data_bool=(data>>0)&0x01;
  x32_out("alarm_from_pll       :",data_bool);  
//-------------------------------------------
  spisend_dac1(dac1.R[59]);
  spisend_dac1(dac1.R[60]);
  spisend_dac1(dac1.R[61]);
  spisend_dac1(dac1.R[62]);
  spisend_dac1(dac1.R[63]);
  spisend_dac1(dac1.R[70]);
  spisend_dac1(dac1.R[71]);
  spisend_dac1(dac1.R[72]);
  spisend_dac1(dac1.R[73]);
  spisend_dac1(dac1.R[74]);
  spisend_dac1(dac1.R[96]);
  
  write_reg_dac1 (108,0x0000);//clear reg_DAC37j82
  
 //*** delay. 
  for (i=0;i<100;i++)   data=spiread_dac1(108);
//***  


 /*
  data_bool=(data>>2)&0x01;//(alarm for lanes 4 through 7)
  x32_out("alarm_rw1_pll   :",data_bool);
*/  
  data_bool=(data>>3)&0x01;//pll (alarm for lanes 0 through 3) 
  x32_out("alarm_rw0_pll        :",data_bool); 
  Transf ("\r\n"); 

  spisend_dac1(dac1.R[3]);
  spisend_dac1(dac1.R[37]);
 
  spisend_dac1(dac1.R[75]);
  spisend_dac1(dac1.R[76]);
  spisend_dac1(dac1.R[77]);
  spisend_dac1(dac1.R[79]);
  spisend_dac1(dac1.R[80]);
  spisend_dac1(dac1.R[81]);
  spisend_dac1(dac1.R[82]);
  spisend_dac1(dac1.R[83]);
  spisend_dac1(dac1.R[84]);
  spisend_dac1(dac1.R[85]);
  spisend_dac1(dac1.R[86]);
  spisend_dac1(dac1.R[87]);
  spisend_dac1(dac1.R[88]);
  spisend_dac1(dac1.R[89]);
  spisend_dac1(dac1.R[90]);
  spisend_dac1(dac1.R[91]);
  spisend_dac1(dac1.R[92]);
  spisend_dac1(dac1.R[93]);
  spisend_dac1(dac1.R[94]);
  spisend_dac1(dac1.R[95]);
  spisend_dac1(dac1.R[96]);
  spisend_dac1(dac1.R[97]);
  
  //---------------------------
  
  
  spisend_dac1(dac1.R[0]);//interp
  spisend_dac1(dac1.R[1]);
  spisend_dac1(dac1.R[2]);//spi 4 enb
  
  spisend_dac1(dac1.R[8]);
  spisend_dac1(dac1.R[9]);
  spisend_dac1(dac1.R[10]);
  spisend_dac1(dac1.R[11]);
  spisend_dac1(dac1.R[12]);
  spisend_dac1(dac1.R[13]);
  spisend_dac1(dac1.R[14]);
  spisend_dac1(dac1.R[15]);
  spisend_dac1(dac1.R[16]);
  spisend_dac1(dac1.R[17]);
  spisend_dac1(dac1.R[18]);
  spisend_dac1(dac1.R[19]);
  spisend_dac1(dac1.R[20]);
  spisend_dac1(dac1.R[21]);
  spisend_dac1(dac1.R[22]);
  spisend_dac1(dac1.R[23]);
  spisend_dac1(dac1.R[24]);
  spisend_dac1(dac1.R[25]);
  
  spisend_dac1(dac1.R[30]);
  spisend_dac1(dac1.R[31]);
  spisend_dac1(dac1.R[32]);
  
  spisend_dac1(dac1.R[34]);
  spisend_dac1(dac1.R[35]);
  spisend_dac1(dac1.R[38]);
  spisend_dac1(dac1.R[40]);
  spisend_dac1(dac1.R[41]);
  spisend_dac1(dac1.R[45]);
  spisend_dac1(dac1.R[46]);
  spisend_dac1(dac1.R[47]);
  spisend_dac1(dac1.R[48]);
  spisend_dac1(dac1.R[52]);
  
  
  
  spisend_dac1(dac1.R[4]);
  spisend_dac1(dac1.R[5]);
  spisend_dac1(dac1.R[6]);
  
  spisend_dac1(dac1.R[3]);
  
  for (i=7;i<26;i++) spisend_dac1(dac1.R[i]);
  
//  Delay(1);
  dac1.cdrvser_sysref_mode  =0x003;//3
  defrag_REG_DAC1 (&dac1);  
  spisend_dac1(dac1.R[36]);
  
  dac1.sysref_mode_link0 = 0x005;//5
  defrag_REG_DAC1 (&dac1);
  spisend_dac1(dac1.R[92]);

  dac1.jesd_reset_n  =1;//1 TEST!!! 
  defrag_REG_DAC1 (&dac1);
  spisend_dac1(dac1.R[74]); 
  
  dac1.init_state    =0x0000;
  defrag_REG_DAC1 (&dac1);
  spisend_dac1(dac1.R[74]); 

 //.. 

 data=spiread_dac1(108);
 data_bool=(data>>12)&0x0f;
 Transf("-------\r\n");
 x32_out("(10)alarm_sysref_err :",data_bool);

 Transf("-------\r\n");  
 data=spiread_dac1(100);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(100):",data);  
 data=spiread_dac1(101);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(101):",data); 
 data=spiread_dac1(102);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(102):",data); 
 data=spiread_dac1(103);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(103):",data); 
 data=spiread_dac1(104);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(104):",data); 
 data=spiread_dac1(105);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(105):",data); 
 data=spiread_dac1(106);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(106):",data); 
 data=spiread_dac1(107);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(107):",data); 
 data=spiread_dac1(108);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(108):",data); 
 data=spiread_dac1(109);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(109):",data); 
   

   
 dac1.sif_txenable=1;//TXENABLE
 defrag_REG_DAC1 (&dac1);
 spisend_dac1(dac1.R[3]);
   
}

void dac1_read_reg (void)
{
	int i=0;
	u16 data;
for (i=0;i<128;i++) 
{
	data=spiread_dac1(i);
	Transf("config[");
    nu_out("",i);
    x32_out("]:",data&0xffff);  
	}
	
}

void DAC1_dsp_init(u8 a)
{
if (a==1)
{
dac1.mixer_ena        =1;      
dac1.mixer_gain       =1;                   
dac1.nco_ena          =1;
dac1.phaseoffsetab    =0x0000;// Phase offset for NCO in DACAB path
dac1.phaseoffsetcd    =0x0000;
dac1.phaseaddab       =76965813944320;// NCO Frequency adjust word for DACAB path. 420 MHz
dac1.phaseaddcd       =76965813944320;	


} else
{
dac1.mixer_ena        =0;      
dac1.mixer_gain       =0;                   
dac1.nco_ena          =0;
dac1.phaseoffsetab    =0x0000;// Phase offset for NCO in DACAB path
dac1.phaseoffsetcd    =0x0000;
dac1.phaseaddab       =0x000000000000;// NCO Frequency adjust word for DACAB path. 420 MHz
dac1.phaseaddcd       =0x000000000000;	
}

dac1.syncsel_mixerab  =9; 
dac1.syncsel_mixercd  =9; 
dac1.syncsel_nco      =0x8; 
dac1.sif_sync         =0;

dac1.fs8              =0;
dac1.fs4              =0;
dac1.fs2              =0;
dac1.fsm4             =0;
dac1.qmc_gainb        =0x400;
dac1.qmc_gainc        =0x400;

dac1.qmc_offseta      =0x0000;
dac1.qmc_offsetb      =0x0000;
dac1.qmc_offsetc      =0x0000;
dac1.qmc_offsetd      =0x0000;
dac1.qmc_gaina        =0x400;

dac1.dither_ena        =0x0;
dac1.dither_mixer_ena  =0x0;
dac1.dither_sra_sel    =0x0;
dac1.dither_zero       =0;

dac1.inv_sinc_ab_ena   =0;
dac1.inv_sinc_cd_ena   =0;
 
defrag_REG_DAC1 (&dac1); 
 
 spisend_dac1(dac1.R[0]);
 spisend_dac1(dac1.R[2]);
 spisend_dac1(dac1.R[18]);
 spisend_dac1(dac1.R[19]);
 spisend_dac1(dac1.R[20]);
 spisend_dac1(dac1.R[21]);
 spisend_dac1(dac1.R[22]);
 spisend_dac1(dac1.R[23]);
 spisend_dac1(dac1.R[24]);
 spisend_dac1(dac1.R[25]);
 spisend_dac1(dac1.R[13]);
 spisend_dac1(dac1.R[14]);
 spisend_dac1(dac1.R[8]);
 spisend_dac1(dac1.R[9]);
 spisend_dac1(dac1.R[10]);
 spisend_dac1(dac1.R[11]);
 spisend_dac1(dac1.R[12]);
 spisend_dac1(dac1.R[38]);
 
 spisend_dac1(dac1.R[31]);
 
 dac1.sif_sync =1;
 defrag_REG_DAC1 (&dac1); 
 spisend_dac1(dac1.R[31]);
 
 
}

void DAC1_coarse_dac (u8 a)
{
	dac1.coarse_dac  =a;
	dac1.sif_sync    =1;
	defrag_REG_DAC1 (&dac1); 
	spisend_dac1(dac1.R[3]);	
}
void DAC1_mixer_gain (u8 a)
{
	dac1.mixer_gain  =a;
	dac1.sif_sync    =1;
	defrag_REG_DAC1 (&dac1); 
	spisend_dac1(dac1.R[2]);	
}
void DAC1_QMC_gain (u16 a)
{
	dac1.qmc_gaina  =a;
	dac1.qmc_gainb  =a;
	dac1.sif_sync    =1;
	defrag_REG_DAC1 (&dac1); 
	spisend_dac1(dac1.R[12]);
	spisend_dac1(dac1.R[13]);	
}
//----------------DAC2-------------------------------

void spisend_dac2 (u32 d) //24бита
{  
   CS_DAC2_0;
   spisend8(((d >> 16)&0x7f));//устанавливаем бит записи 
   spisend8( (d >>  8)&0xff);
   spisend8( (d)      &0xff);

   CS_DAC2_1;
/*   
   Transf("config[");
   nu_out("",(d >> 16)&0x7f);
   x32_out("]:",d&0xffff);  
 */  
}

	
void write_reg_dac2 (u8 adr,u16 d) //24бита
{  
   CS_DAC2_0; 
   spisend8(((adr)    &0x7f));//устанавливаем бит записи 
   spisend8( (d >>  8)&0xff);
   spisend8( (d)      &0xff);
 
   CS_DAC2_1;
   Delay(1);
}

u16 spiread_dac2 (unsigned short adr) //24бита
{
   char dataH; 
   char dataL;
   
   CS_DAC2_0;
   spisend8(((adr)&0xff)|0x80);// устанавливаем бит чтения
   dataH=spisend8(0);
   dataL=spisend8(0);
  
 //  Delay(1);  
   CS_DAC2_1;
   Delay(1);

   return ((dataH<<8)+(dataL)); 
}

u32 alarm_dac2_serdes_pll (u8 clr)
{
	u16 data;

	if (clr==1) spisend_dac2((108<<16)&0xfffff3);

	data=spiread_dac2(108);
	data=(data>>2)&0x3;
	return data;
}

void dac2_info (u32 a)
{
	u16 data;
	u16 data_bool;

Transf ("-------\r\n");	

Delay (100);

  data=spiread_dac2(109);
  data_bool=(data>>0)&0xff;
  x32_out("loss of signal detect:",data_bool);
  x32_out("alarms from lanes    :",data>>8);

  data=spiread_dac2(49);
  data_bool=(data>>0)&0x7; //
  x32_out("memin_pll_lfvolt(3-4):",data_bool);
  
  data=spiread_dac2(108);
  data_bool=(data>>0)&0x01;
  x32_out("alarm_from_pll       :",data_bool);
  data_bool=(data>>2)&0x01;
  x32_out("alarm_rw1_pll        :",data_bool);
  data_bool=(data>>3)&0x01;
  x32_out("alarm_rw0_pll        :",data_bool);
  data_bool=(data>>12)&0x03;
  x32_out("alarm_sysref_ err    :",data_bool);
  
  data=spiread_dac2(100);
  data_bool=(data>>8)&0xff; //
  x32_out("alarm_l_error(0)     :",data_bool);
  data_bool=(data>>0)&0x0f; //
  x32_out("alarm_fifo_flags(0)  :",data_bool);
  
  data=spiread_dac2(101);
  data_bool=(data>>8)&0xff; //
  x32_out("alarm_l_error(1)     :",data_bool);
  data_bool=(data>>0)&0x0f; //
  x32_out("alarm_fifo_flags(1)  :",data_bool);
  
  data=spiread_dac2(65);
  data_bool=(data>>0); //
  x32_out("error count for link0:",data_bool);
 
 
  data=spiread_dac2(66);
  data_bool=(data>>0); //
  x32_out("error count for link1:",data_bool);
 
  data=spiread_dac2(70);
  data_bool=(data>>0)&0x7; //
  x32_out("JESD ID for lain0:",(data_bool>>11)&0x1f);
 /* 
  data=spiread_dac1(70);
  data=(data>>0)&0x7; //
  x32_out("JESD ID for lain1:",(data>>6)&0x1f);
 */ 
  data=spiread_dac2(127);
  data_bool=(data>>0)&0x7; //
  x32_out("versionid        :",(data_bool>>0)&0x7);
  x32_out("vendorid         :",(data_bool>>3)&0x7);
  x32_out("memin_efc_ error :",(data_bool>>10)&0x1f);
  
 data=spiread_dac2(100);
 x32_out("alarm(100):",data);  
 data=spiread_dac2(101);
 x32_out("alarm(101):",data); 
 data=spiread_dac2(102);
 x32_out("alarm(102):",data); 
 data=spiread_dac2(103);
 x32_out("alarm(103):",data); 
 data=spiread_dac2(104);
 x32_out("alarm(104):",data); 
 data=spiread_dac2(105);
 x32_out("alarm(105):",data); 
 data=spiread_dac2(106);
 x32_out("alarm(106):",data); 
 data=spiread_dac2(107);
 x32_out("alarm(107):",data); 
 data=spiread_dac2(108);
 x32_out("alarm(108):",data); 
 data=spiread_dac2(109);
 x32_out("alarm(109):",data); 
  
  	//clear the alarms
write_reg_dac2 (109,0x0000);//clear reg_DAC37j82
write_reg_dac2 (108,0x0000);//clear reg_DAC37j82
write_reg_dac2 (107,0x0000);//clear reg_DAC37j82
write_reg_dac2 (106,0x0000);//clear reg_DAC37j82
write_reg_dac2 (105,0x0000);//clear reg_DAC37j82
write_reg_dac2 (104,0x0000);//clear reg_DAC37j82
write_reg_dac2 (103,0x0000);//clear reg_DAC37j82
write_reg_dac2 (102,0x0000);//clear reg_DAC37j82
write_reg_dac2 (101,0x0000);//clear reg_DAC37j82
write_reg_dac2 (100,0x0000);//clear reg_DAC37j82

spisend_dac2(dac2.R[92]);//clear reg_DAC37j82 err_cnt_ clr_link0 
    
}

void dac2_write (char r)
{
	int i=0;
	u16 data;
	u16 data_bool;
	
	IO("~0 DAC2_PWRDN:0;");
	
	for (i=0;i<128;i++) dac2.R[i]=0;
	
  //	RESETB_DAC_0;
  IO("~0 DAC2_RST:0;");
    data=spiread_dac2(49);
  //	RESETB_DAC_1;	
  IO("~0 DAC2_RST:1;");
  
  INIT_REG_DAC1(&dac2);
  defrag_REG_DAC1 (&dac2);
 
/* 
  	dac1.sif_txenable=0;//TXdesABLE
    defrag_REG_DAC1 (&dac1);
    spisend_dac1(dac1.R[3]);
*/ 

  x32_out("dac2.R[74]:",dac2.R[74]);
 
  dac2.cdrvser_sysref_mode  =0x0000;//3
  defrag_REG_DAC1 (&dac2);  
  spisend_dac2(dac2.R[36]);
  
  dac2.sysref_mode_link0 = 0x0000;//5
  defrag_REG_DAC1 (&dac2);
  spisend_dac2(dac2.R[92]);
  
  dac2.jesd_reset_n  =0;
  dac2.init_state    =0x000f;
  defrag_REG_DAC1 (&dac2);
  spisend_dac2(dac2.R[74]);
  
Transf ("--------\r\n"); 

  spisend_dac2(dac2.R[26]);
  spisend_dac2(dac2.R[49]);
  spisend_dac2(dac2.R[50]);
  spisend_dac2(dac2.R[51]);
  spisend_dac2(dac2.R[2]);//spi 4 enb
//*** delay. 
  for (i=0;i<20;i++)   data=spiread_dac2(49);
//***   
  data=spiread_dac2(49);
  data=(data>>0)&0x7; //
  x32_out("memin_pll_lfvolt(3-4):",data);
  if ((data<3)||(data>4)) Transf("PLL:fail\r\n"); else Transf("PLL:Ok\r\n");
  write_reg_dac2 (108,0x0000);//clear reg_DAC37j82
  data=spiread_dac2(108);
  data_bool=(data>>0)&0x01;
  x32_out("alarm_from_pll       :",data_bool);  
//-------------------------------------------
  spisend_dac2(dac2.R[59]);
  spisend_dac2(dac2.R[60]);
  spisend_dac2(dac2.R[61]);
  spisend_dac2(dac2.R[62]);
  spisend_dac2(dac2.R[63]);
  spisend_dac2(dac2.R[70]);
  spisend_dac2(dac2.R[71]);
  spisend_dac2(dac2.R[72]);
  spisend_dac2(dac2.R[73]);
  spisend_dac2(dac2.R[74]);
  spisend_dac2(dac2.R[96]);
  
  write_reg_dac2 (108,0x0000);//clear reg_DAC37j82
  
 //*** delay. 
  for (i=0;i<100;i++)   data=spiread_dac2(108);
//***  


 /*
  data_bool=(data>>2)&0x01;//(alarm for lanes 4 through 7)
  x32_out("alarm_rw1_pll   :",data_bool);
*/  
  data_bool=(data>>3)&0x01;//pll (alarm for lanes 0 through 3) 
  x32_out("alarm_rw0_pll        :",data_bool); 
  Transf ("\r\n"); 

  spisend_dac2(dac2.R[3]);
  spisend_dac2(dac2.R[37]);
 
  spisend_dac2(dac2.R[75]);
  spisend_dac2(dac2.R[76]);
  spisend_dac2(dac2.R[77]);
  spisend_dac2(dac2.R[79]);
  spisend_dac2(dac2.R[80]);
  spisend_dac2(dac2.R[81]);
  spisend_dac2(dac2.R[82]);
  spisend_dac2(dac2.R[83]);
  spisend_dac2(dac2.R[84]);
  spisend_dac2(dac2.R[85]);
  spisend_dac2(dac2.R[86]);
  spisend_dac2(dac2.R[87]);
  spisend_dac2(dac2.R[88]);
  spisend_dac2(dac2.R[89]);
  spisend_dac2(dac2.R[90]);
  spisend_dac2(dac2.R[91]);
  spisend_dac2(dac2.R[92]);
  spisend_dac2(dac2.R[93]);
  spisend_dac2(dac2.R[94]);
  spisend_dac2(dac2.R[95]);
  spisend_dac2(dac2.R[96]);
  spisend_dac2(dac2.R[97]);
  
  //---------------------------
  
  
  spisend_dac2(dac2.R[0]);//interp
  spisend_dac2(dac2.R[1]);
  spisend_dac2(dac2.R[2]);//spi 4 enb
  
  spisend_dac2(dac2.R[8]);
  spisend_dac2(dac2.R[9]);
  spisend_dac2(dac2.R[10]);
  spisend_dac2(dac2.R[11]);
  spisend_dac2(dac2.R[12]);
  spisend_dac2(dac2.R[13]);
  spisend_dac2(dac2.R[14]);
  spisend_dac2(dac2.R[15]);
  spisend_dac2(dac2.R[16]);
  spisend_dac2(dac2.R[17]);
  spisend_dac2(dac2.R[18]);
  spisend_dac2(dac2.R[19]);
  spisend_dac2(dac2.R[20]);
  spisend_dac2(dac2.R[21]);
  spisend_dac2(dac2.R[22]);
  spisend_dac2(dac2.R[23]);
  spisend_dac2(dac2.R[24]);
  spisend_dac2(dac2.R[25]);
  
  spisend_dac2(dac2.R[30]);
  spisend_dac2(dac2.R[31]);
  spisend_dac2(dac2.R[32]);
  
  spisend_dac2(dac2.R[34]);
  spisend_dac2(dac2.R[35]);
  spisend_dac2(dac2.R[38]);
  spisend_dac2(dac2.R[40]);
  spisend_dac2(dac2.R[41]);
  spisend_dac2(dac2.R[45]);
  spisend_dac2(dac2.R[46]);
  spisend_dac2(dac2.R[47]);
  spisend_dac2(dac2.R[48]);
  spisend_dac2(dac2.R[52]);
  
  
  
  spisend_dac2(dac2.R[4]);
  spisend_dac2(dac2.R[5]);
  spisend_dac2(dac2.R[6]);
  
  spisend_dac2(dac2.R[3]);
  
  for (i=7;i<26;i++) spisend_dac2(dac2.R[i]);
  
//  Delay(1);
  dac2.cdrvser_sysref_mode  =0x003;//3
  defrag_REG_DAC1 (&dac2);  
  spisend_dac2(dac2.R[36]);
  
  dac2.sysref_mode_link0 = 0x005;//5
  defrag_REG_DAC1 (&dac2);
  spisend_dac2(dac2.R[92]);

  dac2.jesd_reset_n  =1;//1 TEST!!! 
  defrag_REG_DAC1 (&dac2);
  spisend_dac2(dac2.R[74]); 
  
  dac2.init_state    =0x0000;
  defrag_REG_DAC1 (&dac2);
  spisend_dac2(dac2.R[74]); 

 //.. 

 data=spiread_dac2(108);
 data_bool=(data>>12)&0x0f;
 Transf("-------\r\n");
 x32_out("(10)alarm_sysref_err :",data_bool);

 Transf("-------\r\n");  
 data=spiread_dac2(100);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(100):",data);  
 data=spiread_dac2(101);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(101):",data); 
 data=spiread_dac2(102);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(102):",data); 
 data=spiread_dac2(103);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(103):",data); 
 data=spiread_dac2(104);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(104):",data); 
 data=spiread_dac2(105);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(105):",data); 
 data=spiread_dac2(106);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(106):",data); 
 data=spiread_dac2(107);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(107):",data); 
 data=spiread_dac2(108);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(108):",data); 
 data=spiread_dac2(109);
 data_bool=(data>>12)&0x0f;
 x32_out("alarm(109):",data); 
   

   
 dac2.sif_txenable=1;//TXENABLE
 defrag_REG_DAC1 (&dac2);
 spisend_dac2(dac2.R[3]);
   
}

void dac2_read_reg (void)
{
	int i=0;
	u16 data;
	
for (i=0;i<128;i++) 
{
	data=spiread_dac2(i);
	Transf("config[");
    nu_out("",i);
    x32_out("]:",data&0xffff);  
	}
}

void DAC2_coarse_dac (u8 a)
{
	dac2.coarse_dac  =a;
	dac2.sif_sync    =1;
	defrag_REG_DAC1 (&dac2); 
	spisend_dac2(dac2.R[3]);	
}

void DAC2_mixer_gain (u8 a)
{
	dac2.mixer_gain  =a;
	dac2.sif_sync    =1;
	defrag_REG_DAC1 (&dac2); 
	spisend_dac2(dac2.R[2]);	
}

void DAC2_QMC_gain (u16 a)
{
	dac2.qmc_gaina  =a;
	dac2.qmc_gainb  =a;
	dac2.sif_sync    =1;
	defrag_REG_DAC1 (&dac2); 
	spisend_dac2(dac2.R[12]);
	spisend_dac2(dac2.R[13]);	
}

void DAC2_dsp_init(u8 a)
{
if (a==1)
{
dac2.mixer_ena        =1;      
dac2.mixer_gain       =1;                   
dac2.nco_ena          =1;
dac2.phaseoffsetab    =0x0000;// Phase offset for NCO in DACAB path
dac2.phaseoffsetcd    =0x0000;
dac2.phaseaddab       =76965813944320;// NCO Frequency adjust word for DACAB path. 420 MHz
dac2.phaseaddcd       =76965813944320;	


} else
{
dac2.mixer_ena        =0;      
dac2.mixer_gain       =0;                   
dac2.nco_ena          =0;
dac2.phaseoffsetab    =0x0000;// Phase offset for NCO in DACAB path
dac2.phaseoffsetcd    =0x0000;
dac2.phaseaddab       =0x000000000000;// NCO Frequency adjust word for DACAB path. 420 MHz
dac2.phaseaddcd       =0x000000000000;	
}

dac2.syncsel_mixerab  =9; 
dac2.syncsel_mixercd  =9; 
dac2.syncsel_nco      =0x8; 
dac2.sif_sync         =0;

dac2.fs8              =0;
dac2.fs4              =0;
dac2.fs2              =0;
dac2.fsm4             =0;
dac2.qmc_gainb        =0x400;
dac2.qmc_gainc        =0x400;

dac2.qmc_offseta      =0x0000;
dac2.qmc_offsetb      =0x0000;
dac2.qmc_offsetc      =0x0000;
dac2.qmc_offsetd      =0x0000;
dac2.qmc_gaina        =0x400;

dac2.dither_ena        =0x0;
dac2.dither_mixer_ena  =0x0;
dac2.dither_sra_sel    =0x0;
dac2.dither_zero       =0;

dac2.inv_sinc_ab_ena   =0;
dac2.inv_sinc_cd_ena   =0;
 
defrag_REG_DAC1 (&dac2); 
 
 spisend_dac2(dac2.R[0]);
 spisend_dac2(dac2.R[2]);
 spisend_dac2(dac2.R[18]);
 spisend_dac2(dac2.R[19]);
 spisend_dac2(dac2.R[20]);
 spisend_dac2(dac2.R[21]);
 spisend_dac2(dac2.R[22]);
 spisend_dac2(dac2.R[23]);
 spisend_dac2(dac2.R[24]);
 spisend_dac2(dac2.R[25]);
 spisend_dac2(dac2.R[13]);
 spisend_dac2(dac2.R[14]);
 spisend_dac2(dac2.R[8]);
 spisend_dac2(dac2.R[9]);
 spisend_dac2(dac2.R[10]);
 spisend_dac2(dac2.R[11]);
 spisend_dac2(dac2.R[12]);
 spisend_dac2(dac2.R[38]);
 
 spisend_dac2(dac2.R[31]);
 
 dac2.sif_sync =1;
 defrag_REG_DAC1 (&dac2); 
 spisend_dac2(dac2.R[31]);
 
 
}
//--------------------------------------------------------
void pll_init (u8 n,u8 m,u8 p,u8 vco,u8 vcoitune,u8 cp_adj)
{
	u16 data;
	u16 data_bool;

  dac1.pll_n=n;
  dac1.pll_m=m;
  dac1.pll_p=p;
  dac1.pll_vco=vco;
  dac1.pll_vcoitune=vcoitune;
  dac1.pll_cp_adj=cp_adj;

  //Program the DAC PLL settings 
  //(config26, config49, config50, config51)
  spisend_dac1(dac1.R[26]);
  spisend_dac1(dac1.R[49]);
  spisend_dac1(dac1.R[50]);
  spisend_dac1(dac1.R[51]);
  
  data=spiread_dac1(49);
  data_bool=(data>>0)&0x07;
  x32_out("memin_pll_lfvolt  :",data_bool); 
}


void init_dac_temp (void)
{
     INIT_REG_DAC1(&dac1);
	 defrag_REG_DAC1 (&dac1);
	 spisend_dac1(dac1.R[2]);
}

void init_dac1 (u8 a)

{
	if (a==1)
	{
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Пришла команда активации DAC!\r\n" );
	 Transf("Программирую DAC:\r" );
     dac1_write(0);
	 Transf("\r\n" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
	}
}

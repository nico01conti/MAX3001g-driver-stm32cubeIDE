/*
 * max30003.h
 *
 *  Created on: Oct 22, 2023
 *      Author: HP
 */

#ifndef INC_MAX30003_H_
#define INC_MAX30003_H_

#include "main.h"

//#include "DEV_CONFIG.h"
#include "stm32u5xx_hal.h" //Modified to f4 from u5 for different board
#include "spi.h"
#include "stdio.h"




#define WREG 0x00
#define RREG 0x01

#define HIGH 1
#define LOW 0

#define true 1
#define false 0

//MAX30003 Registers
#define   NO_OP           0x00
#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_BMUX       0x17
#define   CNFG_BIOZ       0x18
#define   CNFG_BIOZ_LC    0x1A
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   ECG_FIFO_BURST  0x20
#define   ECG_FIFO        0x21
#define   BIOZ_FIFO_BURST 0x22
#define   BIOZ_FIFO       0x23
#define   RTOR            0x25
#define   NO_OP2          0x7F   //CHECK

#define MAX30003_CS_PIN   7
#define CLK_PIN          6
#define RTOR_INTR_MASK     0x04

#define CS_LOW()       LL_GPIO_ResetOutputPin(CS_ECG_GPIO_Port, CS_ECG_Pin);
#define CS_HIGH()      LL_GPIO_SetOutputPin(CS_ECG_GPIO_Port, CS_ECG_Pin);
#define FLAG_TIMEOUT         ((uint32_t)0x1000)



///Status register bits
union Status_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_status
	{
		uint32_t loff_nl    : 1;
		uint32_t loff_nh    : 1;
		uint32_t loff_pl    : 1;
		uint32_t loff_ph    : 1;
		uint32_t bcgmn      : 1;
		uint32_t bcgmp      : 1;
		uint32_t reserved1  : 2;
		uint32_t pllint     : 1;
		uint32_t samp       : 1;
		uint32_t rrint      : 1;
		uint32_t lonint     : 1;
		uint32_t reserved2  : 3;
		uint32_t bcgmon     : 1;
		uint32_t bundr      : 1;
		uint32_t bover      : 1;
		uint32_t bovf       : 1;
		uint32_t bint       : 1;
		uint32_t dcloffint  : 1;
		uint32_t fstint     : 1;
		uint32_t eovf       : 1;
		uint32_t eint       : 1;
		uint32_t reserved3  : 8;
	}bits;
};

///Enable Interrupt registers bits
union EnableInterrupts_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_enint
	{
		uint32_t intb_type    : 2;
		uint32_t reserved1    : 6;
		uint32_t en_pllint    : 1;
		uint32_t en_samp      : 1;
		uint32_t en_rrint     : 1;
		uint32_t en_loint     : 1;
		uint32_t reserved2    : 3;
		uint32_t en_bcgmon    : 1;
		uint32_t en_bundr     : 1;
		uint32_t en_bover     : 1;
		uint32_t en_bovf      : 1;
		uint32_t en_bint      : 1;
		uint32_t en_dcloffint : 1;
		uint32_t en_fstint    : 1;
		uint32_t en_eovf      : 1;
		uint32_t en_eint      : 1;
		uint32_t reserved3    : 8;
	}bits;
};

///Manage Interrupt register bits
union ManageInterrupts_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_manint
	{
		uint32_t samp_it   : 2;
		uint32_t clr_samp  : 1;
		uint32_t reserved1 : 1;
		uint32_t clr_rrint : 2;
		uint32_t clr_fast  : 1;
		uint32_t reserved2 : 9;
		uint32_t bfit      : 3;
		uint32_t efit      : 5;
		uint32_t reserved3 : 8;
	}bits;
};

///Manage Dynamic Modes register bits
union ManageDynamicModes_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_mandyn
	{
		uint32_t bloff_low_it : 8;
		uint32_t bloff_hi_it  : 8;
		uint32_t fast_th      : 6;
		uint32_t fast     	  : 2;
		uint32_t reserved2    : 8;
	}bits;
};

///General Configuration bits
union GeneralConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_gencon
	{
		uint32_t rbiasn     : 1;
		uint32_t rbiasp     : 1;
		uint32_t rbiasv     : 2;
		uint32_t en_rbias   : 2;
		uint32_t vth        : 2;
		uint32_t imag       : 3;
		uint32_t ipol       : 1;
		uint32_t en_dcloff  : 2;
		uint32_t en_bloff   : 2;
		uint32_t reserved1  : 2;
		uint32_t en_bioz    : 1;
		uint32_t en_ecg     : 1;
		uint32_t fmstr      : 2;
		uint32_t en_ulp_lon : 2;
		uint32_t reserved2  : 8;
	}bits;
};

///Cal Configuration bits
union CalConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_calcon
	{
		uint32_t thigh     : 11;
		uint32_t fifty     : 1;
		uint32_t fcal      : 3;
		uint32_t reserved1 : 5;
		uint32_t vmag      : 1;
		uint32_t vmode     : 1;
		uint32_t en_vcal   : 1;
		uint32_t reserved2 : 9;

	}bits;
};

///EMux Configuration bits
union MuxConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_muxcon
	{
		uint32_t reserved1 : 16;
		uint32_t caln_sel  : 2;
		uint32_t calp_sel  : 2;
		uint32_t openn     : 1;
		uint32_t openp     : 1;
		uint32_t reserved2 : 1;
		uint32_t pol       : 1;
		uint32_t reserved3 : 8;
	}bits;
};

///ECG Configuration bits
union ECGConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_ecgconf
	{
		uint32_t reserved1 : 12;
		uint32_t dlpf      : 2;
		uint32_t dhpf      : 1;
		uint32_t reserved2 : 1;
		uint32_t gain      : 2;
		uint32_t reserved3 : 4;
		uint32_t rate      : 2;
		uint32_t reserved4 : 8;
	}bits;
};


///BMux Configuration bits
union BMuxConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_bmuxconf
	{
		uint32_t bmux_fbist     : 2;
		uint32_t reserved1      : 2;
		uint32_t bmux_rmod      : 3;
		uint32_t reserved2      : 1;
		uint32_t bmux_rnom      : 3;
		uint32_t bmux_enbist    : 1;
		uint32_t bmux_cg_mode   : 2;
		uint32_t reserved3      : 2;
		uint32_t bmux_caln_sel	: 2;
		uint32_t bmux_calp_sel  : 2;
		uint32_t bmuxopenn 		: 1;
		uint32_t bmux_openp		: 1;
		uint32_t reserved4 		: 10;
	}bits;

};


///BIOZ Configuration bits
union BIOZConfiguration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_biozconf
	{
		uint32_t bioz_phoff     : 4;
		uint32_t bioz_cgmag     : 3;
		uint32_t bioz_cgmon     : 1;
		uint32_t bioz_fcgen     : 4;
		uint32_t bioz_dlpf      : 2;
		uint32_t bioz_dhpf      : 2;
		uint32_t bioz_gain      : 2;
		uint32_t ln_bioz	    : 1;
		uint32_t ext_rbias      : 1;
		uint32_t bioz_ahpf		: 3;
		uint32_t bioz_rate		: 1;
		uint32_t reserved 		: 8;
	}bits;

};

///BIOZ_LC Configuration bits
union BIOZConfigurationLC_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_biozconfLC
	{
		uint32_t bioz_cgmag_lc  : 4;
		uint32_t bioz_cmres     : 4;
		uint32_t reserved1      : 4;
		uint32_t bistr          : 2;
		uint32_t en_bistr       : 1;
		uint32_t reserved2	    : 4;
		uint32_t buoz_lc2x      : 1;
		uint32_t reserved3		: 3;
		uint32_t bioz_hi_lob	: 1;
		uint32_t reserved4 		: 8;
	}bits;

};


///RtoR1 Configuration bits
union RtoR1Configuration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_rtor1
	{
		uint32_t reserved1 : 8;
		uint32_t ptsf      : 4;
		uint32_t pavg      : 2;
		uint32_t reserved2 : 1;
		uint32_t en_rtor   : 1;
		uint32_t rgain     : 4;
		uint32_t wndw      : 4;
		uint32_t reserved3 : 8;
	}bits;
};

///RtoR2 Configuration bits
union RtoR2Configuration_u
{
	///Access all bits
	uint32_t all;

	///Access individual bits
	struct BitField_rtor2
	{
		uint32_t reserved1 : 8;
		uint32_t rhsf      : 3;
		uint32_t reserved2 : 1;
		uint32_t ravg      : 2;
		uint32_t reserved3 : 2;
		uint32_t hoff      : 6;
		uint32_t reserved4 : 10;
	}bits;
};




typedef enum
{
	SAMPLINGRATE_128 = 128, SAMPLINGRATE_256 = 256, SAMPLINGRATE_512 = 512
}sampRate;


extern unsigned int heartRate;
extern unsigned int RRinterval;
extern signed long ecgdata;

void max30003Begin();
void max30003BeginRtorMode();
void max30001gBeginBIOZ();
void max30003SwReset(void);
void max30003FifoReset(void);
void getHRandRR(void);
void getEcgSamples(void);
int max30003ReadInfo(void);
void max30003SetsamplingRate(uint16_t samplingRate);
void max30003Config();
void max30003Printconf();
void max30001gBeginBothMode4Electrode();
//void max30003RegRead(uint8_t Reg_address, uint8_t * buff);

//void max30003ReadData(int num_samples, uint8_t * readBuffer);

void max30003Synch(void);
//void max30003RegWrite (unsigned char WRITE_ADDRESS, unsigned long data);
void DATA_Write(uint8_t Reg_address, uint16_t NumByteToWrite, unsigned int data);
void DATA_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
static uint8_t SPI_SendReceiveByte(uint8_t byte);

uint8_t SPI_SendByte(uint8_t byte);

__STATIC_INLINE uint32_t buff8_to_int32(uint8_t* pBuffer)
{

	uint32_t out = 0;
	out =  pBuffer[0] << 16;
	out |=  pBuffer[1] << 8;
	out |=  pBuffer[2];

	return out;

}

__STATIC_INLINE uint32_t CorrectSign(int32_t sample, uint8_t msb){

	uint32_t signbit = (1 << (msb - 1));
	uint32_t mask = signbit - 1;

	sample = ((signbit & sample) >> (msb-1)) ? (sample | (~mask)) : sample ;

	return sample;
}


#endif /* INC_MAX30003_H_ */


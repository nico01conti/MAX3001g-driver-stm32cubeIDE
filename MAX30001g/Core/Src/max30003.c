/*
 * max30003.c
 *
 *  Created on: Jun 12, 2021
 *      Author: simon
 */



//////////////////////////////////////////////////////////////////////////////////////////
//
//  Demo code for the MAX30003 breakout board
//
//  Arduino connections:
//
//  |MAX30003 pin label| Pin Function         |Arduino Connection|
//  |----------------- |:--------------------:|-----------------:|
//  | MISO             | Slave Out            |  D12             |
//  | MOSI             | Slave In             |  D11             |
//  | SCLK             | Serial Clock         |  D13             |
//  | CS               | Chip Select          |  D7              |
//  | VCC              | Digital VDD          |  +5V             |
//  | GND              | Digital Gnd          |  Gnd             |
//  | FCLK             | 32K CLOCK            |  -               |
//  | INT1             | Interrupt1           |  02               |
//  | INT2             | Interrupt2           |  -               |
//
//  This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  For information on how to use, visit https://github.com/Protocentral/protocentral-max30003-arduino
//
/////////////////////////////////////////////////////////////////////////////////////////

#include"max30003.h"
uint32_t  LIS3DSHTimeout = FLAG_TIMEOUT;
union Status_u 				   STATUS_r;
union EnableInterrupts_u       EN_INT_r;
//union EnableInterrupts2_u      EN_INT2_r;
union ManageInterrupts_u       MNGR_INT_r;
union ManageDynamicModes_u     MNGR_DYN_r;
union GeneralConfiguration_u   CNFG_GEN_r;
union CalConfiguration_u       CNFG_CAL_r;
union MuxConfiguration_u       CNFG_EMUX_r;
union ECGConfiguration_u       CNFG_ECG_r;
union BMuxConfiguration_u      CNFG_BMUX_r;
union BIOZConfiguration_u      CNFG_BIOZ_r;
union BIOZConfigurationLC_u    CNFG_BIOZ_LC_r;
union RtoR1Configuration_u     CNFG_RTOR1_r;
union RtoR2Configuration_u     CNFG_RTOR2_r;
uint32_t register_buffer = 0;

void max30003SwReset(void)
{
	DATA_Write(SW_RST,3, 0x000000);
	//delay(100);
}

void max30003Synch(void)
{
	DATA_Write(SYNCH,3, 0x000000);
}

void max30003FifoReset(void)
{
	DATA_Write(FIFO_RST,3, 0x000000);
}



int max30003ReadInfo(void)
{
	uint8_t spiTxBuff;
	uint8_t readBuff[4]={0} ;

	CS_LOW();

	spiTxBuff = (INFO << 1 ) | RREG;
	//SPI.transfer(spiTxBuff); //Send register location
//	LIS3DSHTimeout = FLAG_TIMEOUT;
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0) return 0;
//	}

	/* Send a Byte through the SPI peripheral */
	SPI_SendReceiveByte(spiTxBuff);
	//LL_SPI_TransmitData8(SPI1, spiTxBuff);


	for ( int i = 0; i < 3; i++)
	{
		//readBuff[i] = LL_SPI_ReceiveData8(SPI1);
		readBuff[i] = SPI_SendReceiveByte(0x00);
	}

	CS_HIGH();

	if((readBuff[1]&0xf0) == 0x10 ){

		printf("max30003 is ready\r\n");
		printf("Rev ID: %02X\r\n", readBuff[1] & 0xF0); // Use the correct format specifier

		return true;
	}else{

		printf(" max30003 read info error \r\n");
		return 0;
	}

	return false;
}

void max30003ReadData(int num_samples, uint8_t * readBuffer)
{
	uint8_t spiTxBuff;
	CS_LOW();

	spiTxBuff = (ECG_FIFO_BURST<<1 ) | RREG;
	// SPI.transfer(spiTxBuff); //Send register location (not used because code is for arduino)

	SPI_SendByte(spiTxBuff); // Added by me to ensure that SPI communicates register to slave

	for ( int i = 0; i < num_samples*3; ++i)
	{
		readBuffer[i] = SPI_SendByte(0x00);
	}

	CS_HIGH();
}

void max30003Begin()
{
	uint8_t buffer_in[4];

	max30003SwReset();
	//HAL_Delay(100);
	DATA_Write(CNFG_GEN,3, 0x081007);
	//HAL_Delay(100);
	DATA_Read(buffer_in, CNFG_GEN,3);
	//HAL_Delay(100);
	DATA_Write(CNFG_CAL,3, 0x502800);  // 0x700000
	//HAL_Delay(100);
	DATA_Write(CNFG_EMUX,3,0x3B0000);
	//HAL_Delay(100);
	DATA_Write(CNFG_ECG,3, 0x800000);  // d23 - d22 : 10 for 250sps , 00:500 sps
	//HAL_Delay(100);
	DATA_Write(EN_INT,3, 0x800003); // Enables EINT

	DATA_Write(CNFG_RTOR1,3, 0x3fc600);
	//HAL_Delay(100);
	max30003Synch();
	//HAL_Delay(100);


}

void max30003Config(){
	  printf("ADC RESET\r\n");
	  DATA_Write(SW_RST,3, 0x000000);
	  HAL_Delay(100);

    CNFG_GEN_r.bits.en_ecg = 1;     // Enable ECG channel
    CNFG_GEN_r.bits.rbiasn = 1;     // Enable resistive bias on negative input
    CNFG_GEN_r.bits.rbiasp = 1;     // Enable resistive bias on positive input
    CNFG_GEN_r.bits.en_rbias = 1;   // Enable resistive bias
    CNFG_GEN_r.bits.imag = 2;       // Current magnitude = 10nA
    CNFG_GEN_r.bits.en_dcloff = 1;  // Enable DC lead-off detection
	register_buffer = CNFG_GEN_r.all;
    DATA_Write(CNFG_GEN, 3, register_buffer);
    HAL_Delay(100);


    // ECG Config register setting

    CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
    CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
    CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V
    CNFG_ECG_r.bits.rate = 2;       // Sample rate = 128 sps
	register_buffer = CNFG_ECG_r.all;
    DATA_Write(CNFG_ECG, 3, register_buffer);
    HAL_Delay(100);


    //R-to-R configuration

    CNFG_RTOR1_r.bits.en_rtor = 1;           // Enable R-to-R detection
	register_buffer = CNFG_RTOR1_r.all;
    DATA_Write(CNFG_RTOR1, 3, register_buffer);
    HAL_Delay(100);


    //Manage interrupts register setting

    MNGR_INT_r.bits.efit = 0b00011;          // Assert EINT w/ 4 unread samples
    MNGR_INT_r.bits.clr_rrint = 0b01;        // Clear R-to-R on RTOR reg. read back
	register_buffer = MNGR_INT_r.all;
    DATA_Write(MNGR_INT, 3, register_buffer);
    HAL_Delay(100);

    DATA_Write(MNGR_INT, 3, 0x780014);

    //Enable interrupts register setting

    EN_INT_r.all = 0;
    EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
    EN_INT_r.bits.en_rrint = 0;             // Disable R-to-R interrupt
    EN_INT_r.bits.intb_type = 3;            // Open-drain NMOS with internal pullup
	register_buffer = EN_INT_r.all;
    DATA_Write(EN_INT, 3, register_buffer);
    HAL_Delay(100);


    //Dyanmic modes config

    MNGR_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
	register_buffer = MNGR_DYN_r.all;
    DATA_Write(MNGR_DYN, 3, register_buffer);
    HAL_Delay(100);

    // MUX Config

    CNFG_EMUX_r.bits.openn = 0;          // Connect ECGN to AFE channel
    CNFG_EMUX_r.bits.openp = 0;          // Connect ECGP to AFE channel
    CNFG_EMUX_r.bits.pol = 0;            // Invert ECG signal
	register_buffer = CNFG_EMUX_r.all;
    DATA_Write(CNFG_EMUX, 3, register_buffer);
    HAL_Delay(100);



    DATA_Write(SYNCH,3, 0x000000);
    HAL_Delay(100);

}

void max30003Printconf(){
	uint8_t buffer_in[4];

	  printf("ADC CONFIG\r\n");

	  DATA_Read(buffer_in, INFO, 3);
	  printf(" CINFO: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_GEN, 3);
	  printf(" CNFG_GEN: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, STATUS, 3);
	  printf(" STATUS: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, EN_INT , 3);
	  printf(" EN_INT: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, EN_INT2, 3);
	  printf(" EN_INT2: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, MNGR_INT, 3);
	  printf(" MNGR_INT: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, MNGR_DYN, 3);
	  printf(" MNGR_DYN: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_CAL, 3);
	  printf(" CNFG_CAL: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_EMUX, 3);											///READ ECF/EMUX conf
	  printf(" CNFG_EMUX: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_ECG, 3);
	  printf(" CNFG_ECG: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_BMUX, 3);											///READ BIOZ/BMUX conf
	  printf(" CNFG_BMUX: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_BIOZ, 3);
	  printf(" CNFG_BIOZ: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_BIOZ_LC, 3);
	  printf(" CNFG_BIOZ_LC: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_RTOR1, 3);
	  printf(" CNFG_RTOR1: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, CNFG_RTOR2, 3);
	  printf(" CNFG_RTOR2: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, ECG_FIFO_BURST, 3);
	  printf(" ECG_FIFO_BURST: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, ECG_FIFO, 3);
	  printf(" ECG_FIFO: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, BIOZ_FIFO_BURST, 3);
	  printf(" BIOZ_FIFO_BURST: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );

	  DATA_Read(buffer_in, BIOZ_FIFO, 3);
	  printf(" BIOZ_FIFO: %x %x %x \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );


	  DATA_Read(buffer_in, RTOR, 3);
	  printf(" RTOR: %x %x %x  \r\n" ,buffer_in[0],buffer_in[1],buffer_in[2] );
}
void max30003BeginRtorMode1()
{
	max30003SwReset();
	//delay(100);
	DATA_Write(CNFG_GEN, 3, 0x080004);
	//delay(100);
	DATA_Write(CNFG_CAL, 3,0x703000);  // 0x700000
	//delay(100);
	DATA_Write(CNFG_EMUX,3, 0xBB0000);
	//delay(100);
	DATA_Write(CNFG_ECG,3, 0x805000);  // d23 - d22 : 10 for 250sps , 00:500 sps
	//delay(100);
	DATA_Write(CNFG_RTOR1,3, 0x3f8600);
	//delay(100);
	DATA_Write(EN_INT,3, 0x800403);

	DATA_Write(MNGR_INT, 3, 0x780014);
	//delay(100);
	max30003Synch();
	//delay(100);
}

void max30003BeginRtorMode(){
	printf("ADC RESET\r\n");
		  DATA_Write(SW_RST,3, 0x000000);
		  HAL_Delay(100);

	    CNFG_GEN_r.bits.en_ecg = 1;     // Enable ECG channel
	    CNFG_GEN_r.bits.rbiasn = 1;     // Enable resistive bias on negative input
	    CNFG_GEN_r.bits.rbiasp = 1;     // Enable resistive bias on positive input
	    CNFG_GEN_r.bits.en_rbias = 1;   // Enable resistive bias
	    CNFG_GEN_r.bits.imag = 2;       // Current magnitude = 10nA
	    CNFG_GEN_r.bits.en_dcloff = 1;  // Enable DC lead-off detection
		register_buffer = CNFG_GEN_r.all;
	    DATA_Write(CNFG_GEN, 3, register_buffer);
	    HAL_Delay(100);


	    // ECG Config register setting

	    CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
	    CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
	    CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V
	    CNFG_ECG_r.bits.rate = 2;       // Sample rate = 128 sps
		register_buffer = CNFG_ECG_r.all;
	    DATA_Write(CNFG_ECG, 3, register_buffer);
	    HAL_Delay(100);


	    //R-to-R configuration
	    CNFG_RTOR1_r.bits.wndw = 3;
	    CNFG_RTOR1_r.bits.rgain = 15;
	    CNFG_RTOR1_r.bits.en_rtor = 1;           // Enable R-to-R detection
	    CNFG_RTOR1_r.bits.ptsf = 6;
		register_buffer = CNFG_RTOR1_r.all;
	    DATA_Write(CNFG_RTOR1, 3, register_buffer);
	    HAL_Delay(100);

	    //DATA_Write(CNFG_RTOR1,3, 0x3f8600);

	    //Manage interrupts register setting

	    MNGR_INT_r.bits.efit = 15;          // Assert EINT w/ 4 unread samples
	    MNGR_INT_r.bits.clr_rrint = 1;        // Clear R-to-R on RTOR reg. read back
	    MNGR_INT_r.bits.clr_rrint  = 1;
	    MNGR_INT_r.bits.samp_it = 3;
		register_buffer = MNGR_INT_r.all;
	    DATA_Write(MNGR_INT, 3, register_buffer);
	    HAL_Delay(100);

	    //DATA_Write(MNGR_INT, 3, 0x780014);

	    //Enable interrupts register setting

	    EN_INT_r.all = 0;
	    EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
	    EN_INT_r.bits.en_rrint = 1;             // Enable R-to-R interrupt
	    EN_INT_r.bits.intb_type = 3;            // Open-drain NMOS with internal pullup
		register_buffer = EN_INT_r.all;
	    DATA_Write(EN_INT, 3, register_buffer);
	    HAL_Delay(100);


	    //Dyanmic modes config

	    MNGR_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
		register_buffer = MNGR_DYN_r.all;
	    DATA_Write(MNGR_DYN, 3, register_buffer);
	    HAL_Delay(100);

	    // MUX Config

	    CNFG_EMUX_r.bits.openn = 0;          // Connect ECGN to AFE channel
	    CNFG_EMUX_r.bits.openp = 0;          // Connect ECGP to AFE channel
	    CNFG_EMUX_r.bits.pol = 0;            // Invert ECG signal
		register_buffer = CNFG_EMUX_r.all;
	    DATA_Write(CNFG_EMUX, 3, register_buffer);
	    HAL_Delay(100);



	    DATA_Write(SYNCH,3, 0x000000);
	    HAL_Delay(100);
}

void max30001gBeginBIOZ(){
	printf("ADC RESET\r\n");
			  DATA_Write(SW_RST,3, 0x000000);
			  HAL_Delay(100);

		CNFG_GEN_r.bits.en_bioz = 1;    	// Enable BIOZ channel
		CNFG_GEN_r.bits.rbiasn = 1;     	// Enable resistive bias on negative input
		CNFG_GEN_r.bits.rbiasp = 1;    		// Enable resistive bias on positive input
		CNFG_GEN_r.bits.en_rbias = 0b10;	// Enable resistive bias
		CNFG_GEN_r.bits.imag = 2;       	// Current magnitude = 10nA
		CNFG_GEN_r.bits.en_dcloff = 1;  	// Enable DC lead-off detection
		register_buffer = CNFG_GEN_r.all;
		DATA_Write(CNFG_GEN, 3, register_buffer);
		HAL_Delay(100);

		// BIOZ Config register setting

//		CNFG_BIOZ_r.bits.bioz_ahpf = 1;  // ANALOG  LPF cutoff = 40Hz
//		CNFG_BIOZ_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
//		CNFG_BIOZ_r.bits.bioz_gain = 2;  // ECG gain = 40V/V
//		CNFG_BIOZ_r.bits.rate = 2;       // Sample rate = 128 sps
//		register_buffer = CNFG_BIOZ_r.all;
//		DATA_Write(CNFG_BIOZ, 3, register_buffer);
//		HAL_Delay(100);

		DATA_Write(EN_INT, 3, 0x080003);   //Enable BINT IT
		HAL_Delay(100);

		DATA_Write(MNGR_INT, 3, 0x7F0004); //Set BINT Threshold
		HAL_Delay(100);

		DATA_Write(CNFG_BIOZ, 3, 0x801A10);
		HAL_Delay(100);

		DATA_Write(CNFG_BMUX, 3, 0x003040);
		HAL_Delay(100);

		DATA_Write(CNFG_BIOZ_LC, 3, 0x000082);
		HAL_Delay(100);


		DATA_Write(SYNCH,3, 0x000000);
		HAL_Delay(100);


}

void max30001gBeginBothMode4Electrode(){
	printf("ADC RESET\r\n");
		DATA_Write(SW_RST,3, 0x000000);
		HAL_Delay(100);

		DATA_Write(EN_INT, 3, 0x880403);   //Enable BINT IT
		HAL_Delay(100);

		DATA_Write(MNGR_INT, 3, 0x770004); //Set BINT Threshold
		HAL_Delay(100);

		DATA_Write(CNFG_GEN, 3, 0x0c1217);
		HAL_Delay(100);

		DATA_Write(CNFG_BMUX, 3, 0x003040);
		HAL_Delay(100);

		DATA_Write(CNFG_BIOZ, 3, 0x801A10);
		HAL_Delay(100);

		DATA_Write(CNFG_BIOZ_LC, 3, 0x003082);
		HAL_Delay(100);

		DATA_Write(CNFG_ECG, 3, 0x805000);
		HAL_Delay(100);

		DATA_Write(CNFG_RTOR1, 3, 0x3FA300);
		HAL_Delay(100);

		DATA_Write(CNFG_EMUX, 3, 0x000000);
		HAL_Delay(100);

		DATA_Write(SYNCH,3, 0x000000);
		HAL_Delay(100);

}

//not tested
void max30003SetsamplingRate(uint16_t samplingRate)
{
	uint8_t regBuff[4] = {0};
	DATA_Read(regBuff,CNFG_ECG,3);

	switch(samplingRate){
	case SAMPLINGRATE_128:
		regBuff[0] = (regBuff[0] | 0x80 );
		break;

	case SAMPLINGRATE_256:
		regBuff[0] = (regBuff[0] | 0x40 );
		break;

	case SAMPLINGRATE_512:
		regBuff[0] = (regBuff[0] | 0x00 );
		break;

	default :
		////Serial.println("Wrong samplingRate, please choose between 128, 256 or 512");
		break;
	}

	DATA_Write(CNFG_ECG, 3, (uint32_t)regBuff);
}

void getEcgSamples(void)
{
	uint8_t regReadBuff[4];
	DATA_Read(regReadBuff, ECG_FIFO, 3);

	unsigned long data0 = (unsigned long) (regReadBuff[0]);
	data0 = data0 <<24;
	unsigned long data1 = (unsigned long) (regReadBuff[1]);
	data1 = data1 <<16;
	unsigned long data2 = (unsigned long) (regReadBuff[2]);
	data2 = data2 >>6;
	data2 = data2 & 0x03;

	unsigned long data = (unsigned long) (data0 | data1 | data2);
	signed long ecgdata = (signed long) (data);
	printf("ECG sample: %ld\r\n", ecgdata);
}


void getHRandRR(void)
{
	uint8_t regReadBuff[4];
	max30003RegRead(RTOR, regReadBuff);

	unsigned long RTOR_msb = (unsigned long) (regReadBuff[0]);
	unsigned char RTOR_lsb = (unsigned char) (regReadBuff[1]);
	unsigned long rtor = (RTOR_msb<<8 | RTOR_lsb);
	rtor = ((rtor >>2) & 0x3fff) ;

	float hr =  60 /((float)rtor*0.0078125);
	heartRate = (unsigned int)hr;

	unsigned int RR = (unsigned int)rtor* (7.8125) ;  //8ms
	RRinterval = RR;
}


static uint8_t SPI_SendReceiveByte(uint8_t byte) {
    // Write data to the SPI transmit buffer
    LL_SPI_TransmitData8(SPI1, byte);
    while (!LL_SPI_IsActiveFlag_TXC(SPI1)) {}
    // Read received data from the SPI receive buffer
    while (!LL_SPI_IsActiveFlag_RXP(SPI1)) {}
    return LL_SPI_ReceiveData8(SPI1);
}

void DATA_Read(uint8_t* pBuffer, uint8_t Reg_address, uint16_t NumByteToRead)
{
	char ReadAddr;
	if(NumByteToRead > 0x01)
	{
		ReadAddr = (Reg_address<<1 ) | RREG;
	}
	else
	{
		// ReadAddr |= (uint8_t)READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	CS_LOW();

	/* Send the Address of the indexed register */
	SPI_SendReceiveByte(ReadAddr);

	/* Receive the data that will be read from the device (MSB First) */
	while(NumByteToRead > 0x00)
	{
		/* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
		*pBuffer = SPI_SendReceiveByte(0x00);
		NumByteToRead--;
		pBuffer++;
	}

	/* Set chip select High at the end of the transmission */
	CS_HIGH();

}

void DATA_Write(uint8_t Reg_address, uint16_t NumByteToWrite, unsigned int data)
{
	unsigned char WriteAddr;
	unsigned char WriteData;

	if(NumByteToWrite > 0x01)
	{
		WriteAddr = (Reg_address<<1 ) | WREG;
	}
	else
	{
		//    ReadAddr |= (uint8_t)READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	CS_LOW();

	/* Send the Address of the indexed register */
	SPI_SendReceiveByte(WriteAddr);

	/* Receive the data that will be read from the device (MSB First) */
	while(NumByteToWrite > 0x00)
	{
		/* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
		WriteData = (char) ( data >> ((NumByteToWrite -1)*8));
		SPI_SendReceiveByte(WriteData);
		NumByteToWrite--;

	}

	/* Set chip select High at the end of the transmission */
	CS_HIGH();

}








///////////////////////////////////////////////////////////////////////////////////
//OLD MODULE FUNCTIONS
//void max30003RegWrite_OLD(unsigned char WRITE_ADDRESS, unsigned long data)
//{
//    // now combine the register address and the command into one byte:
//    char dataToSend = (WRITE_ADDRESS<<1) | WREG;
//
//     // take the chip select low to select the device:
//    digitalWrite(MAX30003_CS_PIN, LOW);
//
//    LL_SPI_TransmitData8(SPI2,0x00);
//    ////delay(2);
//   // SPI.transfer(dataToSend);
//    //SPI.transfer(data>>16);
//    //SPI.transfer(data>>8);
//    //SPI.transfer(data);
//    ////delay(2);
//
//    // take the chip select high to de-select:
//    digitalWrite(MAX30003_CS_PIN, HIGH);
//}

//void max30003RegWrite(unsigned char WRITE_ADDRESS, unsigned long data)
//{
//	// now combine the register address and the command into one byte:
//	char dataToSend = (WRITE_ADDRESS<<1) | WREG;
//	unsigned char dato = 0;
//
//	// take the chip select low to select the device:
//	CS_LOW();
//	//HAL_Delay(1);
//
//	LIS3DSHTimeout = FLAG_TIMEOUT;
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0) 	while(1);
//	}
//
//	/* Send a Byte through the SPI peripheral */
//	LL_SPI_TransmitData8(SPI2, dataToSend);
//
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0) while(1);
//	}
//
//	/* Send a Byte through the SPI peripheral */
//	dato = (unsigned char)(data>>16);
//	LL_SPI_TransmitData8(SPI2,dato );
//
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0)
//			while(1);
//	}
//
//	/* Send a Byte through the SPI peripheral */
//	dato = (unsigned char)(data>>8);
//	LL_SPI_TransmitData8(SPI2, dato);
//
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0) 	while(1);
//	}
//
//	/* Send a Byte through the SPI peripheral */
//	dato = (unsigned char)(data);
//	LL_SPI_TransmitData8(SPI2, dato);
//
//
//
//	// take the chip select high to de-select:
//	CS_HIGH();
//}
//void max30003RegRead(uint8_t Reg_address, uint8_t * buff)
//{
//	uint8_t spiTxBuff;
//
//	CS_LOW();
//
//	spiTxBuff = (Reg_address<<1 ) | RREG;
//	// SPI.transfer(spiTxBuff); //Send register location
//	LIS3DSHTimeout = FLAG_TIMEOUT;
//	while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
//	{
//		if((LIS3DSHTimeout--) == 0)	while(1);
//	}
//
//	/* Send a Byte through the SPI peripheral */
//	LL_SPI_TransmitData8(SPI2, spiTxBuff);
//
//
//	//	 while(NumByteToRead > 0x00)
//	//	  {
//	//	    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
//	//	    *pBuffer = SPI_SendByte(0x00);
//	//	    NumByteToRead--;
//	//	    pBuffer++;
//	//	  }
//
//
//
//	for ( int i = 0; i < 3; i++)
//	{
//		buff[i] = SPI_SendByte(0x00);
//	}
//
//	CS_HIGH();
//}




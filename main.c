/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Creative Digital Design.
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>
#define HMC5883L_Address 0x0E //used to mark read address
//used to hold the x y z  data from the sensors
struct XYZ_Cord{
	int16 x;
	int16 y;
	int16 z;

};
//This is used for reciveing and tranmitting data for the HMC5883L
union HMC5883L
{
	uint8 array[6];
	struct XYZ_Cord Data;
};
//This is used for reciveing and transmitting data for the LIS331HH
union LIS331HH
{
	uint8 array[6];
	struct XYZ_Cord Data;
};
//This enumerator reperesnts all the register addressses on the HMC588L
enum HMC5883L_REG_Addresses{
	CONFIG_REGA, CONFIG_REGB, MODE_REG, XMSB_REG, XLSB_REG,
	ZMSB_REG, ZLSB_REG, YMSB_REG, YLSB_REG, STAT_REG
} HMC5883L_REG_Addresses;
//this enumerator is used to access all the regs on LIS331HH
enum LIS331HH_REG_Addresses{
	CTRL_REG1 =0x20, CTRL_REG2, CTRL_REG3, CTRL_REG4, CTRL_REG5, 
	HP_FILTER_RESET, REFERENCE, STATUS_REG, 
	OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_z_H,
	INT1_CFG = 0x30, INT1_SOURCE, INT1_THS, INT1_DURATION, INT2_CFG, INT2_SOURCE, INT2_THS, INT2_DURATION
} LIS331HH_REG_Addresses;
enum HMC5883L_Packet{
	REG_ADDRESS,
	COMMAD,
}HMC5883L_I2C_Packet;
//function prototypes
void Self_Test(void);
void LIS331HH_Config();
void HMC5883L_Config();
void Initialization();
void get_LIS331HH_Data(union LIS331HH *Data);
void get_HMC5883L_Data(union HMC5883L *Data);
void get_All_data(uint8 *array);
uint8 i2c_write(uint8 subAddr, uint8 *buffer, uint8 buff_size);
uint8 i2c_read(uint8 subAddr, uint8 *buffer, uint8 buff_size);
void menu();

int main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	//int size = 6;
	int n;
	char buffer[123];
	//uint8 *array;
	union HMC5883L Mag_Data;
	union LIS331HH Accl_Data;

	Initialization();
    CyGlobalIntEnable;  /* Uncomment this line to enable global interrupts. */
    Mag_Data.Data.x = 10;
	Mag_Data.Data.y = 11;
	Mag_Data.Data.z = 12;
	DATA_COM_PutString(" Main \n");
	for(;;)
    {
        CyDelay(10);
		get_HMC5883L_Data(&Mag_Data);
		//DATA_COM_PutArray(array,size);
		//get_LIS331HH_Data(&Accl_Data);
		n= sprintf(buffer, "X: %d \nY: %d \nZ: %d \n",(int) Mag_Data.Data.x, Mag_Data.Data.y,Mag_Data.Data.z);	
		//n= sprintf(buffer, "X: %d \nY: %d \nZ: %d \n",(int) Accl_Data.Data.x, Accl_Data.Data.y,Accl_Data.Data.z);
		DATA_COM_PutArray(buffer,n);
    }
}
void Initialization()
{
	CS_Write(1);
	LIS331HH_SPI_Start();
	HMC5883L_I2C_Start();
	CONFIG_COM_Start();
	DATA_COM_Start();
	DATA_COM_PutString("Initial \n");
	HMC5883L_Config();
	LIS331HH_Config();
}
void LIS331HH_Config()
{
	/* Setup CTRL_REG1
	Bits:
	PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
	PM2PM1PM0: Power mode (001 = Normal Mode)
	DR1DR0: Data rate (00=50Hz, 01=100Hz, 10=400Hz, 11=1000Hz)
	Zen, Yen, Xen: Z enable, Y enable, X enable
	*/
	CS_Write(0);
	CyDelay(1);
	LIS331HH_SPI_WriteTxData(CTRL_REG1);
	LIS331HH_SPI_WriteTxData(0x37); //normal mode, 1000HZ, xyz-enabled
	CS_Write(1);
	CyDelay(100);
	/* Setup CTRL_REG2
	turn off high pass filter
	*/
	CS_Write(0);
	CyDelay(1);
	LIS331HH_SPI_WriteTxData(CTRL_REG2);
	LIS331HH_SPI_WriteTxData(0x00);//HP filteron( cuttoff @8MHz ) = 0x60, HPfilter OFF = 0x00
	CS_Write(1);
	CyDelay(100);
	/* Setup CTRL_REG3
	Bits:
	IHL PP_OD LIR2 I2_CFG1 I2_CFG0 LIR1 I1_CFG1 I1_CFG0
	IHL: interrupt high low (0=active high)
	PP_OD: push-pull/open drain selection (0=push-pull)
	LIR2: latch interrupt request on INT2_SRC register (0=interrupt request not latched)
	I2_CFG1, I2_CFG0: data signal on INT 2 pad control bits
	LIR1: latch interrupt request on INT1_SRC register (0=interrupt request not latched)
	I1_CFG1, I1_CFG0: data signal on INT 1 pad control bits

	Data signal on pad
	I1(2)_CFG1 I1(2)_CFG0 INT 1(2) pad
	0 0 interrupt 1(2) source
	0 1 interrupt 1 source or interrupt 2 source
	1 0 data ready
	1 1 boot running

	*/

	CS_Write(0);
	CyDelay(1);
	LIS331HH_SPI_WriteTxData(CTRL_REG3);
	LIS331HH_SPI_WriteTxData(0x00);
	CS_Write(1);
	CyDelay(100);
	/* Setup CTRL_REG4
	Bits:
	BDU BLE FS1 FS0 STsign 0 ST SIM
	BDU: Block data update (0=continuous update)
	BLE: Big/little endian data (0=accel data LSB at LOW address)
	FS1FS0: Full-scale selection (00 = +/-6G, 01 = +/-12G, 11 = +/-24G)
	STsign: selft-test sign (default 0=plus)
	ST: self-test enable (default 0=disabled)
	SIM: SPI mode selection(default 0=4 wire interface, 1=3 wire interface)
	*/
	CS_Write(0);
	CyDelay(1);
	LIS331HH_SPI_WriteTxData(CTRL_REG4);
	LIS331HH_SPI_WriteTxData(0x30); //24g
	CS_Write(1);
	CyDelay(100);
}
void HMC5883L_Config()
{
	uint8 slave= 0x1E;
	uint8 array[2];
	array [0] = 0x80;
	i2c_write(0x11,array,1);
	
	array [0] = 1;
	i2c_write(0x10,array,1);

//	HMC5883L_I2C_MasterClearStatus();
//	
//	array[REG_ADDRESS] = 0x02u;
//	array[COMMAD] = 0x00u; //continuous measurment mode
//	
//	HMC5883L_I2C_MasterSendStart(slave,0u);
//	HMC5883L_I2C_MasterWriteByte(array[REG_ADDRESS]);
//	HMC5883L_I2C_MasterWriteByte(array[COMMAD]);
//	HMC5883L_I2C_MasterSendStop();
//	CyDelay(10);
////	array[REG_ADDRESS] = 0x10u;
////	array[COMMAD] = 0x3u;
////	HMC5883L_I2C_MasterSendStart(0x0E,0u);
////	HMC5883L_I2C_MasterWriteByte(array[REG_ADDRESS]);
////	HMC5883L_I2C_MasterWriteByte(array[COMMAD]);
////	HMC5883L_I2C_MasterSendStop();
////	CyDelay(10);
}
void get_LIS331HH_Data(union LIS331HH *Data)
{
	
	CS_Write(0);
	CyDelay(10);
	LIS331HH_SPI_WriteTxData(0xE8);//read bit | consecutive measure bit | 0x28 = 0xe8	
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[1] = LIS331HH_SPI_ReadRxData();
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[0] = LIS331HH_SPI_ReadRxData();
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[3] = LIS331HH_SPI_ReadRxData();
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[2] = LIS331HH_SPI_ReadRxData();
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[5] = LIS331HH_SPI_ReadRxData();
	LIS331HH_SPI_WriteTxData(0x00);
	Data->array[4] = LIS331HH_SPI_ReadRxData();
	CS_Write(1);
	CyDelay(100);
}
void get_HMC5883L_Data(union HMC5883L *Data)
{
//	HMC5883L_I2C_MasterClearStatus();
//	HMC5883L_I2C_MasterSendStart(HMC5883L_Address,0); //could throw error here
//	HMC5883L_I2C_MasterWriteByte(0x03); //could throw error here //select register 3, X MSB register
//	HMC5883L_I2C_MasterSendStop();
	uint8 array[6];
	
	
	i2c_read(0x03,array,6);
	
	Data->array[0] =  array[0];
	Data->array[1] =  array[1];
	Data->array[2] =  array[2];
	Data->array[3] =  array[3];
	Data->array[4] =  array[4];
	Data->array[5] =  array[5];
//	HMC5883L_I2C_MasterClearStatus();
//	HMC5883L_I2C_MasterSendStart(HMC5883L_Address,1);
//	Data->array[0]=HMC5883L_I2C_MasterReadByte(1);
//	Data->array[1]=HMC5883L_I2C_MasterReadByte(1);
//	Data->array[2]=HMC5883L_I2C_MasterReadByte(1);
//	Data->array[3]=HMC5883L_I2C_MasterReadByte(1);
//	Data->array[4]=HMC5883L_I2C_MasterReadByte(1);
//	Data->array[5]=HMC5883L_I2C_MasterReadByte(0);
	
}
void menu()
{
	CONFIG_COM_PutString("Welcome to the Desktop Tester Configuration & Testering Tool \n");
	CONFIG_COM_PutString("Menu:\n");
	CONFIG_COM_PutString("1: Test Acclermeter \n");
	CONFIG_COM_PutString("2: Test Magnetmeter \n");
	CONFIG_COM_PutString("3: Read Acclermeter \n");
	CONFIG_COM_PutString("4: Read Magnetmere \n");
	CONFIG_COM_PutString("5: Configuration \n");
}
void get_All_data(uint8 *array)
{
	//union LIS331HH *LIS331HH_Data;
	union HMC5883L *HMC5883L_Data;
	
	//get_LIS331HH_Data(LIS331HH_Data);
	//memcpy(array,LIS331HH_Data->array, 3 * sizeof(uint8)); //copies the Accelerometer data into the array
	get_HMC5883L_Data(HMC5883L_Data);
	memcpy(array + 6, HMC5883L_Data->array, 6 * sizeof(uint8)); // copies the magnetormeter data into the array 
}
uint8 i2c_write(uint8 subAddr, uint8 *buffer, uint8 buff_size) {

	int i;
	uint8 status;
	char array[10];
	HMC5883L_I2C_MasterClearStatus();

	
	//Set subaddr
	status = HMC5883L_I2C_MasterSendStart(HMC5883L_Address, 0);
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		i= sprintf(array, "1: %d",status);
		DATA_COM_PutArray(array,i);
		return status;
	}
   
     
	status = HMC5883L_I2C_MasterWriteByte(subAddr);
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		i= sprintf(array, "2: %d",status);
		DATA_COM_PutArray(array,i);
		return status;
	}
               
	for (i=0;i<buff_size;i++) {
		status = HMC5883L_I2C_MasterWriteByte(buffer[i]);
		if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
			return status;
			i= sprintf(array, "3: %d",status);
		DATA_COM_PutArray(array,i);
		}
	}
                  
		status = HMC5883L_I2C_MasterSendStop();
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		return status;
		i= sprintf(array, "4: %d",status);
		DATA_COM_PutArray(array,i);
	}   
	return status;
}
uint8 i2c_read(uint8 subAddr, uint8 *buffer, uint8 buff_size) {

	uint8 status;
	int i;
	char array[10];
	
	HMC5883L_I2C_MasterClearStatus();
DATA_COM_PutString("make it here?  \n"); 
	//Set subaddr
	status = HMC5883L_I2C_MasterSendStart(HMC5883L_Address, 0);
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		i= sprintf(array, "X: %d",status);
		DATA_COM_PutArray(array,i);
		return status;
	}
        DATA_COM_PutString("make it here?  \n");        
	buffer[0] = HMC5883L_I2C_MasterReadByte(0);
                   
	status = HMC5883L_I2C_MasterWriteByte(subAddr);
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		return status;
	}
               
	status = HMC5883L_I2C_MasterSendRestart(HMC5883L_Address, 1);
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
		return status;
	}               
               
	for (i=0;i<buff_size;i++) {
		buffer[i] = HMC5883L_I2C_MasterReadByte( (i<(buff_size-1) ) );
	}
 
	status = HMC5883L_I2C_MasterSendStop();
	if (status != HMC5883L_I2C_MSTR_NO_ERROR) {
        return status;
 	}  
	return 0;
}

/* [] END OF FILE */

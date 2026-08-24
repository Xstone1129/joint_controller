#include <igh_driver_cr.h> 

/*****************************************************************************/
void ODwrite(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint8_t objectValue)
{
	uint32_t abort_code;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, &objectValue, sizeof(objectValue), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("OD write fail:0X%08x\n",abort_code);
	else{
		printf("OD write successful\n");
	}
}

void ODwrite_u16(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint16_t objectValue)
{
	uint32_t abort_code;
	uint8_t objectBytes[2];
	objectBytes[0] = objectValue & 0xFF;
	objectBytes[1] = (objectValue >> 8) & 0xFF;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, objectBytes, sizeof(objectBytes), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("OD write fail:0X%08x\n",abort_code);
	else{
		printf("OD write successful\n");
	}
}

void ODwrite_u32(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint32_t objectValue)
{
	uint32_t abort_code;
	uint8_t objectBytes[4];
	objectBytes[0] = objectValue & 0xFF;
	objectBytes[1] = (objectValue >> 8) & 0xFF;
	objectBytes[2] = (objectValue >> 16) & 0xFF;
	objectBytes[3] = (objectValue >> 24) & 0xFF;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, objectBytes, sizeof(objectBytes), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("OD write fail:0X%08x\n",abort_code);
	else{
		printf("OD write successful\n");
	}
}

void ODwrite_607F(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex)
{
	uint32_t abort_code;
	uint8_t objectValue[2];
	objectValue[0] = 0x40;
	objectValue[1] = 0x9C;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, objectValue, sizeof(objectValue), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("ODwrite_607F fail:0X%08x\n",abort_code);
	else{
		printf("ODwrite_607F successful\n");
	}
}
void ODwrite_6080(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex)
{
	uint32_t abort_code;
	uint8_t objectValue[2];
	objectValue[0] = 0x40;
	objectValue[1] = 0x9C;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, objectValue, sizeof(objectValue), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("ODwrite_6080 fail:0X%08x\n",abort_code);
	else{
		printf("ODwrite_6080 successful\n");
	}
}

void ODwrite_6065(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex)
{
	uint32_t abort_code;
	uint8_t objectValue[2];
	objectValue[0] = 0x40;
	objectValue[1] = 0x9C;
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, objectValue, sizeof(objectValue), &abort_code);
	/* retVal != 0: Failure */
	if (retVal)
		printf("ODwrite_6065 fail:0X%08x\n",abort_code);
	else{
		printf("ODwrite_6065 successful\n");
	}
}

void initDrive_zero(ec_master_t* master, uint16_t slavePos , uint8_t axismode, uint16_t feedforward_gain)
{
	/* Mode of operation, fixed CSP. */
	ODwrite(master, slavePos, 0x6060, 0x00, axismode);
	/* Configure the position-loop to velocity-loop feed-forward term per axis group. */
	ODwrite_u16(master, slavePos, 0x2382, 0x03, feedforward_gain);
	/* Reset alarm */
	ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

void initDrive_eyou_basic(ec_master_t* master, uint16_t slavePos, uint8_t axismode)
{
	/* EYOU 先只保留基础模式设定和复位，避免写入 Zero 专用对象。 */
	ODwrite(master, slavePos, 0x6060, 0x00, axismode);
	ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

void initDrive_hcfa(ec_master_t* master, uint16_t slavePos , uint8_t axismode)
{

	/* Mode of operation, CSV */
	ODwrite(master, slavePos, 0x6060, 0x00, axismode);  // 0x09 for CSV mode
	ODwrite_607F(master, slavePos, 0x607F, 0x00);  // 0x09 for CSV mode
	ODwrite_6080(master, slavePos, 0x6080, 0x00);  // 0x09 for CSV mode
	ODwrite_6065(master, slavePos, 0x6065, 0x00);  // 0x09 for CSV mode
	/* Reset alarm */
	ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

void init_chage_Drive(ec_master_t* master, uint16_t slavePos , uint8_t axismode)
{

	/* Mode of operation, CSV */
	ODwrite(master, slavePos, 0x6060, 0x00, axismode);  // 0x09 for CSV mode
	/* Reset alarm */
	ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

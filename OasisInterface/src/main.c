/*   Jacob Simeone - O.A.S.I.S. Interface. A link between the central processor
 *   and the coprocessor of the O.A.S.I.S. project.
 *
 *   Copyright (C) 2023  Jacob Simeone
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.

 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

// =============== OASIS INTERFACE ===============

#define OAS_REG_SENS_0 0x01
#define OAS_REG_SENS_1 0x02
#define OAS_REG_SENS_2 0x03
#define OAS_REG_SENS_3 0x04

struct oasis_intf
{
  int fd;   // File descriptor
  int addr; // Slave address
  int err;  // Holds errors from interface
};

typedef struct oasis_intf *oasis_intf_handle;

void
oasis_interface_init(oasis_intf_handle pIntf, char *filename, int slaveAddr)
{
  int file;

  /**
   * CHECK IF FILENAME EXISTS
   *
   * Note that this function (access) is NOT safe to use typically, because in the time that the file is
   * momentarily open, a user can execute an attack in that short time to manipulate it. For our purposes, we really
   * don't care.
   */
  if (access(filename, F_OK) < 0)
  {
    pIntf->err = ENOENT; // No such file
    return;
  }

  file = open(filename, O_RDWR);
  if (file < 0)
  {
    pIntf->err = errno;
    return;
  }

  pIntf->fd = file;

  if (ioctl(file, I2C_SLAVE, slaveAddr) < 0)
  {
    pIntf->err = errno;
    return;
  }

  pIntf->addr = slaveAddr;
}

int
oasis_interface_get_sensor(oasis_intf_handle pIntf, uint8_t regAddr, uint16_t *pDistance)
{
	uint8_t dBuf[2] = {};
	long funcs = 0;

	// Check read functionality
	ioctl(pIntf->fd, I2C_FUNCS, &funcs);
	if (!(funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK)){
		// Indicates this functionality isn't supported
		pIntf->err = ENOTSUP;
		return -1;
	}

	if (i2c_smbus_read_i2c_block_data(pIntf->fd, regAddr, 2, dBuf) <= 0){
		// No data available
		pIntf->err = ENODATA;
		return -1;
	}

	*pDistance = (uint16_t)(dBuf[0] << 8 | dBuf[1]);
	
  return 0;
}

// =============== STATIC VARIABLES ===============

static struct oasis_intf intf = {};

// =============== MAIN ===============

int
main(int argc, char *argv[])
{
  // Retrieve args
  if (argc != 2)
  {
    fprintf(stderr, "Invalid args. Usage: oasis-interface [i2c-dev file]\n");
    exit(1);
  }

	// 0x76 is the address of the BME280 sensor
  oasis_interface_init(&intf, argv[1], 0x76);
  if (intf.err != 0)
  {
    fprintf(stderr, "[%s] Error intitializing I2C interface rc=%d\n", __func__, intf.err);
    if (intf.err == EACCES)
    {
      fprintf(stderr, "\tPermission denied... perhaps you need 'sudo' privileges\n");
    }
    exit(1);
  }

  printf("Initialized I2C OASIS interface\n");
	printf("Interface info (after init):\n\taddr: %d\n\tfd: %d\n\terr: %d\n", intf.addr, intf.fd, intf.err);

  // !TESTING
	// 0xD0 is ID register of BME280
	uint8_t id;
	size_t numRead = 0;
	numRead = i2c_smbus_read_i2c_block_data(intf.fd, 0xD0, 1, &id);
	if (numRead <= 0){
		fprintf(stderr, "Did not read any data...\nAborting...\n");
		exit(1);
	}

	printf("Read the id: 0x%02X from the BME280 sensor\n", id);
  // ! END TESTING
}

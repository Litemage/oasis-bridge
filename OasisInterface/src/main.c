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
oasis_interface_read_reg(oasis_intf_handle pIntf, uint8_t regAddr, uint16_t *regData)
{
  uint8_t dbuf[2] = {0, 0};

  // Check for errors
  if (pIntf->err != 0)
  {
    return -1;
  }

  struct i2c_msg msgs[2] = {
      {
          .addr = pIntf->addr,
          .flags = 0,
          .len = 1,
          .buf = &regAddr,
      },
      {
          .addr = pIntf->addr,
          .flags = I2C_M_RD,
          .len = 2,
          .buf = dbuf,
      },
  };

  struct i2c_rdwr_ioctl_data ioctlRdwr = {
      .msgs = msgs,
      .nmsgs = 2,
  };

  if (ioctl(pIntf->fd, I2C_RDWR, &ioctlRdwr) != 0)
  {
    pIntf->err = errno;
    return -1;
  }

	// TODO: Double-check if this is correct
	// Assign data to buffer
	*regData = (uint16_t)(dbuf[0] << 8 | dbuf[1]);

  return 0;
}

int oasis_interface_get_sensor(oasis_intf_handle pIntf, int index, uint16_t *pDistance){

	uint8_t reg = 0;

	switch (index) {
		case 0:
			index = OAS_REG_SENS_0;
			break;
		case 1:
			index = OAS_REG_SENS_1;
			break;
		case 2:
			index = OAS_REG_SENS_2;
			break;
		case 3:
			index = OAS_REG_SENS_3;
			break;
	}

	if(oasis_interface_read_reg(pIntf, reg, pDistance) < 0){
		// Error reading
		return -1;
	}

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

  oasis_interface_init(&intf, argv[1], 0x40);
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

	// !TESTING
	uint8_t reg = OAS_REG_SENS_0;

	if (write(intf.fd, &reg, 1) < 0){
		fprintf(stderr, "[%s] Failed to write to I2C interface\n", __func__);
		exit(1);
	}

	printf("Successfully wrote to i2c interface\n");

	if (read(intf.fd, &reg, 1) < 0) {
		fprintf(stderr, "[%s] Failed to read from I2C Interface\n", __func__);
		exit(1);
	}

	printf("Successfully read from I2C interface\n");
	// ! END TESTING
}

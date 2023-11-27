/*   O.A.S.I.S. Bridge. A link between the central processor
 *   and the coprocessor of the O.A.S.I.S. project.
 *
 *   Copyright (C) 2023  Jacob Simeone <jsimeone0105 at gamil dot com>
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
 *
 *   ===TODO PARKING===
 *
 *   - [ ] Separate interface into static libraries
 */

#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

// Redis client
#include <hiredis/hiredis.h>

#define SUPER_FREQ_MS (500)
#define OASIS_DB_IP   ("127.0.0.1")
#define OASIS_DB_PORT (6379)

// =============== OASIS BRIDGE ===============

/**
 * @brief "Registers" of peripheral
 *
 */
enum OasisReg
{
  OAS_REG_SENS_0 = 0x10, // Left sensor
  OAS_REG_SENS_1,        // Front sensor
  OAS_REG_SENS_2,        // Right Sensor
  OAS_REG_SENS_3,        // Downward Sensor
};

struct oasis_intf
{
  int fd;               // File descriptor
  int addr;             // Slave address
  int err;              // Holds errors from i2c bridge
  int dbErr;            // Database error
  redisContext *dbCtxt; // Context of database connection
};

typedef struct
{
  // Distances (from distance sensors)
  uint16_t left;
  uint16_t forward;
  uint16_t right;
  uint16_t down;
} SensorVals_t;

/**
 * Type alias for a pointer to a oasis bridge configuration structure.
 *
 * Consumed by pretty much all API functions
 */
typedef struct oasis_intf *oasis_intf_handle;

void
sleep_ms(int ms)
{
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000;
  nanosleep(&ts, NULL);
}

/**
 * @brief Initializes the bridge to coprocessor over i2c, as well as sets the slave address
 *
 * @param pIntf pointer to an empty bridge handle
 * @param filename The name of the device file to access for i2c i.e: "/dev/i2c-0"
 * @param slaveAddr Address of slave, which has implemented the expected O.A.S.I.S target registers
 */
void oasis_bridge_init(oasis_intf_handle pIntf, char *filename, int slaveAddr);

/**
 * @brief Perform an i2c read to retrieve the value of a sensor at some register. Will always read a 16-bit word.
 *
 * @param pIntf An initialized oasis bridge handle
 * @param regAddr 8-bit register address to access on target device
 * @param [out] pDistnace Output buffer for distance (value of 8-bit register regAddr)
 */
int oasis_bridge_get_sensor(oasis_intf_handle pIntf, uint8_t regAddr, uint16_t *pDistance);

/**
 * @brief
 *
 * @param pIntf An initialized oasis bridge handle
 * @param [out] pDbuf Output data buffer for all sensor values
 *
 * @return 0 on success, -1 on error
 */
int oasis_bridge_get_distances(oasis_intf_handle pIntf, SensorVals_t *pDbuf);

/**
 * @brief Stores data into redis db
 *
 * @return int 0 on success, -1 on failure
 */
int oasis_bridge_store_data(SensorVals_t *pSensVals);

// =============== DATABASE INTERFACE ===============

/**
 * @brief Connects to redis at location specified in parameters
 *
 * @param [in] pIp String representing the IP address of redis server
 * @param port Port that the redis server is running on
 * @param [out] pCtxt Pointer to a redis context pointer to give back to user. NULL if failed
 *
 * @return int 0 on success, -1 on fail, errno set accordingly.
 */
int oasis_db_connect(redisContext **pCtxt, char *ip, int port);

/**
 * @brief Post sensor values to a redis stream in local database
 *
 * @param [in] pCtxt Pointer to a redis context used for accessing db
 * @param [in] pSensVals Pointer to a buffer of sensor values that will be posted to database
 */
int oasis_db_post(redisContext *pCtxt, SensorVals_t *pSensVals);

/**
 * @brief Closes connection and frees memory used by context object
 *
 * @param [in] pCtxt Pointer to redis context needing to be freed
 */
void oasis_db_end(redisContext *pCtxt);

// =============== MISC FUNCTIONS ===============

/**
 * @brief Dumps help for program
 */
void help(void);

// =============== STATIC VARIABLES ===============

static struct oasis_intf intf = {};
static SensorVals_t distances = {};
static redisContext *pRedisContext = NULL;

// =============== MAIN ===============

void
db_task()
{
  static int reconCnt = 0;
  int rc;

  // Protect against constant reconnections
  if (reconCnt > 10)
  {
    fprintf(stderr, "Database errored out too many times:(%d), disabling...\n", reconCnt);
    oasis_db_end(pRedisContext);
    return;
  }

  rc = oasis_db_post(pRedisContext, &distances);
  if (rc != 0)
  {
    // Assume connection failure. Ignore errors for now
    redisReconnect(pRedisContext);
    reconCnt++;
  }
}

void
sensor_read_task()
{
  // Ignore errors for now...
  oasis_bridge_get_distances(&intf, &distances);
}

int
main(int argc, char *argv[])
{

  // Retrieve args
  if (argc != 2)
  {
    help();
    exit(1);
  }

  // 0x76 is the address of the BME280 sensor
  oasis_bridge_init(&intf, argv[1], 0x76);
  if (intf.err != 0)
  {
    fprintf(stderr, "[%s] Error intitializing I2C interface rc=%d (%s)\n", __func__, intf.err, strerror(intf.err));
    if (intf.err == EACCES)
    {
      fprintf(stderr, "\tPermission denied... perhaps you need 'sudo' privileges\n");
    }
    exit(1);
  }

  printf("Initialized I2C OASIS bridge\n");

  if (oasis_db_connect(&pRedisContext, OASIS_DB_IP, OASIS_DB_PORT) != 0)
  {
    fprintf(stderr,
            "[%s] Error initializing redis database at [%s:%d] rc=%d (%s)\n",
            __func__,
            OASIS_DB_IP,
            OASIS_DB_PORT,
            errno,
            strerror(errno));
    if (errno == ECONNREFUSED)
    {
      fprintf(stderr, "\tConnection refused, is the redis server running?\n");
    }
    exit(1);
  }

  printf("Initialized connection to Redis DB\n");

  while (1)
  {
    // Handle getting sensor data
    sensor_read_task();

    // Handle database affairs
    db_task();

    printf("Ran task...\n");

    // The thread is eeby and neeby to seeby
    sleep_ms(SUPER_FREQ_MS);
  }
}

// =============== PRIVATE DEFINITIONS ===============

void
oasis_bridge_init(oasis_intf_handle pIntf, char *filename, int slaveAddr)
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
oasis_bridge_get_sensor(oasis_intf_handle pIntf, uint8_t regAddr, uint16_t *pDistance)
{
  uint8_t dBuf[2] = {};
  long funcs = 0;

  // Check read functionality
  ioctl(pIntf->fd, I2C_FUNCS, &funcs);
  if (!(funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK))
  {
    // Indicates this functionality isn't supported
    pIntf->err = ENOTSUP;
    return -1;
  }

  if (i2c_smbus_read_i2c_block_data(pIntf->fd, regAddr, 2, dBuf) <= 0)
  {
    // No data available
    pIntf->err = ENODATA;
    return -1;
  }

  *pDistance = (uint16_t)(dBuf[0] << 8 | dBuf[1]);

  return 0;
}

int
oasis_bridge_get_distances(oasis_intf_handle pIntf, SensorVals_t *pDbuf)
{
  uint16_t tmpBuf;

  if (oasis_bridge_get_sensor(pIntf, OAS_REG_SENS_0, &tmpBuf) != 0)
  {
    return -1;
  }
  pDbuf->left = tmpBuf;

  if (oasis_bridge_get_sensor(pIntf, OAS_REG_SENS_1, &tmpBuf) != 0)
  {
    return -1;
  }
  pDbuf->forward = tmpBuf;

  if (oasis_bridge_get_sensor(pIntf, OAS_REG_SENS_2, &tmpBuf) != 0)
  {
    return -1;
  }
  pDbuf->right = tmpBuf;

  if (oasis_bridge_get_sensor(pIntf, OAS_REG_SENS_3, &tmpBuf) != 0)
  {
    return -1;
  }
  pDbuf->down = tmpBuf;

  return 0;
}

int
oasis_db_connect(redisContext **pCtxt, char *ip, int port)
{
  *pCtxt = redisConnect(ip, port);
  if (*pCtxt == NULL || (*pCtxt)->err)
  {
    if (*pCtxt)
    {
      // Allocated, but failed to connect
      return -1;
    }
    else
    {
      errno = ENOMEM;
      return -1;
    }
  }

  return 0;
}

int
oasis_db_post(redisContext *pCtxt, SensorVals_t *pSensVals)
{
  void *pReply;

  pReply = redisCommand(pCtxt,
                        "XADD robot:sensors * left_cm %d front_cm %d right_cm %d down_cm %d",
                        pSensVals->left,
                        pSensVals->forward,
                        pSensVals->right,
                        pSensVals->down);

  // For now, just assume success if there was no error in the transaction.
  if (pReply == NULL)
  {
    return -1;
  }

  return 0;
}

void
oasis_db_end(redisContext *pCtxt)
{
  redisFree(pCtxt);
}

void
help(void)
{
  printf("O.A.S.I.S bridge service for retrieving data from a co-processor, which implements the expected registers\n"
         "   usage: oasis-bridge [i2c-dev file]\n"
         "   i2c-dev file: a device file representing a i2c device adapter, like '/dev/i2c-0'\n\n");
}

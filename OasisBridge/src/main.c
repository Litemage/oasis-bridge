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
 *   - [X] Add option to post data to redis stream as well as pub/sub
 *   - [X] Move to UART communication interface
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h> // POSIX terminal configuration
#include <time.h>
#include <unistd.h>

// Redis client
#include <hiredis/hiredis.h>

#define PI                 (3.14159265)
#define SUPER_FREQ_MS      (500)
#define OASIS_DB_IP        ("127.0.0.1")
#define OASIS_DB_PORT      (6379)
#define SAMPLE_FREQ        (500)

// Serial interface commands & Settings
#define OASIS_CMD_GET_DATA (0x0A) // Signals remote processor to send data
#define PACKET_SIZE        (8)    // Size of packet we're expecting to receive (SensorValues_t)
#define RX_BUF_SIZE        (64)   // Size of local rx buffer

// =============== OASIS BRIDGE ===============

// TYPES
struct oasis_intf
{
  int err;
  int fd;
  redisContext *dbCtxt; // Context of database connection
};

typedef struct
{
  // Distances (from distance sensors)
  uint16_t left;
  uint16_t forward;
  uint16_t right;
  uint16_t frntSensAngle;
} SensorVals_t;

/**
 * Type alias for a pointer to a oasis bridge configuration structure.
 *
 * Consumed by pretty much all API functions
 */
typedef struct oasis_intf *oasis_intf_handle;

// OASIS BRIDGE FUNCTIONS

/**
 * @brief Tries to open an interface to serial device at filename.
 *   Tries to configure with 8n1, non-canonical mode, 115200 baud rate.
 *   Returns nothing, but pIntf->err will be set with an error if failed.
 *
 * @param pIntf Oasis interface handle to initialize.
 * @param [in] filename Name of device file to attempt to open. Should be of the pattern: "/dev/tty*"
 *
 */
void oasis_bridge_init(oasis_intf_handle pIntf, char *filename);

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
 * @brief Posts data given to redis stream
 *
 * @param [in] pCtxt Initialized redis server context
 * @param [in] pVals sensor values to post to the stream
 *
 * @return int 0 on success, -1 on fail
 */
int oasis_db_post_stream(redisContext *pCtxt, SensorVals_t *pVals);

/**
 * @brief Publishes the given value to the specified topic
 *
 * @param [in] pCtxt Initialized redis server context
 * @param [in] pTopic Topic to publish too. Should be something like: oasis.front
 * @param sensorVal Sensor value to publish to database
 *
 * @return int 0 on success, -1 on fail
 */
int oasis_db_publish(redisContext *pCtxt, char *pTopic, uint16_t sensorVal);

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

/**
 * @brief Uses posix standard functions to sleep for some number of ms
 *   NOT PORTABLE
 *
 * @param ms Number of ms to sleep for
 */
void
sleep_ms(int ms)
{
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000;
  nanosleep(&ts, NULL);
}

// =============== STATIC VARIABLES ===============

static struct oasis_intf intf = {};
static redisContext *pRedisContext = NULL;

// =============== MAIN ===============

void
db_task(SensorVals_t *pSensorVals)
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

  // Posts data to pub/sub in redis
  rc |= oasis_db_post(pRedisContext, pSensorVals);
  // Posts data to stream
  rc |= oasis_db_post_stream(pRedisContext, pSensorVals);
  if (rc != 0)
  {
    // Assume connection failure. Ignore errors for now
    redisReconnect(pRedisContext);
    reconCnt++;
  }
}

int
coprocessor_read_task(SensorVals_t *pSensorVals)
{
  int failedReads = 0;
  uint8_t rxBuf[RX_BUF_SIZE];
  uint8_t txChar = OASIS_CMD_GET_DATA;
  ssize_t txNum, rxNum, numReceived;

  // Note that this request-response code is a little cheesed...
  // Write request character
  txNum = write(intf.fd, &txChar, 1);
  if (txNum < 0)
  {
    fprintf(stderr, "ERROR: write(2) syscall failed rc=%d (%s)\n", errno, strerror(errno));
    sleep_ms(SAMPLE_FREQ);
    return -1;
  }

  while (numReceived < PACKET_SIZE)
  {
    if (failedReads > 3)
    {
      fprintf(stderr, "ERROR: Failed to retrieve message, not enough data received\n");
      return -1;
    }

    rxNum = read(intf.fd, rxBuf + numReceived, sizeof(rxBuf) - numReceived);

    if (rxNum < 0)
    {
      fprintf(stderr, "ERROR: read(2) syscall failed rc=%d (%s)\n", errno, strerror(errno));
      return -1;
    }
    else if (rxNum == 0)
    {
      failedReads++;
      continue;
    }
    else
    {
      numReceived += rxNum;
    }
  }

  // numReceived being 0 indicates an error, also if we have more data than we expected
  if (numReceived != 0 && numReceived == PACKET_SIZE)
  {
    // TESTING
    printf("Rx buf: [");
    for (int i = 0; i < numReceived; ++i)
    {
      printf("%02X ", rxBuf[i]);
    }
    printf("]\n");
    // END

    // Note that blindly copying memory isn't the safest thing in the world
    *pSensorVals = *((SensorVals_t *)rxBuf);
  }
  else
  {
    fprintf(stderr, "ERROR: Received incorrect number of bytes for packet numReceived=%ld\n", numReceived);

    return -1;
  }

  return 0;
}

int
main(int argc, char *argv[])
{
  int rc;
  SensorVals_t vals;

  // Retrieve args
  if (argc != 2)
  {
    help();
    exit(1);
  }

  oasis_bridge_init(&intf, argv[1]);
  if (intf.err != 0)
  {
    fprintf(stderr, "[%s] Error intitializing UART interface rc=%d (%s)\n", __func__, intf.err, strerror(intf.err));
    if (intf.err == EACCES)
    {
      fprintf(stderr,
              "\tPermission denied... perhaps you need 'sudo' privileges?\n\tEnsure that your user is part of the "
              "dialout group. If not, run: \"sudo adduser $USER dialout\"\n");
    }
    exit(1);
  }

  printf("Initialized UART OASIS bridge\n");

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
    if (coprocessor_read_task(&vals) == 0)
    {
      db_task(&vals);
    }
    else
    {
      fprintf(stderr, "ERROR: Coprocessor read task failed\n");
    }

    // The thread is eeby and neeby to seeby
    sleep_ms(SUPER_FREQ_MS);
  }
}

// =============== PRIVATE DEFINITIONS ===============

void
oasis_bridge_init(oasis_intf_handle pIntf, char *filename)
{
  /**
   * Assuming that we are communicating with an arduino with the following RS232 params:
   * - 8 data bits
   * - no parity
   * - 1 stop bit
   * - not using flow control
   * (8n1)
   */
  int file;
  struct termios options;

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

  file = open(filename, O_RDWR | O_NOCTTY);
  if (file < 0)
  {
    pIntf->err = errno;
    return;
  }

  pIntf->fd = file;

  // Get current terminal options
  if (tcgetattr(pIntf->fd, &options) != 0)
  {
    pIntf->err = errno;
  }

  // Set baud rate to 115200
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // Retrieved from man page example for raw data
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_cflag &= ~(CSIZE | PARENB);
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;  // bytes; Number of bytes to retrieve before returning
  options.c_cc[VTIME] = 0; // deciseconds; Timeout before read(2) returns

  tcsetattr(pIntf->fd, TCSANOW, &options);

  /*
   * Flush any data that may already be in the interface
   *
   * Sleep needs to be here for tcflush to work properly,
   *   according to some online sources.
   */
  sleep(2);
  tcflush(pIntf->fd, TCIOFLUSH);
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
oasis_db_post_stream(redisContext *pCtxt, SensorVals_t *pVals)
{
  void *pReply;

  pReply = redisCommand(pCtxt,
                        "XADD oasis:sensors * left_cm %d front_cm %d right_cm %d forward_angle_deg %d",
                        pVals->left,
                        pVals->forward,
                        pVals->right,
                        pVals->frntSensAngle);

  // For now, just assume success if there was no error in the transaction.
  if (pReply == NULL)
  {
    return -1;
  }

  return 0;
}

int
oasis_db_publish(redisContext *pCtxt, char *pTopic, uint16_t sensorVal)
{
  void *pReply;

  pReply = redisCommand(pCtxt, "PUBLISH %s %u", pTopic, sensorVal);

  if (pReply == NULL)
  {
    return -1;
  }

  freeReplyObject(pReply);
  return 0;
}

int
oasis_db_post(redisContext *pCtxt, SensorVals_t *pSensVals)
{
  void *pReply;
  int rc = 0;

  rc |= oasis_db_publish(pCtxt, "oasis.forward", pSensVals->forward);
  rc |= oasis_db_publish(pCtxt, "oasis.left", pSensVals->left);
  rc |= oasis_db_publish(pCtxt, "oasis.right", pSensVals->right);
  rc |= oasis_db_publish(pCtxt, "oasis.forward_angle_deg", pSensVals->frntSensAngle);

  if (rc != 0)
  {
    // Failed to publish in some way
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
  printf("O.A.S.I.S bridge service for retrieving data from a co-processor, over RS-232 interface\n"
         "   usage: oasis-bridge [serial dev file]\n"
         "   serial dev file: a device file representing a serial device adapter, like '/dev/ttyx'\n\n");
}

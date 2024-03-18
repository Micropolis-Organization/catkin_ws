#ifndef __CAN_INTERFACE_HPP__
#define __CAN_INTERFACE_HPP__

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include <sstream>
#include <iomanip>
class CAN_Interface
{
private:
#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000
#define CAN_BUS_SPEED 1000000

   // TODO char
   char *USB_PORT ;
   char CAN_IDENTIFER[3] = "5";

   typedef enum
   {
      CANUSB_SPEED_Default = 0,
      CANUSB_SPEED_1000000 = 0x01,
      CANUSB_SPEED_800000 = 0x02,
      CANUSB_SPEED_500000 = 0x03,
      CANUSB_SPEED_400000 = 0x04,
      CANUSB_SPEED_250000 = 0x05,
      CANUSB_SPEED_200000 = 0x06,
      CANUSB_SPEED_125000 = 0x07,
      CANUSB_SPEED_100000 = 0x08,
      CANUSB_SPEED_50000 = 0x09,
      CANUSB_SPEED_20000 = 0x0a,
      CANUSB_SPEED_10000 = 0x0b,
      CANUSB_SPEED_5000 = 0x0c,
   } CANUSB_SPEED;

   typedef enum
   {
      CANUSB_MODE_NORMAL = 0x00,
      CANUSB_MODE_LOOPBACK = 0x01,
      CANUSB_MODE_SILENT = 0x02,
      CANUSB_MODE_LOOPBACK_SILENT = 0x03,
   } CANUSB_MODE;

   typedef enum
   {
      CANUSB_FRAME_STANDARD = 0x01,
      CANUSB_FRAME_EXTENDED = 0x02,
   } CANUSB_FRAME;

   typedef enum
   {
      CANUSB_INJECT_PAYLOAD_MODE_RANDOM = 0,
      CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
      CANUSB_INJECT_PAYLOAD_MODE_FIXED = 2,
   } CANUSB_PAYLOAD_MODE;

   int terminate_after,
       inject_payload_mode,
       print_traffic;

   float inject_sleep_gap;

   CANUSB_SPEED canusb_int_to_speed(int speed);
   struct timespec timespec_diff(struct timespec start, struct timespec end);
   int generate_checksum(const unsigned char *data, int data_len);
   int frame_is_complete(const unsigned char *frame, int frame_len);
   int frame_send(int tty_fd, const unsigned char *frame, int frame_len);
   int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max);
   int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame);
   int send_data_frame(int tty_fd, CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, const unsigned char data[], int data_length_code);
   int hex_value(int c);
   int convert_from_hex(const char *hex_string, unsigned char *bin_string, int bin_string_len);
   int inject_data_frame(int tty_fd, const char *hex_id, unsigned const char *hex_data, int data_len);
   void dump_data_frames(int tty_fd);
   int adapter_init(const char *tty_device, int baudrate);
   static void sigterm(int signo);


   int c,
       tty_fd,
       baudrate;
 
   float temp;
   char *inject_id = CAN_IDENTIFER;
   CANUSB_SPEED speed;

   char *inject_data ;

   float current_velocity,
         current_steering,
         prev_velocity,
         prev_steering;
         
public:
   void update_loop(float velocity, float steering);
   CAN_Interface(char *port);
   ~CAN_Interface();
};

#endif //__CAN_INTERFACE_HPP__
#include "definitions.hpp"

#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000

char USB_PORT[] = "/dev/ttyUSB1";
#define CAN_BUS_SPEED 1000000
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

static int terminate_after = 1;
static int program_running = 1;
static int inject_payload_mode = CANUSB_INJECT_PAYLOAD_MODE_FIXED;
static float inject_sleep_gap = CANUSB_INJECT_SLEEP_GAP_DEFAULT;
static int print_traffic = 0;

static CANUSB_SPEED canusb_int_to_speed(int speed)
{
  switch (speed)
  {
  case 1000000:
    return CANUSB_SPEED_1000000;
  case 800000:
    return CANUSB_SPEED_800000;
  case 500000:
    return CANUSB_SPEED_500000;
  case 400000:
    return CANUSB_SPEED_400000;
  case 250000:
    return CANUSB_SPEED_250000;
  case 200000:
    return CANUSB_SPEED_200000;
  case 125000:
    return CANUSB_SPEED_125000;
  case 100000:
    return CANUSB_SPEED_100000;
  case 50000:
    return CANUSB_SPEED_50000;
  case 20000:
    return CANUSB_SPEED_20000;
  case 10000:
    return CANUSB_SPEED_10000;
  case 5000:
    return CANUSB_SPEED_5000;
  default:
    return CANUSB_SPEED_Default;
  }
}

struct timespec timespec_diff(struct timespec start, struct timespec end)
{
  struct timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

static int generate_checksum(const unsigned char *data, int data_len)
{
  int i, checksum;

  checksum = 0;
  for (i = 0; i < data_len; i++)
  {
    checksum += data[i];
  }

  return checksum & 0xff;
}

static int frame_is_complete(const unsigned char *frame, int frame_len)
{
  if (frame_len > 0)
  {
    if (frame[0] != 0xaa)
    {
      /* Need to sync on 0xaa at start of frames, so just skip. */
      return 1;
    }
  }

  if (frame_len < 2)
  {
    return 0;
  }

  if (frame[1] == 0x55)
  { /* Command frame... */
    if (frame_len >= 20)
    { /* ...always 20 bytes. */
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else if ((frame[1] >> 4) == 0xc)
  { /* Data frame... */
    if (frame_len >= (frame[1] & 0xf) + 5)
    { /* ...payload and 5 bytes. */
      return 1;
    }
    else
    {
      return 0;
    }
  }

  /* Unhandled frame type. */
  return 1;
}

static int frame_send(int tty_fd, const unsigned char *frame, int frame_len)
{
  int result, i;

  if (print_traffic)
  {
    printf(">>> ");
    for (i = 0; i < frame_len; i++)
    {
      printf("%02x ", frame[i]);
    }
    if (print_traffic > 1)
    {
      printf("    '");
      for (i = 4; i < frame_len - 1; i++)
      {
        printf("%c", isalnum(frame[i]) ? frame[i] : '.');
      }
      printf("'");
    }
    printf("\n");
  }

  result = write(tty_fd, frame, frame_len);
  if (result == -1)
  {
    printf("error");
    fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  return frame_len;
}

static int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max)
{
  int result, frame_len, checksum;
  unsigned char byte;

  if (print_traffic)
    fprintf(stderr, "<<< ");

  frame_len = 0;
  while (program_running)
  {
    result = read(tty_fd, &byte, 1);
    if (result == -1)
    {
      if (errno != EAGAIN && errno != EWOULDBLOCK)
      {
        fprintf(stderr, "read() failed: %s\n", strerror(errno));
        return -1;
      }
    }
    else if (result > 0)
    {
      if (print_traffic)
        fprintf(stderr, "%02x ", byte);

      if (frame_len == frame_len_max)
      {
        fprintf(stderr, "frame_recv() failed: Overflow\n");
        return -1;
      }

      frame[frame_len++] = byte;

      if (frame_is_complete(frame, frame_len))
      {
        break;
      }
    }

    usleep(10);
  }

  if (print_traffic)
    fprintf(stderr, "\n");

  /* Compare checksum for command frames only. */
  if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55))
  {
    checksum = generate_checksum(&frame[2], 17);
    if (checksum != frame[frame_len - 1])
    {
      fprintf(stderr, "frame_recv() failed: Checksum incorrect\n");
      return -1;
    }
  }

  return frame_len;
}

static int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
  int cmd_frame_len;
  unsigned char cmd_frame[20];

  cmd_frame_len = 0;
  cmd_frame[cmd_frame_len++] = 0xaa;
  cmd_frame[cmd_frame_len++] = 0x55;
  cmd_frame[cmd_frame_len++] = 0x12;
  cmd_frame[cmd_frame_len++] = speed;
  cmd_frame[cmd_frame_len++] = frame;
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = mode;
  cmd_frame[cmd_frame_len++] = 0x01;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);

  if (frame_send(tty_fd, cmd_frame, cmd_frame_len) < 0)
  {
    printf("Error");
    return -1;
  }

  return 0;
}

static int send_data_frame(int tty_fd, CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, const unsigned char data[], int data_length_code)
{
#define MAX_FRAME_SIZE 13
  int data_frame_len = 0;
  unsigned char data_frame[MAX_FRAME_SIZE] = {0x00};

  if (data_length_code < 0 || data_length_code > 8)
  {
    fprintf(stderr, "Data length code (DLC) must be between 0 and 8!\n");
    return -1;
  }

  /* Byte 0: Packet Start */
  data_frame[data_frame_len++] = 0xaa;

  /* Byte 1: CAN Bus Data Frame Information */
  data_frame[data_frame_len] = 0x00;
  data_frame[data_frame_len] |= 0xC0; /* Bit 7 Always 1, Bit 6 Always 1 */
  if (frame == CANUSB_FRAME_STANDARD)
    data_frame[data_frame_len] &= 0xDF;           /* STD frame */
  else                                            /* CANUSB_FRAME_EXTENDED */
    data_frame[data_frame_len] |= 0x20;           /* EXT frame */
  data_frame[data_frame_len] &= 0xEF;             /* 0=Data */
  data_frame[data_frame_len] |= data_length_code; /* DLC=data_len */
  data_frame_len++;

  /* Byte 2 to 3: ID */
  data_frame[data_frame_len++] = id_lsb; /* lsb */
  data_frame[data_frame_len++] = id_msb; /* msb */

  /* Byte 4 to (4+data_len): Data */
  for (int i = 0; i < data_length_code; i++)
    data_frame[data_frame_len++] = data[i];

  /* Last byte: End of frame */
  data_frame[data_frame_len++] = 0x55;

  if (frame_send(tty_fd, data_frame, data_frame_len) < 0)
  {
    fprintf(stderr, "Unable to send frame!\n");
    return -1;
  }

  return 0;
}

static int hex_value(int c)
{
  if (c >= 0x30 && c <= 0x39) /* '0' - '9' */
    return c - 0x30;
  else if (c >= 0x41 && c <= 0x46) /* 'A' - 'F' */
    return (c - 0x41) + 10;
  else if (c >= 0x61 && c <= 0x66) /* 'a' - 'f' */
    return (c - 0x61) + 10;
  else
    return -1;
}

static int convert_from_hex(const char *hex_string, unsigned char *bin_string, int bin_string_len)
{
  int n1, n2, high;

  high = -1;
  n1 = n2 = 0;
  while (hex_string[n1] != '\0')
  {
    if (hex_value(hex_string[n1]) >= 0)
    {
      if (high == -1)
      {
        high = hex_string[n1];
      }
      else
      {
        bin_string[n2] = hex_value(high) * 16 + hex_value(hex_string[n1]);
        n2++;
        if (n2 >= bin_string_len)
        {
          printf("hex string truncated to %d bytes\n", n2);
          break;
        }
        high = -1;
      }
    }
    n1++;
  }

  return n2;
}

static int inject_data_frame(int tty_fd, const char *hex_id, unsigned const char *hex_data, int data_len)
{

  unsigned char binary_data[8];
  unsigned char binary_id_lsb = 0, binary_id_msb = 0;
  struct timespec gap_ts;
  struct timeval now;
  int error = 0;

  gap_ts.tv_sec = inject_sleep_gap / 1000;
  gap_ts.tv_nsec = (long)(((long long)(inject_sleep_gap * 1000000)) % 1000000000LL);

  /* Set seed value for pseudo random numbers. */
  gettimeofday(&now, NULL);
  srandom(now.tv_usec);

  // data_len = convert_from_hex(hex_data, binary_data, sizeof(binary_data));
  if (data_len == 0)
  {
    fprintf(stderr, "Unable to convert data from hex to binary!\n");
    return -1;
  }

  switch (strlen(hex_id))
  {
  case 1:
    binary_id_lsb = hex_value(hex_id[0]);
    break;

  case 2:
    binary_id_lsb = (hex_value(hex_id[0]) * 16) + hex_value(hex_id[1]);
    break;

  case 3:
    binary_id_msb = hex_value(hex_id[0]);
    binary_id_lsb = (hex_value(hex_id[1]) * 16) + hex_value(hex_id[2]);
    break;

  default:
    fprintf(stderr, "Unable to convert ID from hex to binary!\n");
    return -1;
  }

  error = send_data_frame(tty_fd, CANUSB_FRAME_STANDARD, binary_id_lsb, binary_id_msb, hex_data, data_len);

  return error;
}

static void dump_data_frames(int tty_fd)
{
  int i, frame_len;
  unsigned char frame[32];
  struct timespec ts;

  while (program_running)
  {
    frame_len = frame_recv(tty_fd, frame, sizeof(frame));

    if (!program_running)
      break;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);

    if (frame_len == -1)
    {
      printf("Frame recieve error!\n");
    }
    else
    {

      if ((frame_len >= 6) &&
          (frame[0] == 0xaa) &&
          ((frame[1] >> 4) == 0xc))
      {
        printf("Frame ID: %02x%02x, Data: ", frame[3], frame[2]);
        for (i = frame_len - 2; i > 3; i--)
        {
          printf("%02x ", frame[i]);
        }
        printf("\n");
      }
      else
      {
        printf("Unknown: ");
        for (i = 0; i <= frame_len; i++)
        {
          printf("%02x ", frame[i]);
        }
        printf("\n");
      }
    }

    if (terminate_after && (--terminate_after == 0))
      program_running = 0;
  }
}

static int adapter_init(const char *tty_device, int baudrate)
{
  int tty_fd, result;
  struct termios2 tio;

  tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (tty_fd == -1)
  {
    fprintf(stderr, "open(%s) failed: %s\n", tty_device, strerror(errno));
    return -1;
  }

  result = ioctl(tty_fd, TCGETS2, &tio);
  if (result == -1)
  {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag = BOTHER | CS8 | CSTOPB;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  result = ioctl(tty_fd, TCSETS2, &tio);
  if (result == -1)
  {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  return tty_fd;
}

static void sigterm(int signo)
{
  program_running = 0;
}
float velocity = 50.0,
      steering = 50.0;
float prev_velocity = 50,
      prev_steering = 50;

// void getRosParam(std::string paramName, auto &paramValue)
// {
//    if (n.getParam(paramName, paramValue))
//    {
//       std::stringstream strg;
//       strg << paramValue;
//       std::string s = strg.str();
//       ROS_INFO("[IWebotsRosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
//    }
//    else
//    {
//       ROS_WARN("[IWebotsRosNode] [PARAM] %s is not set", paramName.c_str());
//    }
// }

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CAN_node");
  ros::NodeHandle n;

  ros::Subscriber velocity_sub = n.subscribe<std_msgs::Float32>("velocity", 1, [&](const std_msgs::Float32::ConstPtr &velocityMsg)
                                                                { velocity = velocityMsg->data; 
                                                                  std::cout<<"velocity: " <<velocity<<std::endl; });

  ros::Subscriber steering_sub = n.subscribe<std_msgs::Float32>("steering_rad", 1, [&](const std_msgs::Float32::ConstPtr &steeringMsg)
                                                                { steering = steeringMsg->data;
                                                                std::cout<<"steering: " <<steering<<std::endl; });
//   ros::Rate loop_rate(10);

  int c, tty_fd;
  char *inject_id = CAN_IDENTIFER;
  CANUSB_SPEED speed = canusb_int_to_speed(CAN_BUS_SPEED);
  // auto port;
  // if (n.getParam("/can_node/port", port));
  // ROS_INFO("port: %s", port);
  int baudrate = CANUSB_TTY_BAUD_RATE_DEFAULT;
  // char hex_id=CAN_IDENTIFER;
  float temp;
  signal(SIGTERM, sigterm);
  signal(SIGHUP, sigterm);
  signal(SIGINT, sigterm);

  tty_fd = adapter_init(USB_PORT, baudrate);
  if (tty_fd == -1)
  {
    printf("Wrong in init adpater");
    return EXIT_FAILURE;
  }

  command_settings(tty_fd, speed, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);
  // char *inject_data = "S:23.6";
  char inject_data[] = "S:%.2f";

  inject_id = "140";
  while (ros::ok())
  {
    std::stringstream s_drive, s_steer;
    if (steering != prev_steering)
    {
      s_steer << "S:" << std::fixed << std::setprecision(2) << steering;
      const char *data_steering = s_steer.str().c_str();
      ROS_INFO("%s", s_steer.str().c_str());

      if (inject_data_frame(tty_fd, inject_id, (unsigned char *)data_steering, strlen(inject_data)) == -1)
      {
        return EXIT_FAILURE;
      }
      // else
      // {
      //   loop_rate.sleep();
      // }
      prev_steering = steering;
    }

    if (velocity != prev_velocity)
    {

      s_drive << "D:" << std::fixed << std::setprecision(2) << velocity;
      const char *data = s_drive.str().c_str();
      ROS_INFO("%s", s_drive.str().c_str());

      if (inject_data_frame(tty_fd, inject_id, (unsigned char *)data, strlen(inject_data)) == -1)
      {
        return EXIT_FAILURE;
      }
      // else
      // {
      //   loop_rate.sleep();
      // }
      prev_velocity = velocity;
    }

    ros::spinOnce();
  }

  return 0;
}

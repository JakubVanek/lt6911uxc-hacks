/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
Copyright (c) 2017, Ben Kazemi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <getopt.h>
#include <string.h>

#include <signal.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <netinet/in.h>

#include <pigpio.h>

#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

// Do the GPIO waggling from here, except that needs root access, plus
// there is variation on pin allocation between the various Pi platforms.
// Provided for reference, but needs tweaking to be useful.
//#define DO_PIN_CONFIG

#define REG_BANK    0xFF
#define REG_CHIP_ID 0x8100
#define REG_MIPI_TX 0x811D
#define MIPI_TX_ON 0xFB
#define MIPI_TX_OFF 0x00
#define REG_I2C_EN  0x80EE
#define REG_WD_DIS  0x8010
#define REG_SYNC_POL 0x8670
#define REG_VSYNC_PIX 0x8671
#define REG_HSYNC_PIX 0x8672
#define REG_VBP_PIX 0x8674
#define REG_VFB_PIX 0x8675
#define REG_HBP_PIX 0x8676
#define REG_HFP_PIX 0x8678
#define REG_VTOTAL 0x867A
#define REG_HTOTAL 0x867C
#define REG_VACTIVE 0x867E
#define REG_HACTIVE 0x8680
#define REG_MIPI_LANES 0x86A2
#define REG_INT_HDMI 0x86A3
#define INT_HDMI_STABILIZED 0x55
#define INT_HDMI_DISCONNECTED 0x88
#define REG_INT_AUDIO 0x86A5
#define INT_AUDIO_DISCONNECTED 0x55
#define INT_AUDIO_SAMPLERATE_HI 0x55
#define INT_AUDIO_SAMPLERATE_LO 0xAA
#define REG_INT_RESPOND 0x86A6

#define TMDS_STABLE (1 << 4)
#define REG_TMDS_CLK_HI 0x8750
#define REG_TMDS_CLK_MID 0x8751
#define REG_TMDS_CLK_LO 0x8752
#define REG_TMDS_FEATURES 0xB0A2
#define FEATURE_HDMI20 0x1

#define REG_AUDIO_SR_HI 0xB0AA
#define REG_AUDIO_SR_LO 0xB0AB

#define REG_MIPI_FRAME_ID 0xD40E
#define REG_MIPI_FRAME_STATUS 0xD414
#define REG_MIPI_CLK_MODE 0xD468

#define PIN_RESET 23
#define PIN_INTERRUPT 24

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

u8 optional_file = 1;
u8 called_quit = 0;

static void setup_gpios(void) {
  vcos_log_error("Configuring GPIOs");
  gpioSetMode(PIN_RESET, PI_OUTPUT);
  gpioSetMode(PIN_INTERRUPT, PI_INPUT);
}

static void reset_chip(void) {
  vcos_log_error("Resetting chip...");
  gpioDelay(100);
  gpioWrite(PIN_RESET, 0);
  gpioDelay(100);
  gpioWrite(PIN_RESET, 1);
  gpioDelay(100);
  vcos_log_error("Done.");
}

struct sensor_regs {
  uint16_t reg;
  uint8_t data;
};


#define ENCODING MMAL_ENCODING_UYVY

#define UNPACK MMAL_CAMERA_RX_CONFIG_UNPACK_NONE
#define PACK MMAL_CAMERA_RX_CONFIG_PACK_NONE

#define I2C_BUS 10
#define I2C_ADDR 0x2B

//#define CSI_IMAGE_ID 0x55 // maybe? from the dump
#define CSI_IMAGE_ID 0x24 // tc358743xbg

void signal_callback_handler(int);

static int i2c_cur_bank = -1;

static void i2c_set_bank(unsigned fd, u8 bank) {
  if (i2c_cur_bank == bank) {
    return;
  }

  i2c_cur_bank = bank;

  int err = i2cWriteByteData(fd, REG_BANK, bank);
  if (err != 0) {
    vcos_log_error("%s: cannot set bank 0x%02x on 0x%x, err %d\n",
                   __func__, bank, I2C_ADDR, err);
  }
}

static inline u8 i2c_rd8(unsigned fd, u16 reg) {
  i2c_set_bank(fd, reg >> 8);

  int rval = i2cReadByteData(fd, reg & 0xFF);

  printf("  i2c: rd 0x%04x -> 0x%02x\n", reg, rval);
  if (rval < 0) {
    vcos_log_error("%s: cannot read u8 from reg 0x%02x from dev 0x%x, err %d\n",
                   __func__, reg, I2C_ADDR, rval);
  }

  return rval;
}

static inline void i2c_wr8(unsigned fd, u16 reg, u8 val) {
  i2c_set_bank(fd, reg >> 8);

  printf("  i2c: wr 0x%04x -> 0x%02x\n", reg, val);
  int rval = i2cWriteByteData(fd, reg & 0xFF, val);
  if (rval < 0) {
    vcos_log_error("%s: cannot write u8 to reg 0x%02x to dev 0x%x, err %d\n",
                   __func__, reg, I2C_ADDR, rval);
  }
}

static inline u16 i2c_rd16(unsigned fd, u16 reg) {
  i2c_set_bank(fd, reg >> 8);

  int rval = i2cReadWordData(fd, reg & 0xFF);

  printf("  i2c: rd 0x%04x -> 0x%04x\n", reg, rval);
  if (rval < 0) {
    vcos_log_error("%s: cannot read u16 from reg 0x%02x from dev 0x%x, err %d\n",
                   __func__, reg, I2C_ADDR, rval);
  }

  return __builtin_bswap16(rval);
}

static inline void i2c_wr16(unsigned fd, u16 reg, u16 val) {
  i2c_set_bank(fd, reg >> 8);

  printf("  i2c: wr 0x%04x -> 0x%04x\n", reg, val);
  int rval = i2cWriteWordData(fd, reg & 0xFF, __builtin_bswap16(val));
  if (rval < 0) {
    vcos_log_error("%s: cannot write u16 to reg 0x%02x to dev 0x%x, err %d\n",
                   __func__, reg, I2C_ADDR, rval);
  }
}

static int comm_enabled = 0;

static int i2c_enable(int fd) {
  if (!comm_enabled) {
    comm_enabled = 1;
    i2c_wr8(fd, REG_I2C_EN, 1);
    i2c_wr8(fd, REG_WD_DIS, 0);
    return 1 /* FIXME BUG very evil HACK */;
  }
  return 1;
}

static void i2c_disable(int fd) {
  if (comm_enabled) {
    comm_enabled = 0;
    i2c_wr8(fd, REG_I2C_EN, 0);
  }
}

static void i2c_restore(int fd, int old) {
  if (!old) {
    i2c_disable(fd);
  }
}

static void print_chip_id(int fd) {
  int oldState = i2c_enable(fd);
  u8 id_hi = i2c_rd8(fd, REG_CHIP_ID + 0);
  u8 id_lo = i2c_rd8(fd, REG_CHIP_ID + 1);
  vcos_log_error("Chip ID: 0x%02x - 0x%02x", id_hi, id_lo);
  i2c_restore(fd, oldState);
}

static inline int no_signal(int fd) {
  int oldState = i2c_enable(fd);
  int tmds_stable = (i2c_rd8(fd, REG_TMDS_CLK_HI) & TMDS_STABLE) == TMDS_STABLE;
  int hdmi_int = i2c_rd8(fd, REG_INT_HDMI) == INT_HDMI_STABILIZED;
  i2c_restore(fd, oldState);

  return tmds_stable || hdmi_int;
}

static inline int no_sync(int fd) {
  return 0;
}

#define CC_RGB_PASSTHROUGH      1
#define CC_RGB_YUV422           2
#define CC_RGB_YUV444           3
#define CC_YUV444_YUV422        4
#define CC_YUV422_YUV444        5
#define COLOR_CONVERSION CC_RGB_PASSTHROUGH

#if COLOR_CONVERSION == CC_RGB_PASSTHROUGH
// RGB through mode
#define r8576  0x00 // 0000 0000 -- RGB full
#define r8573  0x00 // 00000000 -- RGB through
#define r8574  0x00
#define r0004  0x0e24 // 0000 1110 0010 0111
#elif COLOR_CONVERSION == CC_RGB_YUV422
#define r8574  0x08
#define r8573  /* 11000001 */ 0xC1
#define r8576  0x60
#define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_RGB_YUV444
#define r8574  0x08
#define r8573  /* 00000001 */ 0x01
#define r8576  0x60
#define r0004  0x0e24
#elif COLOR_CONVERSION == CC_YUV444_YUV422
#define r8574  0x08
#define r8573  /* 00000001 */ 0x80
#define r8576  0x00
#define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_YUV422_YUV444
#define r8574  0x08
#define r8573  /* 00000001 */ 0x00
#define r8576  0x00
#define r0004  0x0e24
#endif

struct cmds_t {
  uint16_t addr;
  uint32_t value;
  int num_bytes;
};


unsigned char ascii_to_hex(unsigned char c) {
  if (c >= '0' && c <= '9')
    return (c - '0');
  else if (c >= 'A' && c <= 'F')
    return ((c - 'A') + 10);
  else if (c >= 'a' && c <= 'f')
    return ((c - 'a') + 10);
  return 0;
}

void start_camera_streaming(int fd) {
  int oldState = i2c_enable(fd);
  i2c_wr8(fd, REG_INT_RESPOND, 0);
  i2c_wr8(fd, REG_MIPI_TX, MIPI_TX_ON);
  i2c_restore(fd, oldState);
}

void stop_camera_streaming(int fd) {
  int oldState = i2c_enable(fd);
  i2c_wr8(fd, REG_MIPI_TX, MIPI_TX_OFF);
  i2c_restore(fd, oldState);
}

int running = 0;

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  MMAL_STATUS_T status;
  vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length, buffer->pts,
                 buffer->flags);
  //vcos_log_error("File handle: %p", port->userdata);
  int bytes_written = buffer->length;

  if (running) {
    if (optional_file == 1) {
      FILE *file = (FILE *) port->userdata;
      bytes_written = fwrite(buffer->data, 1, buffer->length, file);
      fflush(file);
    }
    mmal_buffer_header_mem_unlock(buffer);

    if (bytes_written != buffer->length) {
      vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
    }

    if (port->is_enabled) {
      status = mmal_port_send_buffer(port, buffer);
      if (status != MMAL_SUCCESS) {
        vcos_log_error("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
      }
    } else
      mmal_buffer_header_release(buffer);

  }
}


/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(const char *filename) {
  FILE *new_handle = NULL;

  if (filename && optional_file == 1) {
    new_handle = fopen(filename, "wb");
  }

  return new_handle;
}

/* Clean up after Ctrl+c */
void signal_callback_handler(int signum) {
  printf("Caught signal %d\nCleanly exiting\n", signum);
  called_quit = 1;
  // goto exit;
  // exit(signum);
}

int main(int argc, char *argv[]) {
  int8_t option_index = 0;
  u32 sleep_duration = 0; //strtol(argv[1], &ptr, 10);
  char *_filename = "test_encode.h264";
  while ((option_index = getopt(argc, argv, "nht:o:")) != -1) {
    switch (option_index) {
    case 'n':
      optional_file = 0;
      break;
    case 't':
      sleep_duration = atoi(optarg); // not type safe
      break;
    case 'o':
      if (strcmp(optarg, "") != 0) {
        _filename = (char *) malloc(1 + strlen(optarg) + strlen(".h264"));
        strcpy(_filename, optarg);
        strcat(_filename, ".h264");
      }
      break;
    case 'h':
      printf("Available options:\n\n");
      printf("\t-t\tDefaults to 0 ms which playbacks until ctrl+c.\n");
      printf("\t-o\tOutput filename, default is 'test_encode.h264'.\n");
      printf("\t-n\tNo output file saved.\n");
      printf("\t-h\tHelp\n");
      return 1;
    default:
      printf("Incorrect option!\n");
      return 1;
    }
  }
  printf("Filename is %s\n", _filename);
  if (sleep_duration != 0)
    printf("Playing back for %u ms.\n", sleep_duration);
  else
    printf("Playing back indefinitely\n");
  u8 playthroughs = 0;
  MMAL_COMPONENT_T *rawcam, *render, *isp, *splitter, *encoder;
  MMAL_STATUS_T status;
  MMAL_PORT_T *output, *input, *isp_input, *isp_output, *encoder_input, *encoder_output;
  MMAL_POOL_T *pool;
  MMAL_CONNECTION_T *connection[4] = {0};
  MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_cfg)}};
  MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing = {{MMAL_PARAMETER_CAMERA_RX_TIMING, sizeof(rx_timing)}};
  int i2c_fd;
  unsigned int width, height, fps, frame_interval;
  unsigned int frame_width, frame_height;

  /* CALL THE SIGNAL HANDLER FUNCTION ON A Ctrl-C EXIT */
  signal(SIGINT, signal_callback_handler);

  bcm_host_init();
  vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

  gpioInitialise();
  setup_gpios();
  reset_chip();

  i2c_fd = i2cOpen(I2C_BUS, I2C_ADDR, 0);
  if (i2c_fd < 0) {
    vcos_log_error("Couldn't open I2C device");
    return -1;
  }
  status = mmal_component_create("vc.ril.rawcam", &rawcam);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create rawcam");
    return -1;
  }
  status = mmal_component_create("vc.ril.video_render", &render);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create render");
    return -1;
  }

  status = mmal_component_create("vc.ril.isp", &isp);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create isp");
    return -1;
  }

  status = mmal_component_create("vc.ril.video_splitter", &splitter);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create isp");
    return -1;
  }

  status = mmal_component_create("vc.ril.video_encode", &encoder);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create encoder");
    return -1;
  }

loop:
  print_chip_id(i2c_fd);

  output = rawcam->output[0];
  isp_input = isp->input[0];
  isp_output = isp->output[0];
  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];
  input = render->input[0];

  // setup CSI configs on rawcam
  status = mmal_port_parameter_get(output, &rx_cfg.hdr);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to get cfg");
    goto component_destroy;
  }
  rx_cfg.image_id = CSI_IMAGE_ID;

  int wasEnabled = i2c_enable(i2c_fd);
  int lanecnt = i2c_rd8(i2c_fd, REG_MIPI_LANES);
  if (lanecnt > 4) {
    lanecnt = 4;
  }
  vcos_log_error("Lane count to select cfg.data_lanes: %u", lanecnt);
  rx_cfg.data_lanes = lanecnt;
  i2c_restore(i2c_fd, wasEnabled);

  rx_cfg.unpack = UNPACK;
  rx_cfg.pack = PACK;
  rx_cfg.embedded_data_lines = 128;
  vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
  status = mmal_port_parameter_set(output, &rx_cfg.hdr);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set cfg");
    goto component_destroy;
  }

  vcos_log_error("Enable rawcam....");

  status = mmal_component_enable(rawcam);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to enable");
    goto component_destroy;
  }
  status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set zero copy");
    goto component_disable;
  }

  // end setup rawcam

  vcos_log_error("Enable isp....");

  status = mmal_component_enable(isp);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to enable");
    goto component_destroy;
  }
  status = mmal_port_parameter_set_boolean(isp_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set zero copy");
    goto component_disable;
  }

  vcos_log_error("Enable splitter....");

  status = mmal_component_enable(splitter);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to enable");
    goto component_destroy;
  }
  status = mmal_port_parameter_set_boolean(splitter->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set zero copy");
    goto component_disable;
  }

  start_camera_streaming(i2c_fd);
  vcos_sleep(500);  //Give a chance to detect signal
  vcos_log_error("Waiting to detect signal...");

  int count = 0;
  while ((count < 20) && (no_sync(i2c_fd) || no_signal(i2c_fd))) {
    vcos_sleep(200);
    count++;
  }
  vcos_log_error("Signal reported");
  wasEnabled = i2c_enable(i2c_fd);
  width = i2c_rd16(i2c_fd, REG_HACTIVE) * 2;
  height = i2c_rd16(i2c_fd, REG_VACTIVE);
  frame_width = i2c_rd16(i2c_fd, REG_HTOTAL) * 2;
  frame_height = i2c_rd16(i2c_fd, REG_VTOTAL);


  if (width == 0) width = 1280;
  if (height == 0) height = 720;
  if (frame_width == 0) frame_width = 1664;
  if (frame_height == 0) frame_height = 748;

  if (playthroughs == 0) {
    playthroughs++;
    vcos_log_error("First playthrough, Goto Loop");
    i2c_restore(i2c_fd, wasEnabled);
    goto loop;
  }

  int tmds_hi = i2c_rd8(i2c_fd, REG_TMDS_CLK_HI);
  int tmds_mid = i2c_rd8(i2c_fd, REG_TMDS_CLK_MID);
  int tmds_lo = i2c_rd8(i2c_fd, REG_TMDS_CLK_LO);

  int tmds_clk = ((tmds_hi & 0xF) << 16) | (tmds_mid << 8) | (tmds_lo << 0);
  int features = i2c_rd8(i2c_fd, REG_TMDS_FEATURES);
  int is_hdmi20 = (features & FEATURE_HDMI20) != 0;

  int pixel_clk = (is_hdmi20 ? tmds_clk * 4 : tmds_clk) * 1000;
  i2c_restore(i2c_fd, wasEnabled);

  if (pixel_clk == 0) pixel_clk = 74500000;

  vcos_log_error("TMDS clk %d, HDMI2.0 %d, pixel clk %d", tmds_clk, is_hdmi20, pixel_clk);

  /* frame interval in milliseconds * 10
   * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
  frame_interval = 10000LL * frame_width * frame_height / pixel_clk;
  fps = 10000 / frame_interval;

  vcos_log_error("Signal is %u x %u, frm_interval %u, so %u fps", width, height, frame_interval, fps);
  vcos_log_error("Frame w x h is %u x %u", frame_width, frame_height);


  output->format->es->video.crop.width = width;
  output->format->es->video.crop.height = height;
  output->format->es->video.width = VCOS_ALIGN_UP(width, 32); //VCOS_ALIGN_UP(WIDTH, 32);
  output->format->es->video.height = VCOS_ALIGN_UP(height, 16); //VCOS_ALIGN_UP(HEIGHT, 16);
  output->format->es->video.frame_rate.num = 10000;
  output->format->es->video.frame_rate.den = frame_interval ? frame_interval : 10000;
  output->format->encoding = ENCODING;
  status = mmal_port_format_commit(output);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("output: Failed port_format_commit");
    mmal_log_dump_port(output);
    goto component_disable;
  }

  output->buffer_size = output->buffer_size_recommended;
  output->buffer_num = 3;
  vcos_log_error("output: buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);

  vcos_log_error("Create connection rawcam output to isp input....");
  status = mmal_connection_create(&connection[0], output, isp_input, MMAL_CONNECTION_FLAG_TUNNELLING);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create connection status %d: rawcam->isp", status);
    goto component_disable;
  }

  // ISP output port does not follow input, so do have to set that one up.
  mmal_format_copy(isp_output->format, isp_input->format);
  isp_output->format->encoding = MMAL_ENCODING_I420;
  vcos_log_error("Setting isp output port format");
  status = mmal_port_format_commit(isp_output);
  isp_output->buffer_size = isp_output->buffer_size_recommended;
  isp_output->buffer_num = isp_output->buffer_num_recommended;
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create connection status %d: rawcam->isp", status);
    goto component_disable;
  }

  //  Encoder setup
  vcos_log_error("Create connection isp output to splitter input....");
  status = mmal_connection_create(&connection[1], isp_output, splitter->input[0], MMAL_CONNECTION_FLAG_TUNNELLING);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create connection status %d: isp->splitter", status);
    goto component_disable;
  }

  vcos_log_error("Create connection splitter output to render input....");
  status = mmal_connection_create(&connection[2], splitter->output[0], input, MMAL_CONNECTION_FLAG_TUNNELLING);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create connection status %d: splitter->render", status);
    goto component_disable;
  }

  vcos_log_error("Create connection splitter output2 to encoder input....");
  status = mmal_connection_create(&connection[3], splitter->output[1], encoder_input, MMAL_CONNECTION_FLAG_TUNNELLING);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create connection status %d: splitter->encoder", status);
    goto component_disable;
  }

  // Only supporting H264 at the moment
  encoder_output->format->encoding = MMAL_ENCODING_H264;

  encoder_output->format->bitrate = 17000000;
  encoder_output->buffer_size = encoder_output->buffer_size_recommended;

  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  encoder_output->buffer_num = 8; //encoder_output->buffer_num_recommended;

  if (encoder_output->buffer_num < encoder_output->buffer_num_min)
    encoder_output->buffer_num = encoder_output->buffer_num_min;

  // We need to set the frame rate on output to 0, to ensure it gets
  // updated correctly from the input framerate when port connected
  encoder_output->format->es->video.frame_rate.num = fps;//0;
  encoder_output->format->es->video.frame_rate.den = 1;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on encoder output port");
  }

  {
    MMAL_PARAMETER_VIDEO_PROFILE_T param;
    param.hdr.id = MMAL_PARAMETER_PROFILE;
    param.hdr.size = sizeof(param);

    param.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;//state->profile;
    param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4; // This is the only value supported

    status = mmal_port_parameter_set(encoder_output, &param.hdr);
    if (status != MMAL_SUCCESS) {
      vcos_log_error("Unable to set H264 profile");
    }
  }

  if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1) != MMAL_SUCCESS) {
    vcos_log_error("Unable to set immutable input flag");
    // Continue rather than abort..
  }

  //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
  if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0) != MMAL_SUCCESS) {
    vcos_log_error("failed to set INLINE HEADER FLAG parameters");
    // Continue rather than abort..
  }

  //set INLINE VECTORS flag to request motion vector estimates
  if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 0) != MMAL_SUCCESS) {
    vcos_log_error("failed to set INLINE VECTORS parameters");
    // Continue rather than abort..
  }

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on video encoder input port");
  }

  vcos_log_error("Enable encoder....");

  status = mmal_component_enable(encoder);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to enable");
    goto component_destroy;
  }
  status = mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set zero copy");
    goto component_disable;
  }

/*
   vcos_log_error("rawcam supported encodings:");
   display_supported_encodings(output);
   vcos_log_error("isp input supported encodings:");
   display_supported_encodings(isp_input);
   vcos_log_error("isp output supported encodings:");
   display_supported_encodings(isp_output);
   vcos_log_error("encoder input supported encodings:");
   display_supported_encodings(encoder_input);
   vcos_log_error("encoder output supported encodings:");
   display_supported_encodings(encoder_output);
   vcos_log_error("render supported encodings:");
   display_supported_encodings(input);
*/


  status = mmal_port_parameter_set_boolean(input, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to set zero copy on video_render");
    goto component_disable;
  }

  if (status == MMAL_SUCCESS) {
    vcos_log_error("Enable connection[0]...");
    vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
    status = mmal_connection_enable(connection[0]);
    if (status != MMAL_SUCCESS) {
      mmal_connection_destroy(connection[0]);
    }
    vcos_log_error("Enable connection[1]...");
    vcos_log_error("buffer size is %d bytes, num %d", isp_output->buffer_size, isp_output->buffer_num);
    status = mmal_connection_enable(connection[1]);
    if (status != MMAL_SUCCESS) {
      mmal_connection_destroy(connection[1]);
    }

    vcos_log_error("Enable connection[2]...");
    vcos_log_error("buffer size is %d bytes, num %d", splitter->output[0]->buffer_size,
                   splitter->output[0]->buffer_num);
    status = mmal_connection_enable(connection[2]);
    if (status != MMAL_SUCCESS) {
      mmal_connection_destroy(connection[2]);
    }

    vcos_log_error("Enable connection[3]...");
    vcos_log_error("buffer size is %d bytes, num %d", splitter->output[1]->buffer_size,
                   splitter->output[1]->buffer_num);
    status = mmal_connection_enable(connection[3]);
    if (status != MMAL_SUCCESS) {
      mmal_connection_destroy(connection[3]);
    }
  }

  // open h264 file and put the file handle in userdata for the encoder output port
  encoder_output->userdata = (void *) open_filename(_filename);
  if (strcmp(_filename, "test_encode.h264") != 0) {
    free(_filename);
    _filename = NULL;
  }
  //Create encoder output buffers

  vcos_log_error("Create pool of %d buffers of size %d", encoder_output->buffer_num, encoder_output->buffer_size);
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
  if (!pool) {
    vcos_log_error("Failed to create pool");
    goto component_disable;
  }

  status = mmal_port_enable(encoder_output, encoder_buffer_callback);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to enable port");
  }

  running = 1;
  int i;
  for (i = 0; i < encoder_output->buffer_num; i++) {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

    if (!buffer) {
      vcos_log_error("Where'd my buffer go?!");
      goto port_disable;
    }
    status = mmal_port_send_buffer(encoder_output, buffer);
    if (status != MMAL_SUCCESS) {
      vcos_log_error("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
      goto port_disable;
    }
    vcos_log_error("Sent buffer %p", buffer);
  }


  // Setup complete
  vcos_log_error("All done. Start streaming...");
  vcos_log_error("View!");

  if (sleep_duration > 0) {
    vcos_log_error("Sleeping for %u ms", sleep_duration);
    vcos_sleep(sleep_duration);
  } else {
    vcos_log_error("Sleeping until you ctrl+c me!");
    while (called_quit != 1) {
      vcos_sleep(100);
    }
  }

// exit:
  running = 0;

  vcos_log_error("Stopping streaming...");
  stop_camera_streaming(i2c_fd);

  playthroughs = 0;

port_disable:

  mmal_connection_disable(connection[0]);
  mmal_connection_destroy(connection[0]);

  mmal_connection_disable(connection[1]);
  mmal_connection_destroy(connection[1]);

  mmal_connection_disable(connection[2]);
  mmal_connection_destroy(connection[2]);

  mmal_connection_disable(connection[3]);
  mmal_connection_destroy(connection[3]);

component_disable:
  status = mmal_component_disable(rawcam);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to disable rawcam");
  }

  status = mmal_component_disable(isp);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to disable isp");
  }

  status = mmal_component_disable(render);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to disable render");
  }

  status = mmal_component_disable(splitter);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to disable splitter");
  }

  status = mmal_component_disable(encoder);
  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to disable encoder");
  }

component_destroy:
  mmal_component_destroy(rawcam);
  mmal_component_destroy(isp);
  mmal_component_destroy(render);
  mmal_component_destroy(splitter);
  mmal_component_destroy(encoder);

  close(i2c_fd);
  return 0;
}


#define MAX_ENCODINGS_NUM 20
typedef struct {
  MMAL_PARAMETER_HEADER_T header;
  MMAL_FOURCC_T encodings[MAX_ENCODINGS_NUM];
} MMAL_SUPPORTED_ENCODINGS_T;

void display_supported_encodings(MMAL_PORT_T *port) {
  MMAL_SUPPORTED_ENCODINGS_T sup_encodings = {{MMAL_PARAMETER_SUPPORTED_ENCODINGS, sizeof(sup_encodings)},
                                              {0}};
  if (mmal_port_parameter_get(port, &sup_encodings.header) == MMAL_SUCCESS) {
    int i;
    int num_encodings = (sup_encodings.header.size - sizeof(sup_encodings.header)) /
                        sizeof(sup_encodings.encodings[0]);
    for (i = 0; i < num_encodings; i++) {
      switch (sup_encodings.encodings[i]) {
      case MMAL_ENCODING_I420:
        vcos_log_error("MMAL_ENCODING_I420");
        break;
      case MMAL_ENCODING_I420_SLICE:
        vcos_log_error("MMAL_ENCODING_I420_SLICE");
        break;
      case MMAL_ENCODING_YV12:
        vcos_log_error("MMAL_ENCODING_YV12");
        break;
      case MMAL_ENCODING_I422:
        vcos_log_error("MMAL_ENCODING_I422");
        break;
      case MMAL_ENCODING_I422_SLICE:
        vcos_log_error("MMAL_ENCODING_I422_SLICE");
        break;
      case MMAL_ENCODING_YUYV:
        vcos_log_error("MMAL_ENCODING_YUYV");
        break;
      case MMAL_ENCODING_YVYU:
        vcos_log_error("MMAL_ENCODING_YVYU");
        break;
      case MMAL_ENCODING_UYVY:
        vcos_log_error("MMAL_ENCODING_UYVY");
        break;
      case MMAL_ENCODING_VYUY:
        vcos_log_error("MMAL_ENCODING_VYUY");
        break;
      case MMAL_ENCODING_NV12:
        vcos_log_error("MMAL_ENCODING_NV12");
        break;
      case MMAL_ENCODING_NV21:
        vcos_log_error("MMAL_ENCODING_NV21");
        break;
      case MMAL_ENCODING_ARGB:
        vcos_log_error("MMAL_ENCODING_ARGB");
        break;
      case MMAL_ENCODING_RGBA:
        vcos_log_error("MMAL_ENCODING_RGBA");
        break;
      case MMAL_ENCODING_ABGR:
        vcos_log_error("MMAL_ENCODING_ABGR");
        break;
      case MMAL_ENCODING_BGRA:
        vcos_log_error("MMAL_ENCODING_BGRA");
        break;
      case MMAL_ENCODING_RGB16:
        vcos_log_error("MMAL_ENCODING_RGB16");
        break;
      case MMAL_ENCODING_RGB24:
        vcos_log_error("MMAL_ENCODING_RGB24");
        break;
      case MMAL_ENCODING_RGB32:
        vcos_log_error("MMAL_ENCODING_RGB32");
        break;
      case MMAL_ENCODING_BGR16:
        vcos_log_error("MMAL_ENCODING_BGR16");
        break;
      case MMAL_ENCODING_BGR24:
        vcos_log_error("MMAL_ENCODING_BGR24");
        break;
      case MMAL_ENCODING_BGR32:
        vcos_log_error("MMAL_ENCODING_BGR32");
        break;
      }
    }
  } else {
    vcos_log_error("Failed to get supported encodings");
  }
}

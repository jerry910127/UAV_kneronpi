#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <libgen.h>


//#define DBG
#define SHORT_DISP

#define SERIAL_PORT_PATH        "/dev/ttyUSB0"

typedef enum
{
    CMD_NA,
    CMD_OUTPUT_MODE,
    CMD_FRAME_TIME,
    CMD_MEASURE_TRI,
    CMD_MEASURE_MODE,
    CMD_RD_MEASURE_M,
    CMD_MAX
} AFBR_S50_COMMAND;


uint8_t szCmdOutputMode[] = { 0x02, 0x41, 0x07, 0xF5, 0x03 };
uint8_t szCmdMeasureMode[] = { 0x02, 0x42, 0x02, 0x48, 0x03 };
//uint8_t szCmdMeasureMode[] = { 0x02, 0x42, 0x09, 0x87, 0x03 };
uint8_t szCmdFrameTime[]  = { 0x02, 0x43, 0x00, 0x1B, 0xFC, 0x0D, 0x40, 0x85, 0x03 };
//uint8_t szCmdFrameTime[]  = { 0x02, 0x43, 0x00, 0x01, 0x86, 0xA0, 0x73, 0x03 };
uint8_t szCmdMeasureTri[] = { 0x02, 0x10, 0xCD, 0x03 };

uint8_t szCmdRdMeasureMode[] = { 0x02, 0x42, 0x29, 0x03 };


struct termios g_tty;
int g_fd;

// FILE OPERATION
static int file_open_and_get_descriptor(const char *fname) {
    int fd;

    fd = open(fname, O_RDWR | O_NONBLOCK);
    if(fd < 0) {
        printf("Could not open file %s...%d\r\n",fname,fd);
    }
    return fd;
}

static int file_write_data(int fd, uint8_t *buff, uint32_t len_buff) {
    return write(fd,buff,len_buff);
}

static int file_read_data(int fd, uint8_t *buff, uint32_t len_buff) {
#if 1
    uint32_t    len_rd = 0;
    uint8_t     data,
                esc_b = 0;

    do  {
        if ( 1 != read( fd, &data, 1 ) )
            break;
        //printf("%02x ", data);

        if ( 0x1B == data )  {
            esc_b = 1;
        }
        else  if ( esc_b )  {
            buff[len_rd++] = 0xFF - data;
            esc_b = 0;
        }
        else  {
            buff[len_rd++] = data;
        }
    } while ( 0x03 != data );
    //printf("\r\n");

#ifndef SHORT_DISP
    if ( len_rd != len_buff )  {
        printf("UART_RD::len  %d ?= %d\r\n", len_rd, len_buff);
    }
#endif

    return len_rd;

#else
    return read(fd,buff,len_buff);
#endif
}

static int file_close(int fd) {
    return close(fd);
}


static void open_serial_port(void) {
    g_fd = file_open_and_get_descriptor(SERIAL_PORT_PATH);
    if(g_fd < 0) {
        printf("Something went wrong while opening the port...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void configure_serial_port(void) {
    if(tcgetattr(g_fd, &g_tty)) {
        printf("Something went wrong while getting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }

    cfsetispeed(&g_tty,B115200);
    cfsetospeed(&g_tty,B115200);

    cfmakeraw(&g_tty);

    if(tcsetattr(g_fd,TCSANOW,&g_tty)) {
        printf("Something went wrong while setting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void close_serial_port(void) {
    file_close(g_fd);
}


static void ack_2nd_data( uint8_t cmd )
{
    uint8_t l_buff[32];
    uint32_t l_len_buff = 5, len;

    if ( CMD_MEASURE_TRI == cmd )  {
        l_len_buff = 23;

        usleep(100000);
        memset(l_buff, 0, l_len_buff);
        len = file_read_data(g_fd, l_buff, l_len_buff);

#ifdef DBG
        printf("Data:\r\n");
        for (uint8_t l_looper=0; l_looper<len; ++l_looper)  {
            printf("%02X ",l_buff[l_looper]);
        }
        printf("\r\n");
#endif

        if ( (0x02 == l_buff[0]) && (0x03 == l_buff[22]) )  {
            uint16_t status = (l_buff[3] << 8) + l_buff[4];

            int t_sec = (l_buff[5] << 24) + (l_buff[6] << 16) + (l_buff[7] << 8) + l_buff[8];
            //int t_usec = (l_buff[9] << 8) + l_buff[10];

            uint16_t    amplitude = (l_buff[18] << 8) + l_buff[19];
            int range = (l_buff[15] << 16) + (l_buff[16] << 8) + l_buff[17];

#ifdef SHORT_DISP
            (void)t_sec;
            (void)amplitude;
            printf("\rRange: %2.4f m     st:%d",
                    ((float)range / 16384.0),
                    (int16_t)status
                    );
            if ( -110 == (int16_t)status )  {
                printf("(STALLED)");
            }
            else  if ( 108 == (int16_t)status )  {
                printf("(NO OBJECT)");
            }
            for (uint8_t idx=0; idx<12; idx++)
                printf(" ");

#else
            printf("\r\n");
            printf("Status: %04X(%d)\r\n", status, (int16_t)status);
            printf("Time Stamp: %d\r\n",
                    t_sec
                    );
            printf("Amplitude: %f\r\n", (float)amplitude / 16.0);
            printf("Signal Quality: %d\r\n", l_buff[20]);
            printf("\r\nRange: %f M\r\n", ((float)range / 16384.0));
#endif
        }
    }

    else  if ( CMD_RD_MEASURE_M == cmd )  {
        l_len_buff = 5;

        usleep(100000);
        memset(l_buff, 0, l_len_buff);
        len = file_read_data(g_fd, l_buff, l_len_buff);

        for (uint8_t l_looper=0; l_looper<len; ++l_looper)  {
            printf("%02X ",l_buff[l_looper]);
        }
        printf("\r\n");
    }
}

static void perform_command( char *cmdArg )
{
    uint8_t cmd = atoi(cmdArg), l_buff[32];
    uint32_t l_len_buff = 5, len;

    if ( (CMD_OUTPUT_MODE == cmd) || !strcmp(cmdArg, "-o") )  {
        cmd = CMD_OUTPUT_MODE;
#ifdef DBG
        printf("Configure Data Output Mode...\r\n");
#endif
        file_write_data( g_fd, szCmdOutputMode, sizeof(szCmdOutputMode) );
    }
    else  if ( (CMD_FRAME_TIME == cmd) || !strcmp(cmdArg, "-f") )  {
        cmd = CMD_FRAME_TIME;
#ifdef DBG
        printf("Configure Frame Time...\r\n");
#endif
        file_write_data( g_fd, szCmdFrameTime, sizeof(szCmdFrameTime) );
    }
    else  if ( (CMD_MEASURE_TRI == cmd) || !strcmp(cmdArg, "-t") )  {
        cmd = CMD_MEASURE_TRI;
#ifdef DBG
        printf("Measurement: Trigger Single Shot...\r\n");
#endif
        file_write_data( g_fd, szCmdMeasureTri, sizeof(szCmdMeasureTri) );
    }
    else  if ( (CMD_MEASURE_MODE == cmd) || !strcmp(cmdArg, "-m") )  {
        cmd = CMD_MEASURE_MODE;
#ifdef DBG
        printf("Configure Measurement Mode...\r\n");
#endif
        file_write_data( g_fd, szCmdMeasureMode, sizeof(szCmdMeasureMode) );
    }
    else  if ( (CMD_RD_MEASURE_M == cmd) || !strcmp(cmdArg, "-rm") )  {
        cmd = CMD_RD_MEASURE_M;
#ifdef DBG
        printf("Read Measurement Mode...\r\n");
#endif
        file_write_data( g_fd, szCmdRdMeasureMode, sizeof(szCmdRdMeasureMode) );
    }
    else  if ( !strcmp(cmdArg, "-h") )  {
        printf("Command:\r\n"
                "\tConfiguration Commands\r\n"
                "\t-o, Data Output Mode,\t0x41\r\n"
                "\t-m, Measure Mode,\t0x42\r\n"
                "\t-f, Frame Time/Rate,\t0x42\r\n"
                "\r\n\tMeasurment Data Command\r\n"
                "\t-t, Measurement: Single Shot,\t0x10\r\n"
                );
        return;
    }

    usleep(100000);
    memset(l_buff,0,l_len_buff);
    len = file_read_data(g_fd,l_buff,l_len_buff);

#ifdef DBG
    uint32_t l_looper;
    printf("\r\nAck:\r\n");
    for(l_looper=0; l_looper<len; ++l_looper) {
        printf("%02X ",l_buff[l_looper]);
    }
    printf("\r\n");
#else
    (void)len;
#endif

    ack_2nd_data(cmd);
}

static void sequence_bytes( int argc, char *argv[] )
{
    uint8_t idx, bw,
            data[32];

    uint8_t buff[320];
    uint32_t len_buff = 5, len;

    // ASCII to HEX
    for ( idx = 1; idx < argc; idx++ )  {
        data[idx-1] = 0;
        for ( bw = 0; bw < 2; bw++ )  {
            if ( argv[idx][bw] >= 'a' )  {
                data[idx-1] |= (argv[idx][bw] - 'a' + 10) << (4 * (1 - bw));
            }
            else  if ( argv[idx][bw] >= 'A' )  {
                data[idx-1] |= (argv[idx][bw] - 'A' + 10) << (4 * (1 - bw));
            }
            else  {
                data[idx-1] |= (argv[idx][bw] - '0') << (4 * (1 - bw));
            }
        }
    }

#ifdef DBG
    for ( idx = 0; idx < argc-1; idx++ )  {
        printf("%02x ", data[idx]);
    }
    printf("\r\n");
#endif

    file_write_data( g_fd, data, argc - 1 );

    usleep(100000);
    memset( buff, 0, len_buff );
    len = file_read_data( g_fd, buff, len_buff );

    uint32_t    looper;
    printf("\r\nAck:\r\n");
    if ( 0x06 == buff[1] )  {
        uint32_t    ts_sec = (buff[2] << 24) + (buff[3] << 16) +
                            (buff[4] << 8) + buff[5];
        uint16_t    ts_usec = (buff[6] << 8) + buff[7];
        printf("Time Stamp: %ds  %dus\r\n", ts_sec, ts_usec);
        buff[len-2] = 0;
        printf("#%d[\r\n%s]\r\n", len - 10, buff + 8);
    }
    else  {
        for(looper=0; looper<len; ++looper) {
            printf("%02X ", buff[looper]);
        }
        printf("\r\n");
    }

    if ( 0x10 == data[1] )  {
      ack_2nd_data(CMD_MEASURE_TRI);
    }
    else  if ( (0x0B != buff[1]) && (5 == argc) )  {
      ack_2nd_data(CMD_RD_MEASURE_M);
    }
}

static void string_command( char *strCmd )
{
    char    data[32];
    int32_t len = strlen(strCmd);

    strcpy( data, strCmd );
    data[len++] = 0x0D;
    data[len++] = 0x0A;
#ifdef DBG
    for ( uint8_t idx=0; idx<len; idx++ )  {
        printf("%02X ", data[idx]);
    }
    printf("\r\n");
#endif

    len = file_write_data( g_fd, (uint8_t *)data, len );
    //printf("write %d bytes\r\n", len);

    usleep(100000);
    memset( data, 0, sizeof(data) );
    len = read( g_fd, &data, 18 );
    if ( len > 0 )  {
        printf("ACK:  [\r\n");
#if 1
        printf("%s]\r\n", data);

#else
        for ( uint8_t idx=0; idx<len; idx++ )  {
            printf("%02X ", data[idx]);
        }
        printf("\r\n");
#endif
    }
}


int main( int argc, char *argv[] ) {
    char    *exec_name;
    exec_name = basename( argv[0] );
    if ( ( exec_name = strstr( exec_name, "tty" ) ) )  {
        char    uart_port[16];
        sprintf( uart_port, "/dev/%s", exec_name );
        g_fd = file_open_and_get_descriptor( uart_port );
        if ( g_fd < 0 )  {
            printf("Something went wrong while opening the port...\r\n");
            return -1;
        }
    }
    else  {
        open_serial_port();
    }

    configure_serial_port();

    if ( 2 == argc )  {
        perform_command( argv[1] );
    }
    else  if ( (3 == argc) && !strcmp(argv[1], "-s") )  {
        string_command( argv[2] );
    }
    else  if ( argc > 3 )  {
        sequence_bytes( argc, argv );
    }

    close_serial_port();

    return 0;
}


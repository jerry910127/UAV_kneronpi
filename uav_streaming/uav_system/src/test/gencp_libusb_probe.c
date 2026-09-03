// gencp_libusb_probe.c
// Probe Miramar GenICam over USB CDC data endpoints directly (EP6 OUT/IN)
// Based on vendor python sample packet layout (little-endian, 20-byte ReadMem command).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#define VID 0x1fc9
#define PID 0x00a3

static void hexdump(const uint8_t *b, int n) {
  for (int i = 0; i < n; i++) printf("%02X ", b[i]);
  printf("\n");
}

// CDC ACM class requests
static int cdc_set_line_coding(libusb_device_handle *h, int if_comm, uint32_t baud) {
  // 7 bytes: dwDTERate(4), bCharFormat(1), bParityType(1), bDataBits(1)
  uint8_t lc[7];
  lc[0] = (uint8_t)(baud & 0xFF);
  lc[1] = (uint8_t)((baud >> 8) & 0xFF);
  lc[2] = (uint8_t)((baud >> 16) & 0xFF);
  lc[3] = (uint8_t)((baud >> 24) & 0xFF);
  lc[4] = 0; // 1 stop bit
  lc[5] = 0; // parity none
  lc[6] = 8; // data bits

  // bmRequestType: 0x21 (Host->Dev, Class, Interface)
  return libusb_control_transfer(
      h, 0x21, 0x20, 0x0000, (uint16_t)if_comm, lc, sizeof(lc), 2000);
}

static int cdc_set_control_line_state(libusb_device_handle *h, int if_comm, int dtr, int rts) {
  uint16_t v = 0;
  if (dtr) v |= 0x0001;
  if (rts) v |= 0x0002;
  // bmRequestType: 0x21 (Host->Dev, Class, Interface)
  return libusb_control_transfer(
      h, 0x21, 0x22, v, (uint16_t)if_comm, NULL, 0, 2000);
}

static void build_readmem_cmd(uint8_t out[20], uint64_t addr, uint16_t len, uint16_t req_id) {
  memset(out, 0, 20);
  // CCD
  out[0] = 0x00;
  out[1] = 0x40; // RequestAck
  out[2] = 0x00;
  out[3] = 0x08; // READMEM_CMD (0x0800) little-endian bytes
  out[4] = 0x0C;
  out[5] = 0x00; // SCD length = 12
  out[6] = (uint8_t)(req_id & 0xFF);
  out[7] = (uint8_t)((req_id >> 8) & 0xFF);

  // SCD: address (uint64 little-endian)
  for (int i = 0; i < 8; i++) out[8 + i] = (uint8_t)((addr >> (8 * i)) & 0xFF);
  // reserved [16],[17] already 0
  out[18] = (uint8_t)(len & 0xFF);
  out[19] = (uint8_t)((len >> 8) & 0xFF);
}

static int bulk_read_exact(libusb_device_handle *h, uint8_t ep_in, uint8_t *buf, int need, int timeout_ms) {
  int got = 0;
  while (got < need) {
    int xfer = 0;
    int r = libusb_bulk_transfer(h, ep_in, buf + got, need - got, &xfer, timeout_ms);
    if (r != 0) return r;
    if (xfer <= 0) return -1;
    got += xfer;
  }
  return 0;
}

static int try_readmem(libusb_device_handle *h, uint8_t ep_out, uint8_t ep_in, uint64_t addr, uint16_t len, uint16_t req_id, int debug) {
  uint8_t cmd[20];
  build_readmem_cmd(cmd, addr, len, req_id);

  if (debug) {
    printf("TX addr=0x%llX len=%u: ", (unsigned long long)addr, len);
    hexdump(cmd, 20);
  }

  int xfer = 0;
  int r = libusb_bulk_transfer(h, ep_out, cmd, 20, &xfer, 2000);
  if (r != 0 || xfer != 20) {
    printf("bulk OUT failed r=%d xfer=%d\n", r, xfer);
    return -1;
  }

  // Expect ack: len + 8 (per vendor sample)【GenicamPythonSampleCode】
  int need = (int)len + 8;
  uint8_t *rb = (uint8_t *)malloc(need);
  if (!rb) return -1;
  memset(rb, 0, need);

  r = bulk_read_exact(h, ep_in, rb, need, 20000);
  if (r != 0) {
    printf("bulk IN failed r=%d\n", r);
    free(rb);
    return -1;
  }

  if (debug) {
    printf("RX(%d): ", need);
    hexdump(rb, need);
  }

  // Validate ACK like vendor sample:
  // rb[0..1]==0, rb[2..3]==0x01 0x08 (READMEM_ACK):contentReference[oaicite:4]{index=4}
  if (rb[0] != 0 || rb[1] != 0 || rb[2] != 0x01 || rb[3] != 0x08) {
    printf("Not READMEM_ACK. hdr=%02X %02X %02X %02X\n", rb[0], rb[1], rb[2], rb[3]);
    free(rb);
    return -1;
  }

  // Payload starts at rb+8
  printf("ACK OK. Payload(ASCII): ");
  for (int i = 8; i < need; i++) {
    uint8_t c = rb[i];
    if (c == 0) break;
    if (c >= 32 && c <= 126) putchar(c);
    else putchar('.');
  }
  printf("\n");

  free(rb);
  return 0;
}

int main(int argc, char **argv) {
  int debug = 0;
  if (argc > 1 && strcmp(argv[1], "-d") == 0) debug = 1;

  // Interfaces from your lsusb: CLI comm=2, CLI data=3; endpoints: OUT=0x06, IN=0x86
  const int if_comm = 2;
  const int if_data = 3;
  const uint8_t ep_out = 0x06;
  const uint8_t ep_in  = 0x86;

  libusb_context *ctx = NULL;
  libusb_init(&ctx);

  libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, VID, PID);
  if (!h) {
    printf("Cannot open device %04x:%04x\n", VID, PID);
    libusb_exit(ctx);
    return 2;
  }

  // Detach kernel driver if claimed by cdc_acm
  if (libusb_kernel_driver_active(h, if_comm) == 1) libusb_detach_kernel_driver(h, if_comm);
  if (libusb_kernel_driver_active(h, if_data) == 1) libusb_detach_kernel_driver(h, if_data);

  int r = libusb_claim_interface(h, if_comm);
  if (r != 0) printf("claim if_comm failed: %d\n", r);
  r = libusb_claim_interface(h, if_data);
  if (r != 0) printf("claim if_data failed: %d\n", r);

  // Mimic what CDC ACM usually does: set baud + DTR/RTS
  int cr = cdc_set_line_coding(h, if_comm, 115200);
  if (debug) printf("SET_LINE_CODING ret=%d\n", cr);
  cr = cdc_set_control_line_state(h, if_comm, 1, 1);
  if (debug) printf("SET_CONTROL_LINE_STATE ret=%d\n", cr);

  // Try vendor sample check (addr=0x04) then Miramar doc check (addr=0x08)
  // Miramar doc says 0x08 returns "OBSIDIAN SENSORS INC." on correct port:contentReference[oaicite:5]{index=5}
  // Vendor sample uses readRegister(4,64) to validate port:contentReference[oaicite:6]{index=6}
  printf("Try addr=0x04 len=64...\n");
  try_readmem(h, ep_out, ep_in, 0x04, 64, 1, debug);

  printf("Try addr=0x08 len=64...\n");
  try_readmem(h, ep_out, ep_in, 0x08, 64, 2, debug);

  libusb_release_interface(h, if_data);
  libusb_release_interface(h, if_comm);
  libusb_close(h);
  libusb_exit(ctx);
  return 0;
}

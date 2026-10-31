#include "jc014.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart4;
static uint8_t screen[240][320];
static uint8_t expected[240][320];
static uint32_t tick, busy_until;
static unsigned int sends, largest_send, protocol_errors, early_sends;
static unsigned int fail_send, ack_index;
static int ack_enabled, mutex_locked, failures;

/* Golden bitmaps for the existing large font, independent of the driver table. */
static const uint8_t font[13][14] = {
  {28,62,99,99,99,99,99,99,99,99,99,99,62,28},
  {12,28,60,12,12,12,12,12,12,12,12,12,62,62},
  {28,62,99,3,3,6,12,24,48,96,96,96,127,127},
  {126,3,3,6,30,30,6,3,3,3,3,6,126,124},
  {6,14,30,54,102,102,102,127,127,6,6,6,6,6},
  {127,127,96,96,126,127,3,3,3,3,3,6,126,124},
  {30,48,96,96,126,99,99,99,99,99,99,99,62,28},
  {127,127,3,6,12,24,24,48,48,48,48,48,48,48},
  {62,99,99,99,62,99,99,99,99,99,99,99,62,28},
  {62,99,99,99,99,99,99,63,3,3,3,6,60,56},
  {0,0,12,12,12,12,127,127,12,12,12,12,0,0},
  {0,0,0,0,0,0,127,127,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,12,12}
};

#define CHECK(condition, ...) do { if (!(condition)) { \
  fprintf(stderr, "FAIL: "); fprintf(stderr, __VA_ARGS__); \
  fputc('\n', stderr); failures++; return; } } while (0)

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &mutex_locked; }
int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout)
{
  (void)mutex; (void)timeout;
  assert(!mutex_locked); mutex_locked = 1; return pdTRUE;
}
int xSemaphoreGive(SemaphoreHandle_t mutex)
{
  (void)mutex; assert(mutex_locked); mutex_locked = 0; return pdTRUE;
}
void vTaskDelay(uint32_t ticks) { tick += ticks; }
uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
  static const uint8_t ack[] = "OK\r\n";
  (void)uart; assert(length == 1);
  if (!ack_enabled) { tick += timeout; return HAL_TIMEOUT; }
  *data = ack[ack_index++ % 4]; tick++; return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
  char commands[10000], *command;
  unsigned int x1, y1, x2, y2, color, x, y;
  uint32_t work_ms = 0;
  (void)uart; (void)timeout;
  sends++;
  if (sends == fail_send) return HAL_ERROR;
  if (tick < busy_until) early_sends++;
  if (length > largest_send) largest_send = length;
  if (length > 500 || length < 2 ||
      data[length - 2] != '\r' || data[length - 1] != '\n') protocol_errors++;
  assert(length < sizeof(commands));
  memcpy(commands, data, length); commands[length] = '\0';
  command = strtok(commands, ";\r\n");
  while (command != NULL) {
    if (sscanf(command, "BOXF(%u,%u,%u,%u,%u)",
               &x1, &y1, &x2, &y2, &color) == 5) {
      if (x1 > x2 || x2 >= 320 || y1 > y2 || y2 >= 240 ||
          (color != 0 && color != 15)) { protocol_errors++; }
      else {
        for (y = y1; y <= y2; y++)
          for (x = x1; x <= x2; x++) screen[y][x] = color == 0;
      }
      work_ms += (x1 == 0 && y1 == 0 && x2 == 319 && y2 == 239) ? 120 : 40;
    } else if (strcmp(command, "DIR(1)") != 0) {
      protocol_errors++;
    }
    command = strtok(NULL, ";\r\n");
  }
  busy_until = tick + work_ms;
  return HAL_OK;
}

static void reset(int use_ack)
{
  memset(screen, 0, sizeof(screen));
  tick = busy_until = sends = largest_send = protocol_errors = early_sends = 0;
  fail_send = ack_index = 0; ack_enabled = use_ack; mutex_locked = 0;
  JC014_Init();
}

static void expected_row(const char *text, unsigned int x, unsigned int y)
{
  static const char characters[] = "0123456789+-.";
  unsigned int i, r, c, px, py;
  for (i = 0; text[i]; i++) {
    const char *glyph = strchr(characters, text[i]);
    assert(glyph != NULL);
    for (r = 0; r < 14; r++) for (c = 0; c < 7; c++)
      if (font[glyph - characters][r] & (1U << (6 - c)))
        for (py = 0; py < 6; py++) for (px = 0; px < 5; px++)
          expected[y + r * 6 + py][x + i * 37 + c * 5 + px] = 1;
  }
}

static void check_expression(const char *payload, const char *top,
                             unsigned int top_x, const char *bottom,
                             unsigned int bottom_x, int use_ack)
{
  reset(use_ack);
  CHECK(JC014_ShowQrPayload(payload) == HAL_OK, "%s returned an error", payload);
  CHECK(protocol_errors == 0, "%s emitted an invalid/oversized batch (largest %u bytes)",
        payload, largest_send);
  CHECK(early_sends == 0, "%s sent a batch before the previous drawing finished", payload);
  CHECK(!mutex_locked, "%s left the mutex locked", payload);
  memset(expected, 0, sizeof(expected));
  expected_row(top, top_x, 26); expected_row(bottom, bottom_x, 130);
  CHECK(memcmp(screen, expected, sizeof(screen)) == 0,
        "%s did not render the complete expected two-half layout", payload);
  printf("PASS: %s (%s ACK), %u batches, largest %u bytes\n",
         payload, use_ack ? "with" : "without", sends, largest_send);
}

static void startup_white(void)
{
  reset(0); memset(screen, 1, sizeof(screen));
  CHECK(JC014_ClearScreen() == HAL_OK, "startup clear failed");
  memset(expected, 0, sizeof(expected));
  CHECK(!memcmp(screen, expected, sizeof(screen)), "startup was not entirely white");
  CHECK(!protocol_errors && !early_sends, "startup command framing/timing failed");
  puts("PASS: startup white screen");
}

static void replace_long_with_short(void)
{
  reset(1);
  CHECK(JC014_ShowNumber("321+123+456+654") == HAL_OK, "initial long display failed");
  CHECK(JC014_ShowNumber("123") == HAL_OK, "short update failed");
  memset(expected, 0, sizeof(expected)); expected_row("123", 105, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "short update left stale lower-row pixels");
  CHECK(!early_sends, "early ACK allowed overlapping batches between updates");
  puts("PASS: long-to-short update clears old content");
}

static void send_error_recovery(void)
{
  reset(0); fail_send = 2;
  CHECK(JC014_ShowNumber("321+123") == HAL_ERROR, "UART error was not propagated");
  CHECK(JC014_GetStatus() == JC014_STATUS_UART_ERROR, "UART error status was not set");
  CHECK(!mutex_locked, "UART error leaked the mutex");
  fail_send = 0;
  CHECK(JC014_ShowNumber("321+123") == HAL_OK, "could not display after a UART error");
  memset(expected, 0, sizeof(expected)); expected_row("321+123", 31, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "retry did not render complete data");
  CHECK(JC014_GetStatus() == JC014_STATUS_OK, "successful retry did not clear error status");
  puts("PASS: UART error and recovery");
}

static void invalid_inputs(void)
{
  reset(0);
  CHECK(JC014_ShowNumber(NULL) == HAL_ERROR, "NULL accepted");
  CHECK(JC014_ShowNumber("") == HAL_ERROR, "empty number accepted");
  CHECK(JC014_ShowNumber("1x") == HAL_ERROR, "invalid glyph accepted");
  CHECK(JC014_ShowNumber("12345678901234567") == HAL_ERROR, "17 large glyphs accepted");
  CHECK(sends == 0 && !mutex_locked, "invalid input wrote to UART or locked mutex");
  puts("PASS: invalid inputs have no display side effects");
}

static void reject_oversized_command(void)
{
  char command[502];
  reset(0); memset(command, 'X', sizeof(command));
  command[sizeof(command) - 1] = '\0';
  CHECK(JC014_SendCommand(command, 0) == HAL_ERROR, "oversized raw command accepted");
  CHECK(sends == 0, "oversized raw command reached the screen");
  puts("PASS: raw commands respect screen length limit");
}

int main(void)
{
  startup_white();
  check_expression("321+123", "321+123", 31, "", 0, 0);
  check_expression("321+123", "321+123", 31, "", 0, 1);
  check_expression("321+123+456+654", "321+123+", 13, "456+654", 31, 0);
  check_expression("321+123+456+654", "321+123+", 13, "456+654", 31, 1);
  check_expression("8888888888888888", "88888888", 13, "88888888", 13, 0);
  check_expression("0123456789+-.", "01234567", 13, "89+-.", 68, 0);
  replace_long_with_short();
  send_error_recovery();
  invalid_inputs();
  reject_oversized_command();
  printf("%d failed checks\n", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}

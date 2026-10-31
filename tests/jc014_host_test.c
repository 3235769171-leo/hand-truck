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
static unsigned int watch_updates, outside_updates;
static unsigned int watch_x1, watch_y1, watch_x2, watch_y2;
static int ack_enabled, mutex_locked, failures;
static char recorded_text[97];

/* Hand-checked seven-segment silhouettes on the 7x14 grid. */
static const uint8_t font[13][14] = {
  {127,127,99,99,99,99,99,99,99,99,99,99,127,127},
  {3,3,3,3,3,3,3,3,3,3,3,3,3,3},
  {127,127,3,3,3,3,127,127,96,96,96,96,127,127},
  {127,127,3,3,3,3,127,127,3,3,3,3,127,127},
  {99,99,99,99,99,99,127,127,3,3,3,3,3,3},
  {127,127,96,96,96,96,127,127,3,3,3,3,127,127},
  {127,127,96,96,96,96,127,127,99,99,99,99,127,127},
  {127,127,3,3,3,3,3,3,3,3,3,3,3,3},
  {127,127,99,99,99,99,127,127,99,99,99,99,127,127},
  {127,127,99,99,99,99,127,127,3,3,3,3,127,127},
  {0,0,0,28,28,28,127,127,28,28,28,0,0,0},
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
        if (watch_updates && (x1 < watch_x1 || x2 > watch_x2 ||
                              y1 < watch_y1 || y2 > watch_y2)) outside_updates++;
        for (y = y1; y <= y2; y++)
          for (x = x1; x <= x2; x++) screen[y][x] = color == 0;
      }
      work_ms += (x1 == 0 && y1 == 0 && x2 == 319 && y2 == 239) ? 120 : 40;
    } else if (strncmp(command, "DC16(", 5) == 0) {
      if (sscanf(command, "DC16(%u,%u,'%96[^']',%u)",
                 &x1, &y1, recorded_text, &color) != 4) protocol_errors++;
      work_ms += 40;
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
  watch_updates = outside_updates = 0;
  recorded_text[0] = '\0';
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

static void duplicate_payload(int use_ack, const char *payload)
{
  unsigned int previous_sends;
  uint32_t previous_tick;
  reset(use_ack);
  CHECK(JC014_ShowQrPayload(payload) == HAL_OK, "first duplicate fixture failed");
  memcpy(expected, screen, sizeof(screen));
  previous_sends = sends; previous_tick = tick;
  CHECK(JC014_ShowQrPayload(payload) == HAL_OK, "duplicate request failed");
  CHECK(sends == previous_sends && tick == previous_tick,
        "identical payload '%s' caused UART activity or waiting", payload);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "duplicate altered the picture");
  puts("PASS: duplicate payload leaves screen and UART unchanged");
}

static void change_one_character(void)
{
  uint32_t previous_tick;
  reset(1);
  CHECK(JC014_ShowNumber("321+123") == HAL_OK, "one-character fixture failed");
  watch_updates = 1; watch_x1 = 253; watch_x2 = 287;
  watch_y1 = 26; watch_y2 = 109; previous_tick = tick;
  CHECK(JC014_ShowNumber("321+124") == HAL_OK, "one-character update failed");
  CHECK(!outside_updates, "one-character update redrew unchanged screen areas");
  CHECK(tick - previous_tick <= 400, "one-character update took too much waiting");
  memset(expected, 0, sizeof(expected)); expected_row("321+124", 31, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "one-character update picture wrong");
  CHECK(!early_sends && !protocol_errors, "one-character update broke the protocol");
  printf("PASS: one-character update, %lu ms simulated waiting\n",
         (unsigned long)(tick - previous_tick));
}

static void resize_and_remove_bottom_row(void)
{
  reset(0);
  CHECK(JC014_ShowNumber("321+123+456+654") == HAL_OK, "bottom-row fixture failed");
  watch_updates = 1; watch_x1 = 0; watch_x2 = 319;
  watch_y1 = 120; watch_y2 = 239;
  CHECK(JC014_ShowNumber("321+123+456") == HAL_OK, "bottom-row resize failed");
  memset(expected, 0, sizeof(expected));
  expected_row("321+123+", 13, 26); expected_row("456", 105, 130);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "resized bottom row left old pixels");
  CHECK(JC014_ShowNumber("321+123+") == HAL_OK, "bottom-row removal failed");
  memset(expected, 0, sizeof(expected)); expected_row("321+123+", 13, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "removed bottom row left old pixels");
  CHECK(!outside_updates, "bottom-only change redrew the top half");
  CHECK(!early_sends && !protocol_errors, "bottom-row update protocol failed");
  puts("PASS: bottom-row resize/removal preserves top half");
}

static void invalidate_after_external_drawing(void)
{
  unsigned int before;
  reset(1);
  CHECK(JC014_ShowNumber("321+123") == HAL_OK, "cache invalidation fixture failed");
  CHECK(JC014_ClearScreen() == HAL_OK, "explicit clear failed");
  before = sends;
  CHECK(JC014_ShowNumber("321+123") == HAL_OK && sends > before,
        "same data after clear was incorrectly skipped");
  CHECK(JC014_SendCommand("BOXF(0,0,10,10,0);\r\n", 50) == HAL_OK,
        "external drawing fixture failed");
  before = sends;
  CHECK(JC014_ShowNumber("321+123") == HAL_OK && sends > before,
        "same data after raw drawing was incorrectly skipped");
  memset(expected, 0, sizeof(expected)); expected_row("321+123", 31, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "external pixels were not cleared");
  puts("PASS: clear/raw commands invalidate cached screen");
}

static void partial_failure_invalidates_cache(void)
{
  reset(1);
  CHECK(JC014_ShowNumber("321+123") == HAL_OK, "partial-error fixture failed");
  fail_send = sends + 2;
  CHECK(JC014_ShowNumber("421+124") == HAL_ERROR, "partial update error was ignored");
  CHECK(!mutex_locked, "partial update failure leaked mutex");
  fail_send = 0;
  CHECK(JC014_ShowNumber("321+123") == HAL_OK, "old data retry failed");
  memset(expected, 0, sizeof(expected)); expected_row("321+123", 31, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "stale cache preserved a partial update");
  puts("PASS: partial failure forces full recovery before caching");
}

static void text_number_text_transition(void)
{
  unsigned int previous_sends;
  reset(1);
  CHECK(JC014_ShowQrPayload("HELLO") == HAL_OK, "text fixture failed");
  CHECK(!strcmp(recorded_text, "HELLO"), "requested text never reached the screen");
  previous_sends = sends;
  CHECK(JC014_ShowQrPayload("HELLO") == HAL_OK && sends == previous_sends,
        "text duplicate was not skipped");
  CHECK(JC014_ShowQrPayload("321+123") == HAL_OK, "text-to-number transition failed");
  memset(expected, 0, sizeof(expected)); expected_row("321+123", 31, 26);
  CHECK(!memcmp(screen, expected, sizeof(screen)), "text-to-number picture wrong");
  previous_sends = sends; recorded_text[0] = '\0';
  CHECK(JC014_ShowQrPayload("HELLO") == HAL_OK && sends > previous_sends,
        "returning to old text was incorrectly skipped");
  CHECK(!strcmp(recorded_text, "HELLO"), "number-to-text sent the wrong text");
  CHECK(!protocol_errors && !early_sends && !mutex_locked,
        "text/number mode change broke the protocol or mutex");
  puts("PASS: text-number-text transitions verify actual outgoing text");
}

static void digit_height_and_speed(void)
{
  char text[2] = {0, 0};
  unsigned int digit, x, y, min_y, max_y;
  for (digit = 0; digit < 10; digit++) {
    reset(0); text[0] = (char)('0' + digit);
    CHECK(JC014_ShowNumber(text) == HAL_OK, "digit height fixture failed");
    min_y = 240; max_y = 0;
    for (y = 0; y < 240; y++) for (x = 0; x < 320; x++)
      if (screen[y][x]) { if (y < min_y) min_y = y; if (y > max_y) max_y = y; }
    CHECK(min_y == 26 && max_y == 109 && (max_y - min_y + 1) * 153 > 12000,
          "digit %u actual ink height is not 84px / above 12mm", digit);
  }
  reset(0);
  CHECK(JC014_ShowNumber("321+123") == HAL_OK && tick <= 1600,
        "short expression did not become faster (waiting %lu ms)", (unsigned long)tick);
  printf("PASS: short expression %lu ms simulated waiting\n", (unsigned long)tick);
  reset(0);
  CHECK(JC014_ShowNumber("321+123+456+654") == HAL_OK && tick <= 3200,
        "long expression did not become faster (waiting %lu ms)", (unsigned long)tick);
  printf("PASS: long expression %lu ms simulated waiting; all digits 84px tall\n",
         (unsigned long)tick);
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
  duplicate_payload(0, "321+123");
  duplicate_payload(1, "321+123+456+654");
  duplicate_payload(0, "HELLO");
  change_one_character();
  resize_and_remove_bottom_row();
  invalidate_after_external_drawing();
  partial_failure_invalidates_cache();
  text_number_text_transition();
  digit_height_and_speed();
  printf("%d failed checks\n", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}

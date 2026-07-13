#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void cmdp_init(void);
void cmdp_task(void);

/* Called from USB CDC RX callback (IRQ context). */
void cmdp_push_bytes(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif

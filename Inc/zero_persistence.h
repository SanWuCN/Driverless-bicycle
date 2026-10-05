#ifndef ZERO_PERSISTENCE_H
#define ZERO_PERSISTENCE_H

#include <stdbool.h>
#include <stdint.h>

enum
{
    ZERO_PERSISTENCE_VALID = (1u << 0),
    ZERO_PERSISTENCE_SAVED_THIS_BOOT = (1u << 1),
    ZERO_PERSISTENCE_ERROR = (1u << 2)
};

void zero_persistence_init(float factory_zero_deg,
                           float allowed_range_deg,
                           float *startup_zero_deg);
void zero_persistence_service(float candidate_zero_deg, bool learning_active);
uint32_t zero_persistence_status(void);
float zero_persistence_stored_zero(void);
uint32_t zero_persistence_sequence(void);

#endif

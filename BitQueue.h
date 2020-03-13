#ifndef BIT_QUEUE_H
#define BIT_QUEUE_H

#include <Arduino.h>

template <uint16_t bit_size = 128>
class BitQueue {
  protected:
    constexpr static uint16_t q_size = static_cast<uint16_t>(ceil(bit_size/64.0));
    constexpr static uint16_t last_bit_index = 63 - (q_size * 64 - bit_size);
    uint64_t queues[q_size];

  public:
    BitQueue() {
      memset(&queues, 0, sizeof(queues));
    }
    uint8_t push_first_pop_last(uint8_t new_first) {
      uint8_t last_bit = static_cast<uint8_t>( (queues[0] >> last_bit_index) & 0x1 );

      for (uint16_t i = 0; i < q_size - 1; ++i) {
        queues[i] = (queues[i] << 1) | (queues[i + 1] >> 63);
      }
      queues[q_size - 1] = (queues[q_size - 1] << 1) | static_cast<uint64_t>(new_first);

      return last_bit;
    }
    void print(Stream& ss = Serial) {
      for (uint16_t i = 0; i < q_size; ++i) {
        for (int8_t j = 63; j >= 0; --j) {
          ss.print(static_cast<uint8_t>( (queues[i] >> j) & 0x1 ));
        }
      }
    }
};

#endif

#pragma once

#include <array>
#include <cstddef>

namespace cyphal::fdcan {

enum class ReceiveFifo {
    Critical,
    Regular,
};

// FOC command/SYNC frames must be handled before register services.  The
// latter can arrive in bursts during activation and must not delay a RUN
// marker past its watchdog.
constexpr std::array<ReceiveFifo, 2> receive_fifo_priority()
{
    return {ReceiveFifo::Critical, ReceiveFifo::Regular};
}

// The application publishes the synchronized-command acknowledgement after
// each CAN provider pass.  Keep that latency bounded even if a small hardware
// FIFO already contains a burst of accepted frames: consuming a whole FIFO in
// one pass can otherwise postpone STAGED behind repeated command retries.
constexpr std::size_t maximum_frames_per_pass(const ReceiveFifo fifo)
{
    return fifo == ReceiveFifo::Critical ? 1U : 1U;
}

}  // namespace cyphal::fdcan

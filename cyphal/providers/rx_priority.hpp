#pragma once

#include <array>

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

}  // namespace cyphal::fdcan

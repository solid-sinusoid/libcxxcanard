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

// FOC command and SYNC share the three-frame critical hardware FIFO.  Drain
// that bounded FIFO in one provider pass so that a control tick cannot lose a
// command or its matching SYNC while register traffic is being serviced.
// Keep the regular FIFO budget at one frame to retain bounded telemetry work.
constexpr std::size_t maximum_frames_per_pass(const ReceiveFifo fifo)
{
    return fifo == ReceiveFifo::Critical ? 3U : 1U;
}

}  // namespace cyphal::fdcan

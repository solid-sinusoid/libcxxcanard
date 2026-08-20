#pragma once

namespace cyphal::fdcan {

// The STM32G431 has a three-element TX event FIFO. The application never
// consumes it, so retaining completion events makes the FIFO permanently full
// during regular 200 Hz state publication. Transmission itself does not need
// these events; the Cyphal TX queue is released once hardware accepts a frame.
constexpr bool store_tx_events() noexcept
{
    return false;
}

}  // namespace cyphal::fdcan

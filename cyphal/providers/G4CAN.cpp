#include "G4CAN.h"
#if (defined(STM32G) || defined(STM32G4) || defined(STM32G0)) && defined(HAL_FDCAN_MODULE_ENABLED)
#include <algorithm>
#include <cstring>

#include "FDCAN_generic.h"

uint32_t G4CAN::len_to_dlc(size_t len) {
    return CanardFDCANLengthToDLC[len];
}

size_t G4CAN::dlc_to_len(uint32_t dlc) {
    return fdcan_dlc_to_len(dlc);
}

void G4CAN::can_loop(bool no_tx) {
    process_canard_rx();
    if (!no_tx) process_canard_tx();
    update_fdcan_status();
}

void G4CAN::process_canard_rx(bool all) {
#if defined(STM32G4)
    // The longest supported FD frame at >=62.5 kbit/s fits in <20 ms
    // even with no BRS and generous stuffing/clock-domain allowance.
    constexpr uint32_t tick_us = 100U;
    constexpr uint32_t frame_margin_ticks = 20'000U / tick_us;
    constexpr uint32_t epoch_limit_ticks = UINT16_MAX - 1U - frame_margin_ticks;
    const bool hardware = (handler->Instance->TSCC & FDCAN_TSCC_TSS) == FDCAN_TIMESTAMP_EXTERNAL &&
        (TIM3->CR1 & TIM_CR1_CEN) != 0U;
    if (!hardware) fifo1_epoch_known = false;
#endif
    // Drain the critical three-frame snapshot first, then one regular frame;
    // a continuously replenished bus must not starve the application loop.
    for (const uint32_t fifo : {FDCAN_RX_FIFO1, FDCAN_RX_FIFO0}) {
        const auto index = fifo == FDCAN_RX_FIFO1 ? 1U : 0U;
#if defined(STM32G4)
        const uint32_t before = TIM2->CNT;
        const bool wrap = (TIM2->SR & TIM_SR_UIF) != 0U;
        const uint32_t after = TIM2->CNT;
        const uint64_t epoch_us = hardware && index == 1U ? utilities.micros_64() : 0;
        if (wrap || after < before) {
            invalidate_rx_receipt();
        }
#endif
        const uint32_t budget = all && fifo == FDCAN_RX_FIFO1 ? 3U : 1U;
        const auto pending = std::min(HAL_FDCAN_GetRxFifoFillLevel(handler, fifo), budget);
#if defined(STM32G4)
        // Sample time before observing empty: arrivals during the check stay newer.
        if (pending == 0U && !wrap && after >= before) {
            rx_empty_tick[index] = before;
            rx_empty_known[index] = true;
            if (hardware && index == 1U) {
                fifo1_epoch_tick = before;
                fifo1_epoch_us = epoch_us;
                fifo1_epoch_known = true;
            }
        }
#endif
        for (uint32_t count = 0; count < pending; ++count) {
            CanardFrame frame;
            uint8_t data[64] = {};
            if (!read_frame_from_fifo(&frame, data, fifo)) break;
            rx_received_after_tick = rx_empty_tick[index];
            // A final fragment alone cannot bound the age of a whole transfer.
            rx_receipt_valid = rx_empty_known[index] && frame.payload_size > 0U &&
                (data[frame.payload_size - 1U] & 0xe0U) == 0xe0U;
#if defined(STM32G4)
            if (hardware && index == 1U && rx_receipt_valid && fifo1_epoch_known) {
                const uint32_t t2 = TIM2->CNT;
                const uint32_t age = static_cast<uint16_t>(TIM3->CNT - rx_hardware_timestamp) + 1U;
                const uint32_t span = t2 - fifo1_epoch_tick;
                const uint64_t us = utilities.micros_64();
                // Require a unique 16-bit epoch; a frame can be on the
                // wire when FIFO reads empty. The TIM2 check also covers
                // a lost TIM7 epoch while interrupts were masked.
                if ((TIM2->SR & TIM_SR_UIF) == 0U && TIM2->CNT >= t2 &&
                    span < epoch_limit_ticks && us - fifo1_epoch_us < epoch_limit_ticks * tick_us &&
                    age <= span + frame_margin_ticks) {
                    rx_sof_tick = t2 - age;
                    rx_sof_valid = true;
                    // FIFO1 is ordered. The previous SOF can bound the
                    // next frame even if the queue never drains.
                    fifo1_epoch_tick = rx_sof_tick;
                    const uint32_t elapsed_us = (age + 1U) * tick_us;
                    fifo1_epoch_us = epoch_us > elapsed_us ? epoch_us - elapsed_us : 0;
                }
            }
            if (count + 1U == pending) {
                // Date future arrivals, not frames already removed from the FIFO.
                const uint32_t drained_tick = TIM2->CNT;
                const uint64_t drained_us = hardware && index == 1U ? utilities.micros_64() : 0;
                const bool drained = HAL_FDCAN_GetRxFifoFillLevel(handler, fifo) == 0U;
                if ((TIM2->SR & TIM_SR_UIF) != 0U || drained_tick < after || TIM2->CNT < drained_tick) {
                    invalidate_rx_receipt();
                } else if (drained) {
                    rx_empty_tick[index] = drained_tick;
                    rx_empty_known[index] = true;
                    if (hardware && index == 1U) {
                        fifo1_epoch_tick = drained_tick;
                        fifo1_epoch_us = drained_us;
                        fifo1_epoch_known = true;
                    }
                }
            }
#endif
            accept_rx_frame(frame);
            rx_receipt_valid = false;
#if defined(STM32G4)
            rx_sof_valid = false;
#endif
            if (!all) return;
        }
    }
}

void G4CAN::update_fdcan_status() {
    static FDCAN_ProtocolStatusTypeDef fdcan_status;
    HAL_FDCAN_GetProtocolStatus(handler, &fdcan_status);
    if (fdcan_status.BusOff) {
        CLEAR_BIT(handler->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

bool G4CAN::read_frame(CanardFrame* rxf, void* data) {
    if (HAL_FDCAN_GetRxFifoFillLevel(handler, FDCAN_RX_FIFO1)) {
        return read_frame_from_fifo(rxf, data, FDCAN_RX_FIFO1);
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(handler, FDCAN_RX_FIFO0)) {
        return read_frame_from_fifo(rxf, data, FDCAN_RX_FIFO0);
    }
    return false;
}

bool G4CAN::read_frame_from_fifo(CanardFrame* rxf, void* data, const uint32_t fifo) {
    uint8_t* RxData = static_cast<uint8_t*>(data);
    if (HAL_FDCAN_GetRxFifoFillLevel(handler, fifo) == 0U) {
        return false;
    }

    FDCAN_RxHeaderTypeDef RxHeader = {};
    if (HAL_FDCAN_GetRxMessage(handler, fifo, &RxHeader, RxData) != HAL_OK) {
        utilities.error_handler();
        return false;
    }

    rxf->extended_can_id = RxHeader.Identifier;
    rx_hardware_timestamp = RxHeader.RxTimestamp;
    rxf->payload_size = dlc_to_len(RxHeader.DataLength);
    rxf->payload = data;
    return true;
}

int G4CAN::write_frame(const CanardTxQueueItem* ti) {
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = ti->frame.extended_can_id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = CanardFDCANLengthToDLC[ti->frame.payload_size];
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x0;

    // Queue mode orders pending frames by CAN identifier.  Do not wait until
    // every hardware slot is empty: a low-priority state frame may otherwise
    // indefinitely prevent a high-priority FOC APPLIED status from entering
    // the controller on a saturated bus.
    if (HAL_FDCAN_GetTxFifoFreeLevel(handler) == 0U) {
        return -1;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(handler, &TxHeader, (uint8_t*)ti->frame.payload) != HAL_OK) {
        return -1;
    }
    return TxHeader.DataLength;
}

HAL_StatusTypeDef apply_filter(
    uint32_t filter_index,
    G4CAN::Handler hfdcan,
    const CanardFilter& filter,
    const uint32_t destination_fifo) {
    FDCAN_FilterTypeDef hw_filter;
    hw_filter.IdType = FDCAN_EXTENDED_ID;
    hw_filter.FilterIndex = filter_index;
    hw_filter.FilterType = FDCAN_FILTER_MASK;
    hw_filter.FilterConfig = destination_fifo;
    hw_filter.FilterID1 = filter.extended_can_id;
    hw_filter.FilterID2 = filter.extended_mask;
    return HAL_FDCAN_ConfigFilter(hfdcan, &hw_filter);
}
#endif

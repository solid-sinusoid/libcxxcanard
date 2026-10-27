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
    // Drain the critical three-frame snapshot first, then one regular frame;
    // a continuously replenished bus must not starve the application loop.
    for (const uint32_t fifo : {FDCAN_RX_FIFO1, FDCAN_RX_FIFO0}) {
        const uint32_t budget = all && fifo == FDCAN_RX_FIFO1 ? 3U : 1U;
        const auto pending = std::min(HAL_FDCAN_GetRxFifoFillLevel(handler, fifo), budget);
        for (uint32_t count = 0; count < pending; ++count) {
            CanardFrame frame;
            uint8_t data[64] = {};
            if (!read_frame_from_fifo(&frame, data, fifo)) break;
            accept_rx_frame(frame);
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

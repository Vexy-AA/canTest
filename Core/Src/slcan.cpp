#include "slcan.hpp"



extern USBD_HandleTypeDef hUsbDeviceFS;
extern CAN_HandleTypeDef hcan;

////////Helper Methods//////////

static bool hex2nibble_error;

static uint8_t nibble2hex(uint8_t x)
{
    // Allocating in RAM because it's faster
    static uint8_t ConversionTable[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}

static uint8_t hex2nibble(char c)
{
    // Must go into RAM, not flash, because flash is slow
    static uint8_t NumConversionTable[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    };

    static uint8_t AlphaConversionTable[] = {
        10, 11, 12, 13, 14, 15
    };

    uint8_t out = 255;

    if (c >= '0' && c <= '9') {
        out = NumConversionTable[int(c) - int('0')];
    } else if (c >= 'a' && c <= 'f') {
        out = AlphaConversionTable[int(c) - int('a')];
    } else if (c >= 'A' && c <= 'F') {
        out = AlphaConversionTable[int(c) - int('A')];
    }

    if (out == 255) {
        hex2nibble_error = true;
    }
    return out;
}

bool SLCAN::CANIface::push_Frame(CANFrame &frame)
{
    CANIface::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.timestamp_us = native_micros64();
    return add_to_rx_queue(frm);
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANIface::handle_FrameDataExt(const char* cmd, bool canfd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.canfd = canfd;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    f.dlc = hex2nibble(cmd[9]);
    if (hex2nibble_error || f.dlc > (canfd?15:8)) {
        return false;
    }
    {
        const char* p = &cmd[10];
        const uint8_t dlen = CANFrame::dlcToDataLength(f.dlc);
        for (unsigned i = 0; i < dlen; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

uint8_t SLCAN::CANFrame::dlcToDataLength(uint8_t dlc)
{
    /*
    Data Length Code      9  10  11  12  13  14  15
    Number of data bytes 12  16  20  24  32  48  64
    */
    if (dlc <= 8) {
        return dlc;
    } else if (dlc == 9) {
        return 12;
    } else if (dlc == 10) {
        return 16;
    } else if (dlc == 11) {
        return 20;
    } else if (dlc == 12) {
        return 24;
    } else if (dlc == 13) {
        return 32;
    } else if (dlc == 14) {
        return 48;
    }
    return 64;
}

uint8_t SLCAN::CANFrame::dataLengthToDlc(uint8_t data_length)
{
    if (data_length <= 8) {
        return data_length;
    } else if (data_length <= 12) {
        return 9;
    } else if (data_length <= 16) {
        return 10;
    } else if (data_length <= 20) {
        return 11;
    } else if (data_length <= 24) {
        return 12;
    } else if (data_length <= 32) {
        return 13;
    } else if (data_length <= 48) {
        return 14;
    }
    return 15;
}
/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANIface::handle_FDFrameDataExt(const char* cmd)
{
#if HAL_CANFD_SUPPORTED
    return false;
#else
    CANFrame f {};
    hex2nibble_error = false;
    f.canfd = true;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    f.dlc = hex2nibble(cmd[9]);
    if (f.dlc > CANFrame::dataLengthToDlc(CANFrame::MaxDataLen)) {
        return false;
    }
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < CANFrame::dlcToDataLength(f.dlc); i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
#endif //#if HAL_CANFD_SUPPORTED
}

bool SLCAN::CANIface::handle_FrameDataStd(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc > CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANIface::handle_FrameRTRExt(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';

    if (f.dlc > CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANIface::handle_FrameRTRStd(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc <= CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

static inline const char* getASCIIStatusCode(bool status)
{
    return status ? "\r" : "\a";
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data> [timestamp msec] [flags]
 * Types:
 *  R - RTR extended
 *  r - RTR standard
 *  T - Data extended
 *  t - Data standard
 * Flags:
 *  L - this frame is a loopback frame; timestamp field contains TX timestamp
 */
int16_t SLCAN::CANIface::reportFrame(const CANFrame& frame, uint64_t timestamp_usec)
{
#if HAL_CANFD_SUPPORTED
    constexpr unsigned SLCANMaxFrameSize = 200;
#else
    constexpr unsigned SLCANMaxFrameSize = 40;
#endif
    uint8_t buffer[SLCANMaxFrameSize] = {'\0'};
    uint8_t* p = &buffer[0];
    /*
    * Frame type
    */
    if (frame.isRemoteTransmissionRequest()) {
        *p++ = frame.isExtended() ? 'R' : 'r';
    } else if (frame.isErrorFrame()) {
        return -1;     // Not supported
    }
#if HAL_CANFD_SUPPORTED
    else if (frame.canfd) {
        *p++ = frame.isExtended() ? 'D' : 'd';
    }
#endif 
    else {
        *p++ = frame.isExtended() ? 'T' : 't';
    }

    /*
    * ID
    */
    {
        const uint32_t id = frame.id & frame.MaskExtID;
        if (frame.isExtended()) {
            *p++ = nibble2hex(id >> 28);
            *p++ = nibble2hex(id >> 24);
            *p++ = nibble2hex(id >> 20);
            *p++ = nibble2hex(id >> 16);
            *p++ = nibble2hex(id >> 12);
        }
        *p++ = nibble2hex(id >> 8);
        *p++ = nibble2hex(id >> 4);
        *p++ = nibble2hex(id >> 0);
    }

    /*
    * DLC
    */
    *p++ = nibble2hex(frame.dlc);

    /*
    * Data
    */
    for (unsigned i = 0; i < CANFrame::dlcToDataLength(frame.dlc); i++) {
        const uint8_t byte = frame.data[i];
        *p++ = nibble2hex(byte >> 4);
        *p++ = nibble2hex(byte);
    }

    /*
    * Timestamp
    */
    {
        // SLCAN format - [0, 60000) milliseconds
        const auto slcan_timestamp = uint16_t(timestamp_usec / 1000U);
        *p++ = nibble2hex(slcan_timestamp >> 12);
        *p++ = nibble2hex(slcan_timestamp >> 8);
        *p++ = nibble2hex(slcan_timestamp >> 4);
        *p++ = nibble2hex(slcan_timestamp >> 0);
    }

    /*
    * Finalization
    */
    *p++ = '\r';
    const auto frame_size = unsigned(p - &buffer[0]);

    /* if (_port->txspace() < frame_size) {
        return 0;
    } */
    //Write to Serial
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &buffer[0], frame_size);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    /* if (!_port->write_locked(&buffer[0], frame_size, _serial_lock_key)) { !!!
        return 0;
    } */
    return 1;
}

//Accepts command string, returns response string or nullptr if no response is needed.
const char* SLCAN::CANIface::processCommand(char* cmd)
{
    /*
    * High-traffic SLCAN commands go first
    */
    if (cmd[0] == 'T' || cmd[0] == 'D') {
        return handle_FrameDataExt(cmd, cmd[0]=='D') ? "Z\r" : "\a";
    } else if (cmd[0] == 't') {
        return handle_FrameDataStd(cmd) ? "z\r" : "\a";
    } else if (cmd[0] == 'R') {
        return handle_FrameRTRExt(cmd) ? "Z\r" : "\a";
    } else if (cmd[0] == 'r' && cmd[1] <= '9') { // The second condition is needed to avoid greedy matching
        // See long commands below
        return handle_FrameRTRStd(cmd) ? "z\r" : "\a";
    }
#if HAL_CANFD_SUPPORTED 
    else if (cmd[0] == 'D') {
        return handle_FDFrameDataExt(cmd) ? "Z\r" : "\a";
    }
#endif

    uint8_t resp_bytes[40];
    uint16_t resp_len;
    /*
    * Regular SLCAN commands
    */
    switch (cmd[0]) {
    case 'S':               // Set CAN bitrate
    case 'O':               // Open CAN in normal mode
    case 'L':               // Open CAN in listen-only mode
    case 'l':               // Open CAN with loopback enabled
    case 'C':               // Close CAN
    case 'M':               // Set CAN acceptance filter ID
    case 'm':               // Set CAN acceptance filter mask
    case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
    case 'Z': {             // Enable/disable RX and loopback timestamping
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'F': {             // Get status flags
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes), "F%02X\r", unsigned(0));    // Returning success for compatibility reasons
        if (resp_len > 0) {
            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, resp_bytes, resp_len);
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
        return nullptr;
    }
    case 'V': {             // HW/SW version
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"V%x%x%x%x\r", 1, 0, 1, 0);
        if (resp_len > 0) {
            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, resp_bytes, resp_len);
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
        return nullptr;
    }
    case 'N': {             // Serial number
        const uint8_t uid_buf_len = 12;
        uint8_t uid_len = uid_buf_len;
        uint8_t unique_id[uid_buf_len];
        char buf[uid_buf_len * 2 + 1] = {'\0'};
        char* pos = &buf[0];
        if (get_system_id_unformatted(unique_id, uid_len)) {
            for (uint8_t i = 0; i < uid_buf_len; i++) {
                *pos++ = nibble2hex(unique_id[i] >> 4);
                *pos++ = nibble2hex(unique_id[i]);
            }
        } 
        *pos++ = '\0';
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"N%s\r", &buf[0]);
        if (resp_len > 0) {
            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, resp_bytes, resp_len);
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
        return nullptr;
    }
    default: {
        break;
    }
    }

    return getASCIIStatusCode(false);
}

// add bytes to parse the received SLCAN Data stream
inline void SLCAN::CANIface::addByte(const uint8_t byte)
{
    /* if (_port == nullptr) {
        return;
    } */
    if ((byte >= 32 && byte <= 126)) {  // Normal printable ASCII character
        if (pos_ < SLCAN_BUFFER_SIZE) {
            buf_[pos_] = char(byte);
            pos_ += 1;
        } else {
            pos_ = 0;   // Buffer overrun; silently drop the data
        }
    } else if (byte == '\r') {  // End of command (SLCAN)

        // Processing the command
        buf_[pos_] = '\0';
        const char* const response = processCommand(reinterpret_cast<char*>(&buf_[0]));
        pos_ = 0;

        // Sending the response if provided
        if (response != nullptr) {
            char resp[10];
            strcpy(resp, response);
            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, reinterpret_cast<uint8_t*>(resp), strlen(response));
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
    } else if (byte == 8 || byte == 127) {  // DEL or BS (backspace)
        if (pos_ > 0) {
            pos_ -= 1;
        }
    } else {    // This also includes Ctrl+C, Ctrl+D
        pos_ = 0;   // Invalid byte - drop the current command
    }
}

uint16_t SLCAN::CANIface::getNumFilters() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->getNumFilters();
    }
    return 0;
}

uint32_t SLCAN::CANIface::getErrorCount() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->getErrorCount();
    }
    return 0;
}

bool SLCAN::CANIface::is_busoff() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->is_busoff();
    }
    return false;
}

void SLCAN::CANIface::flush_tx()
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        _can_iface->flush_tx();
    }

    /* if (_port) {
        _port->flush();
    } */
}

void SLCAN::CANIface::clear_rx()
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        _can_iface->clear_rx();
    }
    rx_queue_.clear();
}

/* bool SLCAN::CANIface::is_initialized() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->is_initialized();
    }
    return false;
} */

/* bool SLCAN::CANIface::select(bool &read, bool &write, const CANFrame* const pending_tx,
                             uint64_t blocking_deadline)
{
    update_slcan_port();
    bool ret = false;
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        ret = _can_iface->select(read, write, pending_tx, blocking_deadline);
    }
    
    // if under passthrough, we only do send when can_iface also allows it
    if (_port->available_locked(_serial_lock_key) || rx_queue_.available()) {
        // allow for receiving messages over slcan
        read = true;
        ret = true;
    }

    return ret;
} */


// send method to transmit the frame through SLCAN interface
int16_t SLCAN::CANIface::send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags)
{
    int16_t ret = 0;
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        ret = _can_iface->send(frame, tx_deadline, flags);
    }

    if (frame.isErrorFrame()
#if !HAL_CANFD_SUPPORTED
        || frame.dlc > 8
#endif
        ) {
        return ret;
    }
    reportFrame(frame, native_micros64());
    return ret;
}

int16_t SLCAN::CANIface::receiveSerial(uint8_t* Buf, uint32_t *Len){
    rxSerial.write(Buf, *Len);
    int32_t nBytes = rxSerial.available();
    while(nBytes--){
        uint8_t b;
        rxSerial.read_byte(&b);
        addByte(b);
        if (!rx_queue_.space()) {
            break;
        }
    }
    if (rx_queue_.available()) {
        //return sendCan();
    }
    return 0;
}
int16_t SLCAN::CANIface::receiveCan(){
    CAN_RxHeaderTypeDef msgHeader;
    uint32_t msgId = 0;
    uint8_t msgData[8];
    
    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &msgHeader, msgData);
        
    if (msgHeader.IDE == CAN_ID_EXT)
    {
        msgId = msgHeader.ExtId;
    }
    else
    {
        msgId = msgHeader.StdId;
    }
    return 0;
}
int16_t SLCAN::CANIface::sendCan(){
    if (rx_queue_.available()) {
        // if we already have something in buffer transmit it
        CanRxItem frm;
        if (!rx_queue_.peek(frm)) {
            return 0;
        }
        /* out_frame = frm.frame;
        rx_time = frm.timestamp_us;
        out_flags = frm.flags; */
        _last_had_activity = HAL_GetTick();
        // Also send this frame over can_iface when in passthrough mode,
        // We just push this frame without caring for priority etc
        if (_can_iface) {
            bool read = false;
            bool write = true;
            _can_iface->select(read, write, &frm.frame, 0); // select without blocking
            if (write && _can_iface->send(frm.frame, native_micros64() + 100000, frm.flags) == 1) {
                    rx_queue_.pop();
                    num_tries = 0;
            } else if (num_tries > 8) {
                rx_queue_.pop();
                num_tries = 0;
            } else {
                num_tries++;
            }
        } else {
            // we just throw away frames if we don't
            // have any can iface to pass through to
            rx_queue_.pop();
        }
        return 1;
    }
    return 0;
}
// receive method to read the frame recorded in the buffer
int16_t SLCAN::CANIface::receive(CANFrame& out_frame, uint64_t& rx_time,
                                 CanIOFlags& out_flags)
{
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        int16_t ret = _can_iface->receive(out_frame, rx_time, out_flags);
        if (ret > 0) {
            // we also pass this frame through to slcan iface,
            // and immediately return
            reportFrame(out_frame, native_micros64()); 
            return ret;
        } else if (ret < 0) {
            return ret;
        }
    }

    // We found nothing in HAL's CANIface recieve, so look in SLCANIface
    /* if (_port == nullptr) {
        return 0;
    } */

    
    /* uint32_t num_bytes = _port->available_locked(_serial_lock_key);
    // flush bytes from port
    while (num_bytes--) {
        uint8_t b;
        if (!_port->read_locked(_serial_lock_key, b)) {
            break;
        }
        addByte(b);
        if (!rx_queue_.space()) {
            break;
        }
    }
    if (rx_queue_.available()) {
        // if we already have something in buffer transmit it
        CanRxItem frm;
        if (!rx_queue_.peek(frm)) {
            return 0;
        }
        out_frame = frm.frame;
        rx_time = frm.timestamp_us;
        out_flags = frm.flags;
        _last_had_activity = HAL_GetTick();
        // Also send this frame over can_iface when in passthrough mode,
        // We just push this frame without caring for priority etc
        if (_can_iface) {
            bool read = false;
            bool write = true;
            _can_iface->select(read, write, &out_frame, 0); // select without blocking
            if (write && _can_iface->send(out_frame, native_micros64() + 100000, out_flags) == 1) {
                    rx_queue_.pop();
                    num_tries = 0;
            } else if (num_tries > 8) {
                rx_queue_.pop();
                num_tries = 0;
            } else {
                num_tries++;
            }
        } else {
            // we just throw away frames if we don't
            // have any can iface to pass through to
            rx_queue_.pop();
        }
        return 1;
    } */
    return 0;
}

int8_t SLCAN::CANIface::sendBack(uint8_t* Buf, uint32_t *Len){
    CDC_Transmit_FS(Buf, *Len);
    return 0;
}


bool get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    len = MIN(12, len);
    memcpy(buf, (const void *)UDID_START, len);
    return true;
}


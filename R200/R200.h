#ifndef R200_h
#define R200_h

#include <stdint.h>
#include <Arduino.h>
//#include <HardwareSerial.h>

#define MAX_RECEIVED_LENGTH 10
#define MAX_SEND_LENGTH 10

// https://docs.electron.id/el-uhf-rmt01/index.html
// https://github.com/m5stack/M5Unit-UHF-RFID/blob/master/examples/Unit_RFID_M5Atom/Unit_RFID_M5Atom.ino


class R200 {

  private:
    HardwareSerial *_serial;
    unsigned long _timeOutTimer;
    unsigned long _timeOutDuration = 500;
    uint8_t _received[MAX_RECEIVED_LENGTH];
    uint8_t _sending[MAX_SEND_LENGTH];
    // How many bytes have been received in the current response
    uint8_t _receivedIndex=0;

    uint8_t calculateCheckSum(uint8_t *buffer);
    uint16_t arrayToUint16(uint8_t *array);

  public:

    R200();
    void poll();
    bool loop();
    bool begin(HardwareSerial *serial = &Serial2, int baud = 115200, uint8_t RxPin = 16, uint8_t TxPin = 17);
    void parseResponse();
    bool available();
    void getModuleInfo();
    bool waitForResponse();
    bool getResponse();

    void setMultiplePollingMode();

 // Commands sent to the reader, and responses received back, are sent as data frames, e.g.
 // Header | Type | Command | ParamLength (2bytes) | Parameter(s) | Checksum | End
 //   AA   |  00  |   07    |      00 03           |   04 02 05   |    15    | DD
 //
 // Frames always start with the header value 0xAA
 // Type indicates a command to the reader (0x00), or a response (0x01), or notification (0x02) back from it
 // Command is the instruction to be performed, or the response from that instruction
 // ParamLength gives 2-byte (MSB then LSB) number of parameters being passed in the frame
 // Params may be zero or more
 // Checksum is the LSB of the sum of bytes from the type to the last instruction parameter (i.e. excluding Frame Header)
 // Frames always end with the tail value 0xDD

  // Position of elements in the frame definition, as offset from the header
  enum R200_FrameStructure : byte {
    R200_HeaderPos = 0x00,
    R200_TypePos = 0x01,
    R200_CommandPos = 0x02,
    R200_ParamLengthMSBPos = 0x03,
    R200_ParamLengthLSBPos = 0x04,
    // Offset of other response elements - parameters, checksum, and frame end - are variable
    // R200_ParamPos = if(R200_ParamLengthMSBPos << 8 + R200_ParamLengthLSBPos) > 0) { 0x05 } else { null }
    // R200_ChecksumPos = 0x05 + (R200_ParamLengthMSBPos << 8 + R200_ParamLengthLSBPos)
    // R200_EndPos = 0x06 + (R200_ParamLengthMSBPos << 8 + R200_ParamLengthLSBPos)
  };


  enum R200_FrameControl : byte {
    R200_FrameHeader = 0xAA,
    R200_FrameEnd = 0xDD,
  };

  enum R200_FrameType : byte {
    FrameType_Command = 0x00,
    FrameType_Response = 0x01,
    FrameType_Notification = 0x02,
  };

  // 35.
	enum R200_Command : byte {
    CMD_GetModuleInfo = 0x03,
    CMD_SinglePollInstruction = 0x22,
    CMD_MultiplePollInstruction = 0x27,
    CMD_StopMultiplePoll = 0x28,
    CMD_SetSelectParameter = 0x0C,
    CMD_GetSelectParameter = 0x0B,
    CMD_SetSendSelectInstruction = 0x12,
    CMD_ReadLabel = 0x39,
    CMD_WriteLabel = 0x49,
    CMD_LockLabel = 0x82,
    CMD_KillTag = 0x65,
    CMD_GetQueryParameters = 0x0D,
    CMD_SetQueryParameters= 0x0E,
    CMD_SetWorkArea = 0x07,
    CMD_SetWorkingChannel = 0xAB,
    CMD_GetWorkingChannel = 0xAA,
    CMD_SetAutoFrequencyHopping = 0xAD,
    CMD_AcquireTransmitPower = 0xB7,
    CMD_SetTransmitPower = 0xB6,
    CMD_SetTransmitContinuousCarrier = 0xB0,
    CMD_GetReceiverDemodulatorParameters = 0xF1,
    CMD_SetReceiverDemodulatorParameters = 0xF0,
    CMD_TestRFInputBlockingSignal = 0xF2,
    CMD_TestChannelRSSI = 0xF3,
    CMD_ControlIOPort = 0x1A,
    CMD_ModuleSleep = 0x17,
    CMD_SetModuleIdleSleepTime = 0x1D,
    CMD_ExecutionFailure = 0xFF,
  };
};
#endif
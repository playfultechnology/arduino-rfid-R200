#ifndef R200_h
#define R200_h

#include <stdint.h>
#include <Arduino.h>

class R200 {
public:


// Header   Type    Command    Length (2bytes)   Parameter   Checksum    End
//   AA      00       07           00 03         04 02 05       15        DD
//
// Header is always 0xAA
// Type is 00 for commands, 01 for responses, and 02 for notifications
// Command is the instruction code
// rNum of parameters about to be specified (2 bytes)
// Parameter(s)
// Checksum is the LSB of the sum of bytes from the type to the last instruction parameter
// Tail is always 0xDD

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
	enum PCD_Instruction : byte {
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
  };
};
#endif
#ifndef CMD_MSGS_H
#define CMD_MSGS_H

#include <cstdint>
#include <algorithm>

typedef void (*freq_update_cb)(double freq, uint32_t emid);

static const uint16_t MAXMSGSZ = 128;

enum MsgType { 
    STATUSREQ = 1,
    STATUSRESP,
    FREQUPDATE
};

enum GaugeCmdType { 
    STDCMD = 1,
    CTRLCMD,
    RAMREAD,
    RAMWRITE,
};

struct EMDriverMsg {
    bool ackbit;
    uint16_t msgid;
    int32_t freq_a, freq_b, freq_c; // Millihertz

    EMDriverMsg();
    EMDriverMsg(uint32_t mHz_a, uint32_t mHz_b, uint32_t mHz_c);
    EMDriverMsg(EMDriverMsg& other);
    EMDriverMsg& operator=(EMDriverMsg& rhs);
};

struct StatusReqMsg {
    MsgType msgtype = MsgType::STATUSREQ;
    uint32_t msgcnt; 
};

struct StatusRespMsg {
    MsgType msgtype = MsgType::STATUSRESP;
    float freq_a;
    float freq_b;
    float freq_c;
};

struct FreqUpdateMsg {
    MsgType msgtype = MsgType::FREQUPDATE;
    double freq;
    uint32_t emid; 
    uint32_t fcnt;
};

#endif // CMD_MSGS_H

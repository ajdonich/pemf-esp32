#include "CmdMsgs.h"

EMDriverMsg::EMDriverMsg() : EMDriverMsg(-1,-1,-1) {}
EMDriverMsg::EMDriverMsg(uint32_t mHz_a, uint32_t mHz_b, uint32_t mHz_c) :
    ackbit(false), msgid(65535), freq_a(mHz_a), freq_b(mHz_b), freq_c(mHz_c) {}

EMDriverMsg::EMDriverMsg(EMDriverMsg& other) : 
    ackbit(other.ackbit), msgid(other.msgid), freq_a(other.freq_a), 
    freq_b(other.freq_b), freq_c(other.freq_c) {}

EMDriverMsg& EMDriverMsg::operator=(EMDriverMsg& rhs) {
    // ackbit = rhs.ackbit;
    // msgid = rhs.msgid;
    freq_a = rhs.freq_a;
    freq_b = rhs.freq_b;
    freq_c = rhs.freq_c;
    return *this;
}

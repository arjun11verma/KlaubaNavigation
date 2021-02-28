#include<DW1000.h>
#include "ProtocolConsts.h"

void setCurrentConfigs()
{
    DW1000.setDefaults();
}

void any_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	int i;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void any_msg_get_ts(const uint8_t *ts_field, uint64_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}


void generic_send(uint8_t *buffer_to_send, int buffer_size, int ts_index, int delayed, int randMax=0) {
    DW1000.newTransmit();
    setCurrentConfigs();
    
    uint32_t txDelay;
    switch (delayed) {
        case SEND_DELAY_FIXED: {
            txDelay = FIXED_DELAY;
            break;
        }
        case SEND_DELAY_RANDOM: {
            txDelay = FIXED_DELAY + random(randMax)*RESP_DURATION;
            break;
        }
    }
    
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(txDelay, DW1000Time::MILLISECONDS);
    // delay sending the message for the given amount
    
    
    txTime = DW1000.setDelay(deltaTime);

    DW1000Time currTime;
    DW1000.getSystemTimestamp(currTime);

    uint64_t txTime64 = txTime.getTimestamp();
    any_msg_set_ts(&buffer_to_send[ts_index], txTime64);

    DW1000.setData(buffer_to_send, buffer_size);
    
    DW1000.startTransmit();
    
}

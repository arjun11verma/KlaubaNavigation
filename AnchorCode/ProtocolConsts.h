#define NONE_TYPE 0x00
#define ERR_TYPE 0xFE
#define NOT_MY_PACKET_TYPE 0xAA
#define POLL_MSG_TYPE 0x20
#define RESP_MSG_TYPE 0x21
#define FINAL_MSG_TYPE 0x22
#define FINAL_MSG_ALL_TYPE 0x23
#define TWR_DONE_TYPE 0x05

//#define DIST_EST_MSG_TYPE 0x24
//#define SYNC_MSG_TYPE 0x25
//#define TWR_ALL_TYPE 0x01
//#define TWR_SPECIFIC_TYPE 0x02
//#define TWR_SOME_ALL_TYPE 0x08
//#define TWR_SOME_SOME_TYPE 0x09
//#define TWR_ALL_CASCADE_TYPE 0x0A
//#define NODES_AVAILABLE 0x04
//#define ALL_REPORT_TWR_TYPE 0x03
//#define TWR_REPORT_FROM_NODE 0x06
//#define NODES_COUNT_TYPE 0x07
//#define ALL_QUAL_REPORT_TYPE 0xB
//#define QUAL_REPORT_FROM_NODE 0xC
//FINAL_MSG_RESP_RX_TS_IDX
#define BROADCAST_ID 0xFF

#define PAN_ID_IDX 1
#define SRC_IDX 2
#define DST_IDX 3
#define SEQ_IDX 4
#define POLL_MSG_POLL_TX_TS_IDX 6
#define POLL_MSG_POLL_NUM_RESP_IDX 11 //number of responder
#define POLL_MSG_POLL_RESP_ORDER_IDX 12 //sets responder schedule

#define FINAL_MSG_FINAL_TX_TS_IDX 6
#define FINAL_MSG_RESP_IDX 11
#define FINAL_MSG_RESP_RX_TS_IDX 12
#define FINAL_MSG_ONE_RESP_ENTRY 6

#define RESP_MSG_PREV_DB_IDX 12
#define RESP_MSG_PREV_RB_IDX 18

#define POLL_MSG_RAND_BOUND_IDX 12

#define ANY_MSG_TS_LEN 5

//#define FIXED_DELAY 3
uint32_t FIXED_DELAY = 4;

typedef enum send_modes{SEND_IMMEDIATE, SEND_DELAY_FIXED, SEND_DELAY_BOARDID, SEND_SHORT_DELAY_BOARDID, SEND_LONG_DELAY_BOARDID, SEND_DELAY_GIVEN, SEND_DELAY_RANDOM} SEND_MODES;


#define RESP_DURATION 2


#define INFINITE_TIME 0xFFFFFFFFFFFFFFFF

#define ACK_TIMEOUT 20

#define TYPICAL_RX_TIMEOUT 10000

#define MAX_POLL_LEN 40
#define MAX_RESP_LEN 24
#define MAX_FINAL_LEN 60

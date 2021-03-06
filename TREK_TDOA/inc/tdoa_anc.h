/* 
 *
 */
#ifndef _TDOA_ANC_H_
#define _TDOA_ANC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "port_deca.h"

#define NSLOTS			8
#define TDMA_SLOT_BITS	26
#define TDMA_NSLOT_BITS 3

#define TDMA_FRAME_BITS (TDMA_SLOT_BITS + TDMA_NSLOT_BITS)
#define TDMA_SLOT_LEN	(1ull<<(TDMA_SLOT_BITS+1))
#define TDMA_FRAME_LEN	(1ull<<(TDMA_FRAME_BITS+1))

#define TDMA_LAST_FRAME(NOW) (NOW & ~(TDMA_FRAME_LEN-1))

#define PREAMBLE_LENGTH_S (128 * 1017.63e-9)
#define PREAMBLE_LENGTH	  (uint64_t)(PREAMBLE_LENGTH_S * 499.2e6 * 128)

#define TDMA_GUARD_LENGTH_S (1e-6)
#define TDMA_GUARD_LENGTH	(uint64_t)(TDMA_GUARD_LENGTH_S * 499.2e6 * 128)

#define RECEIVE_TIMEOUT		250

#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits

#define KF_SYNC             10
#define TS_TX_SIZE          4

#define MAC802154_HEADER_LENGTH 21
#define FRAME_CRC               2

typedef union dwTime_u {
	uint8 raw[5];
	uint64_t full;
	struct {
		uint32 low32;
		uint8 high8;
	}__attribute__((packed));
	struct {
		uint8 low8;
		uint32 high32;
	}__attribute__((packed));
} dwTime_t;

//FSM states
enum state_e {
	syncTdmaState = 0, //Anchor 1 to 5 starts here and rise up to synchronizedState
	syncTimeState,
	synchronizedState, //Anchor 0 is always here
};

enum slotState_e {
	slotRxDone,
	slotTxDone,
};

typedef enum {
	RX_OK,
	RX_TO,
	RX_ERR,
	TX_OK,
} eventState_e;

//This context struct contains all the required global values of the algorithm
struct ctx_s {
	uint8 anchorId;
	enum state_e state;
	enum slotState_e slotState;
	
	uint8 msg_index;

	// Current and next TDMA slot
	uint8 slot;
	uint8 nextSlot;
	
	// TDMA start of frame in local clock
	dwTime_t tdmaFrameStart;

	
	uint8_t packetIds[NSLOTS];
	uint32_t rxTimestamps[NSLOTS];
	uint32_t txTimestamps[NSLOTS];
	uint16_t distances[NSLOTS];

	//Kalman Filter vars
	double xhat[2];
	uint8_t A0_sync;
} ctx;

#define PACKET_TYPE_RANGE 0x21

typedef struct rangePacket_s {
	uint8_t type;
	uint8_t idx;				//index at master
	uint8_t timestamps[NSLOTS][TS_TX_SIZE];	//Relevant time for anchors
	uint16_t distances[NSLOTS];
}__attribute__((packed)) rangePacket_t;

typedef struct packet_s {
	union {
		uint16_t fcf;
		struct {
			uint16_t type:3;
			uint16_t security:1;
			uint16_t framePending:1;
			uint16_t ack:1;
			uint16_t ipan:1;
			uint16_t reserved:3;
			uint16_t destAddrMode:2;
			uint16_t version:2;
			uint16_t srcAddrMode:2;
		} fcf_s;
	};

	uint8_t seq;
	uint16_t pan;
	uint8_t destAddress[8];
	uint8_t sourceAddress[8];

	uint8_t payload[64];
} __attribute__((packed)) packet_t;

uint8 anc_prf, anc_chan;
uint32 tx_failed_count;

void tdoa_init(uint8 s1switch, dwt_config_t *config);

void setupTx(void);
void setupRx(void);
void updateSlot(void);

void setTxData(void);

uint32 adjustRxTime(dwTime_t *time);
dwTime_t transmitTimeForSlot(int slot);

void handleRxPacket(void);

void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);


#define CIR_PWR_OFFSET	0x06
#define TWOPOWER17		131072.0f // 2^17
#define DISTANCE_OF_RADIO_INV 213.139451293f
void dwCorrectTimestamp(dwTime_t* timestamp);
float dwGetReceivePower(void);
float calculatePower(float base, float N);

#ifdef __cplusplus
}
#endif

#endif

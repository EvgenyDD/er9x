/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "er9x.h"
#include <stdlib.h>
#include "templates.h"
#include "menus.h"
#include "pulses.h"
#include "lcd.h"

#include "spi.h"
#include "nrf24l01.h"

#define USING_1MBPS //else - 250kbps
#define TX_POWER TXPOWER_150mW //no matter?? - transmitter contains PA
//#define TX_POWER TXPOWER_30mW
//#define TX_POWER TXPOWER_10mW
//#define TX_POWER TXPOWER_1mW

#define BIND_COUNT 50 // for KN 2sec every 2ms - 1000 packets

#define PAYLOADSIZE 16
#define NFREQCHANNELS 4
#define TXID_SIZE 4


enum {
    FLAG_DR     = 0x01, // Dual Rate
    FLAG_TH     = 0x02, // Throttle Hold
    FLAG_IDLEUP = 0x04, // Idle up
    FLAG_RES1   = 0x08,
    FLAG_RES2   = 0x10,
    FLAG_RES3   = 0x20,
    FLAG_GYRO3  = 0x40, // 00 - 6G mode, 01 - 3G mode
    FLAG_GYROR  = 0x80  // Always 0 so far
};

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10
};


static u8 packet[PAYLOADSIZE];
static u8 packet_sent;

static u8 rf_ch_num;
static u8 tx_id[TXID_SIZE]={178,197,74,47};
static u8 hopping_frequency[NFREQCHANNELS] = {56,250,234,26};

static u8 packet_count;
static u16 bind_counter=100;

static u8 tx_power;
static u16 throttle=0, rudder=0, elevator=0, aileron=0;
static u8 flags=0;

static u8 spiIsInited=0;

static volatile u8 phase;

enum {
    KN_INIT2 = 0,
    KN_INIT2_NO_BIND,
    KN_BIND,
    KN_DATA
};



// Bit vector from bit position
#define BV(bit) (1 << bit)

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

static u8 packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}

// 2-bytes CRC
#define CRC_CONFIG (BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO))

static void kn_init()
{
    NRF24L01_Initialize();

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, CRC_CONFIG); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);    // Disable retransmit
    NRF24L01_SetPower(TX_POWER);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);   // bytes of data payload for pipe 0

    NRF24L01_Activate(0x73);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1); // Dynamic payload for data pipe 0
    // Enable: Dynamic Payload Length, Payload with ACK , W_TX_PAYLOAD_NOACK
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, BV(NRF2401_1D_EN_DPL) | BV(NRF2401_1D_EN_ACK_PAY) | BV(NRF2401_1D_EN_DYN_ACK));

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
    //printf("Trying to switch banks\n");
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        //printf("BK2421 detected\n");
//        long nul = 0;
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        NRF24L01_WriteRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
        NRF24L01_WriteRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
        NRF24L01_WriteRegisterMulti(0x03, (u8 *) "\xF9\x00\x39\x21", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xC1\x96\x9A\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
//        NRF24L01_WriteRegisterMulti(0x06, (u8 *) &nul, 4);
//        NRF24L01_WriteRegisterMulti(0x07, (u8 *) &nul, 4);
//        NRF24L01_WriteRegisterMulti(0x08, (u8 *) &nul, 4);
//        NRF24L01_WriteRegisterMulti(0x09, (u8 *) &nul, 4);
//        NRF24L01_WriteRegisterMulti(0x0A, (u8 *) &nul, 4);
//        NRF24L01_WriteRegisterMulti(0x0B, (u8 *) &nul, 4);
        NRF24L01_WriteRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xC7\x96\x9A\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xC1\x96\x9A\x1B", 4);
    } else {
        //printf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
}


static void kn_init2()
{
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    packet_sent = 0;
    packet_count = 0;
    rf_ch_num = 0;

    // Turn radio power on
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, CRC_CONFIG | BV(NRF24L01_00_PWR_UP));
    // delayMicroseconds(150);

    NRF24L01_SetTxRxMode(TX_EN);
}


static void set_tx_for_bind()
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 83);
    NRF24L01_SetBitrate(NRF24L01_BR_1M); // 1Mbps for binding
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8 *) "KNDZK", 5);
}


static void set_tx_for_data()
{
    u8 tx_addr[5];
    for (int i = 0; i < TXID_SIZE; ++i)
        tx_addr[i] = tx_id[i];
    tx_addr[4] = 'K';

#ifdef USING_1MBPS
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
#else
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
#endif

    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, tx_addr, 5);
}


#define PACKET_COUNT_SHIFT 5
#define RF_CHANNEL_SHIFT 2

static void send_packet(u8 bind)
{
    static u8 rf_ch;
    if (bind) {
        rf_ch = 83;
        packet[0]  = 'K';
        packet[1]  = 'N';
        packet[2]  = 'D';
        packet[3]  = 'Z';
        packet[4]  = tx_id[0];
        packet[5]  = tx_id[1];
        packet[6]  = tx_id[2];
        packet[7]  = tx_id[3];
        packet[8]  = hopping_frequency[0];
        packet[9]  = hopping_frequency[1];
        packet[10] = hopping_frequency[2];
        packet[11] = hopping_frequency[3];
        packet[12] = 0x00;
        packet[13] = 0x00;
        packet[14] = 0x00;
#ifdef USING_1MBPS
        packet[15] = 0x01;
#else
        packet[15] = 0x00;
#endif
    } else {

    	rf_ch = hopping_frequency[rf_ch_num];

        // Each packet is repeated 4 times on the same channel
        // We're not strictly repeating them, rather we
        // send new packet on the same frequency, so the
        // receiver gets the freshest command. As receiver
        // hops to a new frequency as soon as valid packet
        // received it does not matter that the packet is
        // not the same one repeated twice - nobody checks this

        // NB! packet_count overflow is handled and used in
        // callback.
        if (++packet_count == 4)
            rf_ch_num = (rf_ch_num + 1) & 0x03;

       // read_controls(&throttle, &aileron, &elevator, &rudder, &flags);
        throttle = (g_chans512[2]+1023)>>1;
        aileron = (g_chans512[3]+1023)>>1;
        elevator = (g_chans512[1]+1023)>>1;
        rudder = (g_chans512[0]+1023)>>1;

		if (g_chans512[4] <= 100) flags &= ~FLAG_DR;
		else flags |= FLAG_DR;

		if (g_chans512[5] <= 100) flags &= ~FLAG_TH;
		else flags |= FLAG_TH;

		if (g_chans512[6] <= 100) flags &= ~FLAG_IDLEUP;
		else flags |= FLAG_IDLEUP;

		if (g_chans512[7] <= 100) flags &= ~FLAG_GYRO3;
		else flags |= FLAG_GYRO3;

        packet[0]  = (throttle >> 8) & 0xFF;
        packet[1]  = throttle & 0xFF;
        packet[2]  = (aileron >> 8) & 0xFF;
        packet[3]  = aileron  & 0xFF;
        packet[4]  = (elevator >> 8) & 0xFF;
        packet[5]  = elevator & 0xFF;
        packet[6]  = (rudder >> 8) & 0xFF;
        packet[7]  = rudder & 0xFF;
        // Trims, middle is 0x64 (100) 0-200
        packet[8]  = 0x64; // T
        packet[9]  = 0x64; // A
        packet[10] = 0x64; // E
        packet[11] = 0x64; // R
        packet[12] = flags;
        packet[13] = (packet_count << PACKET_COUNT_SHIFT) | (rf_ch_num << RF_CHANNEL_SHIFT);
        packet[14] = packet[15] = 0x00;
    }

    packet_sent = 0;
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(packet, sizeof(packet));

    packet_sent = 1;

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
    if (! rf_ch_num && tx_power != TX_POWER) {
        //Keep transmit power updated
        tx_power = TX_POWER;
        NRF24L01_SetPower(tx_power);
    }
}



void kn_initAll(uint8_t protocol)
{
	BitReset(spiIsInited, 0);

	if(protocol == PROTO_NRF)
	{//init
		SPI_Init();
		kn_init();
		phase = KN_INIT2;// : KN_INIT2_NO_BIND;

		PORTB |= (1<<1)|(1<<2);
		DDRB &= ~((1<<1)|(1<<2));
		BitSet(spiIsInited, 1);
	}
	else
	{//deinit
		if(BitIsSet(spiIsInited, 1))
			NRF24L01_Reset();
		SPI_DeInit();
		BitReset(spiIsInited, 1);
	}

	BitSet(spiIsInited, 0);
}

void kn_callback()
{
	heartbeat |= HEART_TIMER2Mhz;
	if(!spiIsInited) return;

	// T, R, E, A, DR, TH, IDLEUP, GYRO3
	DDRB |= (1<<1)|(1<<2);

	switch (phase)
	{
	    case KN_INIT2:
	        bind_counter = BIND_COUNT;
	        kn_init2();
	        phase = KN_BIND;
	        set_tx_for_bind();
	        break;

	    case KN_BIND:
	        if (packet_sent && packet_ack() != PKT_ACKED)
	            break;
	        send_packet(1);
	        if (--bind_counter == 0)
	        {
	            phase = KN_DATA;
	            set_tx_for_data();
	        }
	        break;

	    case KN_DATA:
	        if (packet_count >= 4)
	            packet_count = 0;
	        else
	        {
	            if (packet_sent && packet_ack() != PKT_ACKED)
	                break;
	            send_packet(0);
	        }
	        break;
	    }

	PORTB |= (1<<1)|(1<<2);
	DDRB &= ~((1<<1)|(1<<2));
}

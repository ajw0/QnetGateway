/*
 *   Copyright (C) 2018-2020 by Thomas A. Early N7TAE
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#pragma once

#include <atomic>
#include <cstring>
#include <string>
#include <queue>

#include <netinet/in.h>
#include "Random.h"	// for streamid generation
#include "UnixDgramSocket.h"
#include "Base.h"

#define CALL_SIZE 8
#define IP_SIZE 15
#define BT_BATCH_SIZE 4  // Number of frames per Bluetooth batch

// Reply types from Icom radio
enum REPLY_TYPE
{
	RT_TIMEOUT,      // No data / 0xFF
	RT_ERROR,        // Read error
	RT_UNKNOWN,      // Unrecognized packet type
	RT_HEADER,       // Header packet (type 0x10)
	RT_DATA,         // Single voice frame (type 0x12) - USB mode
	RT_HEADER_ACK,   // Header acknowledgment (type 0x21)
	RT_DATA_ACK,     // Voice acknowledgment (type 0x23) - USB mode
	RT_PONG,         // Pong response (type 0x03)
	RT_DATA_BT       // Batched voice frames (type 0x13) - Bluetooth mode
};

// Single AMBE voice frame structure
// Used in both individual and batched packets
struct ambe_frame {
	unsigned char sequence;  // Frame sequence number (modulo 21)
	unsigned char ambe[9];   // AMBE voice data
	unsigned char text[3];   // Slow data (call signs, etc.)
};

// Icom Terminal and Access Point Mode data structure
#pragma pack(push, 1)
using SITAP = struct itap_tag
{
	unsigned char length;  // Packet length
	// 41 for header
	// 16 for single voice frame
	// 56 for batched voice frames (Bluetooth)

	unsigned char type;    // Packet type
	// Receive (from radio):
	//   0x03U - pong
	//   0x10U - header
	//   0x11U - header acknowledgment
	//   0x12U - single voice frame (USB)
	//   0x13U - batched voice frames (Bluetooth, 4 frames)
	// Transmit (to radio):
	//   0x02U - ping
	//   0x20U - header
	//   0x21U - header acknowledgment
	//   0x22U - single voice frame
	//   0x23U - batched voice frames (if supported)

	union
	{
		// Header packet (41 bytes)
		struct
		{
			unsigned char flag[3];  // Flags
			unsigned char r2[8];    // RPT2 callsign
			unsigned char r1[8];    // RPT1 callsign
			unsigned char ur[8];    // URCALL (destination)
			unsigned char my[8];    // MYCALL (source)
			unsigned char nm[4];    // Suffix
		} header;

		// Single voice frame (16 bytes) - USB mode
		struct
		{
			unsigned char counter;  // Frame counter (resets with each header)
			ambe_frame frame;       // Voice frame
		} voice;

		// Batched voice frames (56 bytes) - Bluetooth mode
		struct
		{
			unsigned char counter;       // Batch counter
			unsigned char count;         // Number of frames in batch (typically 4)
			ambe_frame frames[BT_BATCH_SIZE];  // Array of voice frames
		} voice_bt;
	};
};
#pragma pack(pop)

// Frame wrapper class for queuing
class CFrame
{
public:
	CFrame(const unsigned char *buf)
	{
		memcpy(&frame.length, buf, buf[0]);
	}

	CFrame(const CFrame &from)
	{
		memcpy(&frame.length, from.data(), from.size());
	}

	~CFrame() {}

	size_t size() const { return (size_t)frame.length; }

	const unsigned char *data() const { return &frame.length; }

private:
	SITAP frame;
};

class CQnetBT : public CModem
{
public:
	// Constructor / Destructor
	CQnetBT(int mod) : CModem(mod) {}
	~CQnetBT() {}

	// Main interface
	bool Initialize(const std::string &cfgfile);
	void Run();
	void Close();

private:
	// Packet processing
	bool ProcessGateway(const int len, const unsigned char *raw);
	bool ProcessBT(const unsigned char *raw);
	bool ProcessBatchedVoice(const unsigned char *raw, unsigned int len);

	// Serial communication
	int OpenBT();
	bool SendToIcom(const unsigned char *buf);
	REPLY_TYPE GetBTData(unsigned char *buf);

	// Utilities
	void calcPFCS(const unsigned char *packet, unsigned char *pfcs);
	void DumpPacket(const char *title, const unsigned char *buf);

	// Configuration
	bool ReadConfig(const std::string &path);

	// Config data
	char RPTR_MOD;
	std::string BT_DEVICE, RPTR;
	bool LOG_QSO, LOG_DEBUG, AP_MODE;

	// Runtime parameters
	int serfd;                    // Serial file descriptor
	unsigned char tapcounter;     // Packet counter

	// Helpers
	CRandom random;               // Stream ID generator

	// Unix sockets for gateway communication
	CUnixDgramWriter ToGate;      // Send to gateway
	CUnixDgramReader FromGate;    // Receive from gateway

	// Outgoing packet queue
	std::queue<CFrame> queue;
	bool acknowledged;            // ACK status for flow control
};

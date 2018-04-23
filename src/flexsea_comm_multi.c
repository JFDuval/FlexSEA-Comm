/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-comm' Communication stack
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] flexsea_comm: Data-Link layer of the FlexSEA protocol
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-09 | jfduval | Initial GPL-3.0 release
	*
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//FlexSEA comm. prototype:
//=======================
//[HEADER][# of BYTES][DATA...][CHECKSUM][FOOTER]
//=> Number of bytes includes the ESCAPE bytes
//=> Checksum is done on the payload (data + ESCAPEs) and on the BYTES byte.

//To transmit a message:
//======================
// 1) Place the payload in an array (no header, no footer: pure data)
// 2) Call comm_gen_str(your_data_array, number_of_bytes)
// 2b) It will return the index of the last byte of the message (add 1 for the length)
// 2c) The message is in comm_str[]
// 3) Send comm_str[] (x_puts(comm_str, msg_length));

//To receive a message:
//=====================
// 1) Assuming that you have dealt with all the previous messages, call comm_str_payload();
//    to fill the buffer with zeros
// 2) Every time you receive a byte update the buffer: comm_update_rx_buffer(your_new_byte);
// 3) Call payload_str_available_in_buffer = comm_decode_str(). If you get >= 1, read the
//    comm_str_payload buffer and do something with the data!
// 4) At this point you might want to flush the read payload from rx_buf

//****************************************************************************
// Include(s)
//****************************************************************************

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "flexsea_comm_multi.h"
#include "flexsea_payload.h"
#include "flexsea.h"
//****************************************************************************
// Variable(s)
//****************************************************************************

MultiCommPeriph usbMultiPeriph;

//****************************************************************************
// Private Function Prototypes(s)
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialize CommPeriph to defaults:
void initMultiPeriph(MultiCommPeriph *cp, Port port, PortType pt, circularBuffer_t* rx_cb)
{
	cp->port = port;
	cp->portType = pt;
	cp->transState = TS_UNKNOWN;

	cp->bytesReadyFlag = 0;
	cp->unpackedPacketsAvailable = 0;

	circ_buff_init(rx_cb);
	cp->circularBuff = rx_cb;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************


uint16_t unpack_multi_payload_cb(circularBuffer_t *cb, MultiWrapper* p)
{
	int bufSize = circ_buff_get_size(cb);

	int foundString = 0, foundFrame = 0, bytes, possibleFooterPos;
	int lastPossibleHeaderIndex = bufSize - 5;
	int headerPos = -1, lastHeaderPos = -1;
	uint8_t checksum = 0;

	int headers = 0, footers = 0;
	while(!foundString && lastHeaderPos < lastPossibleHeaderIndex)
	{
		headerPos = circ_buff_search(cb, HEADER, lastHeaderPos+1);
		//if we can't find a header, we quit searching for strings
		if(headerPos == -1) break;

		headers++;
		foundFrame = 0;
		if(headerPos <= lastPossibleHeaderIndex)
		{
			bytes = circ_buff_peak(cb, headerPos + 1);
			possibleFooterPos = headerPos + 4 + bytes; //header=1, numbytes=1, multiinfo=1, data=bytes, crc=1, footer=1;
			foundFrame = (possibleFooterPos < bufSize && circ_buff_peak(cb, possibleFooterPos) == FOOTER);
		}

		if(foundFrame)
		{
			footers++;
			//checksum only adds actual data, not any of the frame stuff
			checksum = circ_buff_checksum(cb, headerPos+3, possibleFooterPos-1);

			//if checksum is valid than we found a valid string
			foundString = (checksum == circ_buff_peak(cb, possibleFooterPos-1));
		}

		//either we found a frame and it had a valid checksum, or we want to try the next value of i
		lastHeaderPos = headerPos;
	}

	int numBytesInPackedString = 0;
	if(foundString)
	{
		numBytesInPackedString = headerPos + bytes + 5;
		uint8_t multiInfo = circ_buff_peak(cb, headerPos + 2);
		uint8_t packetId = (multiInfo >> 6) & 0x03;
		uint8_t frameId  = (multiInfo >> 3) & 0x07;
		uint8_t lastFrameInPacket  = multiInfo & 0x07;

		//why are we recording the packed data? just to have a linear buffer for the next part?
		circ_buff_read_section(cb, p->packed[frameId], headerPos, bytes + 5);

		//if we just received the first frame of a new packet, we can throw out all the old info,
		//the previous multi packet was either completed and parsed, or is incomplete and useless anyways
		if(packetId != p->currentMultiPacket && frameId == 0)
			resetToPacketId(p, packetId);

		//this should always be true except in some strange case with out of order frames
		if(packetId == p->currentMultiPacket)
		{
			//Note that in this implementation we parse each frame as we receive it and we require them to be received in order
			//Alternatively, since we store each frame anyways, we could wait to parse until we have received all of them
			//Then, we'd be able to request a frame that was missed, or gracefully deal with a frame out of order fault

			int k, lastValueWasEscape = 0;
			for(k = 0; k < bytes; k++)
			{
				int index = k+3; //zeroth value is header, first value is numbytes, second value is multiInfo, thrid value is actual data
				if(p->packed[frameId][index] == ESCAPE && (!lastValueWasEscape))
				{
					lastValueWasEscape = 1;
				}
				else
				{
					lastValueWasEscape = 0;
					p->unpacked[p->unpackedIdx++] = p->packed[frameId][index];
				}
			}

			//set the multi's map to record we received this frame
			p->frameMap |= (1 << frameId);

			if(frameId == lastFrameInPacket)
				p->isMultiComplete = 1;
		}
	}

	return numBytesInPackedString;
}

uint8_t tryParse(MultiCommPeriph *cp) {
	if(!(cp->bytesReadyFlag > 0)) return 0;

	cp->bytesReadyFlag--;	// = 0;
	uint8_t error = 0;

	uint16_t numBytesConverted = \
			unpack_multi_payload_cb(cp->circularBuff, &cp->in);

	if(numBytesConverted > 0)
		error = circ_buff_move_head(cp->circularBuff, numBytesConverted);

	return numBytesConverted > 0 && !error;
}

void setMsgInfo(uint8_t* outbuf, uint8_t xid, uint8_t rid, uint8_t cmdcode, uint8_t cmdtype) {
	outbuf[P_XID] = xid;
	outbuf[P_RID] = xid;

	outbuf[P_CMD] = 0;
	outbuf[P_CMD] |= (cmdcode << 1);
	if(cmdtype == RX_PTYPE_READ)
		outbuf[P_CMD] |= 0x01;

	//TODO: include an actual timestamp
	outbuf[P_TIMESTAMP] = 0;
}

//Takes the payload, in p->unpacked, adds ESCAPES, checksum, header, ...
//Adds escapes, checksums, etc, and breaks it up into packets
//returns 1 on error, 0 on success
#define BYTE_NEEDS_ESCAPE(x) (((x) == HEADER) || ((x) == FOOTER) || ((x) == ESCAPE))
uint8_t packMultiPacket(MultiWrapper* p) {

	const uint16_t OVERHEAD = 5;
	//space per frame that we can fit the underlying unpacked string into
	const uint16_t SPACE = PACKET_WRAPPER_LEN - OVERHEAD;
	const uint16_t DATAOFFSET = 3;

	uint16_t i=0, j;
	uint8_t frameId = 0;

	while(i < p->unpackedIdx && frameId < MAX_FRAMES_PER_MULTI_PACKET)
	{
			uint8_t *frame = p->packed[frameId];
			j=0;
			frame[0] = HEADER;							// set the frame's header
			while(j < (SPACE-1) && i < p->unpackedIdx)		// fill in the data
			{
				if (BYTE_NEEDS_ESCAPE(p->unpacked[i]))
				{
					frame[DATAOFFSET+(j++)] = ESCAPE;
				}
				frame[DATAOFFSET+(j++)] = p->unpacked[i++];
			}

			//if the next byte doesn't need an escape, then we can add it
			if(j < SPACE && i < p->unpackedIdx && !BYTE_NEEDS_ESCAPE(p->unpacked[i]))
				frame[DATAOFFSET+(j++)] = p->unpacked[i++];

			frame[1] = j;								// set the frame's num bytes

			//checksum only adds actual data, not any of the frame stuff
			uint8_t crcsum = 0;
			for(j=0; j < frame[1]; j++)
				crcsum += frame[DATAOFFSET+j];
			uint8_t crcpos = DATAOFFSET + frame[1];
			frame[crcpos] = crcsum;						// set the crc
			frame[crcpos+1] = FOOTER;					// set the footer

			//if there's still data to pack then we must move onto the next frame
			if(i < p->unpackedIdx)
				frameId++;
	}
	//check if it all fit in our multi packet
	if(frameId >= MAX_FRAMES_PER_MULTI_PACKET)
		//if it did not all fit we return an error
		return 1;

	// if it did all fit we just need to fill the multiInfo byte now that we know how many frames we have
	// frameId now holds the id of the last frame in the packet
	uint8_t lastFrameIdInPacket = frameId;
	uint8_t multiInfoPos = 2;

	p->frameMap = 0;									// set the multiInfo
	for(frameId=0; frameId <= lastFrameIdInPacket; frameId++)
	{
		p->packed[frameId][multiInfoPos] = ((p->currentMultiPacket << 6) | (frameId << 3) | lastFrameIdInPacket);
		p->frameMap |= (1 << frameId);
	}

	//set isMultiComplete low, meaning that the sending of the packet is not complete
	p->isMultiComplete = 0;
	return 0;
}

//TODO: make this be able to send up stream / down stream?
// Just note that this only works if this device is communicating with plan.
uint8_t parseReadyMultiString(MultiCommPeriph* cp)
{
	// ensure multi is actually ready to be parsed
	if(!cp->in.isMultiComplete) return PARSE_DEFAULT;

	// set flag low to avoid double parsing
	cp->in.isMultiComplete = 0;

	uint8_t *cp_str = cp->in.unpacked;

	uint8_t cmd = 0, cmd_7bits = 0;
	unsigned int id = 0;
	uint8_t pType = RX_PTYPE_INVALID;

	//Command
	cmd = cp_str[P_CMD];		//CMD w/ R/W bit
	cmd_7bits = CMD_7BITS(cmd);	//CMD code, no R/W information

	id = get_rid(cp_str);

	//First, get RID code
	if(id == ID_MATCH)
	{
		uint8_t info[2] = {0,0};
		info[0] = (uint8_t)cp->port;

		cp->in.destinationPort = PORT_NONE;	//We are home
		pType = packetType(cp_str);

		//It's addressed to me. Function pointer array will call
		//the appropriate handler (as defined in flexsea_system):
		if((cmd_7bits <= MAX_CMD_CODE) && (pType <= RX_PTYPE_MAX_INDEX))
		{
			// NOTE: in a response, we need to reserve bytes for XID, RID, CMD, and TIMESTAMP
			const uint8_t RESERVEDBYTES = 4;

			// initialize the response length to 0
			// Our index is the length of response.
			cp->out.unpackedIdx = 0;

			//Call handler:
			(*flexsea_multipayload_ptr[cmd_7bits][pType]) (cp->in.unpacked, info, cp->out.unpacked + RESERVEDBYTES, &cp->out.unpackedIdx);

			uint8_t error = 0;

			//If there is a response we need to route it or w/e
			if(cp->out.unpackedIdx) {
				setMsgInfo(cp->out.unpacked, cp_str[P_RID], cp_str[P_XID], cmd_7bits, RX_PTYPE_REPLY);

				// adjust the index, as this now represents the length including reserved bytes
				cp->out.unpackedIdx += RESERVEDBYTES;

				// set multipacket id's to match
				cp->out.currentMultiPacket = cp->in.currentMultiPacket;

				error = packMultiPacket(&cp->out);
			}

			cp->in.frameMap = 0;
			if(!error)
				return PARSE_SUCCESSFUL;
			else
				return PARSE_DEFAULT;
		}
		else
		{	//if the cmd bits or pType are invalid return error
			return PARSE_DEFAULT;
		}
	}
	// else give up
	return PARSE_DEFAULT;
}

void resetToPacketId(MultiWrapper* p, uint8_t id)
{
	p->currentMultiPacket = id;
	p->unpackedIdx = 0;
	p->frameMap = 0;
	memset(p->unpacked, 0, UNPACKED_BUFF_SIZE);
}

#ifdef __cplusplus
}
#endif

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
#include "flexsea_sys_def.h"
#include "flexsea_comm_multi.h"
#include "flexsea_payload.h"
#include "flexsea.h"
#include "flexsea_device_spec.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

MultiCommPeriph comm_multi_periph[NUMBER_OF_PORTS];

//****************************************************************************
// Private Function Prototypes(s)
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

void initMultiWrapper(MultiWrapper *w)
{
	int i;
	for(i=0;i<MAX_FRAMES_PER_MULTI_PACKET;i++)
		memset(w->packed[i], 0, PACKET_WRAPPER_LEN);

	memset(w->unpacked, 0, UNPACKED_BUFF_SIZE);
	w->unpackedIdx = 0;
}

//Initialize CommPeriph to defaults:
void initMultiPeriph(MultiCommPeriph *cp, Port port, PortType pt)
{
	cp->port = port;
	cp->portType = pt;
	cp->transState = TS_UNKNOWN;

	cp->bytesReadyFlag = 0;
	cp->unpackedPacketsAvailable = 0;
	cp->parsingCachedIndex = 0;

	initMultiWrapper(&(cp->in));
	initMultiWrapper(&(cp->out));

	circ_buff_init(&cp->circularBuff);
}

//****************************************************************************
// Private Function(s)
//****************************************************************************


uint16_t unpack_multi_payload_cb_cached(circularBuffer_t *cb, MultiWrapper* p, int *cacheStart)
{
	int bufSize = circ_buff_get_size(cb);

	if(*cacheStart > bufSize)
	{
		*cacheStart = 0;
	}

	if(*cacheStart == bufSize)
	{
		return 0;
	}

	int foundString = 0, foundFrame = 0, bytes, possibleFooterPos;
	int lastPossibleHeaderIndex = bufSize - MULTI_NUM_OVERHEAD_BYTES_FRAME;

	int headerPos = (*cacheStart)-1;
	int lastHeaderPos = headerPos;

	uint8_t checksum = 0;

	int headers = 0, footers = 0;
	while(!foundString && lastHeaderPos < lastPossibleHeaderIndex)
	{
		headerPos = circ_buff_search(cb, MULTI_SOF, lastHeaderPos+1);
		//if we can't find a header, we quit searching for strings
		if(headerPos == -1) break;

		headers++;
		foundFrame = 0;
		if(headerPos <= lastPossibleHeaderIndex)
		{
			bytes = circ_buff_peak(cb, headerPos + 1);
			possibleFooterPos = MULTI_EOF_POS_FROM_SOF(headerPos, bytes);
			foundFrame = (possibleFooterPos < bufSize && circ_buff_peak(cb, possibleFooterPos) == MULTI_EOF);
		}

		if(foundFrame)
		{
			footers++;
			//checksum only adds actual data, not any of the frame stuff
			checksum = circ_buff_checksum(cb, MULTI_DATA_POS_FROM_SOF(headerPos) , possibleFooterPos-1);

			//if checksum is valid than we found a valid string
			foundString = (checksum == circ_buff_peak(cb, possibleFooterPos-1));
		}

		//either we found a frame and it had a valid checksum, or we want to try the next value of i
		lastHeaderPos = headerPos;
	}

	int numBytesInPackedString = 0;
	if(foundString)
	{
		numBytesInPackedString = headerPos + bytes + MULTI_NUM_OVERHEAD_BYTES_FRAME;
		uint8_t multiInfo = circ_buff_peak(cb, headerPos + 2);
		uint8_t packetId = MULTI_PACKETID(multiInfo);
		uint8_t frameId  = MULTI_THIS_FRAMEID(multiInfo);
		uint8_t lastFrameInPacket  = MULTI_LAST_FRAMEID(multiInfo);

		//why are we recording the packed data? just to have a linear buffer for the next part?
		circ_buff_read_section(cb, p->packed[frameId], headerPos, bytes + MULTI_NUM_OVERHEAD_BYTES_FRAME);

		//if we just received the first frame of a new packet, we can throw out all the old info,
		//the previous multi packet was either completed and parsed, or is incomplete and useless anyways
		if(frameId == 0 && ((packetId != p->currentMultiPacket)
//				|| (p->frameMap & 0x01)
				))
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
				int index = k+3; //zeroth value is header, first value is numbytes, second value is multiInfo, third value is actual data
				if(p->packed[frameId][index] == MULTI_ESC && (!lastValueWasEscape))
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

	// update the cached header value
	if(foundString)
	{
		*cacheStart = numBytesInPackedString;
	}
	else
	{
		*cacheStart = lastHeaderPos;
		if(lastHeaderPos < bufSize-1)
		{
			bytes = circ_buff_peak(cb, headerPos + 1);
			possibleFooterPos = MULTI_EOF_POS_FROM_SOF(headerPos, bytes);
			if(possibleFooterPos < bufSize)
				*cacheStart = bufSize;
		}
	}

	return numBytesInPackedString;
}

uint16_t unpack_multi_payload_cb(circularBuffer_t *cb, MultiWrapper* p)
{
	int bufSize = circ_buff_get_size(cb);

	int foundString = 0, foundFrame = 0, bytes, possibleFooterPos;
	int lastPossibleHeaderIndex = bufSize - MULTI_NUM_OVERHEAD_BYTES_FRAME;
	int headerPos = -1, lastHeaderPos = -1;
	uint8_t checksum = 0;

	int headers = 0, footers = 0;
	while(!foundString && lastHeaderPos < lastPossibleHeaderIndex)
	{
		headerPos = circ_buff_search(cb, MULTI_SOF, lastHeaderPos+1);
		//if we can't find a header, we quit searching for strings
		if(headerPos == -1) break;

		headers++;
		foundFrame = 0;
		if(headerPos <= lastPossibleHeaderIndex)
		{
			bytes = circ_buff_peak(cb, headerPos + 1);
			possibleFooterPos = MULTI_EOF_POS_FROM_SOF(headerPos, bytes);
			foundFrame = (possibleFooterPos < bufSize && circ_buff_peak(cb, possibleFooterPos) == MULTI_EOF);
		}

		if(foundFrame)
		{
			footers++;
			//checksum only adds actual data, not any of the frame stuff
			checksum = circ_buff_checksum(cb, MULTI_DATA_POS_FROM_SOF(headerPos) , possibleFooterPos-1);

			//if checksum is valid than we found a valid string
			foundString = (checksum == circ_buff_peak(cb, possibleFooterPos-1));
		}

		//either we found a frame and it had a valid checksum, or we want to try the next value of i
		lastHeaderPos = headerPos;
	}

	int numBytesInPackedString = 0;
	if(foundString)
	{
		numBytesInPackedString = headerPos + bytes + MULTI_NUM_OVERHEAD_BYTES_FRAME;
		uint8_t multiInfo = circ_buff_peak(cb, headerPos + 2);
		uint8_t packetId = MULTI_PACKETID(multiInfo);
		uint8_t frameId  = MULTI_THIS_FRAMEID(multiInfo);
		uint8_t lastFrameInPacket  = MULTI_LAST_FRAMEID(multiInfo);

		//why are we recording the packed data? just to have a linear buffer for the next part?
		circ_buff_read_section(cb, p->packed[frameId], headerPos, bytes + MULTI_NUM_OVERHEAD_BYTES_FRAME);

		//if we just received the first frame of a new packet, we can throw out all the old info,
		//the previous multi packet was either completed and parsed, or is incomplete and useless anyways
		if(frameId == 0 && ((packetId != p->currentMultiPacket)
//				|| (p->frameMap & 0x01)
				))
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
				int index = k+3; //zeroth value is header, first value is numbytes, second value is multiInfo, third value is actual data
				if(p->packed[frameId][index] == MULTI_ESC && (!lastValueWasEscape))
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
	if(!(cp->bytesReadyFlag > 0)) return 1;

	cp->bytesReadyFlag--;	// = 0;
	uint8_t error = 0;

	uint16_t numBytesConverted = \
			unpack_multi_payload_cb(&cp->circularBuff, &cp->in);

	if(numBytesConverted > 0)
		error = circ_buff_move_head(&cp->circularBuff, numBytesConverted);

	uint8_t retCode = 0;
	retCode |= (error ? 2 : 0);
	retCode |= (!numBytesConverted ? 4 : 0);

	return retCode;
}

void setMsgInfo(uint8_t* outbuf, uint8_t xid, uint8_t rid, uint8_t cmdcode, uint8_t cmdtype, uint32_t timestamp) {
	outbuf[MP_XID] = xid;
	outbuf[MP_RID] = rid;

	memcpy(outbuf + MP_TSTP, &timestamp, sizeof(uint32_t));

	outbuf[MP_CMD1] = (cmdcode << 1);
	if(cmdtype == RX_PTYPE_READ)
		outbuf[MP_CMD1] |= 0x01;
}

//Takes the payload, in p->unpacked, adds ESCAPES, checksum, header, ...
//Adds escapes, checksums, etc, and breaks it up into packets
//returns 1 on error, 0 on success
#define BYTE_NEEDS_ESCAPE(x) (((x) == MULTI_SOF) || ((x) == MULTI_EOF) || ((x) == MULTI_ESC))
uint8_t packMultiPacket(MultiWrapper* p) {

	//space per frame that we can fit the underlying unpacked string into
	const uint16_t SPACE = PACKET_WRAPPER_LEN - MULTI_NUM_OVERHEAD_BYTES_FRAME;

	uint16_t i=0, j;
	uint8_t frameId = 0;

	while(i < p->unpackedIdx && frameId < MAX_FRAMES_PER_MULTI_PACKET)
	{
			uint8_t *frame = p->packed[frameId];
			j=0;
			frame[0] = MULTI_SOF;							// set the start of frame byte
			while(j < (SPACE-1) && i < p->unpackedIdx)		// fill in the data
			{
				if (BYTE_NEEDS_ESCAPE(p->unpacked[i]))
				{
					frame[MULTI_DATA_OFFSET+(j++)] = MULTI_ESC;
				}
				frame[MULTI_DATA_OFFSET+(j++)] = p->unpacked[i++];
			}

			//if the next byte doesn't need an escape, then we can add it
			if(j < SPACE && i < p->unpackedIdx && !BYTE_NEEDS_ESCAPE(p->unpacked[i]))
				frame[MULTI_DATA_OFFSET+(j++)] = p->unpacked[i++];

			frame[1] = j;								// set the frame's num bytes

			//checksum only adds actual data, not any of the frame stuff
			uint8_t checksum = 0;
			for(j=0; j < frame[1]; j++)
				checksum += frame[MULTI_DATA_OFFSET+j];

			uint8_t checksumPos = MULTI_CHECKSUM_POS_FROM_SOF(0, frame[1]);
			frame[checksumPos] = checksum;						// set the checksum
			frame[checksumPos+1] = MULTI_EOF;					// set the end of frame byte

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
	uint8_t multiInfoPos = MULTI_INFO_POS_FROM_SOF(0);

	p->frameMap = 0;									// set the multiInfo
	for(frameId=0; frameId <= lastFrameIdInPacket; frameId++)
	{
		p->packed[frameId][multiInfoPos] = MULTI_GENINFO(p->currentMultiPacket, frameId, lastFrameIdInPacket);
		p->frameMap |= (1 << frameId);
	}

	//set isMultiComplete low, meaning that the sending of the packet is not complete
	p->isMultiComplete = 0;
	return 0;
}

uint8_t receiveAndPackResponse(uint8_t cmd_7bits, uint8_t pType, MultiPacketInfo* info, MultiCommPeriph* cp)
{
	// initialize the response length to 0
	// Our index is the length of response.
	cp->out.unpackedIdx = 0;

	//Call handler, 							NOTE: in a response, we need to reserve bytes for XID, RID, CMD, and TIMESTAMP
	(*flexsea_multipayload_ptr[cmd_7bits][pType])(cp->in.unpacked + MP_DATA1, info, cp->out.unpacked + MP_DATA1, &cp->out.unpackedIdx);

	uint8_t error = 0;
	//If there is a response we need to route it or w/e
	if(cp->out.unpackedIdx + MULTI_PACKET_OVERHEAD >= UNPACKED_BUFF_SIZE) {
		error = 1; // raise an error flag
	}	else if (cp->out.unpackedIdx) {
		//TODO: fill with an actual cross device timestamp
		setMsgInfo(cp->out.unpacked, info->rid, info->xid, cmd_7bits, RX_PTYPE_REPLY, *fx_dev_timestamp);
		// adjust the index, as this now represents the length including reserved bytes
		cp->out.unpackedIdx += MULTI_PACKET_OVERHEAD;

		// set multipacket id's to match
		cp->out.currentMultiPacket = cp->in.currentMultiPacket;
		error = packMultiPacket(&cp->out);
	}
	cp->in.frameMap = 0;
	return error;
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
	cmd = cp_str[MP_CMD1];		//CMD w/ R/W bit
	cmd_7bits = CMD_7BITS(cmd);	//CMD code, no R/W information

	MultiPacketInfo info;
	info.portIn = cp->port;
	info.xid = cp_str[MP_XID];
	info.rid = cp_str[MP_RID];

	//First, get RID code
	id = get_rid(cp_str);
	if(id == ID_MATCH)
	{
		cp->in.destinationPort = PORT_NONE;	//We are home

		//TODO: figure out how to determine if message is actually from slave
#ifdef BOARD_TYPE_FLEXSEA_PLAN
		pType = RX_PTYPE_REPLY;
#else
		pType = (cp_str[MP_CMD1] & 0x01) ? RX_PTYPE_READ : RX_PTYPE_WRITE;
#endif
		//It's addressed to me. Function pointer array will call
		//the appropriate handler (as defined in flexsea_system):
		if((cmd_7bits <= MAX_CMD_CODE) && (pType <= RX_PTYPE_MAX_INDEX))
		{
			uint8_t error = receiveAndPackResponse(cmd_7bits, pType, &info, cp);
			if(error)
				return PARSE_DEFAULT;
		}
	}
	else if(cp->in.unpacked[MP_RID] == 0 && cmd_7bits == CMD_SYSDATA)
	{
		cp->in.unpacked[MP_DATA1] = SYSDATA_WHO_AM_I_FLAG; // results in whoami msg
		uint8_t error = receiveAndPackResponse(cmd_7bits, RX_PTYPE_READ, &info, cp);
		if(error)
			return PARSE_DEFAULT;
	}
	// else give up
	return PARSE_SUCCESSFUL;
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

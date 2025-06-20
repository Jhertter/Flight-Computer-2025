#include "sam_m8q.h"

SFE_UBLOX_GPS::SFE_UBLOX_GPS(void)
{
    // Constructor
    currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use
    moduleQueried.versionNumber = false;
}

// Initialize the Serial port
bool SFE_UBLOX_GPS::begin(i2c_inst_t &i2c, uint8_t deviceAddress)
{
    commType = COMM_TYPE_I2C;
    _i2cPort = &i2c; // Grab which port the user wants us to use

    // We expect caller to begin their I2C port, with the speed of their choice external to the library
    // But if they forget, we start the hardware here.

    // We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
    // ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
    // hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
    //_i2cPort->begin();

    _gpsI2Caddress = deviceAddress; // Store the I2C address from user

    // Attempt isConnected up to 3 times if required
    bool success = isConnected();

    if (!success)
        success = isConnected();

    if (!success)
        success = isConnected();

    return (success);
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase the transaction
// size if the platform supports it
// Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
void SFE_UBLOX_GPS::setI2CTransactionSize(uint8_t transactionSize)
{
    i2cTransactionSize = transactionSize;
}
uint8_t SFE_UBLOX_GPS::getI2CTransactionSize(void)
{
    return (i2cTransactionSize);
}

const char *SFE_UBLOX_GPS::statusString(sfe_ublox_status_e stat)
{
    switch (stat)
    {
    case SFE_UBLOX_STATUS_SUCCESS:
        return "Success";
        break;
    case SFE_UBLOX_STATUS_FAIL:
        return "General Failure";
        break;
    case SFE_UBLOX_STATUS_CRC_FAIL:
        return "CRC Fail";
        break;
    case SFE_UBLOX_STATUS_TIMEOUT:
        return "Timeout";
        break;
    case SFE_UBLOX_STATUS_COMMAND_NACK:
        return "Command not acknowledged (NACK)";
        break;
    case SFE_UBLOX_STATUS_OUT_OF_RANGE:
        return "Out of range";
        break;
    case SFE_UBLOX_STATUS_INVALID_ARG:
        return "Invalid Arg";
        break;
    case SFE_UBLOX_STATUS_INVALID_OPERATION:
        return "Invalid operation";
        break;
    case SFE_UBLOX_STATUS_MEM_ERR:
        return "Memory Error";
        break;
    case SFE_UBLOX_STATUS_HW_ERR:
        return "Hardware Error";
        break;
    case SFE_UBLOX_STATUS_DATA_SENT:
        return "Data Sent";
        break;
    case SFE_UBLOX_STATUS_DATA_RECEIVED:
        return "Data Received";
        break;
    case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
        return "I2C Comm Failure";
        break;
    case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
        return "Data Packet Overwritten";
        break;
    default:
        return "Unknown Status";
        break;
    }
    return "None";
}

void SFE_UBLOX_GPS::factoryReset()
{
    // Copy default settings to permanent
    // Note: this does not load the permanent configuration into the current configuration. Calling factoryDefault() will do that.
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_CFG;
    packetCfg.len = 13;
    packetCfg.startingSpot = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        payloadCfg[0 + i] = 0xff; // clear mask: copy default config to permanent config
        payloadCfg[4 + i] = 0x00; // save mask: don't save current to permanent
        payloadCfg[8 + i] = 0x00; // load mask: don't copy permanent config to current
    }
    payloadCfg[12] = 0xff;      // all forms of permanent memory
    sendCommand(&packetCfg, 0); // don't expect ACK
    hardReset();                // cause factory default config to actually be loaded and used cleanly
}

void SFE_UBLOX_GPS::hardReset()
{
    // Issue hard reset
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RST;
    packetCfg.len = 4;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = 0xff;       // cold start
    payloadCfg[1] = 0xff;       // cold start
    payloadCfg[2] = 0;          // 0=HW reset
    payloadCfg[3] = 0;          // reserved
    sendCommand(&packetCfg, 0); // don't expect ACK
}

// Changes the I2C address that the u-blox module responds to
// 0x42 is the default but can be changed with this command
bool SFE_UBLOX_GPS::setI2CAddress(uint8_t deviceAddress, uint16_t maxWait)
{
    // Get the current config values for the I2C port
    getPortSettings(COM_PORT_I2C, maxWait); // This will load the payloadCfg array with current port settings

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_PRT;
    packetCfg.len = 20;
    packetCfg.startingSpot = 0;

    // payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg[4] = deviceAddress << 1; // DDC mode LSB

    if (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
    {
        // Success! Now change our internal global.
        _gpsI2Caddress = deviceAddress; // Store the I2C address from user
        return (true);
    }
    return (false);
}

// //Want to see the NMEA messages on the Serial port? Here's how
// void SFE_UBLOX_GPS::setNMEAOutputPort(Stream &nmeaOutputPort)
// {
//   _nmeaOutputPort = &nmeaOutputPort; //Store the port from user
// }

// Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GPS::checkUblox(uint8_t requestedClass, uint8_t requestedID)
{
    return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}

// Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GPS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    if (commType == COMM_TYPE_I2C)
        return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
    return false;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool SFE_UBLOX_GPS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    if (to_ms_since_boot(get_absolute_time()) - lastCheck >= i2cPollingWait)
    {
        // Get the number of bytes available from the module
        uint16_t bytesAvailable = 0;
        uint8_t reg = 0xFD;
        if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, &reg, 1, true) == PICO_ERROR_GENERIC) // Send a restart command. Do not release bus.
            return (false);                                                                    // Sensor did not ACK

        // This checks how many data bytes are available to read
        if (i2c_get_read_available(_i2cPort) < 2)
        {
            uint8_t msb = 0;
            i2c_read_blocking(_i2cPort, _gpsI2Caddress, &msb, 1, true); // Send a restart command. Do not release bus.
            uint8_t lsb = 0;
            i2c_read_blocking(_i2cPort, _gpsI2Caddress, &lsb, 1, true); // Send a restart command. Do not release bus.
            if (lsb == 0xFF)
            {
                if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
                    printf("checkUbloxI2C: u-blox bug, length lsb is 0xFF\n");

                // I believe this is a u-blox bug. Device should never present an 0xFF.
                lastCheck = to_ms_since_boot(get_absolute_time()); // Put off checking to avoid I2C bus traffic
                return (false);
            }
            bytesAvailable = (uint16_t)msb << 8 | lsb;
        }

        if (bytesAvailable == 0)
        {
            if (_printDebug == true)
                printf("checkUbloxI2C: OK, zero bytes available\n");

            lastCheck = to_ms_since_boot(get_absolute_time()); // Put off checking to avoid I2C bus traffic
            return (false);
        }

        // Check for undocumented bit error. We found this doing logic scans.
        // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
        // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
        if (bytesAvailable & ((uint16_t)1 << 15))
        {
            // Clear the MSbit
            bytesAvailable &= ~((uint16_t)1 << 15);
            if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
                printf("checkUbloxI2C: Bytes available error: %d\n", bytesAvailable);
        }

        if (bytesAvailable > 100)
        {
            if (_printDebug == true)
                printf("checkUbloxI2C: Large packet of %d bytes received\n", bytesAvailable);
        }
        else
        {
            if (_printDebug == true)
                printf("checkUbloxI2C: Reading %d bytes\n", bytesAvailable);
        }

        while (bytesAvailable)
        {
            // reg = 0xFF;
            // if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, &reg, 1, true) == PICO_ERROR_GENERIC) // Send a restart command. Do not release bus.
            // {
            //     printf("linea 247: PICO_ERROR_GENERIC\n");
            //     return (false);   // Sensor did not ACK
            // }

            // Limit to 32 bytes or whatever the buffer limit is for given platform
            uint16_t bytesToRead = bytesAvailable;
            if (bytesToRead > i2cTransactionSize)
                bytesToRead = i2cTransactionSize;

            // TRY_AGAIN:

            // if (i2c_get_read_available(_i2cPort))
            // {
            // printf("checkUbloxI2C: i2c_get_read_available\n");
            for (uint16_t x = 0; x < bytesToRead; x++)
            {
                uint8_t incoming = 0;
                i2c_read_blocking(_i2cPort, _gpsI2Caddress, &incoming, 1, true); // Send a restart command. Do not release bus.

                // Check to see if the first read is 0x7F. If it is, the module is not ready
                // to respond. Stop, wait, and try again
                // if (x == 0)
                // {
                //     if (incoming == 0x7F)
                //     {
                //         if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
                //             printf("checkUbloxU2C: u-blox error, module not ready with data");

                //         sleep_ms(5); // In logic analyzation, the module starting responding after 1.48ms
                //         goto TRY_AGAIN;
                //     }
                // }

                process(incoming, incomingUBX, requestedClass, requestedID); // Process this valid character
            }
            // }
            // else
            // {
            //     if (_printDebug == true)
            //         printf("checkUbloxI2C: No bytes available to read from u-blox\n");
            //     return (false); // Sensor did not respond
            // }

            bytesAvailable -= bytesToRead;
        }
    }

    return (true);

} // end checkUbloxI2C()

// Processes NMEA and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void SFE_UBLOX_GPS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    if ((currentSentence == NONE) || (currentSentence == NMEA))
    {
        if (incoming == 0xB5) // UBX binary frames start with 0xB5, aka μ
        {
            // This is the start of a binary sentence. Reset flags.
            // We still don't know the response class
            ubxFrameCounter = 0;
            currentSentence = UBX;
            // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
            packetBuf.counter = 0;
            ignoreThisPayload = false; // We should not ignore this payload - yet
            // Store data in packetBuf until we know if we have a requested class and ID match
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
        }
        else if (incoming == '$')
        {
            currentSentence = NMEA;
        }
        else if (incoming == 0xD3) // RTCM frames start with 0xD3
        {
            rtcmFrameCounter = 0;
            currentSentence = RTCM;
        }
        else
        {
            // This character is unknown or we missed the previous start of a sentence
        }
    }

    // Depending on the sentence, pass the character to the individual processor
    if (currentSentence == UBX)
    {
        printf("process: UBX\n");
        // Decide what type of response this is
        if ((ubxFrameCounter == 0) && (incoming != 0xB5))      // ISO 'μ'
            currentSentence = NONE;                            // Something went wrong. Reset.
        else if ((ubxFrameCounter == 1) && (incoming != 0x62)) // ASCII 'b'
            currentSentence = NONE;                            // Something went wrong. Reset.
        // Note to future self:
        // There may be some duplication / redundancy in the next few lines as processUBX will also
        // load information into packetBuf, but we'll do it here too for clarity
        else if (ubxFrameCounter == 2) // Class
        {
            // Record the class in packetBuf until we know what to do with it
            packetBuf.cls = incoming; // (Duplication)
            rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
            rollingChecksumB = 0;
            packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
            packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
            packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
        }
        else if (ubxFrameCounter == 3) // ID
        {
            // Record the ID in packetBuf until we know what to do with it
            packetBuf.id = incoming; // (Duplication)
            // We can now identify the type of response
            // If the packet we are receiving is not an ACK then check for a class and ID match
            if (packetBuf.cls != UBX_CLASS_ACK)
            {
                // This is not an ACK so check for a class and ID match
                if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
                {
                    // This is not an ACK and we have a class and ID match
                    // So start diverting data into incomingUBX (usually packetCfg)
                    activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
                    incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
                    incomingUBX->id = packetBuf.id;
                    incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
                }
                // This is not an ACK and we do not have a complete class and ID match
                // So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
                else if ((packetBuf.cls == requestedClass) &&
                         (((packetBuf.id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH || requestedID == UBX_NAV_DOP)) ||
                          ((packetBuf.id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_DOP)) ||
                          ((packetBuf.id == UBX_NAV_DOP) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_HPPOSLLH))))
                {
                    // This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
                    activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
                    incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
                    incomingUBX->id = packetBuf.id;
                    incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
                    if (_printDebug == true)
                    {
                        printf("process: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x%0x, Message ID: 0x%0x\n", requestedID, packetBuf.id);
                    }
                }
                else if ((packetBuf.cls == requestedClass) &&
                         (((packetBuf.id == UBX_HNR_ATT) && (requestedID == UBX_HNR_INS || requestedID == UBX_HNR_PVT)) ||
                          ((packetBuf.id == UBX_HNR_INS) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_PVT)) ||
                          ((packetBuf.id == UBX_HNR_PVT) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_INS))))
                {
                    // This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
                    activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
                    incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
                    incomingUBX->id = packetBuf.id;
                    incomingUBX->counter = packetBuf.counter; // Copy over the .counter too

                    if (_printDebug == true)
                    {
                        printf("process: auto HNR ATT/INS/PVT collision: Requested ID: 0x%0x, Message ID: 0x%0x\n", requestedID, packetBuf.id);
                    }
                }
                else
                {
                    // This is not an ACK and we do not have a class and ID match
                    // so we should keep diverting data into packetBuf and ignore the payload
                    ignoreThisPayload = true;
                }
            }
            else
            {
                // This is an ACK so it is to early to do anything with it
                // We need to wait until we have received the length and data bytes
                // So we should keep diverting data into packetBuf
            }
        }
        else if (ubxFrameCounter == 4) // Length LSB
        {
            // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
            packetBuf.len = incoming; // (Duplication)
        }
        else if (ubxFrameCounter == 5) // Length MSB
        {
            // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
            packetBuf.len |= incoming << 8; // (Duplication)
        }
        else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
        {
            if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
            {
                if (_printDebug == true)
                {
                    printf("process: ZERO LENGTH packet received: Class: 0x%0x, ID: 0x%0x\n", packetBuf.cls, packetBuf.id);
                }
                // If length is zero (!) this will be the first byte of the checksum so record it
                packetBuf.checksumA = incoming;
            }
            else
            {
                // The length is not zero so record this byte in the payload
                packetBuf.payload[0] = incoming;
            }
        }
        else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
        {
            if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
            {
                // If length is zero (!) this will be the second byte of the checksum so record it
                packetBuf.checksumB = incoming;
            }
            else if (packetBuf.len == 1) // Check if length is one
            {
                // The length is one so this is the first byte of the checksum
                packetBuf.checksumA = incoming;
            }
            else // Length is >= 2 so this must be a payload byte
            {
                packetBuf.payload[1] = incoming;
            }
            // Now that we have received two payload bytes, we can check for a matching ACK/NACK
            if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
                && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
                && (packetBuf.payload[0] == requestedClass)        // and if the class matches
                && (packetBuf.payload[1] == requestedID))          // and if the ID matches
            {
                if (packetBuf.len == 2) // Check if .len is 2
                {
                    // Then this is a matching ACK so copy it into packetAck
                    activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
                    packetAck.cls = packetBuf.cls;
                    packetAck.id = packetBuf.id;
                    packetAck.len = packetBuf.len;
                    packetAck.counter = packetBuf.counter;
                    packetAck.payload[0] = packetBuf.payload[0];
                    packetAck.payload[1] = packetBuf.payload[1];
                }
                else
                {
                    if (_printDebug == true)
                    {
                        printf("process: ACK received with .len != 2: Class: 0x%0x, ID: 0x%0x, len: %d\n", packetBuf.cls, packetBuf.id, packetBuf.len);
                    }
                }
            }
        }

        // Divert incoming into the correct buffer
        if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
            processUBX(incoming, &packetAck, requestedClass, requestedID);
        else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
            processUBX(incoming, incomingUBX, requestedClass, requestedID);
        else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
            processUBX(incoming, &packetBuf, requestedClass, requestedID);

        // Finally, increment the frame counter
        ubxFrameCounter++;
    }
    else if (currentSentence == NMEA)
    {
        printf("process: NMEA\n");
        processNMEA(incoming); // Process each NMEA character
    }
    else if (currentSentence == RTCM)
    {
        printf("process: RTCM\n");
        processRTCMframe(incoming); // Deal with RTCM bytes
    }
}

// We need to be able to identify an RTCM packet and then the length
// so that we know when the RTCM message is completely received and we then start
// listening for other sentences (like NMEA or UBX)
// RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
// http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
// https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
// Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
// RTCM 3.2 bytes look like this:
// Byte 0: Always 0xD3
// Byte 1: 6-bits of zero
// Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
// byte 3 + 4 bits: Msg type 12 bits
// Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
void SFE_UBLOX_GPS::processRTCMframe(uint8_t incoming)
{
    if (rtcmFrameCounter == 1)
    {
        rtcmLen = (incoming & 0x03) << 8; // Get the last two bits of this byte. Bits 8&9 of 10-bit length
    }
    else if (rtcmFrameCounter == 2)
    {
        rtcmLen |= incoming; // Bits 0-7 of packet length
        rtcmLen += 6;        // There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
    }
    /*else if (rtcmFrameCounter == 3)
    {
      rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
    }
    else if (rtcmFrameCounter == 4)
    {
      rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
    }*/

    rtcmFrameCounter++;

    processRTCM(incoming); // Here is where we expose this byte to the user

    if (rtcmFrameCounter == rtcmLen)
    {
        // We're done!
        currentSentence = NONE; // Reset and start looking for next sentence type
    }
}

// This function is called for each byte of an RTCM frame
// Ths user can overwrite this function and process the RTCM frame as they please
// Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
    // Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

    //_debugSerial->write(incoming); //An example of passing this byte out the serial port

    // Debug printing
    //   _debugSerial->print(F(" "));
    //   if(incoming < 0x10) _debugSerial->print(F("0"));
    //   if(incoming < 0x10) _debugSerial->print(F("0"));
    //   _debugSerial->print(incoming, HEX);
    //   if(rtcmFrameCounter % 16 == 0) _debugSerial->println();
}

// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array
// size is MAX_PAYLOAD_SIZE bytes. startingSpot can be set so we only record
// a subset of bytes within a larger packet.
void SFE_UBLOX_GPS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    size_t max_payload_size = (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG) ? MAX_PAYLOAD_SIZE : 2;
    bool overrun = false;

    // Add all incoming bytes to the rolling checksum
    // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
    if (incomingUBX->counter < incomingUBX->len + 4)
        addToChecksum(incoming);

    if (incomingUBX->counter == 0)
    {
        incomingUBX->cls = incoming;
    }
    else if (incomingUBX->counter == 1)
    {
        incomingUBX->id = incoming;
    }
    else if (incomingUBX->counter == 2) // Len LSB
    {
        incomingUBX->len = incoming;
    }
    else if (incomingUBX->counter == 3) // Len MSB
    {
        incomingUBX->len |= incoming << 8;
    }
    else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
    {
        incomingUBX->checksumA = incoming;
    }
    else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
    {
        incomingUBX->checksumB = incoming;

        currentSentence = NONE; // We're done! Reset the sentence to being looking for a new start char

        // Validate this sentence
        if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
        {
            incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

            // Let's check if the class and ID match the requestedClass and requestedID
            // Remember - this could be a data packet or an ACK packet
            if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
            {
                incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
            }

            // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
            else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
            {
                incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
            }

            // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
            else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
            {
                incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
                if (_printDebug == true)
                {
                    printf("processUBX: NACK received: Requested Class: 0x%0x, Requested ID: 0x%0x\n", requestedClass, requestedID);
                }
            }

            // This is not an ACK and we do not have a complete class and ID match
            // So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
            else if ((incomingUBX->cls == requestedClass) &&
                     (((incomingUBX->id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH || requestedID == UBX_NAV_DOP)) ||
                      ((incomingUBX->id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_DOP)) ||
                      ((incomingUBX->id == UBX_NAV_DOP) && (requestedID == UBX_NAV_PVT || requestedID == UBX_NAV_HPPOSLLH))))
            {
                // This isn't the message we are looking for...
                // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
                if (_printDebug == true)
                {
                    printf("processUBX: auto NAV PVT/HPPOSLLH/DOP collision: Requested ID: 0x%0x, Message ID: 0x%0x\n", requestedID, incomingUBX->id);
                }
            }
            // Let's do the same for the HNR messages
            else if ((incomingUBX->cls == requestedClass) &&
                     (((incomingUBX->id == UBX_HNR_ATT) && (requestedID == UBX_HNR_INS || requestedID == UBX_HNR_PVT)) ||
                      ((incomingUBX->id == UBX_HNR_INS) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_PVT)) ||
                      ((incomingUBX->id == UBX_HNR_PVT) && (requestedID == UBX_HNR_ATT || requestedID == UBX_HNR_INS))))
            {
                // This isn't the message we are looking for...
                // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
                if (_printDebug == true)
                {
                    printf("processUBX: auto HNR ATT/INS/PVT collision: Requested ID: 0x%0x, Message ID: 0x%0x\n", requestedID, incomingUBX->id);
                }
            }

            if (_printDebug == true)
            {
                printf("Incoming: Size: %d, Received: ", incomingUBX->len);
                ;
                printPacket(incomingUBX);

                if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
                {
                    printf("packetCfg now valid\n");
                }
                if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
                {
                    printf("packetAck now valid\n");
                }
                if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
                {
                    printf("packetCfg classAndIDmatch\n");
                }
                if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
                {
                    printf("packetAck classAndIDmatch\n");
                }
            }

            // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
            if (ignoreThisPayload == false)
            {
                processUBXpacket(incomingUBX);
            }
        }
        else // Checksum failure
        {
            incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

            // Let's check if the class and ID match the requestedClass and requestedID.
            // This is potentially risky as we are saying that we saw the requested Class and ID
            // but that the packet checksum failed. Potentially it could be the class or ID bytes
            // that caused the checksum error!
            if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
            {
                incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
            }
            // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
            else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
            {
                incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
            }
            if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
            {
                // Drive an external pin to allow for easier logic analyzation
                printf("Checksum failed: checksumA: %d, checksumB: %d ", incomingUBX->checksumA, incomingUBX->checksumB);

                printf(" rollingChecksumA: %d, rollingChecksumB: %d\n", rollingChecksumA, rollingChecksumB);

                printf("Failed : Size: %d, Received: ", incomingUBX->len);
                printPacket(incomingUBX);
            }
        }
    }
    else // Load this byte into the payload array
    {
        // If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
        uint16_t startingSpot = incomingUBX->startingSpot;
        if (incomingUBX->cls == UBX_CLASS_NAV && incomingUBX->id == UBX_NAV_PVT)
            startingSpot = 0;
        // Check if this is payload data which should be ignored
        if (ignoreThisPayload == false)
        {
            // Begin recording if counter goes past startingSpot
            if ((incomingUBX->counter - 4) >= startingSpot)
            {
                // Check to see if we have room for this byte
                if (((incomingUBX->counter - 4) - startingSpot) < max_payload_size) // If counter = 208, starting spot = 200, we're good to record.
                {
                    incomingUBX->payload[incomingUBX->counter - 4 - startingSpot] = incoming; // Store this byte into payload array
                }
                else
                {
                    overrun = true;
                }
            }
        }
    }

    // Increment the counter
    incomingUBX->counter++;

    if (overrun || (incomingUBX->counter == MAX_PAYLOAD_SIZE))
    {
        // Something has gone very wrong
        currentSentence = NONE; // Reset the sentence to being looking for a new start char
    }
    if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
    {
        if (overrun)
            printf("processUBX: buffer overrun detected\n");
        else
            printf("processUBX: counter hit MAX_PAYLOAD_SIZE\n");
    }
}

// Once a packet has been received and validated, identify this packet's class/id and update internal flags
// Note: if the user requests a PVT or a HPPOSLLH message using a custom packet, the data extraction will
//       not work as expected beacuse extractLong etc are hardwired to packetCfg payloadCfg. Ideally
//       extractLong etc should be updated so they receive a pointer to the packet buffer.
void SFE_UBLOX_GPS::processUBXpacket(ubxPacket *msg)
{
    switch (msg->cls)
    {
    case UBX_CLASS_NAV:
        if (msg->id == UBX_NAV_PVT && msg->len == 92)
        {
            // Parse various byte fields into global vars
            constexpr int startingSpot = 0; // fixed value used in processUBX

            timeOfWeek = extractLong(0);
            gpsMillisecond = extractLong(0) % 1000; // Get last three digits of iTOW
            gpsYear = extractInt(4);
            gpsMonth = extractByte(6);
            gpsDay = extractByte(7);
            gpsHour = extractByte(8);
            gpsMinute = extractByte(9);
            gpsSecond = extractByte(10);
            gpsDateValid = extractByte(11) & 0x01;
            gpsTimeValid = (extractByte(11) & 0x02) >> 1;
            gpsNanosecond = extractSignedLong(16); // Includes milliseconds

            fixType = extractByte(20 - startingSpot);
            gnssFixOk = extractByte(21 - startingSpot) & 0x1;           // Get the 1st bit
            diffSoln = (extractByte(21 - startingSpot) >> 1) & 0x1;     // Get the 2nd bit
            carrierSolution = extractByte(21 - startingSpot) >> 6;      // Get 6th&7th bits of this byte
            headVehValid = (extractByte(21 - startingSpot) >> 5) & 0x1; // Get the 5th bit
            SIV = extractByte(23 - startingSpot);
            longitude = extractSignedLong(24 - startingSpot);
            latitude = extractSignedLong(28 - startingSpot);
            altitude = extractSignedLong(32 - startingSpot);
            altitudeMSL = extractSignedLong(36 - startingSpot);
            horizontalAccEst = extractLong(40 - startingSpot);
            verticalAccEst = extractLong(44 - startingSpot);
            nedNorthVel = extractSignedLong(48 - startingSpot);
            nedEastVel = extractSignedLong(52 - startingSpot);
            nedDownVel = extractSignedLong(56 - startingSpot);
            groundSpeed = extractSignedLong(60 - startingSpot);
            headingOfMotion = extractSignedLong(64 - startingSpot);
            speedAccEst = extractLong(68 - startingSpot);
            headingAccEst = extractLong(72 - startingSpot);
            pDOP = extractInt(76 - startingSpot);
            invalidLlh = extractByte(78 - startingSpot) & 0x1;
            headVeh = extractSignedLong(84 - startingSpot);
            magDec = extractSignedInt(88 - startingSpot);
            magAcc = extractInt(90 - startingSpot);

            // Mark all datums as fresh (not read before)
            moduleQueried.gpsiTOW = true;
            moduleQueried.gpsYear = true;
            moduleQueried.gpsMonth = true;
            moduleQueried.gpsDay = true;
            moduleQueried.gpsHour = true;
            moduleQueried.gpsMinute = true;
            moduleQueried.gpsSecond = true;
            moduleQueried.gpsDateValid = true;
            moduleQueried.gpsTimeValid = true;
            moduleQueried.gpsNanosecond = true;

            moduleQueried.all = true;
            moduleQueried.gnssFixOk = true;
            moduleQueried.diffSoln = true;
            moduleQueried.headVehValid = true;
            moduleQueried.longitude = true;
            moduleQueried.latitude = true;
            moduleQueried.altitude = true;
            moduleQueried.altitudeMSL = true;
            moduleQueried.horizontalAccEst = true;
            moduleQueried.verticalAccEst = true;
            moduleQueried.nedNorthVel = true;
            moduleQueried.nedEastVel = true;
            moduleQueried.nedDownVel = true;
            moduleQueried.SIV = true;
            moduleQueried.fixType = true;
            moduleQueried.carrierSolution = true;
            moduleQueried.groundSpeed = true;
            moduleQueried.headingOfMotion = true;
            moduleQueried.speedAccEst = true;
            moduleQueried.headingAccEst = true;
            moduleQueried.pDOP = true;
            moduleQueried.invalidLlh = true;
            moduleQueried.headVeh = true;
            moduleQueried.magDec = true;
            moduleQueried.magAcc = true;
        }
        else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == 36)
        {
            timeOfWeek = extractLong(4);
            highResLongitude = extractSignedLong(8);
            highResLatitude = extractSignedLong(12);
            elipsoid = extractSignedLong(16);
            meanSeaLevel = extractSignedLong(20);
            highResLongitudeHp = extractSignedChar(24);
            highResLatitudeHp = extractSignedChar(25);
            elipsoidHp = extractSignedChar(26);
            meanSeaLevelHp = extractSignedChar(27);
            horizontalAccuracy = extractLong(28);
            verticalAccuracy = extractLong(32);

            highResModuleQueried.all = true;
            highResModuleQueried.highResLatitude = true;
            highResModuleQueried.highResLatitudeHp = true;
            highResModuleQueried.highResLongitude = true;
            highResModuleQueried.highResLongitudeHp = true;
            highResModuleQueried.elipsoid = true;
            highResModuleQueried.elipsoidHp = true;
            highResModuleQueried.meanSeaLevel = true;
            highResModuleQueried.meanSeaLevelHp = true;
            highResModuleQueried.geoidSeparation = true;
            highResModuleQueried.horizontalAccuracy = true;
            highResModuleQueried.verticalAccuracy = true;
            moduleQueried.gpsiTOW = true; // this can arrive via HPPOS too.

            /*
                  if (_printDebug == true)
                  {
                    _debugSerial->print(F("Sec: "));
                    _debugSerial->print(((float)extractLong(4)) / 1000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("LON: "));
                    _debugSerial->print(((float)(int32_t)extractLong(8)) / 10000000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("LAT: "));
                    _debugSerial->print(((float)(int32_t)extractLong(12)) / 10000000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("ELI M: "));
                    _debugSerial->print(((float)(int32_t)extractLong(16)) / 1000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("MSL M: "));
                    _debugSerial->print(((float)(int32_t)extractLong(20)) / 1000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("LON HP: "));
                    _debugSerial->print(extractSignedChar(24));
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("LAT HP: "));
                    _debugSerial->print(extractSignedChar(25));
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("ELI HP: "));
                    _debugSerial->print(extractSignedChar(26));
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("MSL HP: "));
                    _debugSerial->print(extractSignedChar(27));
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("HA 2D M: "));
                    _debugSerial->print(((float)(int32_t)extractLong(28)) / 10000.0f);
                    _debugSerial->print(F(" "));
                    _debugSerial->print(F("VERT M: "));
                    _debugSerial->println(((float)(int32_t)extractLong(32)) / 10000.0f);
                  }
            */
        }
        else if (msg->id == UBX_NAV_DOP && msg->len == 18)
        {
            geometricDOP = extractInt(4);
            positionDOP = extractInt(6);
            timeDOP = extractInt(8);
            verticalDOP = extractInt(10);
            horizontalDOP = extractInt(12);
            northingDOP = extractInt(14);
            eastingDOP = extractInt(16);
            dopModuleQueried.all = true;
            dopModuleQueried.geometricDOP = true;
            dopModuleQueried.positionDOP = true;
            dopModuleQueried.timeDOP = true;
            dopModuleQueried.verticalDOP = true;
            dopModuleQueried.horizontalDOP = true;
            dopModuleQueried.northingDOP = true;
            dopModuleQueried.eastingDOP = true;
        }
        break;
    case UBX_CLASS_HNR:
        if (msg->id == UBX_HNR_ATT && msg->len == 32)
        {
            // Parse various byte fields into global vars
            hnrAtt.iTOW = extractLong(0);
            hnrAtt.roll = extractSignedLong(8);
            hnrAtt.pitch = extractSignedLong(12);
            hnrAtt.heading = extractSignedLong(16);
            hnrAtt.accRoll = extractLong(20);
            hnrAtt.accPitch = extractLong(24);
            hnrAtt.accHeading = extractLong(28);

            hnrAttQueried = true;
        }
        else if (msg->id == UBX_HNR_INS && msg->len == 36)
        {
            // Parse various byte fields into global vars
            hnrVehDyn.iTOW = extractLong(8);
            hnrVehDyn.xAngRate = extractSignedLong(12);
            hnrVehDyn.yAngRate = extractSignedLong(16);
            hnrVehDyn.zAngRate = extractSignedLong(20);
            hnrVehDyn.xAccel = extractSignedLong(24);
            hnrVehDyn.yAccel = extractSignedLong(28);
            hnrVehDyn.zAccel = extractSignedLong(32);

            uint32_t bitfield0 = extractLong(0);
            hnrVehDyn.xAngRateValid = (bitfield0 & 0x00000100) > 0;
            hnrVehDyn.yAngRateValid = (bitfield0 & 0x00000200) > 0;
            hnrVehDyn.zAngRateValid = (bitfield0 & 0x00000400) > 0;
            hnrVehDyn.xAccelValid = (bitfield0 & 0x00000800) > 0;
            hnrVehDyn.yAccelValid = (bitfield0 & 0x00001000) > 0;
            hnrVehDyn.zAccelValid = (bitfield0 & 0x00002000) > 0;

            hnrDynQueried = true;
        }
        else if (msg->id == UBX_HNR_PVT && msg->len == 72)
        {
            // Parse various byte fields into global vars
            hnrPVT.iTOW = extractLong(0);
            hnrPVT.year = extractInt(4);
            hnrPVT.month = extractByte(6);
            hnrPVT.day = extractByte(7);
            hnrPVT.hour = extractByte(8);
            hnrPVT.min = extractByte(9);
            hnrPVT.sec = extractByte(10);
            hnrPVT.nano = extractSignedLong(12);
            hnrPVT.gpsFix = extractByte(16);
            hnrPVT.lon = extractSignedLong(20);
            hnrPVT.lat = extractSignedLong(24);
            hnrPVT.height = extractSignedLong(28);
            hnrPVT.hMSL = extractSignedLong(32);
            hnrPVT.gSpeed = extractSignedLong(36);
            hnrPVT.speed = extractSignedLong(40);
            hnrPVT.headMot = extractSignedLong(44);
            hnrPVT.headVeh = extractSignedLong(48);
            hnrPVT.hAcc = extractLong(52);
            hnrPVT.vAcc = extractLong(56);
            hnrPVT.sAcc = extractLong(60);
            hnrPVT.headAcc = extractLong(64);

            uint8_t valid = extractByte(11);
            hnrPVT.validDate = (valid & 0x01) > 0;
            hnrPVT.validTime = (valid & 0x02) > 0;
            hnrPVT.fullyResolved = (valid & 0x04) > 0;

            uint8_t flags = extractByte(17);
            hnrPVT.gpsFixOK = (flags & 0x01) > 0;
            hnrPVT.diffSoln = (flags & 0x02) > 0;
            hnrPVT.WKNSET = (flags & 0x04) > 0;
            hnrPVT.TOWSET = (flags & 0x08) > 0;
            hnrPVT.headVehValid = (flags & 0x10) > 0;

            hnrPVTQueried = true;
        }
    }
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e SFE_UBLOX_GPS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
    sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

    calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

    if (_printDebug == true)
    {
        printf("\nSending: ");
        printPacket(outgoingUBX);
    }

    if (commType == COMM_TYPE_I2C)
    {
        retVal = sendI2cCommand(outgoingUBX, maxWait);
        if (retVal != SFE_UBLOX_STATUS_SUCCESS)
        {
            if (_printDebug == true)
                printf("Send I2C Command failed\n");
            return retVal;
        }
    }

    if (maxWait > 0)
    {
        // Depending on what we just sent, either we need to look for an ACK or not
        if (outgoingUBX->cls == UBX_CLASS_CFG)
        {
            if (_printDebug == true)
                printf("sendCommand: Waiting for ACK response\n");
            retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
        }
        else
        {
            if (_printDebug == true)
                printf("sendCommand: Waiting for No ACK response\n");
            retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
        }
    }
    return retVal;
}

// Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e SFE_UBLOX_GPS::sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
    // Point at 0xFF data register
    const int buffer_size = 100;
    uint8_t reg[buffer_size] = {0};
    reg[0] = 0xFF;                                                                         // Dummy register address
    if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, reg, 1, false) == PICO_ERROR_GENERIC) // Send a dummy byte to set the register pointer
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);                                        // Sensor did not ACK

    // Write header bytes
    reg[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
    reg[1] = UBX_SYNCH_2; // b
    reg[2] = outgoingUBX->cls;
    reg[3] = outgoingUBX->id;
    reg[4] = outgoingUBX->len & 0xFF; // LSB
    reg[5] = outgoingUBX->len >> 8;   // MSB

    int msgLen = 6;

    // if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, reg, 6, true) == PICO_ERROR_GENERIC) // Send header bytes
    //     return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);                                        // Sensor did not ACK

    // Write payload. Limit the sends into 32 byte chunks
    // This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
    uint16_t bytesToSend = outgoingUBX->len;

    //"The number of data bytes must be at least 2 to properly distinguish
    // from the write access to set the address counter in random read accesses."
    uint16_t startSpot = 0;
    while (bytesToSend > 1)
    {
        uint8_t len = bytesToSend;
        if (len > i2cTransactionSize)
            len = i2cTransactionSize;

        for (uint16_t x = 0; x < len; x++)
        {
            reg[x + msgLen] = outgoingUBX->payload[startSpot + x]; // Write a portion of the payload to the bus
            msgLen++;
        }

        if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, reg, len, true) == PICO_ERROR_GENERIC) // Don't release bus
            return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);                                         // Sensor did not ACK

        //*outgoingUBX->payload += len; //Move the pointer forward
        startSpot += len; // Move the pointer forward
        bytesToSend -= len;
    }

    // Write checksum
    int size = 0;
    if (bytesToSend == 1)
    {
        reg[msgLen] = outgoingUBX->payload[startSpot]; // Write the last byte of the payload
        size = 1;
    }
    reg[msgLen + size] = outgoingUBX->checksumA; // Checksum A
    reg[msgLen + size] = outgoingUBX->checksumB; // Checksum B
    // All done transmitting bytes. Release bus.
    if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, reg, msgLen + 2 + size, false) == PICO_ERROR_GENERIC) // Send checksum bytes
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);                                                        // Sensor did not ACK
    return (SFE_UBLOX_STATUS_SUCCESS);
}

// Returns true if I2C device ack's
bool SFE_UBLOX_GPS::isConnected(uint16_t maxWait)
{
    uint8_t reg[1] = {0};
    if (commType == COMM_TYPE_I2C)
    {
        if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, reg, 1, false) == PICO_ERROR_GENERIC) // Send a dummy byte to set the register pointer
            return false;       // Sensor did not ack
    }

    if (setI2COutput(COM_TYPE_UBX) == false) // Set the I2C output to UXB
    {
        if (_printDebug == true)
            printf("isConnected: setI2COutput failed\n");
        return false; // Sensor did not ack
    }

    // Query navigation rate to see whether we get a meaningful response
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RATE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait); // Poll the navigation rate

    // In this case, we don't acutally care what the navigation rate is, we're just polling it to indicate a connection.
    // So we return true if result is DATA_RECEIVED or DATA_OVERWRITTEN (just in case the RATE was overwritten by an auto packet).
    if ((result == SFE_UBLOX_STATUS_DATA_RECEIVED) || (result == SFE_UBLOX_STATUS_DATA_OVERWRITTEN))
        return (true);
    else
        return (false);
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void SFE_UBLOX_GPS::calcChecksum(ubxPacket *msg)
{
    msg->checksumA = 0;
    msg->checksumB = 0;

    msg->checksumA += msg->cls;
    msg->checksumB += msg->checksumA;

    msg->checksumA += msg->id;
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len & 0xFF);
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len >> 8);
    msg->checksumB += msg->checksumA;

    for (uint16_t i = 0; i < msg->len; i++)
    {
        msg->checksumA += msg->payload[i];
        msg->checksumB += msg->checksumA;
    }
}

// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
void SFE_UBLOX_GPS::addToChecksum(uint8_t incoming)
{
    rollingChecksumA += incoming;
    rollingChecksumB += rollingChecksumA;
}

// Pretty prints the current ubxPacket
void SFE_UBLOX_GPS::printPacket(ubxPacket *packet)
{
    if (_printDebug == true)
    {
        printf("CLS: ");
        if (packet->cls == UBX_CLASS_NAV) // 1
            printf("NAV");
        else if (packet->cls == UBX_CLASS_ACK) // 5
            printf("ACK");
        else if (packet->cls == UBX_CLASS_CFG) // 6
            printf("CFG");
        else if (packet->cls == UBX_CLASS_MON) // 0x0A
            printf("MON");
        else
            printf("0x%0x", packet->cls);

        printf(" ID: ");
        if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
            printf("PVT");
        else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_RATE)
            printf("RATE");
        else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
            printf("SAVE");
        else
            printf("0x%0x", packet->id);

        printf(" Len: 0x%0x", packet->len);

        // Only print the payload is ignoreThisPayload is false otherwise
        // we could be printing gibberish from beyond the end of packetBuf
        if (ignoreThisPayload == false)
        {
            printf(" Payload: ");

            for (int x = 0; x < packet->len; x++)
                printf(" 0x%0x", packet->payload[x]);
        }
        else
            printf(" Payload: IGNORED");

        printf("\n");
    }
}

//=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
//"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
// Some messages from other classes also use the same acknowledgement mechanism.

// When we poll or get a setting, we will receive _both_ a config packet and an ACK
// If the poll or get request is not valid, we will receive _only_ a NACK

// If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
// If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
// For all other gets or polls, .len will be 0 initially
//(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

// If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
// But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

// If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
//(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

// We need to think carefully about how interleaved PVT packets affect things.
// It is entirely possible that our packetCfg and packetAck were received successfully
// but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
// or _starts_ to arrive (remember that Serial data can arrive very slowly).

// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
// Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
// Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
// Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GPS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
    outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
    packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
    packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    unsigned long startTime = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - startTime < maxTime)
    {
        if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
        {

            // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
            // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
            // then we can be confident that the data in outgoingUBX is valid
            if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: valid data and valid ACK received after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
            }

            // We can be confident that the data packet (if we are going to get one) will always arrive
            // before the matching ACK. So if we sent a config packet which only produces an ACK
            // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
            // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
            // as these may have been changed by a PVT packet.
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: no data and valid ACK received after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
            }

            // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
            // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
            // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
            // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
            // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
            // So we cannot use outgoingUBX->valid as part of this check.
            // Note: the addition of packetBuf should make this check redundant!
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: data being OVERWRITTEN after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
            }

            // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
            // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
            else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: CRC failed after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
            }

            // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
            // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
            // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
            // but outgoingUBX->cls and outgoingUBX->id would not match...
            // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
            // the packet was definitely NACK'd otherwise we are possibly just guessing...
            // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
            else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
            }

            // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
            // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
            // If we were playing safe, we should return FAIL instead
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: VALID data and INVALID ACK received after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
            }

            // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
            // then we return a FAIL. This must be a double checksum failure?
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: INVALID data and INVALID ACK received after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
            }

            // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
            // then the ACK has not yet been received and we should keep waiting for it
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
            {
                if (_printDebug == true)
                {
                    printf("waitForACKResponse: valid data after %f msec. Waiting for ACK.\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
            }

        } // checkUbloxInternal == true

        sleep_us(500);
    } // while (to_ms_since_boot(get_absolute_time()) - startTime < maxTime)

    // We have timed out...
    // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
    // even though we did not get an ACK
    if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
    {
        if (_printDebug == true)
        {
            printf("waitForACKResponse: TIMEOUT with valid data after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
    }

    if (_printDebug == true)
    {
        printf("waitForACKResponse: TIMEOUT after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
    }

    return (SFE_UBLOX_STATUS_TIMEOUT);
}

// For non-CFG queries no ACK is sent so we use this function
// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GPS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
    outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
    packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
    packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    unsigned long startTime = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - startTime < maxTime)
    {
        if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
        {

            // If outgoingUBX->classAndIDmatch is VALID
            // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
            // then we can be confident that the data in outgoingUBX is valid
            if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
            {
                if (_printDebug == true)
                {
                    printf("waitForNoACKResponse: valid data with CLS/ID match after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
            }

            // If the outgoingUBX->classAndIDmatch is VALID
            // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
            // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
            // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
            // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
            // So we cannot use outgoingUBX->valid as part of this check.
            // Note: the addition of packetBuf should make this check redundant!
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
            {
                if (_printDebug == true)
                {
                    printf("waitForNoACKResponse: data being OVERWRITTEN after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
            }

            // If outgoingUBX->classAndIDmatch is NOT_DEFINED
            // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
            else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
            {
                // if (_printDebug == true)
                // {
                //   _debugSerial->print(F("waitForNoACKResponse: valid but UNWANTED data after "));
                //   _debugSerial->print(to_ms_since_boot(get_absolute_time()) - startTime);
                //   _debugSerial->print(F(" msec. Class: "));
                //   _debugSerial->print(outgoingUBX->cls);
                //   _debugSerial->print(F(" ID: "));
                //   _debugSerial->print(outgoingUBX->id);
                // }
            }

            // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
            else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
            {
                if (_printDebug == true)
                {
                    printf("waitForNoACKResponse: CLS/ID match but failed CRC after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
                }
                return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
            }
        }

        sleep_us(500);
    }

    if (_printDebug == true)
    {
        printf("waitForNoACKResponse: TIMEOUT after %f msec\n", (float)(to_ms_since_boot(get_absolute_time()) - startTime));
    }

    return (SFE_UBLOX_STATUS_TIMEOUT);
}

// Save current configuration to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GPS::saveConfiguration(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_CFG;
    packetCfg.len = 12;
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    packetCfg.payload[4] = 0xFF; // Set any bit in the saveMask field to save current config to Flash and BBR
    packetCfg.payload[5] = 0xFF;

    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Save the selected configuration sub-sections to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GPS::saveConfigSelective(uint32_t configMask, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_CFG;
    packetCfg.len = 12;
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    packetCfg.payload[4] = configMask & 0xFF; // Set the appropriate bits in the saveMask field to save current config to Flash and BBR
    packetCfg.payload[5] = (configMask >> 8) & 0xFF;
    packetCfg.payload[6] = (configMask >> 16) & 0xFF;
    packetCfg.payload[7] = (configMask >> 24) & 0xFF;

    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Reset module to factory defaults
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GPS::factoryDefault(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_CFG;
    packetCfg.len = 12;
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    packetCfg.payload[0] = 0xFF; // Set any bit in the clearMask field to clear saved config
    packetCfg.payload[1] = 0xFF;
    packetCfg.payload[8] = 0xFF; // Set any bit in the loadMask field to discard current config and rebuild from lower non-volatile memory layers
    packetCfg.payload[9] = 0xFF;

    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, load the payload with data that can then be extracted to 8, 16, or 32 bits
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
sfe_ublox_status_e SFE_UBLOX_GPS::getVal(uint32_t key, uint8_t layer, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALGET;
    packetCfg.len = 4 + 4 * 1; // While multiple keys are allowed, we will send only one key at a time
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    // VALGET uses different memory layer definitions to VALSET
    // because it can only return the value for one layer.
    // So we need to fiddle the layer here.
    // And just to complicate things further, the ZED-F9P only responds
    // correctly to layer 0 (RAM) and layer 7 (Default)!
    uint8_t getLayer = 7;                         // 7 is the "Default Layer"
    if ((layer & VAL_LAYER_RAM) == VAL_LAYER_RAM) // Did the user request the RAM layer?
    {
        getLayer = 0; // Layer 0 is RAM
    }

    payloadCfg[0] = 0;        // Message Version - set to 0
    payloadCfg[1] = getLayer; // Layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    if (_printDebug == true)
    {
        printf("getVal: key: 0x%0x\n", key);
    }

    // Send VALGET command with this key

    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);
    if (_printDebug == true)
    {
        printf("getVal: sendCommand returned: %s\n", statusString(retVal));
    }

    // Verify the response is the correct length as compared to what the user called (did the module respond with 8-bits but the user called getVal32?)
    // Response is 8 bytes plus cfg data
    // if(packet->len > 8+1)

    // The response is now sitting in payload, ready for extraction
    return (retVal);
}

// Given a key, return its value
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::getVal8(uint32_t key, uint8_t layer, uint16_t maxWait)
{
    if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (0);

    return (extractByte(8));
}
uint16_t SFE_UBLOX_GPS::getVal16(uint32_t key, uint8_t layer, uint16_t maxWait)
{
    if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (0);

    return (extractInt(8));
}
uint32_t SFE_UBLOX_GPS::getVal32(uint32_t key, uint8_t layer, uint16_t maxWait)
{
    if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (0);

    return (extractLong(8));
}

// Form 32-bit key from group/id/size
uint32_t SFE_UBLOX_GPS::createKey(uint16_t group, uint16_t id, uint8_t size)
{
    uint32_t key = 0;
    key |= (uint32_t)id;
    key |= (uint32_t)group << 16;
    key |= (uint32_t)size << 28;
    return (key);
}

// Given a group, ID and size, return the value of this config spot
// The 32-bit key is put together from group/ID/size. See other getVal to send key directly.
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
    uint32_t key = createKey(group, id, size);
    return getVal8(key, layer, maxWait);
}
uint16_t SFE_UBLOX_GPS::getVal16(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
    uint32_t key = createKey(group, id, size);
    return getVal16(key, layer, maxWait);
}
uint32_t SFE_UBLOX_GPS::getVal32(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
    uint32_t key = createKey(group, id, size);
    return getVal32(key, layer, maxWait);
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::setVal(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
    return setVal16(key, value, layer, maxWait);
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 2; // 4 byte header, 4 byte key ID, 2 bytes of value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value >> 8 * 0; // Value LSB
    payloadCfg[9] = value >> 8 * 1;

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set an 8-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::setVal8(uint32_t key, uint8_t value, uint8_t layer, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 1; // 4 byte header, 4 byte key ID, 1 byte value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value; // Value

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set a 32-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::setVal32(uint32_t key, uint32_t value, uint8_t layer, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 4; // 4 byte header, 4 byte key ID, 4 bytes of value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value >> 8 * 0; // Value LSB
    payloadCfg[9] = value >> 8 * 1;
    payloadCfg[10] = value >> 8 * 2;
    payloadCfg[11] = value >> 8 * 3;

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit value
// Default layer is BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::newCfgValset32(uint32_t key, uint32_t value, uint8_t layer)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 4; // 4 byte header, 4 byte key ID, 4 bytes of value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < MAX_PAYLOAD_SIZE; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value >> 8 * 0; // Value LSB
    payloadCfg[9] = value >> 8 * 1;
    payloadCfg[10] = value >> 8 * 2;
    payloadCfg[11] = value >> 8 * 3;

    // All done
    return (true);
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 16-bit value
// Default layer is BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::newCfgValset16(uint32_t key, uint16_t value, uint8_t layer)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 2; // 4 byte header, 4 byte key ID, 2 bytes of value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < MAX_PAYLOAD_SIZE; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value >> 8 * 0; // Value LSB
    payloadCfg[9] = value >> 8 * 1;

    // All done
    return (true);
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 8-bit value
// Default layer is BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::newCfgValset8(uint32_t key, uint8_t value, uint8_t layer)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + 1; // 4 byte header, 4 byte key ID, 1 byte value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint16_t x = 0; x < MAX_PAYLOAD_SIZE; x++)
        packetCfg.payload[x] = 0;

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Load user's value
    payloadCfg[8] = value; // Value

    // All done
    return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit value
uint8_t SFE_UBLOX_GPS::addCfgValset32(uint32_t key, uint32_t value)
{
    // Load key into outgoing payload
    payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
    payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
    payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
    payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

    // Load user's value
    payloadCfg[packetCfg.len + 4] = value >> 8 * 0; // Value LSB
    payloadCfg[packetCfg.len + 5] = value >> 8 * 1;
    payloadCfg[packetCfg.len + 6] = value >> 8 * 2;
    payloadCfg[packetCfg.len + 7] = value >> 8 * 3;

    // Update packet length: 4 byte key ID, 4 bytes of value
    packetCfg.len = packetCfg.len + 4 + 4;

    // All done
    return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 16-bit value
uint8_t SFE_UBLOX_GPS::addCfgValset16(uint32_t key, uint16_t value)
{
    // Load key into outgoing payload
    payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
    payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
    payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
    payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

    // Load user's value
    payloadCfg[packetCfg.len + 4] = value >> 8 * 0; // Value LSB
    payloadCfg[packetCfg.len + 5] = value >> 8 * 1;

    // Update packet length: 4 byte key ID, 2 bytes of value
    packetCfg.len = packetCfg.len + 4 + 2;

    // All done
    return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 8-bit value
uint8_t SFE_UBLOX_GPS::addCfgValset8(uint32_t key, uint8_t value)
{
    // Load key into outgoing payload
    payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
    payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
    payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
    payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

    // Load user's value
    payloadCfg[packetCfg.len + 4] = value; // Value

    // Update packet length: 4 byte key ID, 1 byte value
    packetCfg.len = packetCfg.len + 4 + 1;

    // All done
    return (true);
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 32-bit value
uint8_t SFE_UBLOX_GPS::sendCfgValset32(uint32_t key, uint32_t value, uint16_t maxWait)
{
    // Load keyID and value into outgoing payload
    addCfgValset32(key, value);

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 16-bit value
uint8_t SFE_UBLOX_GPS::sendCfgValset16(uint32_t key, uint16_t value, uint16_t maxWait)
{
    // Load keyID and value into outgoing payload
    addCfgValset16(key, value);

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 8-bit value
uint8_t SFE_UBLOX_GPS::sendCfgValset8(uint32_t key, uint8_t value, uint16_t maxWait)
{
    // Load keyID and value into outgoing payload
    addCfgValset8(key, value);

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the current TimeMode3 settings - these contain survey in statuses
bool SFE_UBLOX_GPS::getSurveyMode(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_TMODE3;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

// Control Survey-In for NEO-M8P
bool SFE_UBLOX_GPS::setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
    if (getSurveyMode(maxWait) == false) // Ask module for the current TimeMode3 settings. Loads into payloadCfg.
        return (false);

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_TMODE3;
    packetCfg.len = 40;
    packetCfg.startingSpot = 0;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        packetCfg.payload[x] = 0;

    // payloadCfg should be loaded with poll response. Now modify only the bits we care about
    payloadCfg[2] = mode; // Set mode. Survey-In and Disabled are most common. Use ECEF (not LAT/LON/ALT).

    // svinMinDur is U4 (uint32_t) but we'll only use a uint16_t (waiting more than 65535 seconds seems excessive!)
    payloadCfg[24] = observationTime & 0xFF; // svinMinDur in seconds
    payloadCfg[25] = observationTime >> 8;   // svinMinDur in seconds
    payloadCfg[26] = 0;                      // Truncate to 16 bits
    payloadCfg[27] = 0;                      // Truncate to 16 bits

    // svinAccLimit is U4 (uint32_t) in 0.1mm.
    uint32_t svinAccLimit = (uint32_t)(requiredAccuracy * 10000.0); // Convert m to 0.1mm
    payloadCfg[28] = svinAccLimit & 0xFF;                           // svinAccLimit in 0.1mm increments
    payloadCfg[29] = svinAccLimit >> 8;
    payloadCfg[30] = svinAccLimit >> 16;
    payloadCfg[31] = svinAccLimit >> 24;

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Begin Survey-In for NEO-M8P
bool SFE_UBLOX_GPS::enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
    return (setSurveyMode(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait));
}

// Stop Survey-In for NEO-M8P
bool SFE_UBLOX_GPS::disableSurveyMode(uint16_t maxWait)
{
    return (setSurveyMode(SVIN_MODE_DISABLE, 0, 0, maxWait));
}

// Reads survey in status and sets the global variables
// for status, position valid, observation time, and mean 3D StdDev
// Returns true if commands was successful
bool SFE_UBLOX_GPS::getSurveyStatus(uint16_t maxWait)
{
    // Reset variables
    svin.active = false;
    svin.valid = false;
    svin.observationTime = 0;
    svin.meanAccuracy = 0;

    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SVIN;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if ((sendCommand(&packetCfg, maxWait)) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);                                                       // If command send fails then bail

    // We got a response, now parse the bits into the svin structure

    // dur (Passed survey-in observation time) is U4 (uint32_t) seconds. We truncate to 16 bits
    //(waiting more than 65535 seconds (18.2 hours) seems excessive!)
    uint32_t tmpObsTime = extractLong(8);
    if (tmpObsTime <= 0xFFFF)
    {
        svin.observationTime = (uint16_t)tmpObsTime;
    }
    else
    {
        svin.observationTime = 0xFFFF;
    }

    // meanAcc is U4 (uint32_t) in 0.1mm. We convert this to float.
    uint32_t tempFloat = extractLong(28);
    svin.meanAccuracy = ((float)tempFloat) / 10000.0; // Convert 0.1mm to m

    svin.valid = payloadCfg[36];  // 1 if survey-in position is valid, 0 otherwise
    svin.active = payloadCfg[37]; // 1 if survey-in in progress, 0 otherwise

    return (true);
}

// Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
bool SFE_UBLOX_GPS::getPortSettings(uint8_t portID, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_PRT;
    packetCfg.len = 1;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = portID;

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

// Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool SFE_UBLOX_GPS::setPortOutput(uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
    // Get the current config values for this port ID
    // if (getPortSettings(portID, maxWait) == false)
    //     if(_printDebug)
    //         printf("getPortSettings: FAILED\n");
    //     return (false); // Something went wrong. Bail.

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_PRT;
    packetCfg.len = 20;
    packetCfg.startingSpot = 0;

    // payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg[0] = portID; // Port ID
    // payloadCfg[1] = 0;     // Reserved
    // payloadCfg[2] = 0;     // TxReady
    // payloadCfg[4] = 0;     // mode
    // payloadCfg[8] = 0;     // reserved
    payloadCfg[12] = 3;     // inProtoMask - inNMEA and inUBX
    payloadCfg[14] = outStreamSettings; // OutProtocolMask LSB - Set outStream bits
    // payloadCfg[16] = 0;     // flags
    // payloadCfg[18] = 0;     // reserved

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool SFE_UBLOX_GPS::setPortInput(uint8_t portID, uint8_t inStreamSettings, uint16_t maxWait)
{
    // Get the current config values for this port ID
    // This will load the payloadCfg array with current port settings
    if (getPortSettings(portID, maxWait) == false)
        return (false); // Something went wrong. Bail.

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_PRT;
    packetCfg.len = 20;
    packetCfg.startingSpot = 0;

    // payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg[12] = inStreamSettings; // InProtocolMask LSB - Set inStream bits

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
bool SFE_UBLOX_GPS::setI2COutput(uint8_t comSettings, uint16_t maxWait)
{
    return (setPortOutput(COM_PORT_I2C, comSettings, maxWait));
}
bool SFE_UBLOX_GPS::setUART1Output(uint8_t comSettings, uint16_t maxWait)
{
    return (setPortOutput(COM_PORT_UART1, comSettings, maxWait));
}
bool SFE_UBLOX_GPS::setUART2Output(uint8_t comSettings, uint16_t maxWait)
{
    return (setPortOutput(COM_PORT_UART2, comSettings, maxWait));
}
bool SFE_UBLOX_GPS::setUSBOutput(uint8_t comSettings, uint16_t maxWait)
{
    return (setPortOutput(COM_PORT_USB, comSettings, maxWait));
}
bool SFE_UBLOX_GPS::setSPIOutput(uint8_t comSettings, uint16_t maxWait)
{
    return (setPortOutput(COM_PORT_SPI, comSettings, maxWait));
}

// Set the rate at which the module will give us an updated navigation solution
// Expects a number that is the updates per second. For example 1 = 1Hz, 2 = 2Hz, etc.
// Max is 40Hz(?!)
bool SFE_UBLOX_GPS::setNavigationFrequency(uint8_t navFreq, uint16_t maxWait)
{
    // if(updateRate > 40) updateRate = 40; //Not needed: module will correct out of bounds values

    // Adjust the I2C polling timeout based on update rate
    i2cPollingWait = 1000 / (((int)navFreq) * 4); // This is the number of ms to wait between checks for new I2C data

    // Query the module for the latest lat/long
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RATE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // This will load the payloadCfg array with current settings of the given register
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);                                                     // If command send fails then bail

    uint16_t measurementRate = 1000 / navFreq;

    // payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg[0] = measurementRate & 0xFF; // measRate LSB
    payloadCfg[1] = measurementRate >> 8;   // measRate MSB

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the rate at which the module is outputting nav solutions
uint8_t SFE_UBLOX_GPS::getNavigationFrequency(uint16_t maxWait)
{
    // Query the module for the latest lat/long
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RATE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // This will load the payloadCfg array with current settings of the given register
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);                                                     // If command send fails then bail

    // payloadCfg is now loaded with current bytes. Get what we need
    uint16_t measurementRate = extractInt(0); // Pull from payloadCfg at measRate LSB

    measurementRate = 1000 / measurementRate; // This may return an int when it's a float, but I'd rather not return 4 bytes
    return (measurementRate);
}

// In case no config access to the GPS is possible and PVT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoPVT(bool enabled, bool implicitUpdate)
{
    bool changes = autoPVT != enabled || autoPVTImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoPVT = enabled;
        autoPVTImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getPVT
// works.
bool SFE_UBLOX_GPS::setAutoPVT(bool enable, uint16_t maxWait)
{
    return setAutoPVT(enable, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getPVT
// works.
bool SFE_UBLOX_GPS::setAutoPVT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_NAV;
    payloadCfg[1] = UBX_NAV_PVT;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoPVT = enable;
        autoPVTImplicitUpdate = implicitUpdate;
    }
    moduleQueried.all = false;
    return ok;
}

// In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate)
{
    bool changes = autoHPPOSLLH != enabled || autoHPPOSLLHImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoHPPOSLLH = enabled;
        autoHPPOSLLHImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getHPPOSLLH
// works.
bool SFE_UBLOX_GPS::setAutoHPPOSLLH(bool enable, uint16_t maxWait)
{
    return setAutoHPPOSLLH(enable, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getHPPOSLLH
// works.
bool SFE_UBLOX_GPS::setAutoHPPOSLLH(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_NAV;
    payloadCfg[1] = UBX_NAV_HPPOSLLH;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoHPPOSLLH = enable;
        autoHPPOSLLHImplicitUpdate = implicitUpdate;
    }
    highResModuleQueried.all = false;
    return ok;
}

// In case no config access to the GPS is possible and DOP is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoDOP(bool enabled, bool implicitUpdate)
{
    bool changes = autoDOP != enabled || autoDOPImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoDOP = enabled;
        autoDOPImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getDOP
// works.
bool SFE_UBLOX_GPS::setAutoDOP(bool enable, uint16_t maxWait)
{
    return setAutoDOP(enable, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GPS. This changes the way getDOP
// works.
bool SFE_UBLOX_GPS::setAutoDOP(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_NAV;
    payloadCfg[1] = UBX_NAV_DOP;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoDOP = enable;
        autoDOPImplicitUpdate = implicitUpdate;
    }
    dopModuleQueried.all = false;
    return ok;
}

// Configure a given message type for a given port (UART1, I2C, SPI, etc)
bool SFE_UBLOX_GPS::configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait)
{
    // Poll for the current settings for a given message
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 2;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = msgClass;
    payloadCfg[1] = msgID;

    // This will load the payloadCfg array with current settings of the given register
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);                                                     // If command send fails then bail

    // Now send it back with new mods
    packetCfg.len = 8;

    // payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg[2 + portID] = sendRate; // Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every 2nd navigation solution.

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable a given message type, default of 1 per update rate (usually 1 per second)
bool SFE_UBLOX_GPS::enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t rate, uint16_t maxWait)
{
    return (configureMessage(msgClass, msgID, portID, rate, maxWait));
}
// Disable a given message type on a given port
bool SFE_UBLOX_GPS::disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait)
{
    return (configureMessage(msgClass, msgID, portID, 0, maxWait));
}

bool SFE_UBLOX_GPS::enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t rate, uint16_t maxWait)
{
    return (configureMessage(UBX_CLASS_NMEA, msgID, portID, rate, maxWait));
}
bool SFE_UBLOX_GPS::disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait)
{
    return (enableNMEAMessage(msgID, portID, 0, maxWait));
}

// Given a message number turns on a message ID for output over a given portID (UART, I2C, SPI, USB, etc)
// To disable a message, set secondsBetween messages to 0
// Note: This function will return false if the message is already enabled
// For base station RTK output we need to enable various sentences

// NEO-M8P has four:
// 1005 = 0xF5 0x05 - Stationary RTK reference ARP
// 1077 = 0xF5 0x4D - GPS MSM7
// 1087 = 0xF5 0x57 - GLONASS MSM7
// 1230 = 0xF5 0xE6 - GLONASS code-phase biases, set to once every 10 seconds

// ZED-F9P has six:
// 1005, 1074, 1084, 1094, 1124, 1230

// Much of this configuration is not documented and instead discerned from u-center binary console
bool SFE_UBLOX_GPS::enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait)
{
    return (configureMessage(UBX_RTCM_MSB, messageNumber, portID, sendRate, maxWait));
}

// Disable a given message on a given port by setting secondsBetweenMessages to zero
bool SFE_UBLOX_GPS::disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait)
{
    return (enableRTCMmessage(messageNumber, portID, 0, maxWait));
}

// Add a new geofence using UBX-CFG-GEOFENCE
//  bool SFE_UBLOX_GPS::addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence, byte pinPolarity, byte pin, uint16_t maxWait)
//  {
//    if (currentGeofenceParams.numFences >= 4)
//      return (false); // Quit if we already have four geofences defined

//   // Store the new geofence parameters
//   currentGeofenceParams.lats[currentGeofenceParams.numFences] = latitude;
//   currentGeofenceParams.longs[currentGeofenceParams.numFences] = longitude;
//   currentGeofenceParams.rads[currentGeofenceParams.numFences] = radius;
//   currentGeofenceParams.numFences = currentGeofenceParams.numFences + 1; // Increment the number of fences

//   packetCfg.cls = UBX_CLASS_CFG;
//   packetCfg.id = UBX_CFG_GEOFENCE;
//   packetCfg.len = (currentGeofenceParams.numFences * 12) + 8;
//   packetCfg.startingSpot = 0;

//   payloadCfg[0] = 0;                               // Message version = 0x00
//   payloadCfg[1] = currentGeofenceParams.numFences; // numFences
//   payloadCfg[2] = confidence;                      // confLvl = Confidence level 0-4 (none, 68%, 95%, 99.7%, 99.99%)
//   payloadCfg[3] = 0;                               // reserved1
//   if (pin > 0)
//   {
//     payloadCfg[4] = 1; // enable PIO combined fence state
//   }
//   else
//   {
//     payloadCfg[4] = 0; // disable PIO combined fence state
//   }
//   payloadCfg[5] = pinPolarity; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
//   payloadCfg[6] = pin;         // PIO pin
//   payloadCfg[7] = 0;           //reserved2
//   payloadCfg[8] = currentGeofenceParams.lats[0] & 0xFF;
//   payloadCfg[9] = currentGeofenceParams.lats[0] >> 8;
//   payloadCfg[10] = currentGeofenceParams.lats[0] >> 16;
//   payloadCfg[11] = currentGeofenceParams.lats[0] >> 24;
//   payloadCfg[12] = currentGeofenceParams.longs[0] & 0xFF;
//   payloadCfg[13] = currentGeofenceParams.longs[0] >> 8;
//   payloadCfg[14] = currentGeofenceParams.longs[0] >> 16;
//   payloadCfg[15] = currentGeofenceParams.longs[0] >> 24;
//   payloadCfg[16] = currentGeofenceParams.rads[0] & 0xFF;
//   payloadCfg[17] = currentGeofenceParams.rads[0] >> 8;
//   payloadCfg[18] = currentGeofenceParams.rads[0] >> 16;
//   payloadCfg[19] = currentGeofenceParams.rads[0] >> 24;
//   if (currentGeofenceParams.numFences >= 2)
//   {
//     payloadCfg[20] = currentGeofenceParams.lats[1] & 0xFF;
//     payloadCfg[21] = currentGeofenceParams.lats[1] >> 8;
//     payloadCfg[22] = currentGeofenceParams.lats[1] >> 16;
//     payloadCfg[23] = currentGeofenceParams.lats[1] >> 24;
//     payloadCfg[24] = currentGeofenceParams.longs[1] & 0xFF;
//     payloadCfg[25] = currentGeofenceParams.longs[1] >> 8;
//     payloadCfg[26] = currentGeofenceParams.longs[1] >> 16;
//     payloadCfg[27] = currentGeofenceParams.longs[1] >> 24;
//     payloadCfg[28] = currentGeofenceParams.rads[1] & 0xFF;
//     payloadCfg[29] = currentGeofenceParams.rads[1] >> 8;
//     payloadCfg[30] = currentGeofenceParams.rads[1] >> 16;
//     payloadCfg[31] = currentGeofenceParams.rads[1] >> 24;
//   }
//   if (currentGeofenceParams.numFences >= 3)
//   {
//     payloadCfg[32] = currentGeofenceParams.lats[2] & 0xFF;
//     payloadCfg[33] = currentGeofenceParams.lats[2] >> 8;
//     payloadCfg[34] = currentGeofenceParams.lats[2] >> 16;
//     payloadCfg[35] = currentGeofenceParams.lats[2] >> 24;
//     payloadCfg[36] = currentGeofenceParams.longs[2] & 0xFF;
//     payloadCfg[37] = currentGeofenceParams.longs[2] >> 8;
//     payloadCfg[38] = currentGeofenceParams.longs[2] >> 16;
//     payloadCfg[39] = currentGeofenceParams.longs[2] >> 24;
//     payloadCfg[40] = currentGeofenceParams.rads[2] & 0xFF;
//     payloadCfg[41] = currentGeofenceParams.rads[2] >> 8;
//     payloadCfg[42] = currentGeofenceParams.rads[2] >> 16;
//     payloadCfg[43] = currentGeofenceParams.rads[2] >> 24;
//   }
//   if (currentGeofenceParams.numFences >= 4)
//   {
//     payloadCfg[44] = currentGeofenceParams.lats[3] & 0xFF;
//     payloadCfg[45] = currentGeofenceParams.lats[3] >> 8;
//     payloadCfg[46] = currentGeofenceParams.lats[3] >> 16;
//     payloadCfg[47] = currentGeofenceParams.lats[3] >> 24;
//     payloadCfg[48] = currentGeofenceParams.longs[3] & 0xFF;
//     payloadCfg[49] = currentGeofenceParams.longs[3] >> 8;
//     payloadCfg[50] = currentGeofenceParams.longs[3] >> 16;
//     payloadCfg[51] = currentGeofenceParams.longs[3] >> 24;
//     payloadCfg[52] = currentGeofenceParams.rads[3] & 0xFF;
//     payloadCfg[53] = currentGeofenceParams.rads[3] >> 8;
//     payloadCfg[54] = currentGeofenceParams.rads[3] >> 16;
//     payloadCfg[55] = currentGeofenceParams.rads[3] >> 24;
//   }
//   return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
// }

// Clear all geofences using UBX-CFG-GEOFENCE
bool SFE_UBLOX_GPS::clearGeofences(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_GEOFENCE;
    packetCfg.len = 8;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = 0; // Message version = 0x00
    payloadCfg[1] = 0; // numFences
    payloadCfg[2] = 0; // confLvl
    payloadCfg[3] = 0; // reserved1
    payloadCfg[4] = 0; // disable PIO combined fence state
    payloadCfg[5] = 0; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
    payloadCfg[6] = 0; // PIO pin
    payloadCfg[7] = 0; // reserved2

    currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Clear the antenna control settings using UBX-CFG-ANT
// This function is hopefully redundant but may be needed to release
// any PIO pins pre-allocated for antenna functions
bool SFE_UBLOX_GPS::clearAntPIO(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_ANT;
    packetCfg.len = 4;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = 0x10; // Antenna flag mask: set the recovery bit
    payloadCfg[1] = 0;
    payloadCfg[2] = 0xFF; // Antenna pin configuration: set pinSwitch and pinSCD to 31
    payloadCfg[3] = 0xFF; // Antenna pin configuration: set pinOCD to 31, set reconfig bit

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Returns the combined geofence state using UBX-NAV-GEOFENCE
bool SFE_UBLOX_GPS::getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_GEOFENCE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the geofence status. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);

    currentGeofenceState.status = payloadCfg[5];    // Extract the status
    currentGeofenceState.numFences = payloadCfg[6]; // Extract the number of geofences
    currentGeofenceState.combState = payloadCfg[7]; // Extract the combined state of all geofences
    if (currentGeofenceState.numFences > 0)
        currentGeofenceState.states[0] = payloadCfg[8]; // Extract geofence 1 state
    if (currentGeofenceState.numFences > 1)
        currentGeofenceState.states[1] = payloadCfg[10]; // Extract geofence 2 state
    if (currentGeofenceState.numFences > 2)
        currentGeofenceState.states[2] = payloadCfg[12]; // Extract geofence 3 state
    if (currentGeofenceState.numFences > 3)
        currentGeofenceState.states[3] = payloadCfg[14]; // Extract geofence 4 state

    return (true);
}

// Power Save Mode
// Enables/Disables Low Power Mode using UBX-CFG-RXM
bool SFE_UBLOX_GPS::powerSaveMode(bool power_save, uint16_t maxWait)
{
    // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
    uint8_t protVer = getProtocolVersionHigh(maxWait);
    /*
    if (_printDebug == true)
    {
      _debugSerial->print(F("Protocol version is "));
      _debugSerial->println(protVer);
    }
    */
    if (protVer >= 27)
    {
        if (_printDebug == true)
        {
            printf("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version");
        }
        return (false);
    }

    // Now let's change the power setting using UBX-CFG-RXM
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RXM;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current power management settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);

    if (power_save)
    {
        payloadCfg[1] = 1; // Power Save Mode
    }
    else
    {
        payloadCfg[1] = 0; // Continuous Mode
    }

    packetCfg.len = 2;
    packetCfg.startingSpot = 0;

    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get Power Save Mode
// Returns the current Low Power Mode using UBX-CFG-RXM
// Returns 255 if the sendCommand fails
uint8_t SFE_UBLOX_GPS::getPowerSaveMode(uint16_t maxWait)
{
    // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
    uint8_t protVer = getProtocolVersionHigh(maxWait);
    /*
    if (_printDebug == true)
    {
      _debugSerial->print(F("Protocol version is "));
      _debugSerial->println(protVer);
    }
    */
    if (protVer >= 27)
    {
        if (_printDebug == true)
        {
            printf("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version");
        }
        return (255);
    }

    // Now let's read the power setting using UBX-CFG-RXM
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RXM;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current power management settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (255);

    return (payloadCfg[1]); // Return the low power mode
}

// Powers off the GPS device for a given duration to reduce power consumption.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool SFE_UBLOX_GPS::powerOff(uint32_t durationInMs, uint16_t maxWait)
{
    // Power off device using UBX-RXM-PMREQ
    packetCfg.cls = UBX_CLASS_RXM; // 0x02
    packetCfg.id = UBX_RXM_PMREQ;  // 0x41
    packetCfg.len = 8;
    packetCfg.startingSpot = 0;

    // duration
    // big endian to little endian, switch byte order
    payloadCfg[0] = (durationInMs >> (8 * 0)) & 0xff;
    payloadCfg[1] = (durationInMs >> (8 * 1)) & 0xff;
    payloadCfg[2] = (durationInMs >> (8 * 2)) & 0xff;
    payloadCfg[3] = (durationInMs >> (8 * 3)) & 0xff;

    payloadCfg[4] = 0x02; // Flags : set the backup bit
    payloadCfg[5] = 0x00; // Flags
    payloadCfg[6] = 0x00; // Flags
    payloadCfg[7] = 0x00; // Flags

    if (maxWait != 0)
    {
        // check for "not acknowledged" command
        return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
    }
    else
    {
        sendCommand(&packetCfg, maxWait);
        return false; // can't tell if command not acknowledged if maxWait = 0
    }
}

// Powers off the GPS device for a given duration to reduce power consumption.
// While powered off it can be woken up by creating a falling or rising voltage edge on the specified pin.
// NOTE: The GPS seems to be sensitve to signals on the pins while powered off. Works best when Microcontroller is in deepsleep.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool SFE_UBLOX_GPS::powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources, bool forceWhileUsb, uint16_t maxWait)
{
    // Power off device using UBX-RXM-PMREQ
    packetCfg.cls = UBX_CLASS_RXM; // 0x02
    packetCfg.id = UBX_RXM_PMREQ;  // 0x41
    packetCfg.len = 16;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = 0x00; // message version

    // bytes 1-3 are reserved - and must be set to zero
    payloadCfg[1] = 0x00;
    payloadCfg[2] = 0x00;
    payloadCfg[3] = 0x00;

    // duration
    // big endian to little endian, switch byte order
    payloadCfg[4] = (durationInMs >> (8 * 0)) & 0xff;
    payloadCfg[5] = (durationInMs >> (8 * 1)) & 0xff;
    payloadCfg[6] = (durationInMs >> (8 * 2)) & 0xff;
    payloadCfg[7] = (durationInMs >> (8 * 3)) & 0xff;

    // flags

    // disables USB interface when powering off, defaults to true
    if (forceWhileUsb)
    {
        payloadCfg[8] = 0x06; // force | backup
    }
    else
    {
        payloadCfg[8] = 0x02; // backup only (leave the force bit clear - module will stay on if USB is connected)
    }

    payloadCfg[9] = 0x00;
    payloadCfg[10] = 0x00;
    payloadCfg[11] = 0x00;

    // wakeUpSources

    // wakeupPin mapping, defaults to VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0

    // Possible values are:
    // VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX
    // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0
    // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1
    // VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS

    payloadCfg[12] = (wakeupSources >> (8 * 0)) & 0xff;
    payloadCfg[13] = (wakeupSources >> (8 * 1)) & 0xff;
    payloadCfg[14] = (wakeupSources >> (8 * 2)) & 0xff;
    payloadCfg[15] = (wakeupSources >> (8 * 3)) & 0xff;

    if (maxWait != 0)
    {
        // check for "not acknowledged" command
        return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
    }
    else
    {
        sendCommand(&packetCfg, maxWait);
        return false; // can't tell if command not acknowledged if maxWait = 0
    }
}

// Change the dynamic platform model using UBX-CFG-NAV5
// Possible values are:
// PORTABLE,STATIONARY,PEDESTRIAN,AUTOMOTIVE,SEA,
// AIRBORNE1g,AIRBORNE2g,AIRBORNE4g,WRIST,BIKE
// WRIST is not supported in protocol versions less than 18
// BIKE is supported in protocol versions 19.2
bool SFE_UBLOX_GPS::setDynamicModel(dynModel newDynamicModel, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_NAV5;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current navigation model settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (false);

    payloadCfg[0] = 0x01;            // mask: set only the dyn bit (0)
    payloadCfg[1] = 0x00;            // mask
    payloadCfg[2] = newDynamicModel; // dynModel

    packetCfg.len = 36;
    packetCfg.startingSpot = 0;

    return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the dynamic platform model using UBX-CFG-NAV5
// Returns 255 if the sendCommand fails
uint8_t SFE_UBLOX_GPS::getDynamicModel(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_NAV5;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current navigation model settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
        return (255);

    return (payloadCfg[2]); // Return the dynamic model
}

// Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GPS::extractLong(uint8_t spotToStart)
{
    uint32_t val = 0;
    val |= (uint32_t)payloadCfg[spotToStart + 0] << 8 * 0;
    val |= (uint32_t)payloadCfg[spotToStart + 1] << 8 * 1;
    val |= (uint32_t)payloadCfg[spotToStart + 2] << 8 * 2;
    val |= (uint32_t)payloadCfg[spotToStart + 3] << 8 * 3;
    return (val);
}

// Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t SFE_UBLOX_GPS::extractSignedLong(uint8_t spotToStart)
{
    union // Use a union to convert from uint32_t to int32_t
    {
        uint32_t unsignedLong;
        int32_t signedLong;
    } unsignedSigned;

    unsignedSigned.unsignedLong = extractLong(spotToStart);
    return (unsignedSigned.signedLong);
}

// Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GPS::extractInt(uint8_t spotToStart)
{
    uint16_t val = 0;
    val |= (uint16_t)payloadCfg[spotToStart + 0] << 8 * 0;
    val |= (uint16_t)payloadCfg[spotToStart + 1] << 8 * 1;
    return (val);
}

// Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t SFE_UBLOX_GPS::extractSignedInt(int8_t spotToStart)
{
    union // Use a union to convert from uint16_t to int16_t
    {
        uint16_t unsignedInt;
        int16_t signedInt;
    } stSignedInt;

    stSignedInt.unsignedInt = extractInt(spotToStart);
    return (stSignedInt.signedInt);
}

// Given a spot, extract a byte from the payload
uint8_t SFE_UBLOX_GPS::extractByte(uint8_t spotToStart)
{
    return (payloadCfg[spotToStart]);
}

// Given a spot, extract a signed 8-bit value from the payload
int8_t SFE_UBLOX_GPS::extractSignedChar(uint8_t spotToStart)
{
    return ((int8_t)payloadCfg[spotToStart]);
}

// Get the current year
uint16_t SFE_UBLOX_GPS::getYear(uint16_t maxWait)
{
    if (moduleQueried.gpsYear == false)
        getPVT(maxWait);
    moduleQueried.gpsYear = false; // Since we are about to give this to user, mark this data as stale
    return (gpsYear);
}

// Get the current month
uint8_t SFE_UBLOX_GPS::getMonth(uint16_t maxWait)
{
    if (moduleQueried.gpsMonth == false)
        getPVT(maxWait);
    moduleQueried.gpsMonth = false; // Since we are about to give this to user, mark this data as stale
    return (gpsMonth);
}

// Get the current day
uint8_t SFE_UBLOX_GPS::getDay(uint16_t maxWait)
{
    if (moduleQueried.gpsDay == false)
        getPVT(maxWait);
    moduleQueried.gpsDay = false; // Since we are about to give this to user, mark this data as stale
    return (gpsDay);
}

// Get the current hour
uint8_t SFE_UBLOX_GPS::getHour(uint16_t maxWait)
{
    if (moduleQueried.gpsHour == false)
        getPVT(maxWait);
    moduleQueried.gpsHour = false; // Since we are about to give this to user, mark this data as stale
    return (gpsHour);
}

// Get the current minute
uint8_t SFE_UBLOX_GPS::getMinute(uint16_t maxWait)
{
    if (moduleQueried.gpsMinute == false)
        getPVT(maxWait);
    moduleQueried.gpsMinute = false; // Since we are about to give this to user, mark this data as stale
    return (gpsMinute);
}

// Get the current second
uint8_t SFE_UBLOX_GPS::getSecond(uint16_t maxWait)
{
    if (moduleQueried.gpsSecond == false)
        getPVT(maxWait);
    moduleQueried.gpsSecond = false; // Since we are about to give this to user, mark this data as stale
    return (gpsSecond);
}

// Get the current date validity
bool SFE_UBLOX_GPS::getDateValid(uint16_t maxWait)
{
    if (moduleQueried.gpsDateValid == false)
        getPVT(maxWait);
    moduleQueried.gpsDateValid = false; // Since we are about to give this to user, mark this data as stale
    return (gpsDateValid);
}

// Get the current time validity
bool SFE_UBLOX_GPS::getTimeValid(uint16_t maxWait)
{
    if (moduleQueried.gpsTimeValid == false)
        getPVT(maxWait);
    moduleQueried.gpsTimeValid = false; // Since we are about to give this to user, mark this data as stale
    return (gpsTimeValid);
}

uint32_t SFE_UBLOX_GPS::getSpeedAccEst(uint16_t maxWait)
{
    if (moduleQueried.speedAccEst == false)
        getPVT(maxWait);
    moduleQueried.speedAccEst = false; // Since we are about to give this to user, mark this data as stale
    return (speedAccEst);
}

uint32_t SFE_UBLOX_GPS::getHeadingAccEst(uint16_t maxWait)
{
    if (moduleQueried.headingAccEst == false)
        getPVT(maxWait);
    moduleQueried.headingAccEst = false; // Since we are about to give this to user, mark this data as stale
    return (headingAccEst);
}

bool SFE_UBLOX_GPS::getInvalidLlh(uint16_t maxWait)
{
    if (moduleQueried.invalidLlh == false)
        getPVT(maxWait);
    moduleQueried.invalidLlh = false; // Since we are about to give this to user, mark this data as stale
    return (invalidLlh);
}

int32_t SFE_UBLOX_GPS::getHeadVeh(uint16_t maxWait)
{
    if (moduleQueried.headVeh == false)
        getPVT(maxWait);
    moduleQueried.headVeh = false; // Since we are about to give this to user, mark this data as stale
    return (headVeh);
}

int16_t SFE_UBLOX_GPS::getMagDec(uint16_t maxWait)
{
    if (moduleQueried.magDec == false)
        getPVT(maxWait);
    moduleQueried.magDec = false; // Since we are about to give this to user, mark this data as stale
    return (magDec);
}

uint16_t SFE_UBLOX_GPS::getMagAcc(uint16_t maxWait)
{
    if (moduleQueried.magAcc == false)
        getPVT(maxWait);
    moduleQueried.magAcc = false; // Since we are about to give this to user, mark this data as stale
    return (magAcc);
}

// Get the current millisecond
uint16_t SFE_UBLOX_GPS::getMillisecond(uint16_t maxWait)
{
    if (moduleQueried.gpsiTOW == false)
        getPVT(maxWait);
    moduleQueried.gpsiTOW = false; // Since we are about to give this to user, mark this data as stale
    return (gpsMillisecond);
}

// Get the current nanoseconds - includes milliseconds
int32_t SFE_UBLOX_GPS::getNanosecond(uint16_t maxWait)
{
    if (moduleQueried.gpsNanosecond == false)
        getPVT(maxWait);
    moduleQueried.gpsNanosecond = false; // Since we are about to give this to user, mark this data as stale
    return (gpsNanosecond);
}

// Get the latest Position/Velocity/Time solution and fill all global variables
bool SFE_UBLOX_GPS::getPVT(uint16_t maxWait)
{
    if (autoPVT && autoPVTImplicitUpdate)
    {
        // The GPS is automatically reporting, we just check whether we got unread data
        if (_printDebug == true)
        {
            printf("getPVT: Autoreporting");
        }
        checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT);
        return moduleQueried.all;
    }
    else if (autoPVT && !autoPVTImplicitUpdate)
    {
        if (_printDebug == true)
        {
            printf("getPVT: Exit immediately");
        }
        // Someone else has to call checkUblox for us...
        return (false);
    }
    else
    {
        if (_printDebug == true)
        {
            printf("getPVT: Polling");
        }
        // The GPS is not automatically reporting navigation position so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_NAV;
        packetCfg.id = UBX_NAV_PVT;
        packetCfg.len = 0;
        // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+MAX_PAYLOAD_SIZE = 84 bytes Note:now hard-coded in processUBX

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            if (_printDebug == true)
            {
                printf("getPVT: data was OVERWRITTEN by another NAV message (but that's OK)");
            }
            return (true);
        }

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            if (_printDebug == true)
            {
                printf("getPVT: data was OVERWRITTEN by a HNR message (and that's not OK)");
            }
            return (false);
        }

        if (_printDebug == true)
        {
            printf("getPVT: retVal: %s\n", statusString(retVal));
        }
        return (false);
    }
}

uint32_t SFE_UBLOX_GPS::getTimeOfWeek(uint16_t maxWait /* = 250*/)
{
    if (moduleQueried.gpsiTOW == false)
        getPVT(maxWait);
    moduleQueried.gpsiTOW = false; // Since we are about to give this to user, mark this data as stale
    return (timeOfWeek);
}

int32_t SFE_UBLOX_GPS::getHighResLatitude(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.highResLatitude == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.highResLatitude = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (highResLatitude);
}

int8_t SFE_UBLOX_GPS::getHighResLatitudeHp(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.highResLatitudeHp == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.highResLatitudeHp = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (highResLatitudeHp);
}

int32_t SFE_UBLOX_GPS::getHighResLongitude(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.highResLongitude == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.highResLongitude = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (highResLongitude);
}

int8_t SFE_UBLOX_GPS::getHighResLongitudeHp(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.highResLongitudeHp == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.highResLongitudeHp = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (highResLongitudeHp);
}

int32_t SFE_UBLOX_GPS::getElipsoid(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.elipsoid == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.elipsoid = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (elipsoid);
}

int8_t SFE_UBLOX_GPS::getElipsoidHp(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.elipsoidHp == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.elipsoidHp = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (elipsoidHp);
}

int32_t SFE_UBLOX_GPS::getMeanSeaLevel(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.meanSeaLevel == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.meanSeaLevel = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (meanSeaLevel);
}

int8_t SFE_UBLOX_GPS::getMeanSeaLevelHp(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.meanSeaLevelHp == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.meanSeaLevelHp = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (meanSeaLevelHp);
}

// getGeoidSeparation is currently redundant. The geoid separation seems to only be provided in NMEA GGA and GNS messages.
int32_t SFE_UBLOX_GPS::getGeoidSeparation(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.geoidSeparation == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.geoidSeparation = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (geoidSeparation);
}

uint32_t SFE_UBLOX_GPS::getHorizontalAccuracy(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.horizontalAccuracy == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.horizontalAccuracy = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (horizontalAccuracy);
}

uint32_t SFE_UBLOX_GPS::getVerticalAccuracy(uint16_t maxWait /* = 250*/)
{
    if (highResModuleQueried.verticalAccuracy == false)
        getHPPOSLLH(maxWait);
    highResModuleQueried.verticalAccuracy = false; // Since we are about to give this to user, mark this data as stale
    highResModuleQueried.all = false;

    return (verticalAccuracy);
}

bool SFE_UBLOX_GPS::getHPPOSLLH(uint16_t maxWait)
{
    if (autoHPPOSLLH && autoHPPOSLLHImplicitUpdate)
    {
        if (_printDebug == true)
        {
            printf("getHPPOSLLH: Autoreporting");
        }
        checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH);
        return highResModuleQueried.all;
    }
    else if (autoHPPOSLLH && !autoHPPOSLLHImplicitUpdate)
    {
        if (_printDebug == true)
        {
            printf("getHPPOSLLH: Exit immediately");
        }
        // Someone else has to call checkUblox for us...
        return (false);
    }
    else
    {
        if (_printDebug == true)
        {
            printf("getHPPOSLLH: Polling");
        }
        // The GPS is not automatically reporting navigation position so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_NAV;
        packetCfg.id = UBX_NAV_HPPOSLLH;
        packetCfg.len = 0;

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            return (true);
        }
        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            return (false);
        }

        return (false);
    }
}

uint16_t SFE_UBLOX_GPS::getGeometricDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.geometricDOP == false)
        getDOP(maxWait);
    dopModuleQueried.geometricDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (geometricDOP);
}

uint16_t SFE_UBLOX_GPS::getPositionDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.positionDOP == false)
        getDOP(maxWait);
    dopModuleQueried.positionDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (positionDOP);
}

uint16_t SFE_UBLOX_GPS::getTimeDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.timeDOP == false)
        getDOP(maxWait);
    dopModuleQueried.timeDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (timeDOP);
}

uint16_t SFE_UBLOX_GPS::getVerticalDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.verticalDOP == false)
        getDOP(maxWait);
    dopModuleQueried.verticalDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (verticalDOP);
}

uint16_t SFE_UBLOX_GPS::getHorizontalDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.horizontalDOP == false)
        getDOP(maxWait);
    dopModuleQueried.horizontalDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (horizontalDOP);
}

uint16_t SFE_UBLOX_GPS::getNorthingDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.northingDOP == false)
        getDOP(maxWait);
    dopModuleQueried.northingDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (northingDOP);
}

uint16_t SFE_UBLOX_GPS::getEastingDOP(uint16_t maxWait /* = 250*/)
{
    if (dopModuleQueried.eastingDOP == false)
        getDOP(maxWait);
    dopModuleQueried.eastingDOP = false; // Since we are about to give this to user, mark this data as stale
    dopModuleQueried.all = false;

    return (eastingDOP);
}

bool SFE_UBLOX_GPS::getDOP(uint16_t maxWait)
{
    if (autoDOP && autoDOPImplicitUpdate)
    {
        // The GPS is automatically reporting, we just check whether we got unread data
        checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_DOP);
        return dopModuleQueried.all;
    }
    else if (autoDOP && !autoDOPImplicitUpdate)
    {
        return (false);
    }
    else
    {

        // The GPS is not automatically reporting navigation position so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_NAV;
        packetCfg.id = UBX_NAV_DOP;
        packetCfg.len = 0;

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            return (true);
        }

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            return (false);
        }

        return (false);
    }
}

// Get the current 3D high precision positional accuracy - a fun thing to watch
// Returns a long representing the 3D accuracy in millimeters
uint32_t SFE_UBLOX_GPS::getPositionAccuracy(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
        return (0);                                                         // If command send fails then bail

    uint32_t tempAccuracy = extractLong(24); // We got a response, now extract a long beginning at a given position

    if ((tempAccuracy % 10) >= 5)
        tempAccuracy += 5; // Round fraction of mm up to next mm if .5 or above
    tempAccuracy /= 10;    // Convert 0.1mm units to mm

    return (tempAccuracy);
}

// Get the current latitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLatitude(uint16_t maxWait)
{
    if (moduleQueried.latitude == false)
        getPVT(maxWait);
    moduleQueried.latitude = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (latitude);
}

// Get the current longitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLongitude(uint16_t maxWait)
{
    if (moduleQueried.longitude == false)
        getPVT(maxWait);
    moduleQueried.longitude = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (longitude);
}

// Get the current altitude in mm according to ellipsoid model
int32_t SFE_UBLOX_GPS::getAltitude(uint16_t maxWait)
{
    if (moduleQueried.altitude == false)
        getPVT(maxWait);
    moduleQueried.altitude = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (altitude);
}

// Get the current altitude in mm according to mean sea level
// Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
// Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
int32_t SFE_UBLOX_GPS::getAltitudeMSL(uint16_t maxWait)
{
    if (moduleQueried.altitudeMSL == false)
        getPVT(maxWait);
    moduleQueried.altitudeMSL = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (altitudeMSL);
}

int32_t SFE_UBLOX_GPS::getHorizontalAccEst(uint16_t maxWait)
{
    if (moduleQueried.horizontalAccEst == false)
        getPVT(maxWait);
    moduleQueried.horizontalAccEst = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (horizontalAccEst);
}

int32_t SFE_UBLOX_GPS::getVerticalAccEst(uint16_t maxWait)
{
    if (moduleQueried.verticalAccEst == false)
        getPVT(maxWait);
    moduleQueried.verticalAccEst = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (verticalAccEst);
}

int32_t SFE_UBLOX_GPS::getNedNorthVel(uint16_t maxWait)
{
    if (moduleQueried.nedNorthVel == false)
        getPVT(maxWait);
    moduleQueried.nedNorthVel = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (nedNorthVel);
}

int32_t SFE_UBLOX_GPS::getNedEastVel(uint16_t maxWait)
{
    if (moduleQueried.nedEastVel == false)
        getPVT(maxWait);
    moduleQueried.nedEastVel = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (nedEastVel);
}

int32_t SFE_UBLOX_GPS::getNedDownVel(uint16_t maxWait)
{
    if (moduleQueried.nedDownVel == false)
        getPVT(maxWait);
    moduleQueried.nedDownVel = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (nedDownVel);
}

// Get the number of satellites used in fix
uint8_t SFE_UBLOX_GPS::getSIV(uint16_t maxWait)
{
    if (moduleQueried.SIV == false)
        getPVT(maxWait);
    moduleQueried.SIV = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (SIV);
}

// Get the current fix type
// 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
uint8_t SFE_UBLOX_GPS::getFixType(uint16_t maxWait)
{
    if (moduleQueried.fixType == false)
    {
        getPVT(maxWait);
    }
    moduleQueried.fixType = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (fixType);
}

// Get whether we have a valid fix (i.e within DOP & accuracy masks)
bool SFE_UBLOX_GPS::getGnssFixOk(uint16_t maxWait)
{
    if (moduleQueried.gnssFixOk == false)
        getPVT(maxWait);
    moduleQueried.gnssFixOk = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (gnssFixOk);
}

// Get whether differential corrections were applied
bool SFE_UBLOX_GPS::getDiffSoln(uint16_t maxWait)
{
    if (moduleQueried.diffSoln == false)
        getPVT(maxWait);
    moduleQueried.diffSoln = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (diffSoln);
}

// Get the carrier phase range solution status
// Useful when querying module to see if it has high-precision RTK fix
// 0=No solution, 1=Float solution, 2=Fixed solution
uint8_t SFE_UBLOX_GPS::getCarrierSolutionType(uint16_t maxWait)
{
    if (moduleQueried.carrierSolution == false)
        getPVT(maxWait);
    moduleQueried.carrierSolution = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (carrierSolution);
}

// Get whether head vehicle valid or not
bool SFE_UBLOX_GPS::getHeadVehValid(uint16_t maxWait)
{
    if (moduleQueried.headVehValid == false)
        getPVT(maxWait);
    moduleQueried.headVehValid = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (headVehValid);
}

// Get the ground speed in mm/s
int32_t SFE_UBLOX_GPS::getGroundSpeed(uint16_t maxWait)
{
    if (moduleQueried.groundSpeed == false)
        getPVT(maxWait);
    moduleQueried.groundSpeed = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (groundSpeed);
}

// Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
int32_t SFE_UBLOX_GPS::getHeading(uint16_t maxWait)
{
    if (moduleQueried.headingOfMotion == false)
        getPVT(maxWait);
    moduleQueried.headingOfMotion = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (headingOfMotion);
}

// Get the positional dillution of precision * 10^-2 (dimensionless)
uint16_t SFE_UBLOX_GPS::getPDOP(uint16_t maxWait)
{
    if (moduleQueried.pDOP == false)
        getPVT(maxWait);
    moduleQueried.pDOP = false; // Since we are about to give this to user, mark this data as stale
    moduleQueried.all = false;

    return (pDOP);
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GPS::getProtocolVersionHigh(uint16_t maxWait)
{
    if (moduleQueried.versionNumber == false)
        getProtocolVersion(maxWait);
    return (versionHigh);
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GPS::getProtocolVersionLow(uint16_t maxWait)
{
    if (moduleQueried.versionNumber == false)
        getProtocolVersion(maxWait);
    return (versionLow);
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
bool SFE_UBLOX_GPS::getProtocolVersion(uint16_t maxWait)
{
    // Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
    packetCfg.cls = UBX_CLASS_MON;
    packetCfg.id = UBX_MON_VER;

    packetCfg.len = 0;
    packetCfg.startingSpot = 40; // Start at first "extended software information" string

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
        return (false);                                                     // If command send fails then bail

    // Payload should now contain ~220 characters (depends on module type)

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("MON VER Payload:"));
    //   for (int location = 0; location < packetCfg.len; location++)
    //   {
    //     if (location % 30 == 0)
    //       _debugSerial->println();
    //     _debugSerial->write(payloadCfg[location]);
    //   }
    //   _debugSerial->println();
    // }

    // We will step through the payload looking at each extension field of 30 bytes
    for (uint8_t extensionNumber = 0; extensionNumber < 10; extensionNumber++)
    {
        // Now we need to find "PROTVER=18.00" in the incoming byte stream
        if ((payloadCfg[(30 * extensionNumber) + 0] == 'P') && (payloadCfg[(30 * extensionNumber) + 6] == 'R'))
        {
            versionHigh = (payloadCfg[(30 * extensionNumber) + 8] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 9] - '0');  // Convert '18' to 18
            versionLow = (payloadCfg[(30 * extensionNumber) + 11] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 12] - '0'); // Convert '00' to 00
            moduleQueried.versionNumber = true;                                                                                  // Mark this data as new

            return (true); // Success!
        }
    }

    return (false); // We failed
}

// Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GPS::flushPVT()
{
    // Mark all datums as stale (read before)
    moduleQueried.gpsiTOW = false;
    moduleQueried.gpsYear = false;
    moduleQueried.gpsMonth = false;
    moduleQueried.gpsDay = false;
    moduleQueried.gpsHour = false;
    moduleQueried.gpsMinute = false;
    moduleQueried.gpsSecond = false;
    moduleQueried.gpsDateValid = false;
    moduleQueried.gpsTimeValid = false;
    moduleQueried.gpsNanosecond = false;

    moduleQueried.all = false;
    moduleQueried.gnssFixOk = false;
    moduleQueried.diffSoln = false;
    moduleQueried.headVehValid = false;
    moduleQueried.longitude = false;
    moduleQueried.latitude = false;
    moduleQueried.altitude = false;
    moduleQueried.altitudeMSL = false;
    moduleQueried.SIV = false;
    moduleQueried.fixType = false;
    moduleQueried.carrierSolution = false;
    moduleQueried.groundSpeed = false;
    moduleQueried.headingOfMotion = false;
    moduleQueried.speedAccEst = false;
    moduleQueried.headingAccEst = false;
    moduleQueried.pDOP = false;
    moduleQueried.invalidLlh = false;
    moduleQueried.headVeh = false;
    moduleQueried.magDec = false;
    moduleQueried.magAcc = false;
}

// Mark all the HPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GPS::flushHPPOSLLH()
{
    // Mark all datums as stale (read before)
    highResModuleQueried.all = false;
    highResModuleQueried.highResLatitude = false;
    highResModuleQueried.highResLatitudeHp = false;
    highResModuleQueried.highResLongitude = false;
    highResModuleQueried.highResLongitudeHp = false;
    highResModuleQueried.elipsoid = false;
    highResModuleQueried.elipsoidHp = false;
    highResModuleQueried.meanSeaLevel = false;
    highResModuleQueried.meanSeaLevelHp = false;
    highResModuleQueried.geoidSeparation = false;
    highResModuleQueried.horizontalAccuracy = false;
    highResModuleQueried.verticalAccuracy = false;
    // moduleQueried.gpsiTOW = false; // this can arrive via HPPOS too.
}

// Mark all the DOP data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GPS::flushDOP()
{
    // Mark all DOPs as stale (read before)
    dopModuleQueried.all = false;
    dopModuleQueried.geometricDOP = false;
    dopModuleQueried.positionDOP = false;
    dopModuleQueried.timeDOP = false;
    dopModuleQueried.verticalDOP = false;
    dopModuleQueried.horizontalDOP = false;
    dopModuleQueried.northingDOP = false;
    dopModuleQueried.eastingDOP = false;
}

// Relative Positioning Information in NED frame
// Returns true if commands was successful
// Note:
//   RELPOSNED on the M8 is only 40 bytes long
//   RELPOSNED on the F9 is 64 bytes long and contains much more information
bool SFE_UBLOX_GPS::getRELPOSNED(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_RELPOSNED;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
        return (false);                                                     // If command send fails then bail

    // We got a response, now parse the bits

    uint16_t refStationID = extractInt(2);
    //_debugSerial->print(F("refStationID: "));
    //_debugSerial->println(refStationID));

    int32_t tempRelPos;

    tempRelPos = extractSignedLong(8);
    relPosInfo.relPosN = ((float)tempRelPos) / 100.0; // Convert cm to m

    tempRelPos = extractSignedLong(12);
    relPosInfo.relPosE = ((float)tempRelPos) / 100.0; // Convert cm to m

    tempRelPos = extractSignedLong(16);
    relPosInfo.relPosD = ((float)tempRelPos) / 100.0; // Convert cm to m

    if (packetCfg.len == 40)
    {
        // The M8 version does not contain relPosLength or relPosHeading
        relPosInfo.relPosLength = 0;
        relPosInfo.relPosHeading = 0;
    }
    else
    {
        relPosInfo.relPosLength = extractSignedLong(20);
        relPosInfo.relPosHeading = extractSignedLong(24);
    }

    if (packetCfg.len == 40)
    {
        relPosInfo.relPosHPN = payloadCfg[20];
        relPosInfo.relPosHPE = payloadCfg[21];
        relPosInfo.relPosHPD = payloadCfg[22];
        relPosInfo.relPosHPLength = 0; // The M8 version does not contain relPosHPLength
    }
    else
    {
        relPosInfo.relPosHPN = payloadCfg[32];
        relPosInfo.relPosHPE = payloadCfg[33];
        relPosInfo.relPosHPD = payloadCfg[34];
        relPosInfo.relPosHPLength = payloadCfg[35];
    }

    uint32_t tempAcc;

    if (packetCfg.len == 40)
    {
        tempAcc = extractLong(24);
        relPosInfo.accN = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
        tempAcc = extractLong(28);
        relPosInfo.accE = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
        tempAcc = extractLong(32);
        relPosInfo.accD = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
    }
    else
    {
        tempAcc = extractLong(36);
        relPosInfo.accN = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
        tempAcc = extractLong(40);
        relPosInfo.accE = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
        tempAcc = extractLong(44);
        relPosInfo.accD = ((float)tempAcc) / 10000.0; // Convert 0.1 mm to m
    }

    uint8_t flags;

    if (packetCfg.len == 40)
    {
        flags = payloadCfg[36];
    }
    else
    {
        flags = payloadCfg[60];
    }

    relPosInfo.gnssFixOk = flags & (1 << 0);
    relPosInfo.diffSoln = flags & (1 << 1);
    relPosInfo.relPosValid = flags & (1 << 2);
    relPosInfo.carrSoln = (flags & (0b11 << 3)) >> 3;
    relPosInfo.isMoving = flags & (1 << 5);
    relPosInfo.refPosMiss = flags & (1 << 6);
    relPosInfo.refObsMiss = flags & (1 << 7);

    return (true);
}

bool SFE_UBLOX_GPS::getEsfInfo(uint16_t maxWait)
{
    // Requesting Data from the receiver
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false); // If command send fails then bail

    // checkUblox();

    // payload should be loaded.
    imuMeas.version = extractByte(4);
    imuMeas.fusionMode = extractByte(12);
    ubloxSen.numSens = extractByte(15);

    // Individual Status Sensor in different function
    return (true);
}

//
bool SFE_UBLOX_GPS::getEsfIns(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_INS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false); // If command send fails then bail

    // checkUblox();

    // Validity of each sensor value below
    uint32_t validity = extractLong(0);

    imuMeas.xAngRateVald = (validity & 0x0100) >> 8;
    imuMeas.yAngRateVald = (validity & 0x0200) >> 9;
    imuMeas.zAngRateVald = (validity & 0x0400) >> 10;
    imuMeas.xAccelVald = (validity & 0x0800) >> 11;
    imuMeas.yAccelVald = (validity & 0x1000) >> 12;
    imuMeas.zAccelVald = (validity & 0x2000) >> 13;

    imuMeas.xAngRate = extractSignedLong(12); // 0.001 deg/s
    imuMeas.yAngRate = extractSignedLong(16); // 0.001 deg/s
    imuMeas.zAngRate = extractSignedLong(20); // 0.001 deg/s

    imuMeas.xAccel = extractSignedLong(24); // 0.01 m/s^2
    imuMeas.yAccel = extractSignedLong(28); // 0.01 m/s^2
    imuMeas.zAccel = extractSignedLong(32); // 0.01 m/s^2

    return (true);
}

//
bool SFE_UBLOX_GPS::getEsfDataInfo(uint16_t maxWait)
{

    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_MEAS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false); // If command send fails then bail

    // checkUblox();

    uint32_t timeStamp = extractLong(0);
    uint32_t flags = extractInt(4);

    uint8_t timeSent = flags & 0x03; // timeSent is 2-bit: 0 = none, 1 = on Ext0, 2 = on Ext1
    uint8_t timeEdge = (flags & 0x04) >> 2;
    uint8_t tagValid = (flags & 0x08) >> 3;
    uint8_t numMeas = (flags & 0xF800) >> 11;

    if (numMeas > DEF_NUM_SENS) // Truncate numMeas if required
        numMeas = DEF_NUM_SENS;

    uint8_t byteOffset = 4;

    for (uint8_t i = 0; i < numMeas; i++)
    {
        uint32_t bitField = extractLong(8 + (byteOffset * i));
        imuMeas.dataType[i] = (bitField & 0x3F000000) >> 24;
        imuMeas.data[i] = (bitField & 0xFFFFFF);
    }

    numMeas = (flags & 0xF800) >> 11; // Restore numMeas

    if (packetCfg.len > (8 + (4 * numMeas))) // The calibTtag is optional - only extract it if it is present
    {
        uint8_t startOfTtag = 8 + (4 * numMeas); // Calculate where the Ttag data starts
        if (numMeas > DEF_NUM_SENS)              // Truncate numMeas again if required
            numMeas = DEF_NUM_SENS;
        for (uint8_t i = 0; i < numMeas; i++)
        {
            imuMeas.dataTStamp[i] = extractLong(startOfTtag); // calibTtag is only appended once
        }
    }

    return (true);
}

bool SFE_UBLOX_GPS::getEsfRawDataInfo(uint16_t maxWait)
{

    // Need to know the number of sensor to get the correct data
    // Rate selected in UBX-CFG-MSG is not respected
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_RAW;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false); // If command send fails then bail

    // checkUblox();

    uint32_t bitField = extractLong(4);
    imuMeas.rawDataType = (bitField & 0xFF000000) >> 24;
    imuMeas.rawData = (bitField & 0xFFFFFF);

    imuMeas.rawTStamp = extractLong(8);

    return (true);
}

// Note: senor numbering starts at 1 (not 0)
sfe_ublox_status_e SFE_UBLOX_GPS::getSensState(uint8_t sensor, uint16_t maxWait)
{

    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (SFE_UBLOX_STATUS_FAIL); // If command send fails then bail

    ubloxSen.numSens = extractByte(15);

    if (sensor > ubloxSen.numSens)
        return (SFE_UBLOX_STATUS_OUT_OF_RANGE);

    // checkUblox();

    uint8_t offset = 4;

    // Only the last sensor value checked will remain.
    for (uint8_t i = 0; i < sensor; i++)
    {

        uint8_t sensorFieldOne = extractByte(16 + (offset * i));
        uint8_t sensorFieldTwo = extractByte(17 + (offset * i));
        ubloxSen.freq = extractByte(18 + (offset * i));
        uint8_t sensorFieldThr = extractByte(19 + offset * i);

        ubloxSen.senType = (sensorFieldOne & 0x3F);
        ubloxSen.isUsed = (sensorFieldOne & 0x40) >> 6;
        ubloxSen.isReady = (sensorFieldOne & 0x80) >> 7;

        ubloxSen.calibStatus = sensorFieldTwo & 0x03;
        ubloxSen.timeStatus = (sensorFieldTwo & 0xC) >> 2;

        ubloxSen.badMeas = (sensorFieldThr & 0x01);
        ubloxSen.badTag = (sensorFieldThr & 0x02) >> 1;
        ubloxSen.missMeas = (sensorFieldThr & 0x04) >> 2;
        ubloxSen.noisyMeas = (sensorFieldThr & 0x08) >> 3;
    }

    return (SFE_UBLOX_STATUS_SUCCESS);
}

bool SFE_UBLOX_GPS::getVehAtt(uint16_t maxWait)
{

    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_ATT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (SFE_UBLOX_STATUS_FAIL); // If command send fails then bail

    // checkUblox();

    vehAtt.roll = extractSignedLong(8);     // 0.00001 deg
    vehAtt.pitch = extractSignedLong(12);   // 0.00001 deg
    vehAtt.heading = extractSignedLong(16); // 0.00001 deg

    vehAtt.accRoll = extractLong(20);    // 0.00001 deg
    vehAtt.accPitch = extractLong(24);   // 0.00001 deg
    vehAtt.accHeading = extractLong(28); // 0.00001 deg

    return (true);
}

// Set the ECEF or Lat/Long coordinates of a receiver
// This imediately puts the receiver in TIME mode (fixed) and will begin outputting RTCM sentences if enabled
// This is helpful once an antenna's position has been established. See this tutorial: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station#gather-raw-gnss-data
//  For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
//  For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
bool SFE_UBLOX_GPS::setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_TMODE3;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current TimeMode3 settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false);

    packetCfg.len = 40;

    // Clear packet payload
    for (uint8_t x = 0; x < packetCfg.len; x++)
        payloadCfg[x] = 0;

    // customCfg should be loaded with poll response. Now modify only the bits we care about
    payloadCfg[2] = 2; // Set mode to fixed. Use ECEF (not LAT/LON/ALT).

    if (latLong == true)
        payloadCfg[3] = (uint8_t)(1 << 0); // Set mode to fixed. Use LAT/LON/ALT.

    // Set ECEF X or Lat
    payloadCfg[4] = (ecefXOrLat >> 8 * 0) & 0xFF; // LSB
    payloadCfg[5] = (ecefXOrLat >> 8 * 1) & 0xFF;
    payloadCfg[6] = (ecefXOrLat >> 8 * 2) & 0xFF;
    payloadCfg[7] = (ecefXOrLat >> 8 * 3) & 0xFF; // MSB

    // Set ECEF Y or Long
    payloadCfg[8] = (ecefYOrLon >> 8 * 0) & 0xFF; // LSB
    payloadCfg[9] = (ecefYOrLon >> 8 * 1) & 0xFF;
    payloadCfg[10] = (ecefYOrLon >> 8 * 2) & 0xFF;
    payloadCfg[11] = (ecefYOrLon >> 8 * 3) & 0xFF; // MSB

    // Set ECEF Z or Altitude
    payloadCfg[12] = (ecefZOrAlt >> 8 * 0) & 0xFF; // LSB
    payloadCfg[13] = (ecefZOrAlt >> 8 * 1) & 0xFF;
    payloadCfg[14] = (ecefZOrAlt >> 8 * 2) & 0xFF;
    payloadCfg[15] = (ecefZOrAlt >> 8 * 3) & 0xFF; // MSB

    // Set high precision parts
    payloadCfg[16] = ecefXOrLatHP;
    payloadCfg[17] = ecefYOrLonHP;
    payloadCfg[18] = ecefZOrAltHP;

    return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

bool SFE_UBLOX_GPS::setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latlong, uint16_t maxWait)
{
    return (setStaticPosition(ecefXOrLat, 0, ecefYOrLon, 0, ecefZOrAlt, 0, latlong, maxWait));
}

// Push (e.g.) RTCM data directly to the module
// Returns true if all numDataBytes were pushed successfully
// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
bool SFE_UBLOX_GPS::pushRawData(uint8_t *dataBytes, size_t numDataBytes)
{
    // I2C: split the data up into packets of i2cTransactionSize
    size_t bytesLeftToWrite = numDataBytes;
    size_t bytesWrittenTotal = 0;

    while (bytesLeftToWrite > 0)
    {
        size_t bytesToWrite; // Limit bytesToWrite to i2cTransactionSize
        if (bytesLeftToWrite > i2cTransactionSize)
            bytesToWrite = i2cTransactionSize;
        else
            bytesToWrite = bytesLeftToWrite;

        bytesWrittenTotal += bytesToWrite; // Update the totals
        bytesLeftToWrite -= bytesToWrite;
        dataBytes += bytesToWrite; // Point to fresh data

        if (bytesLeftToWrite > 0)
        {
            if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, dataBytes, 1, true) == PICO_ERROR_GENERIC) // Send a restart command. Do not release bus.
                return (false);                                                                         // Sensor did not ACK
        }
        else
        {
            if (i2c_write_blocking(_i2cPort, _gpsI2Caddress, dataBytes, 1, false) == PICO_ERROR_GENERIC) // We're done. Release bus.
                return (false);                                                                          // Sensor did not ACK
        }
    }

    return (bytesWrittenTotal == numDataBytes);
}

// Set the High Navigation Rate
// Returns true if the setHNRNavigationRate is successful
bool SFE_UBLOX_GPS::setHNRNavigationRate(uint8_t rate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_HNR;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current HNR settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false);

    // Load the new navigation rate into payloadCfg
    payloadCfg[0] = rate;

    // Update the navigation rate
    sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait); // We are only expecting an ACK

    // Adjust the I2C polling timeout based on update rate
    if (result == SFE_UBLOX_STATUS_DATA_SENT)
        i2cPollingWait = 1000 / (((int)rate) * 4); // This is the number of ms to wait between checks for new I2C data

    return (result == SFE_UBLOX_STATUS_DATA_SENT);
}

// Get the High Navigation Rate
// Returns 0 if the getHNRNavigationRate fails
uint8_t SFE_UBLOX_GPS::getHNRNavigationRate(uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_HNR;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current HNR settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (0);

    // Return the navigation rate
    return (payloadCfg[0]);
}

// In case no config access to the GPS is possible and HNR attitude is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoHNRAtt(bool enabled, bool implicitUpdate)
{
    bool changes = autoHNRAtt != enabled || autoHNRAttImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoHNRAtt = enabled;
        autoHNRAttImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic HNR attitude message generation by the GPS. This changes the way getHNRAtt
// works.
bool SFE_UBLOX_GPS::setAutoHNRAtt(bool enable, uint16_t maxWait)
{
    return setAutoHNRAtt(enable, true, maxWait);
}

// Enable or disable automatic HNR attitude message generation by the GPS. This changes the way getHNRAtt
// works.
bool SFE_UBLOX_GPS::setAutoHNRAtt(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_HNR;
    payloadCfg[1] = UBX_HNR_ATT;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoHNRAtt = enable;
        autoHNRAttImplicitUpdate = implicitUpdate;
    }
    hnrAttQueried = false; // Mark data as stale
    return ok;
}

// Get the HNR Attitude data
//  Returns true if the get HNR attitude is successful. Data is returned in hnrAtt
//  Note: if hnrAttQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrAtt immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Att is processed
bool SFE_UBLOX_GPS::getHNRAtt(uint16_t maxWait)
{
    if (autoHNRAtt && autoHNRAttImplicitUpdate)
    {
        checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_ATT);
        if (hnrAttQueried)
        {
            hnrAttQueried = false; // Mark data as stale as we assume the user will read it after this
            return true;
        }
        return false;
    }
    else if (autoHNRAtt && !autoHNRAttImplicitUpdate)
    {
        return (false);
    }
    else
    {
        // The GPS is not automatically reporting HNR attitude so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_HNR;
        packetCfg.id = UBX_HNR_ATT;
        packetCfg.len = 0;

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            return (false);
        }

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            return (true);
        }

        return (false);
    }

    return (false); // Trap. We should never get here...
}

// In case no config access to the GPS is possible and HNR vehicle dynamics is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoHNRDyn(bool enabled, bool implicitUpdate)
{
    bool changes = autoHNRDyn != enabled || autoHNRDynImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoHNRDyn = enabled;
        autoHNRDynImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GPS. This changes the way getHNRDyn
// works.
bool SFE_UBLOX_GPS::setAutoHNRDyn(bool enable, uint16_t maxWait)
{
    return setAutoHNRDyn(enable, true, maxWait);
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GPS. This changes the way getHNRDyn
// works.
bool SFE_UBLOX_GPS::setAutoHNRDyn(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_HNR;
    payloadCfg[1] = UBX_HNR_INS;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoHNRDyn = enable;
        autoHNRDynImplicitUpdate = implicitUpdate;
    }
    hnrDynQueried = false; // Mark data as stale
    return ok;
}

// Get the HNR vehicle dynamics data
//  Returns true if the get HNR vehicle dynamics is successful. Data is returned in hnrVehDyn
//  Note: if hnrDynQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrVehDyn immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Dyn is processed
bool SFE_UBLOX_GPS::getHNRDyn(uint16_t maxWait)
{
    if (autoHNRDyn && autoHNRDynImplicitUpdate)
    {
        // The GPS is automatically reporting, we just check whether we got unread data
        checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_INS);
        if (hnrDynQueried)
        {
            hnrDynQueried = false; // Mark data as stale as we assume the user will read it after this
            return true;
        }
        return false;
    }
    else if (autoHNRDyn && !autoHNRDynImplicitUpdate)
    {
        // Someone else has to call checkUblox for us...
        return (false);
    }
    else
    {
        // The GPS is not automatically reporting HNR vehicle dynamics so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_HNR;
        packetCfg.id = UBX_HNR_INS;
        packetCfg.len = 0;

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            return (false);
        }

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            return (true);
        }

        return (false);
    }

    return (false); // Trap. We should never get here...
}

// In case no config access to the GPS is possible and HNR PVT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GPS::assumeAutoHNRPVT(bool enabled, bool implicitUpdate)
{
    bool changes = autoHNRPVT != enabled || autoHNRPVTImplicitUpdate != implicitUpdate;
    if (changes)
    {
        autoHNRPVT = enabled;
        autoHNRPVTImplicitUpdate = implicitUpdate;
    }
    return changes;
}

// Enable or disable automatic HNR PVT message generation by the GPS. This changes the way getHNRPVT
// works.
bool SFE_UBLOX_GPS::setAutoHNRPVT(bool enable, uint16_t maxWait)
{
    return setAutoHNRPVT(enable, true, maxWait);
}

// Enable or disable automatic HNR PVT message generation by the GPS. This changes the way getHNRPVT
// works.
bool SFE_UBLOX_GPS::setAutoHNRPVT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_MSG;
    packetCfg.len = 3;
    packetCfg.startingSpot = 0;
    payloadCfg[0] = UBX_CLASS_HNR;
    payloadCfg[1] = UBX_HNR_PVT;
    payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

    bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        autoHNRPVT = enable;
        autoHNRPVTImplicitUpdate = implicitUpdate;
    }
    hnrPVTQueried = false; // Mark data as stale
    return ok;
}

// Get the HNR PVT data
//  Returns true if the get HNR PVT is successful. Data is returned in hnrPVT
//  Note: if hnrPVTQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrPVT immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR PVT is processed
bool SFE_UBLOX_GPS::getHNRPVT(uint16_t maxWait)
{
    if (autoHNRPVT && autoHNRPVTImplicitUpdate)
    {
        // The GPS is automatically reporting, we just check whether we got unread data
        checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_PVT);
        if (hnrPVTQueried)
        {
            hnrPVTQueried = false; // Mark data as stale as we assume the user will read it after this
            return true;
        }
        return false;
    }
    else if (autoHNRPVT && !autoHNRPVTImplicitUpdate)
    {
        // Someone else has to call checkUblox for us...
        return (false);
    }
    else
    {
        // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
        packetCfg.cls = UBX_CLASS_HNR;
        packetCfg.id = UBX_HNR_PVT;
        packetCfg.len = 0;

        // The data is parsed as part of processing the response
        sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

        if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
            return (true);

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_NAV))
        {
            return (false);
        }

        if ((retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.cls == UBX_CLASS_HNR))
        {
            return (true);
        }

        return (false);
    }

    return (false); // Trap. We should never get here...
}
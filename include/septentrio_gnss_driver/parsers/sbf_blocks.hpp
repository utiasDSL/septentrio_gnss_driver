// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include <cstdint>

//! Using maximum value of NR_OF_LOGICALCHANNELS for SBF definitions
static const uint8_t NR_OF_LOGICALCHANNELS = 80;
//! Inmarsat is a British satellite telecommunications company.
static const uint8_t MAX_NB_INMARSATCHANNELS = 1;
//! Using maximum value of MAX_NR_OF_SIGNALS_PER_SATELLITE for SBF definitions
static const uint8_t MAX_NR_OF_SIGNALS_PER_SATELLITE = 7;
//! Using maximum value of NR_OF_ANTENNAS for SBF definitions
static const uint8_t NR_OF_ANTENNAS = 3;
//! Maximum number of antennas that mosaic etc. can handle
static const uint8_t MAXSB_NBRANTENNA = 4;
//! Max number of bytes that ChannelSatInfo sub-block can consist of
static const uint8_t MAXSB_CHANNELSATINFO =
    (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS);
//! Max number of bytes that ChannelStateInfo sub-block can consist of
static const uint16_t MAXSB_CHANNELSTATEINFO =
    (MAXSB_CHANNELSATINFO * MAXSB_NBRANTENNA);
//! Max number of bytes that MeasEpochChannelType1 sub-block can consist of
static const uint8_t MAXSB_MEASEPOCH_T1 =
    (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS);
//! Max number of bytes that MeasEpochChannelType2 sub-block can consist of
static const uint16_t MAXSB_MEASEPOCH_T2 =
    ((MAXSB_MEASEPOCH_T1) *
     (((MAX_NR_OF_SIGNALS_PER_SATELLITE) * (NR_OF_ANTENNAS)) - 1));
//! Max number of vector info sub-blocks
static const uint8_t MAXSB_NBVECTORINFO = 30;

//! 0x24 is ASCII for $ - 1st byte in each message
static const uint8_t SBF_SYNC_1 = 0x24;
//! 0x40 is ASCII for @ - 2nd byte to indicate SBF block
static const uint8_t SBF_SYNC_2 = 0x40;

// C++
#include <algorithm>
#include <type_traits>
// Boost
#include <boost/spirit/include/qi.hpp>
// ROSaic
#ifdef ROS2
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#endif
#ifdef ROS1
#include <septentrio_gnss_driver/abstraction/typedefs_ros1.hpp>
#endif
#include <septentrio_gnss_driver/parsers/parsing_utilities.hpp>

/**
 * @file sbf_structs.hpp
 * @brief Declares and defines structs into which SBF blocks are unpacked then
 * shipped to handler functions
 * @date 17/08/20
 */

/**
 * @class ChannelStateInfo
 * @brief Struct for the SBF sub-block "ChannelStateInfo"
 */
struct ChannelStateInfo
{
    uint8_t antenna;
    uint16_t tracking_status;
    uint16_t pvt_status;
    uint16_t pvt_info;
};

/**
 * @class ChannelSatInfo
 * @brief Struct for the SBF sub-block "ChannelSatInfo"
 */
struct ChannelSatInfo
{
    uint8_t sv_id;
    uint8_t freq_nr;
    uint16_t az_rise_set;
    uint16_t health_status;
    int8_t elev;
    uint8_t n2;
    uint8_t rx_channel;

    std::vector<ChannelStateInfo> stateInfo;
};

/**
 * @class ChannelStatus
 * @brief Struct for the SBF block "ChannelStatus"
 */
struct ChannelStatus
{
    BlockHeaderMsg block_header;

    uint8_t n;
    uint8_t sb1_length;
    uint8_t sb2_length;

    std::vector<ChannelSatInfo> satInfo;
};

/**
 * @class DOP
 * @brief Struct for the SBF block "DOP"
 */
struct Dop
{
    BlockHeaderMsg block_header;

    uint8_t nr_sv;
    double pdop;
    double tdop;
    double hdop;
    double vdop;
    float hpl;
    float vpl;
};

/**
 * @class ReceiverSetupT
 * @brief Struct for the SBF block "ReceiverSetup"
 */
struct ReceiverSetupT
{
    BlockHeaderMsg block_header;

    std::string marker_name;
    std::string marker_number;
    std::string observer;
    std::string agency;
    std::string rx_serial_number;
    std::string rx_name;
    std::string rx_version;
    std::string ant_serial_nbr;
    std::string ant_type;
    float delta_h; /* [m] */
    float delta_e; /* [m] */
    float delta_n; /* [m] */
    std::string marker_type;
    std::string gnss_fw_version;
    std::string product_name;
    double latitude;
    double longitude;
    float height;
    std::string station_code;
    uint8_t monument_idx;
    uint8_t receiver_idx;
    std::string country_code;

    void toROSMsg(ReceiverSetupMsg &msg) const
    {
        msg.block_header = this->block_header;
        msg.marker_name = this->marker_name;
        msg.marker_nbr = this->marker_number;
        msg.observer = this->observer;
        msg.agency = this->agency;
        msg.rx_serial_nbr = this->rx_serial_number;
        msg.rx_name = this->rx_name;
        msg.rx_version = this->rx_version;
        msg.ant_serial_nbr = this->ant_serial_nbr;
        msg.ant_type = this->ant_type;
        msg.delta_h = this->delta_h;
        msg.delta_e = this->delta_e;
        msg.delta_n = this->delta_n;
        msg.marker_type = this->marker_type;
        msg.gnss_fw_version = this->gnss_fw_version;
        msg.product_name = this->product_name;
        msg.latitude = this->latitude;
        msg.longitude = this->longitude;
        msg.height = this->height;
        msg.station_code = this->station_code;
        msg.monument_idx = this->monument_idx;
        msg.receiver_idx = this->receiver_idx;
        msg.country_code = this->country_code;
    }
};

/**
 * @class QualityIndT
 * @brief Struct for the SBF block "QualityInd"
 */
struct QualityIndT
{
    BlockHeaderMsg block_header;

    uint8_t n = 0;

    std::vector<uint16_t> indicators;

    void toROSMsg(QuantityIndMsg& msg) const
    {
        msg.block_header = this->block_header;
        msg.n = this->n;
        for (const auto& ind : this->indicators)
        {
            msg.indicators.push_back(ind);
        }
    }
};

/**
 * @brief Struct for the SBF sub-block "AGCState"
 */
struct AgcStateT
{
    uint8_t frontend_id;
    int8_t gain;
    uint8_t sample_var;
    uint8_t blanking_stat;
};

/**
 * @class ReceiverStatusT
 * @brief Struct for the SBF block "ReceiverStatus"
 */
struct ReceiverStatusT
{
    BlockHeaderMsg block_header;

    uint8_t cpu_load;
    uint8_t ext_error;
    uint32_t up_time;
    uint32_t rx_status;
    uint32_t rx_error;
    uint8_t n;
    uint8_t sb_length;
    uint8_t cmd_count;
    uint8_t temperature;

    std::vector<AgcStateT> agc_state;

    void toROSMsg(ReceiverStatusMsg& ros_msg) const
    {
        ros_msg.block_header = this->block_header;
        ros_msg.cpu_load = this->cpu_load;
        ros_msg.ext_error = this->ext_error;
        ros_msg.up_time = this->up_time;
        ros_msg.rx_error = this->rx_error;
        ros_msg.rx_state = this->rx_status;
        ros_msg.n = this->n;
        ros_msg.sb_length = this->sb_length;
        ros_msg.cmd_count = this->cmd_count;
        ros_msg.temperature = this->temperature;
        for (const auto& [frontend_id, gain, sample_var, blanking_stat] : this->agc_state)
        {
            AGCStateMsg agc_msg;
            agc_msg.blanking_stat = blanking_stat;
            agc_msg.sample_var = sample_var;
            agc_msg.front_end_id = frontend_id;
            agc_msg.gain = gain;
            ros_msg.agc_states.emplace_back(agc_msg);
        }
    }
};

/**
 * @brief CRC look-up table for fast computation of the 16-bit CRC for SBF blocks.
 *
 * Provided by Septenrio (c) 2020 Septentrio N.V./S.A., Belgium.
 */
static const std::array<uint16_t, 256> CRC_LOOK_UP = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129,
    0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252,
    0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
    0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861,
    0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5,
    0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b,
    0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9,
    0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3,
    0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676,
    0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,
    0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
    0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36,
    0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

namespace qi = boost::spirit::qi;

/**
 * setDoNotUse
 * @brief Sets scalar to Do-Not-Use value
 */
template <typename Val>
void setDoNotUse(Val& s)
{
    static_assert(
        std::is_same<uint16_t, Val>::value || std::is_same<uint32_t, Val>::value ||
        std::is_same<uint64_t, Val>::value || std::is_same<float, Val>::value ||
        std::is_same<double, Val>::value);

    if (std::is_same<uint16_t, Val>::value)
    {
        s = 65535;
    } else if (std::is_same<uint32_t, Val>::value)
    {
        s = 4294967295ul;
    } else if (std::is_same<float, Val>::value)
    {
        s = std::numeric_limits<float>::quiet_NaN();
    } else if (std::is_same<double, Val>::value)
    {
        s = std::numeric_limits<double>::quiet_NaN();
    }
    // TODO add more
}

/**
 * doNotUseToNaN
 * @brief Sets scalar to Do-Not-Use value to NaN
 */
template <typename Val>
void doNotUseToNaN(Val& s)
{
    static_assert(std::is_same<float, Val>::value ||
                  std::is_same<double, Val>::value);

    if constexpr (std::is_same<float, Val>::value)
    {
        if (s == -2e10f)
            s = std::numeric_limits<float>::quiet_NaN();
    } else if constexpr (std::is_same<double, Val>::value)
    {
        if (s == -2e10)
            s = std::numeric_limits<double>::quiet_NaN();
    }
}

/**
 * qiLittleEndianParser
 * @brief Qi little endian parsers for numeric values
 */
template <typename It, typename Val>
void qiLittleEndianParser(It& it, Val& val)
{
    static_assert(
        std::is_same<int8_t, Val>::value || std::is_same<uint8_t, Val>::value ||
        std::is_same<int16_t, Val>::value || std::is_same<uint16_t, Val>::value ||
        std::is_same<int32_t, Val>::value || std::is_same<uint32_t, Val>::value ||
        std::is_same<int64_t, Val>::value || std::is_same<uint64_t, Val>::value ||
        std::is_same<float, Val>::value || std::is_same<double, Val>::value);

    if constexpr (std::is_same<int8_t, Val>::value)
    {
        qi::parse(it, it + 1, qi::char_, val);
    } else if constexpr (std::is_same<uint8_t, Val>::value)
    {
        qi::parse(it, it + 1, qi::byte_, val);
    } else if constexpr ((std::is_same<int16_t, Val>::value) ||
                         (std::is_same<uint16_t, Val>::value))
    {
        qi::parse(it, it + 2, qi::little_word, val);
    } else if constexpr ((std::is_same<int32_t, Val>::value) ||
                         (std::is_same<uint32_t, Val>::value))
    {
        qi::parse(it, it + 4, qi::little_dword, val);
    } else if constexpr ((std::is_same<int64_t, Val>::value) ||
                         (std::is_same<uint64_t, Val>::value))
    {
        qi::parse(it, it + 8, qi::little_qword, val);
    } else if constexpr (std::is_same<float, Val>::value)
    {
        qi::parse(it, it + 4, qi::little_bin_float, val);
        doNotUseToNaN(val);
    } else if constexpr (std::is_same<double, Val>::value)
    {
        qi::parse(it, it + 8, qi::little_bin_double, val);
        doNotUseToNaN(val);
    }
}

/**
 * qiCharsToStringParser
 * @brief Qi parser for char array to string
 */
template <typename It>
void qiCharsToStringParser(It& it, std::string& val, std::size_t num)
{
    val.clear();
    qi::parse(it, it + num, qi::repeat(num)[qi::char_], val);
    // remove string termination characters '\0'
    val.erase(std::remove(val.begin(), val.end(), '\0'), val.end());
}

/**
 * BlockHeaderParser
 * @brief Qi based parser for the SBF block "BlockHeader" plus receiver time stamp
 */
template <typename It, typename Hdr>
[[nodiscard]] bool BlockHeaderParser(ROSaicNodeBase* node, It& it, Hdr& block_header)
{
    qiLittleEndianParser(it, block_header.sync_1);
    if (block_header.sync_1 != SBF_SYNC_1)
    {
        node->log(log_level::ERROR, "BlockHeaderParser error: Wrong sync byte 1.");
        return false;
    }
    qiLittleEndianParser(it, block_header.sync_2);
    if (block_header.sync_2 != SBF_SYNC_2)
    {
        node->log(log_level::ERROR, "BlockHeaderParser error: Wrong sync byte 2.");
        return false;
    }
    qiLittleEndianParser(it, block_header.crc);
    uint16_t ID;
    qiLittleEndianParser(it, ID);
    block_header.id = ID & 8191;      // lower 13 bits are id
    block_header.revision = ID >> 13; // upper 3 bits are revision
    qiLittleEndianParser(it, block_header.length);
    qiLittleEndianParser(it, block_header.tow);
    qiLittleEndianParser(it, block_header.wnc);
    return true;
}

/**
 * ChannelStateInfoParser
 * @brief Qi based parser for the SBF sub-block "ChannelStateInfo"
 */
template <typename It>
void ChannelStateInfoParser(It& it, ChannelStateInfo& msg, uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.antenna);
    ++it; // reserved
    qiLittleEndianParser(it, msg.tracking_status);
    qiLittleEndianParser(it, msg.pvt_status);
    qiLittleEndianParser(it, msg.pvt_info);
    std::advance(it, sb2_length - 8); // skip padding
};

/**
 * ChannelSatInfoParser
 * @brief Qi based parser or the SBF sub-block "ChannelSatInfo"
 */
template <typename It>
[[nodiscard]] bool ChannelSatInfoParser(ROSaicNodeBase* node, It& it,
                                        ChannelSatInfo& msg, uint8_t sb1_length,
                                        uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    std::advance(it, 2); // reserved
    qiLittleEndianParser(it, msg.az_rise_set);
    qiLittleEndianParser(it, msg.health_status);
    qiLittleEndianParser(it, msg.elev);
    qiLittleEndianParser(it, msg.n2);
    if (msg.n2 > MAXSB_CHANNELSTATEINFO)
    {
        node->log(log_level::ERROR, "ChannelSatInfoParser error: Too many ChannelStateInfo " +
                                        std::to_string(msg.n2));
        return false;
    }
    qiLittleEndianParser(it, msg.rx_channel);
    ++it;                              // reserved
    std::advance(it, sb1_length - 12); // skip padding
    msg.stateInfo.resize(msg.n2);
    for (auto& stateInfo : msg.stateInfo)
    {
        ChannelStateInfoParser(it, stateInfo, sb2_length);
    }
    return true;
};

/**
 * ChannelStatusParser
 * @brief Qi based parser for the SBF block "ChannelStatus"
 */
template <typename It>
[[nodiscard]] bool ChannelStatusParser(ROSaicNodeBase* node, It it, It itEnd,
                                       ChannelStatus& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4013)
    {
        node->log(log_level::ERROR, "ChannelStatusParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_CHANNELSATINFO)
    {
        node->log(log_level::ERROR,
                  "ChannelStatusParser error: Too many ChannelSatInfo " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb1_length);
    qiLittleEndianParser(it, msg.sb2_length);
    std::advance(it, 3); // reserved
    msg.satInfo.resize(msg.n);
    for (auto& satInfo : msg.satInfo)
    {
        if (!ChannelSatInfoParser(node, it, satInfo, msg.sb1_length, msg.sb2_length))
            return false;
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ChannelStatusParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * DOPParser
 * @brief Qi based parser for the SBF block "DOP"
 */
template <typename It>
[[nodiscard]] bool DOPParser(ROSaicNodeBase* node, It it, It itEnd, Dop& msg)
{

    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4001)
    {
        node->log(log_level::ERROR, "DOPParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.nr_sv);
    ++it; // reserved
    uint16_t temp;
    qiLittleEndianParser(it, temp);
    msg.pdop = temp / 100.0;
    qiLittleEndianParser(it, temp);
    msg.tdop = temp / 100.0;
    qiLittleEndianParser(it, temp);
    msg.hdop = temp / 100.0;
    qiLittleEndianParser(it, temp);
    msg.vdop = temp / 100.0;
    qiLittleEndianParser(it, msg.hpl);
    qiLittleEndianParser(it, msg.vpl);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "DOPParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * MeasEpochChannelType2Parser
 * @brief Qi based parser for the SBF sub-block "MeasEpochChannelType2"
 */
template <typename It>
void MeasEpochChannelType2Parser(It& it, MeasEpochChannelType2Msg& msg,
                                 uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.type);
    qiLittleEndianParser(it, msg.lock_time);
    qiLittleEndianParser(it, msg.cn0);
    qiLittleEndianParser(it, msg.offsets_msb);
    qiLittleEndianParser(it, msg.carrier_msb);
    qiLittleEndianParser(it, msg.obs_info);
    qiLittleEndianParser(it, msg.code_offset_lsb);
    qiLittleEndianParser(it, msg.carrier_lsb);
    qiLittleEndianParser(it, msg.doppler_offset_lsb);
    std::advance(it, sb2_length - 12); // skip padding
};

/**
 * MeasEpochChannelType1Parser
 * @brief Qi based parser for the SBF sub-block "MeasEpochChannelType1"
 */
template <typename It>
[[nodiscard]] bool MeasEpochChannelType1Parser(ROSaicNodeBase* node, It& it,
                                               MeasEpochChannelType1Msg& msg,
                                               uint8_t sb1_length,
                                               uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.rx_channel);
    qiLittleEndianParser(it, msg.type);
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.misc);
    qiLittleEndianParser(it, msg.code_lsb);
    qiLittleEndianParser(it, msg.doppler);
    qiLittleEndianParser(it, msg.carrier_lsb);
    qiLittleEndianParser(it, msg.carrier_msb);
    qiLittleEndianParser(it, msg.cn0);
    qiLittleEndianParser(it, msg.lock_time);
    qiLittleEndianParser(it, msg.obs_info);
    qiLittleEndianParser(it, msg.n2);
    std::advance(it, sb1_length - 20); // skip padding
    if (msg.n2 > MAXSB_MEASEPOCH_T2)
    {
        node->log(log_level::ERROR, "MeasEpochChannelType1Parser error: Too many MeasEpochChannelType2 " +
                                        std::to_string(msg.n2));
        return false;
    }
    msg.type2.resize(msg.n2);
    for (auto& type2 : msg.type2)
    {
        MeasEpochChannelType2Parser(it, type2, sb2_length);
    }
    return true;
};

/**
 * MeasEpochParser
 * @brief Qi based parser for the SBF block "MeasEpoch"
 */
template <typename It>
[[nodiscard]] bool MeasEpochParser(ROSaicNodeBase* node, It it, It itEnd,
                                   MeasEpochMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4027)
    {
        node->log(log_level::ERROR, "MeasEpochParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_MEASEPOCH_T1)
    {
        node->log(log_level::ERROR, "MeasEpochParser error: Too many MeasEpochChannelType1 " +
                                        std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb1_length);
    qiLittleEndianParser(it, msg.sb2_length);
    qiLittleEndianParser(it, msg.common_flags);
    if (msg.block_header.revision > 0)
        qiLittleEndianParser(it, msg.cum_clk_jumps);
    ++it; // reserved
    msg.type1.resize(msg.n);
    for (auto& type1 : msg.type1)
    {
        if (!MeasEpochChannelType1Parser(node, it, type1, msg.sb1_length,
                                         msg.sb2_length))
            return false;
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "MeasEpochParser error: iterator past end.");
        return false;
    }
    return true;
};


/**
 * MeasExtraChannelParser
 * @brief Qi based parser for the SBF sub-block "MeasExtraChannel"
 */
template <typename It>
[[nodiscard]] bool MeasExtraChannelParser(ROSaicNodeBase* node, It& it,
                                          MeasExtraChannelMsg& msg,
                                          uint8_t sb_length,
                                          uint8_t revision)
{
    qiLittleEndianParser(it, msg.rx_channel);
    qiLittleEndianParser(it, msg.type);
    qiLittleEndianParser(it, msg.mp_correction);
    qiLittleEndianParser(it, msg.smoothing_correction);
    qiLittleEndianParser(it, msg.code_var);
    qiLittleEndianParser(it, msg.carrier_var);
    qiLittleEndianParser(it, msg.lock_time);

    uint8_t skip_bytes = 12;
    if (revision > 0)
    {
        qiLittleEndianParser(it, msg.cum_loss_cont);
        qiLittleEndianParser(it, msg.carrier_var);
        skip_bytes += 2; // skip padding
    }

    if (revision > 1)
    {
        qiLittleEndianParser(it, msg.info);
        skip_bytes += 1; // skip padding
    }

    if (revision > 2)
    {
        qiLittleEndianParser(it, msg.misc);
        // ToDo: this loops strange,
        // ToDo: but it seems that either the manuel or the firmware is inconsistant
        skip_bytes += 2;
    }
    std::advance(it, sb_length - skip_bytes); // skip padding
    return true;
};

/**
 * MeasExtraParser
 * @brief Qi based parser for the SBF block "MeasExtra"
 */
template <typename It>
[[nodiscard]] bool MeasExtraParser(ROSaicNodeBase* node, It it, It itEnd,
                                   MeasExtraMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4000)
    {
        node->log(log_level::ERROR, "MeasExtraParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    qiLittleEndianParser(it, msg.doppler_var_factor);

    msg.channels.resize(msg.n);
    for (auto& channel : msg.channels)
    {
        if (!MeasExtraChannelParser(node, it, channel, msg.sb_length, msg.block_header.revision))
            return false;
    }
    if (it > itEnd)
    {
        std::cout << "Iterator offset = " << std::distance(itEnd, it) << "\n";
        node->log(log_level::ERROR, "MeasExtraParser error: iterator past end. Rev: " + std::to_string(msg.block_header.revision));
        node->log(log_level::ERROR, "MeasExtraParser error: n " + std::to_string(msg.n));
        node->log(log_level::ERROR, "MeasExtraParser error: sb_length " + std::to_string(msg.sb_length));

        return false;
    }
    return true;
};

/**
 * GALAuthStatus
 * @brief Qi based parser for the SBF block "GALAuthStatus"
 */
template <typename It>
[[nodiscard]] bool GalAuthStatusParser(ROSaicNodeBase* node, It it, It itEnd,
                                       GalAuthStatusMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4245)
    {
        node->log(log_level::ERROR, "GalAuthStatusParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.osnma_status);
    qiLittleEndianParser(it, msg.trusted_time_delta);
    qiLittleEndianParser(it, msg.gal_active_mask);
    qiLittleEndianParser(it, msg.gal_authentic_mask);
    qiLittleEndianParser(it, msg.gps_active_mask);
    qiLittleEndianParser(it, msg.gps_authentic_mask);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GalAuthStatusParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * RFBandParser
 * @brief Qi based parser for the SBF sub-block "RFBand"
 */
template <typename It>
void RfBandParser(It& it, RfBandMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.frequency);
    qiLittleEndianParser(it, msg.bandwidth);
    qiLittleEndianParser(it, msg.info);
    qiLittleEndianParser(it, msg.power);
    std::advance(it, sb_length - 8); // skip padding
};

/**
 * RFStatusParser
 * @brief Qi based parser for the SBF block "RFStatus"
 */
template <typename It>
[[nodiscard]] bool RfStatusParser(ROSaicNodeBase* node, It it, It itEnd,
                                  RfStatusMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4092)
    {
        node->log(log_level::ERROR, "RfStatusParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    qiLittleEndianParser(it, msg.flags);
    std::advance(it, 3); // reserved
    msg.rfband.resize(msg.n);
    for (auto& rfband : msg.rfband)
    {
        RfBandParser(it, rfband, msg.sb_length);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "RfStatusParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * ReceiverSetupParser
 * @brief Qi based parser for the SBF block "ReceiverSetup"
 */
template <typename It>
[[nodiscard]] bool ReceiverSetupParser(ROSaicNodeBase* node, It it, It itEnd,
                                       ReceiverSetupT& msg, ReceiverSetupMsg& ros_msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5902)
    {
        node->log(log_level::ERROR, "ReceiverSetupParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    std::advance(it, 2); // reserved
    qiCharsToStringParser(it, msg.marker_name, 60);
    qiCharsToStringParser(it, msg.marker_number, 20);
    qiCharsToStringParser(it, msg.observer, 20);
    qiCharsToStringParser(it, msg.agency, 40);
    qiCharsToStringParser(it, msg.rx_serial_number, 20);
    qiCharsToStringParser(it, msg.rx_name, 20);
    qiCharsToStringParser(it, msg.rx_version, 20);
    qiCharsToStringParser(it, msg.ant_serial_nbr, 20);
    qiCharsToStringParser(it, msg.ant_type, 20);
    qiLittleEndianParser(it, msg.delta_h);
    qiLittleEndianParser(it, msg.delta_e);
    qiLittleEndianParser(it, msg.delta_n);
    if (msg.block_header.revision > 0)
        qiCharsToStringParser(it, msg.marker_type, 20);
    if (msg.block_header.revision > 1)
        qiCharsToStringParser(it, msg.gnss_fw_version, 40);
    if (msg.block_header.revision > 2)
        qiCharsToStringParser(it, msg.product_name, 40);
    if (msg.block_header.revision > 3)
    {
        qiLittleEndianParser(it, msg.latitude);
        qiLittleEndianParser(it, msg.longitude);
        qiLittleEndianParser(it, msg.height);
        qiCharsToStringParser(it, msg.station_code, 10);
        qiLittleEndianParser(it, msg.monument_idx);
        qiLittleEndianParser(it, msg.receiver_idx);
        qiCharsToStringParser(it, msg.country_code, 3);
    } else
    {
        setDoNotUse(msg.latitude);
        setDoNotUse(msg.longitude);
        setDoNotUse(msg.height);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ReceiverSetupParser error: iterator past end.");
        return false;
    }
    msg.toROSMsg(ros_msg);
    return true;
};

/**
 * ReceiverTimeParser
 * @brief Struct for the SBF block "ReceiverTime"
 */
template <typename It>
[[nodiscard]] bool ReceiverTimesParser(ROSaicNodeBase* node, It it, It itEnd,
                                       ReceiverTimeMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5914)
    {
        node->log(log_level::ERROR, "ReceiverTimesParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.utc_year);
    qiLittleEndianParser(it, msg.utc_month);
    qiLittleEndianParser(it, msg.utc_day);
    qiLittleEndianParser(it, msg.utc_hour);
    qiLittleEndianParser(it, msg.utc_min);
    qiLittleEndianParser(it, msg.utc_second);
    qiLittleEndianParser(it, msg.delta_ls);
    qiLittleEndianParser(it, msg.sync_level);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ReceiverTimesParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PVTCartesianParser
 * @brief Qi based parser for the SBF block "PVTCartesian"
 */
template <typename It>
[[nodiscard]] bool PVTCartesianParser(ROSaicNodeBase* node, It it, It itEnd,
                                      PVTCartesianMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4006)
    {
        node->log(log_level::ERROR, "PVTCartesianParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.vx);
    qiLittleEndianParser(it, msg.vy);
    qiLittleEndianParser(it, msg.vz);
    qiLittleEndianParser(it, msg.cog);
    qiLittleEndianParser(it, msg.rx_clk_bias);
    qiLittleEndianParser(it, msg.rx_clk_drift);
    qiLittleEndianParser(it, msg.time_system);
    qiLittleEndianParser(it, msg.datum);
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.wa_corr_info);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.mean_corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    qiLittleEndianParser(it, msg.alert_flag);
    if (msg.block_header.revision > 0)
    {
        qiLittleEndianParser(it, msg.nr_bases);
        qiLittleEndianParser(it, msg.ppp_info);
    }
    if (msg.block_header.revision > 1)
    {
        qiLittleEndianParser(it, msg.latency);
        qiLittleEndianParser(it, msg.h_accuracy);
        qiLittleEndianParser(it, msg.v_accuracy);
        qiLittleEndianParser(it, msg.misc);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "PVTCartesianParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * PVTGeodeticParser
 * @brief Qi based parser for the SBF block "PVTGeodetic"
 */
template <typename It>
[[nodiscard]] bool PVTGeodeticParser(ROSaicNodeBase* node, It it, It itEnd,
                                     PVTGeodeticMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4007)
    {
        node->log(log_level::ERROR, "PVTGeodeticParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.latitude);
    qiLittleEndianParser(it, msg.longitude);
    qiLittleEndianParser(it, msg.height);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.vn);
    qiLittleEndianParser(it, msg.ve);
    qiLittleEndianParser(it, msg.vu);
    qiLittleEndianParser(it, msg.cog);
    qiLittleEndianParser(it, msg.rx_clk_bias);
    qiLittleEndianParser(it, msg.rx_clk_drift);
    qiLittleEndianParser(it, msg.time_system);
    qiLittleEndianParser(it, msg.datum);
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.wa_corr_info);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.mean_corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    qiLittleEndianParser(it, msg.alert_flag);
    if (msg.block_header.revision > 0)
    {
        qiLittleEndianParser(it, msg.nr_bases);
        qiLittleEndianParser(it, msg.ppp_info);
    }
    if (msg.block_header.revision > 1)
    {
        qiLittleEndianParser(it, msg.latency);
        qiLittleEndianParser(it, msg.h_accuracy);
        qiLittleEndianParser(it, msg.v_accuracy);
        qiLittleEndianParser(it, msg.misc);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "PVTGeodeticParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * AttEulerParser
 * @brief Qi based parser for the SBF block "AttEuler"
 */
template <typename It>
[[nodiscard]] bool AttEulerParser(ROSaicNodeBase* node, It it, It itEnd,
                                  AttEulerMsg& msg, bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5938)
    {
        node->log(log_level::ERROR, "AttEulerParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.mode);
    std::advance(it, 2); // reserved
    qiLittleEndianParser(it, msg.heading);
    qiLittleEndianParser(it, msg.pitch);
    qiLittleEndianParser(it, msg.roll);
    qiLittleEndianParser(it, msg.pitch_dot);
    qiLittleEndianParser(it, msg.roll_dot);
    qiLittleEndianParser(it, msg.heading_dot);
    if (use_ros_axis_orientation)
    {
        msg.heading = -msg.heading + 90;
        msg.pitch = -msg.pitch;
        msg.pitch_dot = -msg.pitch_dot;
        msg.heading_dot = -msg.heading_dot;
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "AttEulerParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * AttCovEulerParser
 * @brief Qi based parser for the SBF block "AttCovEuler"
 */
template <typename It>
[[nodiscard]] bool AttCovEulerParser(ROSaicNodeBase* node, It it, It itEnd,
                                     AttCovEulerMsg& msg,
                                     bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5939)
    {
        node->log(log_level::ERROR, "AttCovEulerParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    ++it; // reserved
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_headhead);
    qiLittleEndianParser(it, msg.cov_pitchpitch);
    qiLittleEndianParser(it, msg.cov_rollroll);
    qiLittleEndianParser(it, msg.cov_headpitch);
    qiLittleEndianParser(it, msg.cov_headroll);
    qiLittleEndianParser(it, msg.cov_pitchroll);
    if (use_ros_axis_orientation)
    {
        msg.cov_headroll = -msg.cov_headroll;
        msg.cov_pitchroll = -msg.cov_pitchroll;
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "AttCovEulerParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VectorInfoCartParser
 * @brief Qi based parser for the SBF sub-block "VectorInfoCart"
 */
template <typename It>
void VectorInfoCartParser(It& it, VectorInfoCartMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.misc);
    qiLittleEndianParser(it, msg.delta_x);
    qiLittleEndianParser(it, msg.delta_y);
    qiLittleEndianParser(it, msg.delta_z);
    qiLittleEndianParser(it, msg.delta_vx);
    qiLittleEndianParser(it, msg.delta_vy);
    qiLittleEndianParser(it, msg.delta_vz);
    qiLittleEndianParser(it, msg.azimuth);
    qiLittleEndianParser(it, msg.elevation);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    std::advance(it, sb_length - 52); // skip padding
};

/**
 * BaseVectorCartParser
 * @brief Qi based parser for the SBF block "BaseVectorCart"
 */
template <typename It>
[[nodiscard]] bool BaseVectorCartParser(ROSaicNodeBase* node, It it, It itEnd,
                                        BaseVectorCartMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4043)
    {
        node->log(log_level::ERROR, "BaseVectorCartParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_NBVECTORINFO)
    {
        node->log(log_level::ERROR,
                  "BaseVectorCartParser error: Too many VectorInfoCart " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb_length);
    msg.vector_info_cart.resize(msg.n);
    for (auto& vector_info_cart : msg.vector_info_cart)
    {
        VectorInfoCartParser(it, vector_info_cart, msg.sb_length);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BaseVectorCartParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VectorInfoGeodParser
 * @brief Qi based parser for the SBF sub-block "VectorInfoGeod"
 */
template <typename It>
void VectorInfoGeodParser(It& it, VectorInfoGeodMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.misc);
    qiLittleEndianParser(it, msg.delta_east);
    qiLittleEndianParser(it, msg.delta_north);
    qiLittleEndianParser(it, msg.delta_up);
    qiLittleEndianParser(it, msg.delta_ve);
    qiLittleEndianParser(it, msg.delta_vn);
    qiLittleEndianParser(it, msg.delta_vu);
    qiLittleEndianParser(it, msg.azimuth);
    qiLittleEndianParser(it, msg.elevation);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    std::advance(it, sb_length - 52); // skip padding
};

/**
 * BaseVectorGeodParser
 * @brief Qi based parser for the SBF block "BaseVectorGeod"
 */
template <typename It>
[[nodiscard]] bool BaseVectorGeodParser(ROSaicNodeBase* node, It it, It itEnd,
                                        BaseVectorGeodMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4028)
    {
        node->log(log_level::ERROR, "BaseVectorGeodParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_NBVECTORINFO)
    {
        node->log(log_level::ERROR,
                  "BaseVectorGeodParser error: Too many VectorInfoGeod " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb_length);
    msg.vector_info_geod.resize(msg.n);
    for (auto& vector_info_geod : msg.vector_info_geod)
    {
        VectorInfoGeodParser(it, vector_info_geod, msg.sb_length);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BaseVectorGeodParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * INSNavCartParser
 * @brief Qi based parser for the SBF block "INSNavCart"
 */
template <typename It>
[[nodiscard]] bool INSNavCartParser(ROSaicNodeBase* node, It it, It itEnd,
                                    INSNavCartMsg& msg,
                                    bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if ((msg.block_header.id != 4225) && (msg.block_header.id != 4229))
    {
        node->log(log_level::ERROR, "INSNavCartParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.gnss_mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.info);
    qiLittleEndianParser(it, msg.gnss_age);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);
    qiLittleEndianParser(it, msg.accuracy);
    qiLittleEndianParser(it, msg.latency);
    qiLittleEndianParser(it, msg.datum);
    ++it; // reserved
    qiLittleEndianParser(it, msg.sb_list);
    if ((msg.sb_list & 1) != 0)
    {
        qiLittleEndianParser(it, msg.x_std_dev);
        qiLittleEndianParser(it, msg.y_std_dev);
        qiLittleEndianParser(it, msg.z_std_dev);
    } else
    {
        setDoNotUse(msg.x_std_dev);
        setDoNotUse(msg.y_std_dev);
        setDoNotUse(msg.z_std_dev);
    }
    if ((msg.sb_list & 2) != 0)
    {
        qiLittleEndianParser(it, msg.heading);
        qiLittleEndianParser(it, msg.pitch);
        qiLittleEndianParser(it, msg.roll);
        if (use_ros_axis_orientation)
        {
            msg.heading = -msg.heading + 90;
            msg.pitch = -msg.pitch;
        }
    } else
    {
        setDoNotUse(msg.heading);
        setDoNotUse(msg.pitch);
        setDoNotUse(msg.roll);
    }
    if ((msg.sb_list & 4) != 0)
    {
        qiLittleEndianParser(it, msg.heading_std_dev);
        qiLittleEndianParser(it, msg.pitch_std_dev);
        qiLittleEndianParser(it, msg.roll_std_dev);
    } else
    {
        setDoNotUse(msg.heading_std_dev);
        setDoNotUse(msg.pitch_std_dev);
        setDoNotUse(msg.roll_std_dev);
    }
    if ((msg.sb_list & 8) != 0)
    {
        qiLittleEndianParser(it, msg.vx);
        qiLittleEndianParser(it, msg.vy);
        qiLittleEndianParser(it, msg.vz);
    } else
    {
        setDoNotUse(msg.vx);
        setDoNotUse(msg.vy);
        setDoNotUse(msg.vz);
    }
    if ((msg.sb_list & 16) != 0)
    {
        qiLittleEndianParser(it, msg.vx_std_dev);
        qiLittleEndianParser(it, msg.vy_std_dev);
        qiLittleEndianParser(it, msg.vz_std_dev);
    } else
    {
        setDoNotUse(msg.vx_std_dev);
        setDoNotUse(msg.vy_std_dev);
        setDoNotUse(msg.vz_std_dev);
    }
    if ((msg.sb_list & 32) != 0)
    {
        qiLittleEndianParser(it, msg.xy_cov);
        qiLittleEndianParser(it, msg.xz_cov);
        qiLittleEndianParser(it, msg.yz_cov);
    } else
    {
        setDoNotUse(msg.xy_cov);
        setDoNotUse(msg.xz_cov);
        setDoNotUse(msg.yz_cov);
    }
    if ((msg.sb_list & 64) != 0)
    {
        qiLittleEndianParser(it, msg.heading_pitch_cov);
        qiLittleEndianParser(it, msg.heading_roll_cov);
        qiLittleEndianParser(it, msg.pitch_roll_cov);
        if (use_ros_axis_orientation)
        {
            msg.heading_roll_cov = -msg.heading_roll_cov;
            msg.pitch_roll_cov = -msg.pitch_roll_cov;
        }
    } else
    {
        setDoNotUse(msg.heading_pitch_cov);
        setDoNotUse(msg.heading_roll_cov);
        setDoNotUse(msg.pitch_roll_cov);
    }
    if ((msg.sb_list & 128) != 0)
    {
        qiLittleEndianParser(it, msg.vx_vy_cov);
        qiLittleEndianParser(it, msg.vx_vz_cov);
        qiLittleEndianParser(it, msg.vy_vz_cov);
    } else
    {
        setDoNotUse(msg.vx_vy_cov);
        setDoNotUse(msg.vx_vz_cov);
        setDoNotUse(msg.vy_vz_cov);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "INSNavCartParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PosCovCartesianParser
 * @brief Qi based parser for the SBF block "PosCovCartesian"
 */
template <typename It>
[[nodiscard]] bool PosCovCartesianParser(ROSaicNodeBase* node, It it, It itEnd,
                                         PosCovCartesianMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5905)
    {
        node->log(log_level::ERROR, "PosCovCartesianParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_xx);
    qiLittleEndianParser(it, msg.cov_yy);
    qiLittleEndianParser(it, msg.cov_zz);
    qiLittleEndianParser(it, msg.cov_bb);
    qiLittleEndianParser(it, msg.cov_xy);
    qiLittleEndianParser(it, msg.cov_xz);
    qiLittleEndianParser(it, msg.cov_xb);
    qiLittleEndianParser(it, msg.cov_yz);
    qiLittleEndianParser(it, msg.cov_yb);
    qiLittleEndianParser(it, msg.cov_zb);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "PosCovCartesianParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PosCovGeodeticParser
 * @brief Qi based parser for the SBF block "PosCovGeodetic"
 */
template <typename It>
[[nodiscard]] bool PosCovGeodeticParser(ROSaicNodeBase* node, It it, It itEnd,
                                        PosCovGeodeticMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5906)
    {
        node->log(log_level::ERROR, "PosCovGeodeticParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_latlat);
    qiLittleEndianParser(it, msg.cov_lonlon);
    qiLittleEndianParser(it, msg.cov_hgthgt);
    qiLittleEndianParser(it, msg.cov_bb);
    qiLittleEndianParser(it, msg.cov_latlon);
    qiLittleEndianParser(it, msg.cov_lathgt);
    qiLittleEndianParser(it, msg.cov_latb);
    qiLittleEndianParser(it, msg.cov_lonhgt);
    qiLittleEndianParser(it, msg.cov_lonb);
    qiLittleEndianParser(it, msg.cov_hb);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "PosCovGeodeticParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelCovCartesianParser
 * @brief Qi based parser for the SBF block "VelCovCartesian"
 */
template <typename It>
[[nodiscard]] bool VelCovCartesianParser(ROSaicNodeBase* node, It it, It itEnd,
                                         VelCovCartesianMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5907)
    {
        node->log(log_level::ERROR, "VelCovCartesianParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_vxvx);
    qiLittleEndianParser(it, msg.cov_vyvy);
    qiLittleEndianParser(it, msg.cov_vzvz);
    qiLittleEndianParser(it, msg.cov_dtdt);
    qiLittleEndianParser(it, msg.cov_vxvy);
    qiLittleEndianParser(it, msg.cov_vxvz);
    qiLittleEndianParser(it, msg.cov_vxdt);
    qiLittleEndianParser(it, msg.cov_vyvz);
    qiLittleEndianParser(it, msg.cov_vydt);
    qiLittleEndianParser(it, msg.cov_vzdt);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "VelCovCartesianParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelCovGeodeticParser
 * @brief Qi based parser for the SBF block "VelCovGeodetic"
 */
template <typename It>
[[nodiscard]] bool VelCovGeodeticParser(ROSaicNodeBase* node, It it, It itEnd,
                                        VelCovGeodeticMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5908)
    {
        node->log(log_level::ERROR, "VelCovGeodeticParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_vnvn);
    qiLittleEndianParser(it, msg.cov_veve);
    qiLittleEndianParser(it, msg.cov_vuvu);
    qiLittleEndianParser(it, msg.cov_dtdt);
    qiLittleEndianParser(it, msg.cov_vnve);
    qiLittleEndianParser(it, msg.cov_vnvu);
    qiLittleEndianParser(it, msg.cov_vndt);
    qiLittleEndianParser(it, msg.cov_vevu);
    qiLittleEndianParser(it, msg.cov_vedt);
    qiLittleEndianParser(it, msg.cov_vudt);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "VelCovGeodeticParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * QualityIndParser
 * @brief @brief Qi based parser for the SBF block "QualityInd"
 */
template <typename It>
[[nodiscard]] bool QualityIndParser(ROSaicNodeBase* node, It it, It itEnd,
                                    QualityIndT& msg, QuantityIndMsg& ros_msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4082)
    {
        node->log(log_level::ERROR, "QualityIndParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > 40)
    {
        node->log(log_level::ERROR,
                  "QualityIndParser error: Too many indicators " + std::to_string(msg.n));
        return false;
    }
    ++it; // reserved
    msg.indicators.resize(msg.n);
    std::vector<uint16_t> indicators;
    for (auto& indicator : msg.indicators)
    {
        qiLittleEndianParser(it, indicator);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "QualityIndParser error: iterator past end.");
        return false;
    }
    msg.toROSMsg(ros_msg);
    return true;
};

/**
 * AgcStateParser
 * @brief Struct for the SBF sub-block "AGCState"
 */
template <typename It>
void AgcStateParser(It it, AgcStateT& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.frontend_id);
    qiLittleEndianParser(it, msg.gain);
    qiLittleEndianParser(it, msg.sample_var);
    qiLittleEndianParser(it, msg.blanking_stat);
    std::advance(it, sb_length - 4); // skip padding
};

/**
 * ReceiverStatusParser
 * @brief Struct for the SBF block "ReceiverStatus"
 */
template <typename It>
[[nodiscard]] bool ReceiverStatusParser(ROSaicNodeBase* node, It it, It itEnd,
                                        ReceiverStatusT& msg, ReceiverStatusMsg& ros_msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4014)
    {
        node->log(log_level::ERROR, "ReceiverStatusParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.cpu_load);
    qiLittleEndianParser(it, msg.ext_error);
    qiLittleEndianParser(it, msg.up_time);
    qiLittleEndianParser(it, msg.rx_status);
    qiLittleEndianParser(it, msg.rx_error);
    qiLittleEndianParser(it, msg.n);
    if (msg.n > 18)
    {
        node->log(log_level::ERROR,
                  "ReceiverStatusParser error: Too many AGCState " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb_length);
    qiLittleEndianParser(it, msg.cmd_count);
    qiLittleEndianParser(it, msg.temperature);
    msg.agc_state.resize(msg.n);
    for (auto& agc_state : msg.agc_state)
    {
        AgcStateParser(it, agc_state, msg.sb_length);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ReceiverStatusParser error: iterator past end.");
        return false;
    }
    msg.toROSMsg(ros_msg);
    return true;
};

/**
 * ReceiverTimeParser
 * @brief Struct for the SBF block "ReceiverTime"
 */
template <typename It>
[[nodiscard]] bool ReceiverTimeParser(ROSaicNodeBase* node, It it, It itEnd,
                                      ReceiverTimeMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5914)
    {
        node->log(log_level::ERROR, "ReceiverTimeParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.utc_year);
    qiLittleEndianParser(it, msg.utc_month);
    qiLittleEndianParser(it, msg.utc_day);
    qiLittleEndianParser(it, msg.utc_hour);
    qiLittleEndianParser(it, msg.utc_min);
    qiLittleEndianParser(it, msg.utc_second);
    qiLittleEndianParser(it, msg.delta_ls);
    qiLittleEndianParser(it, msg.sync_level);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ReceiverTimeParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * INSNavGeodParser
 * @brief Qi based parser for the SBF block "INSNavGeod"
 */
template <typename It>
[[nodiscard]] bool INSNavGeodParser(ROSaicNodeBase* node, It it, It itEnd,
                                    INSNavGeodMsg& msg,
                                    bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if ((msg.block_header.id != 4226) && (msg.block_header.id != 4230))
    {
        node->log(log_level::ERROR, "INSNavGeodParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.gnss_mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.info);
    qiLittleEndianParser(it, msg.gnss_age);
    qiLittleEndianParser(it, msg.latitude);
    qiLittleEndianParser(it, msg.longitude);
    qiLittleEndianParser(it, msg.height);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.accuracy);
    qiLittleEndianParser(it, msg.latency);
    qiLittleEndianParser(it, msg.datum);
    ++it; // reserved
    qiLittleEndianParser(it, msg.sb_list);
    if ((msg.sb_list & 1) != 0)
    {
        qiLittleEndianParser(it, msg.latitude_std_dev);
        qiLittleEndianParser(it, msg.longitude_std_dev);
        qiLittleEndianParser(it, msg.height_std_dev);
    } else
    {
        setDoNotUse(msg.latitude_std_dev);
        setDoNotUse(msg.longitude_std_dev);
        setDoNotUse(msg.height_std_dev);
    }
    if ((msg.sb_list & 2) != 0)
    {
        qiLittleEndianParser(it, msg.heading);
        qiLittleEndianParser(it, msg.pitch);
        qiLittleEndianParser(it, msg.roll);
        if (use_ros_axis_orientation)
        {
            msg.heading = -msg.heading + 90;
            msg.pitch = -msg.pitch;
        }
    } else
    {
        setDoNotUse(msg.heading);
        setDoNotUse(msg.pitch);
        setDoNotUse(msg.roll);
    }
    if ((msg.sb_list & 4) != 0)
    {
        qiLittleEndianParser(it, msg.heading_std_dev);
        qiLittleEndianParser(it, msg.pitch_std_dev);
        qiLittleEndianParser(it, msg.roll_std_dev);
    } else
    {
        setDoNotUse(msg.heading_std_dev);
        setDoNotUse(msg.pitch_std_dev);
        setDoNotUse(msg.roll_std_dev);
    }
    if ((msg.sb_list & 8) != 0)
    {
        qiLittleEndianParser(it, msg.ve);
        qiLittleEndianParser(it, msg.vn);
        qiLittleEndianParser(it, msg.vu);
    } else
    {
        setDoNotUse(msg.ve);
        setDoNotUse(msg.vn);
        setDoNotUse(msg.vu);
    }
    if ((msg.sb_list & 16) != 0)
    {
        qiLittleEndianParser(it, msg.ve_std_dev);
        qiLittleEndianParser(it, msg.vn_std_dev);
        qiLittleEndianParser(it, msg.vu_std_dev);
    } else
    {
        setDoNotUse(msg.ve_std_dev);
        setDoNotUse(msg.vn_std_dev);
        setDoNotUse(msg.vu_std_dev);
    }
    if ((msg.sb_list & 32) != 0)
    {
        qiLittleEndianParser(it, msg.latitude_longitude_cov);
        qiLittleEndianParser(it, msg.latitude_height_cov);
        qiLittleEndianParser(it, msg.longitude_height_cov);
    } else
    {
        setDoNotUse(msg.latitude_longitude_cov);
        setDoNotUse(msg.latitude_height_cov);
        setDoNotUse(msg.longitude_height_cov);
    }
    if ((msg.sb_list & 64) != 0)
    {
        qiLittleEndianParser(it, msg.heading_pitch_cov);
        qiLittleEndianParser(it, msg.heading_roll_cov);
        qiLittleEndianParser(it, msg.pitch_roll_cov);
        if (use_ros_axis_orientation)
        {
            msg.heading_roll_cov = -msg.heading_roll_cov;
            msg.pitch_roll_cov = -msg.pitch_roll_cov;
        }
    } else
    {
        setDoNotUse(msg.heading_pitch_cov);
        setDoNotUse(msg.heading_roll_cov);
        setDoNotUse(msg.pitch_roll_cov);
    }
    if ((msg.sb_list & 128) != 0)
    {
        qiLittleEndianParser(it, msg.ve_vn_cov);
        qiLittleEndianParser(it, msg.ve_vu_cov);
        qiLittleEndianParser(it, msg.vn_vu_cov);
    } else
    {
        setDoNotUse(msg.ve_vn_cov);
        setDoNotUse(msg.ve_vu_cov);
        setDoNotUse(msg.vn_vu_cov);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "INSNavGeodParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * IMUSetupParser
 * @brief Qi based parser for the SBF block "IMUSetup"
 */
template <typename It>
[[nodiscard]] bool IMUSetupParser(ROSaicNodeBase* node, It it, It itEnd,
                                  IMUSetupMsg& msg, bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4224)
    {
        node->log(log_level::ERROR, "IMUSetupParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    ++it; // reserved
    qiLittleEndianParser(it, msg.serial_port);
    qiLittleEndianParser(it, msg.ant_lever_arm_x);
    qiLittleEndianParser(it, msg.ant_lever_arm_y);
    qiLittleEndianParser(it, msg.ant_lever_arm_z);
    qiLittleEndianParser(it, msg.theta_x);
    qiLittleEndianParser(it, msg.theta_y);
    qiLittleEndianParser(it, msg.theta_z);
    if (use_ros_axis_orientation)
    {
        msg.ant_lever_arm_y = -msg.ant_lever_arm_y;
        msg.ant_lever_arm_z = -msg.ant_lever_arm_z;
        msg.theta_x = parsing_utilities::wrapAngle180to180(msg.theta_x - 180.0f);
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "IMUSetupParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelSensorSetupParser
 * @brief Qi based parser for the SBF block "VelSensorSetup"
 */
template <typename It>
[[nodiscard]] bool VelSensorSetupParser(ROSaicNodeBase* node, It it, It itEnd,
                                        VelSensorSetupMsg& msg,
                                        bool use_ros_axis_orientation)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4244)
    {
        node->log(log_level::ERROR, "VelSensorSetupParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    ++it; // reserved
    qiLittleEndianParser(it, msg.port);
    qiLittleEndianParser(it, msg.lever_arm_x);
    qiLittleEndianParser(it, msg.lever_arm_y);
    qiLittleEndianParser(it, msg.lever_arm_z);
    if (use_ros_axis_orientation)
    {
        msg.lever_arm_y = -msg.lever_arm_y;
        msg.lever_arm_z = -msg.lever_arm_z;
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "VelSensorSetupParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * ExtSensorMeasParser
 * @brief Qi based parser for the SBF block "ExtSensorMeas"
 */
template <typename It>
[[nodiscard]] bool
ExtSensorMeasParser(ROSaicNodeBase* node, It it, It itEnd, ExtSensorMeasMsg& msg,
                    bool use_ros_axis_orientation, bool& hasImuMeas)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4050)
    {
        node->log(log_level::ERROR, "ExtSensorMeasParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    if (msg.sb_length != 28)
    {
        node->log(log_level::ERROR,
                  "ExtSensorMeasParser error: Wrong sb_length " + std::to_string(msg.sb_length));
        return false;
    }

    msg.acceleration_x = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration_y = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration_z = std::numeric_limits<double>::quiet_NaN();

    msg.angular_rate_x = std::numeric_limits<double>::quiet_NaN();
    msg.angular_rate_y = std::numeric_limits<double>::quiet_NaN();
    msg.angular_rate_z = std::numeric_limits<double>::quiet_NaN();

    msg.velocity_x = std::numeric_limits<double>::quiet_NaN();
    msg.velocity_y = std::numeric_limits<double>::quiet_NaN();
    msg.velocity_z = std::numeric_limits<double>::quiet_NaN();

    msg.std_dev_x = std::numeric_limits<double>::quiet_NaN();
    msg.std_dev_y = std::numeric_limits<double>::quiet_NaN();
    msg.std_dev_z = std::numeric_limits<double>::quiet_NaN();

    msg.sensor_temperature = std::numeric_limits<float>::quiet_NaN();
    msg.zero_velocity_flag = std::numeric_limits<double>::quiet_NaN();

    msg.source.resize(msg.n);
    msg.sensor_model.resize(msg.n);
    msg.type.resize(msg.n);
    msg.obs_info.resize(msg.n);
    bool hasAcc = false;
    bool hasOmega = false;
    hasImuMeas = false;
    for (size_t i = 0; i < msg.n; i++)
    {
        qiLittleEndianParser(it, msg.source[i]);
        qiLittleEndianParser(it, msg.sensor_model[i]);
        qiLittleEndianParser(it, msg.type[i]);
        qiLittleEndianParser(it, msg.obs_info[i]);

        switch (msg.type[i])
        {
        case 0:
        {
            qiLittleEndianParser(it, msg.acceleration_x);
            qiLittleEndianParser(it, msg.acceleration_y);
            qiLittleEndianParser(it, msg.acceleration_z);
            hasAcc = true;
            break;
        }
        case 1:
        {
            qiLittleEndianParser(it, msg.angular_rate_x);
            qiLittleEndianParser(it, msg.angular_rate_y);
            qiLittleEndianParser(it, msg.angular_rate_z);
            hasOmega = true;
            break;
        }
        case 3:
        {
            int16_t temp;
            qiLittleEndianParser(it, temp);
            if (temp != -32768)
                msg.sensor_temperature = temp / 100.0f;
            else
                msg.sensor_temperature = std::numeric_limits<float>::quiet_NaN();
            std::advance(it, 22); // reserved
            break;
        }
        case 4:
        {
            qiLittleEndianParser(it, msg.velocity_x);
            qiLittleEndianParser(it, msg.velocity_y);
            qiLittleEndianParser(it, msg.velocity_z);
            qiLittleEndianParser(it, msg.std_dev_x);
            qiLittleEndianParser(it, msg.std_dev_y);
            qiLittleEndianParser(it, msg.std_dev_z);
            if (use_ros_axis_orientation)
            {
                msg.velocity_y = -msg.velocity_y;
                msg.velocity_z = -msg.velocity_z;
            }
            break;
        }
        case 20:
        {
            qiLittleEndianParser(it, msg.zero_velocity_flag);
            std::advance(it, 16); // reserved
            break;
        }
        default:
        {
            node->log(
                log_level::DEBUG,
                "ExtSensorMeasParser Unknown external sensor measurement type in SBF ExtSensorMeas: " +
                    std::to_string(msg.type[i]));
            std::advance(it, 24);
            break;
        }
        }
    }
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "ExtSensorMeasParser error: iterator past end.");
        return false;
    }
    hasImuMeas = hasAcc && hasOmega;
    return true;
};

/**
 * GPSNavParser
 * @brief Qi based parser for the SBF block "GPSNav"
 */
template <typename It>
[[nodiscard]] bool GPSNavParser(ROSaicNodeBase* node, It it, It itEnd,
                                GPSNavMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5891)
    {
        node->log(log_level::ERROR, "GPSNavParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved1);
    qiLittleEndianParser(it, msg.wn);
    qiLittleEndianParser(it, msg.ca_or_p_on_l2);
    qiLittleEndianParser(it, msg.ura);
    qiLittleEndianParser(it, msg.health);
    qiLittleEndianParser(it, msg.l2_data_flag);
    qiLittleEndianParser(it, msg.iodc);
    qiLittleEndianParser(it, msg.iode2);
    qiLittleEndianParser(it, msg.iode3);
    qiLittleEndianParser(it, msg.fit_int_flg);
    qiLittleEndianParser(it, msg.reserved2);
    qiLittleEndianParser(it, msg.t_gd);
    qiLittleEndianParser(it, msg.t_oc);
    qiLittleEndianParser(it, msg.a_f2);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.c_rs);
    qiLittleEndianParser(it, msg.del_n);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.c_uc);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.c_us);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.t_oe);
    qiLittleEndianParser(it, msg.c_ic);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.c_is);
    qiLittleEndianParser(it, msg.i_0);
    qiLittleEndianParser(it, msg.c_rc);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.i_dot);
    qiLittleEndianParser(it, msg.wnt_oc);
    qiLittleEndianParser(it, msg.wnt_oe);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GPSNavParse error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GPSAlmParser
 * @brief Qi based parser for the SBF block "GPSAlm"
 */
template <typename It>
[[nodiscard]] bool GPSAlmParser(ROSaicNodeBase* node, It it, It itEnd,
                                GPSAlmMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5892)
    {
        node->log(log_level::ERROR, "GPSAlmParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.t_oa);
    qiLittleEndianParser(it, msg.delta_i);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.wn_a);
    qiLittleEndianParser(it, msg.config);
    qiLittleEndianParser(it, msg.health8);
    qiLittleEndianParser(it, msg.health6);
    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GPSAlmParser error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * GPSIonParser
 * @brief Qi based parser for the SBF block "GPSIon"
 */
template <typename It>
[[nodiscard]] bool GPSIonParser(ROSaicNodeBase* node, It it, It itEnd,
                                GPSIonMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5893)
    {
        node->log(log_level::ERROR, "GPSIonParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.alpha_0);
    qiLittleEndianParser(it, msg.alpha_1);
    qiLittleEndianParser(it, msg.alpha_2);
    qiLittleEndianParser(it, msg.alpha_3);
    qiLittleEndianParser(it, msg.beta_0);
    qiLittleEndianParser(it, msg.beta_1);
    qiLittleEndianParser(it, msg.beta_2);
    qiLittleEndianParser(it, msg.beta_3);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GPSIonParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GPSUtcParser
 * @brief Qi based parser for the SBF block "GPSIon"
 */
template <typename It>
[[nodiscard]] bool GPSUtcParser(ROSaicNodeBase* node, It it, It itEnd,
                                GPSUtcMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5894)
    {
        node->log(log_level::ERROR, "GPSUtcParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.a_1);
    qiLittleEndianParser(it, msg.a_0);
    qiLittleEndianParser(it, msg.t_ot);
    qiLittleEndianParser(it, msg.wn_t);
    qiLittleEndianParser(it, msg.del_t_ls);
    qiLittleEndianParser(it, msg.wn_lsf);
    qiLittleEndianParser(it, msg.dn);
    qiLittleEndianParser(it, msg.del_t_lsf);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GPSUtcParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GLONavParser
 * @brief Qi based parser for the SBF block "GLONav"
 */
template <typename It>
[[nodiscard]] bool GLONavParser(ROSaicNodeBase* node, It it, It itEnd,
                                GLONavMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4004)
    {
        node->log(log_level::ERROR, "GLONavParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);
    qiLittleEndianParser(it, msg.dx);
    qiLittleEndianParser(it, msg.dy);
    qiLittleEndianParser(it, msg.dz);
    qiLittleEndianParser(it, msg.ddx);
    qiLittleEndianParser(it, msg.ddz);
    qiLittleEndianParser(it, msg.gamma);
    qiLittleEndianParser(it, msg.tau);
    qiLittleEndianParser(it, msg.dtau);
    qiLittleEndianParser(it, msg.t_oe);
    qiLittleEndianParser(it, msg.wn_toe);
    qiLittleEndianParser(it, msg.p1);
    qiLittleEndianParser(it, msg.p2);
    qiLittleEndianParser(it, msg.capital_e);
    qiLittleEndianParser(it, msg.capital_b);
    qiLittleEndianParser(it, msg.tb);
    qiLittleEndianParser(it, msg.capital_m);
    qiLittleEndianParser(it, msg.capital_p);
    qiLittleEndianParser(it, msg.l_health_flag);
    qiLittleEndianParser(it, msg.p4);
    qiLittleEndianParser(it, msg.n_t);
    qiLittleEndianParser(it, msg.f_t);
    qiLittleEndianParser(it, msg.p4);

    if (msg.block_header.revision > 0)
    {
        qiLittleEndianParser(it, msg.capital_c);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GLONavParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GLOAlmParser
 * @brief Qi based parser for the SBF block "GLOAlm"
 */
template <typename It>
[[nodiscard]] bool GLOAlmParser(ROSaicNodeBase* node, It it, It itEnd,
                                GLOAlmMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4005)
    {
        node->log(log_level::ERROR, "GLOAlmParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    qiLittleEndianParser(it, msg.epsilon);
    qiLittleEndianParser(it, msg.t_oa);
    qiLittleEndianParser(it, msg.delta_i);
    qiLittleEndianParser(it, msg.glo_lambda);
    qiLittleEndianParser(it, msg.t_ln);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.delta_t);
    qiLittleEndianParser(it, msg.d_delta_t);
    qiLittleEndianParser(it, msg.tau);
    qiLittleEndianParser(it, msg.wn_a);
    qiLittleEndianParser(it, msg.capital_c);
    qiLittleEndianParser(it, msg.capital_n);
    qiLittleEndianParser(it, msg.capital_m);
    qiLittleEndianParser(it, msg.capital_n_4);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GLOAlmParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GLOTimeParser
 * @brief Qi based parser for the SBF block "GLOTime"
 */
template <typename It>
[[nodiscard]] bool GLOTimeParser(ROSaicNodeBase* node, It it, It itEnd,
                                 GLOTimeMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4036)
    {
        node->log(log_level::ERROR, "GLOTimeParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    qiLittleEndianParser(it, msg.n_4);
    qiLittleEndianParser(it, msg.kp);
    qiLittleEndianParser(it, msg.capital_n);
    qiLittleEndianParser(it, msg.tau_gps);
    qiLittleEndianParser(it, msg.tau_c);
    qiLittleEndianParser(it, msg.b1);
    qiLittleEndianParser(it, msg.b2);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GLOTimeParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GALNavParser
 * @brief Qi based parser for the SBF block "GALNav"
 */
template <typename It>
[[nodiscard]] bool GALNavParser(ROSaicNodeBase* node, It it, It itEnd,
                                GALNavMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4002)
    {
        node->log(log_level::ERROR, "GALNavParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.i_0);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.i_dot);
    qiLittleEndianParser(it, msg.del_n);
    qiLittleEndianParser(it, msg.c_uc);
    qiLittleEndianParser(it, msg.c_us);
    qiLittleEndianParser(it, msg.c_rc);
    qiLittleEndianParser(it, msg.c_rs);
    qiLittleEndianParser(it, msg.c_ic);
    qiLittleEndianParser(it, msg.c_is);
    qiLittleEndianParser(it, msg.t_oe);
    qiLittleEndianParser(it, msg.t_oc);
    qiLittleEndianParser(it, msg.a_f2);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.wnt_oe);
    qiLittleEndianParser(it, msg.wnt_oc);
    qiLittleEndianParser(it, msg.iod_nav);
    qiLittleEndianParser(it, msg.health_ossol);
    qiLittleEndianParser(it, msg.healteh_prs);
    qiLittleEndianParser(it, msg.sisa_l1_e5a);
    qiLittleEndianParser(it, msg.sisa_l1_e5b);
    qiLittleEndianParser(it, msg.sisa_l1a_e6a);
    qiLittleEndianParser(it, msg.bgd_l1_e5a);
    qiLittleEndianParser(it, msg.bgd_l1_e5b);
    qiLittleEndianParser(it, msg.bgd_l1a_e6a);
    qiLittleEndianParser(it, msg.c_nav_enc);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALNavParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GALAlmParser
 * @brief Qi based parser for the SBF block "GALAlm"
 */
template <typename It>
[[nodiscard]] bool GALAlmParser(ROSaicNodeBase* node, It it, It itEnd,
                                GALAlmMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4003)
    {
        node->log(log_level::ERROR, "GALAlmParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.t_oa);
    qiLittleEndianParser(it, msg.delta_i);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.wn_a);
    qiLittleEndianParser(it, msg.sv_id_a);
    qiLittleEndianParser(it, msg.health);
    qiLittleEndianParser(it, msg.iod_a);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALAlmParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GALIonParser
 * @brief Qi based parser for the SBF block "GALIon"
 */
template <typename It>
[[nodiscard]] bool GALIonParser(ROSaicNodeBase* node, It it, It itEnd,
                                GALIonMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4030)
    {
        node->log(log_level::ERROR, "GALIonParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.a_i0);
    qiLittleEndianParser(it, msg.a_i1);
    qiLittleEndianParser(it, msg.a_i2);
    qiLittleEndianParser(it, msg.storm_flags);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALIonParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GALUtcParser
 * @brief Qi based parser for the SBF block "GALUtc"
 */
template <typename It>
[[nodiscard]] bool GALUtcParser(ROSaicNodeBase* node, It it, It itEnd,
                                GALUtcMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4031)
    {
        node->log(log_level::ERROR, "GALUtcParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.a_1);
    qiLittleEndianParser(it, msg.a_0);
    qiLittleEndianParser(it, msg.t_ot);
    qiLittleEndianParser(it, msg.wn_ot);
    qiLittleEndianParser(it, msg.del_t_ls);
    qiLittleEndianParser(it, msg.wn_lsf);
    qiLittleEndianParser(it, msg.dn);
    qiLittleEndianParser(it, msg.del_t_lsf);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALUtcParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GALGstGpsParser
 * @brief Qi based parser for the SBF block "GALGstGps"
 */
template <typename It>
[[nodiscard]] bool GALGstGpsParser(ROSaicNodeBase* node, It it, It itEnd,
                                   GALGstGpsMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4032)
    {
        node->log(log_level::ERROR, "GALGstGpsParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.a_1g);
    qiLittleEndianParser(it, msg.a_0g);
    qiLittleEndianParser(it, msg.t_og);
    qiLittleEndianParser(it, msg.wn_og);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALGstGpsParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * BDSNavParser
 * @brief Qi based parser for the SBF block "BDSNav"
 */
template <typename It>
[[nodiscard]] bool BDSNavParser(ROSaicNodeBase* node, It it, It itEnd,
                                BDSNavMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4081)
    {
        node->log(log_level::ERROR, "BDSNavParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.wn);
    qiLittleEndianParser(it, msg.ura);
    qiLittleEndianParser(it, msg.sat_h1);
    qiLittleEndianParser(it, msg.iod_c);
    qiLittleEndianParser(it, msg.iod_e);
    qiLittleEndianParser(it, msg.reserved2);
    qiLittleEndianParser(it, msg.t_gd1);
    qiLittleEndianParser(it, msg.t_gd2);
    qiLittleEndianParser(it, msg.t_oc);
    qiLittleEndianParser(it, msg.a_f2);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.c_rs);
    qiLittleEndianParser(it, msg.del_n);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.c_uc);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.c_us);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.t_oe);
    qiLittleEndianParser(it, msg.c_ic);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.c_is);
    qiLittleEndianParser(it, msg.i_0);
    qiLittleEndianParser(it, msg.c_rc);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.i_dot);
    qiLittleEndianParser(it, msg.wnt_oc);
    qiLittleEndianParser(it, msg.wnt_oe);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GALGstGpsParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * BDSAlmParser
 * @brief Qi based parser for the SBF block "BDSAlm"
 */
template <typename It>
[[nodiscard]] bool BDSAlmParser(ROSaicNodeBase* node, It it, It itEnd,
                                BDSAlmMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4119)
    {
        node->log(log_level::ERROR, "BDSAlmParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.wn_a);
    qiLittleEndianParser(it, msg.t_oa);
    qiLittleEndianParser(it, msg.sqrt_a);
    qiLittleEndianParser(it, msg.ecc);
    qiLittleEndianParser(it, msg.omega);
    qiLittleEndianParser(it, msg.m_0);
    qiLittleEndianParser(it, msg.omega_0);
    qiLittleEndianParser(it, msg.omega_dot);
    qiLittleEndianParser(it, msg.delta_i);
    qiLittleEndianParser(it, msg.a_f0);
    qiLittleEndianParser(it, msg.a_f1);
    qiLittleEndianParser(it, msg.health);
    qiLittleEndianParser(it, msg.reserved[0]);
    qiLittleEndianParser(it, msg.reserved[1]);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BDSAlmParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * BDSIonParser
 * @brief Qi based parser for the SBF block "BDSIon"
 */
template <typename It>
[[nodiscard]] bool BDSIonParser(ROSaicNodeBase* node, It it, It itEnd,
                                BDSIonMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4120)
    {
        node->log(log_level::ERROR, "BDSIonParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.alpha_0);
    qiLittleEndianParser(it, msg.alpha_1);
    qiLittleEndianParser(it, msg.alpha_2);
    qiLittleEndianParser(it, msg.alpha_3);
    qiLittleEndianParser(it, msg.beta_0);
    qiLittleEndianParser(it, msg.beta_1);
    qiLittleEndianParser(it, msg.beta_2);
    qiLittleEndianParser(it, msg.beta_3);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BDSIonParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * BDSUtcParser
 * @brief Qi based parser for the SBF block "BDSUtc"
 */
template <typename It>
[[nodiscard]] bool BDSUtcParser(ROSaicNodeBase* node, It it, It itEnd,
                                BDSUtcMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4121)
    {
        node->log(log_level::ERROR, "BDSUtcParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.a_1);
    qiLittleEndianParser(it, msg.a_0);
    qiLittleEndianParser(it, msg.del_t_ls);
    qiLittleEndianParser(it, msg.wn_lsf);
    qiLittleEndianParser(it, msg.dn);
    qiLittleEndianParser(it, msg.del_t_lsf);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BDSUtcParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * FastCorrParser
 * @brief Qi based parser for the SBF sub-block "FastCorr"
 */
template <typename It>
void FastCorrParser(It& it, FastCorrMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.prn_mask_no);
    qiLittleEndianParser(it, msg.udrei);
    std::advance(it, 2); // skip reserved
    qiLittleEndianParser(it, msg.prc);
    std::advance(it, sb_length - 8); // skip padding
};

/**
 * GEOFastCorrParser
 * @brief Qi based parser for the SBF block "GEOFastCorr"
 */
template <typename It>
[[nodiscard]] bool GEOFastCorrParser(ROSaicNodeBase* node, It it, It itEnd,
                                     GEOFastCorrMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5927)
    {
        node->log(log_level::ERROR, "GEOFastCorrParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.mt);
    qiLittleEndianParser(it, msg.iod_prn);
    qiLittleEndianParser(it, msg.iod_fast_corr);
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    msg.fast_corr.resize(msg.n);
    for (auto & fast_corr : msg.fast_corr)
    {
        FastCorrParser(it, fast_corr, msg.sb_length);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GEOFastCorrParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GEONavParser
 * @brief Qi based parser for the SBF block "GEONav"
 */
template <typename It>
[[nodiscard]] bool GEONavParser(ROSaicNodeBase* node, It it, It itEnd,
                                GEONavMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5896)
    {
        node->log(log_level::ERROR, "GEONavParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.iod_n);
    qiLittleEndianParser(it, msg.ura);
    qiLittleEndianParser(it, msg.t0);
    qiLittleEndianParser(it, msg.x_g);
    qiLittleEndianParser(it, msg.y_g);
    qiLittleEndianParser(it, msg.z_g);
    qiLittleEndianParser(it, msg.dx_g);
    qiLittleEndianParser(it, msg.dy_g);
    qiLittleEndianParser(it, msg.dz_g);
    qiLittleEndianParser(it, msg.ddx_g);
    qiLittleEndianParser(it, msg.ddy_g);
    qiLittleEndianParser(it, msg.ddz_g);
    qiLittleEndianParser(it, msg.ag_f0);
    qiLittleEndianParser(it, msg.ag_f1);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GEONavParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GEONetworkTimeParser
 * @brief Qi based parser for the SBF block "GEONetworkTime"
 */
template <typename It>
[[nodiscard]] bool GEONetworkTimeParser(ROSaicNodeBase* node, It it, It itEnd,
                                        GEONetworkTimeMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5918)
    {
        node->log(log_level::ERROR, "GEONetworkTimeParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.a_1);
    qiLittleEndianParser(it, msg.a_0);
    qiLittleEndianParser(it, msg.t_ot);
    qiLittleEndianParser(it, msg.wn_t);
    qiLittleEndianParser(it, msg.del_t_ls);
    qiLittleEndianParser(it, msg.wn_lsf);
    qiLittleEndianParser(it, msg.dn);
    qiLittleEndianParser(it, msg.del_t_lsf);
    qiLittleEndianParser(it, msg.utc_std);
    qiLittleEndianParser(it, msg.gps_wn);
    qiLittleEndianParser(it, msg.gps_tow);
    qiLittleEndianParser(it, msg.glonass_id);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GEONetworkTimeParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * GEOAlmParser
 * @brief Qi based parser for the SBF block "GEOAlm"
 */
template <typename It>
[[nodiscard]] bool GEOAlmParser(ROSaicNodeBase* node, It it, It itEnd,
                                GEOAlmMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5918)
    {
        node->log(log_level::ERROR, "GEOAlmParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.reserved0);
    qiLittleEndianParser(it, msg.health);
    qiLittleEndianParser(it, msg.t_oa);
    qiLittleEndianParser(it, msg.x_g);
    qiLittleEndianParser(it, msg.y_g);
    qiLittleEndianParser(it, msg.z_g);
    qiLittleEndianParser(it, msg.dx_g);
    qiLittleEndianParser(it, msg.dy_g);
    qiLittleEndianParser(it, msg.dz_g);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GEOAlmParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * LTCorrParser
 * @brief Qi based parser for the SBF sub-block "LTCorr"
 */
template <typename It>
void LTCorrParser(It& it, LTCorrMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.velocity_code);
    qiLittleEndianParser(it, msg.prn_mask_no);
    qiLittleEndianParser(it, msg.iod_prn);
    qiLittleEndianParser(it, msg.iod_eph);
    qiLittleEndianParser(it, msg.dx);
    qiLittleEndianParser(it, msg.dy);
    qiLittleEndianParser(it, msg.dz);
    qiLittleEndianParser(it, msg.ddx);
    qiLittleEndianParser(it, msg.ddy);
    qiLittleEndianParser(it, msg.ddz);
    qiLittleEndianParser(it, msg.da_f0);
    qiLittleEndianParser(it, msg.da_f1);
    qiLittleEndianParser(it, msg.t_oe);
    std::advance(it, sb_length - 40); // skip padding
};

/**
 * GEOLongTermCorrParser
 * @brief Qi based parser for the SBF block "GEOLongTermCorr"
 */
template <typename It>
[[nodiscard]] bool GEOLongTermCorrParser(ROSaicNodeBase* node, It it, It itEnd,
                                         GEOLongTermCorrMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5932)
    {
        node->log(log_level::ERROR, "GEOLongTermCorrParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.prn);
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    std::advance(it, 3); // skip reserved
    msg.lt_corr.resize(msg.n);
    for (auto & lt_corr : msg.lt_corr)
    {
        LTCorrParser(it, lt_corr, msg.sb_length);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "GEOLongTermCorrParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * DiffCorrInParser
 * @brief Qi based parser for the SBF block "DiffCorrIn"
 */
template <typename It>
[[nodiscard]] bool DiffCorrInParser(ROSaicNodeBase* node, It it, It itEnd,
                                    DiffCorrInMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5919)
    {
        node->log(log_level::ERROR, "DiffCorrInParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.source);

    if (msg.mode == DiffCorrInMsg::RTCMV2)
    {
        for (size_t i = 0; i < 2; i++)
        {
            uint32_t temp;
            qiLittleEndianParser(it, temp);
            msg.rtcm2_words.emplace_back(temp);
        }
        const auto N = 2 + ((msg.rtcm2_words[1]>>9) & 0x1f);
        for (size_t i = 2; i < N; i++)
        {
            uint32_t temp;
            qiLittleEndianParser(it, temp);
            msg.rtcm2_words.emplace_back(temp);
        }
    }
    else
    {
        do
        {
            uint8_t temp;
            qiLittleEndianParser(it, temp);
            if (msg.mode == DiffCorrInMsg::CMRV2)
                msg.cmr_message.emplace_back(temp);
            else if (msg.mode == DiffCorrInMsg::RTCMV3)
                msg.rtcm3_message.emplace_back(temp);
            else if (msg.mode == DiffCorrInMsg::RTCMV)
                msg.rtcmv_message.emplace_back(temp);
        } while (it != itEnd);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "DiffCorrInParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * BaseStationParser
 * @brief Qi based parser for the SBF block "BaseStation"
 */
template <typename It>
[[nodiscard]] bool BaseStationParser(ROSaicNodeBase* node, It it, It itEnd,
                                     BaseStationMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5949)
    {
        node->log(log_level::ERROR, "BaseStationParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.base_station_id);
    qiLittleEndianParser(it, msg.base_type);
    qiLittleEndianParser(it, msg.source);
    qiLittleEndianParser(it, msg.datum);
    qiLittleEndianParser(it, msg.reserved);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "BaseStationParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * SatInfoParser
 * @brief Qi based parser for the SBF sub-block "SatInfo"
 */
template <typename It>
void SatInfoParser(It& it, SatInfoMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    qiLittleEndianParser(it, msg.azimuth);
    qiLittleEndianParser(it, msg.elevation);
    qiLittleEndianParser(it, msg.rise_set);
    qiLittleEndianParser(it, msg.satellite_info);
    std::advance(it, sb_length - 8); // skip padding
};

/**
 * SatVisibilityParser
 * @brief Qi based parser for the SBF block "SatVisibility"
 */
template <typename It>
[[nodiscard]] bool SatVisibilityParser(ROSaicNodeBase* node, It it, It itEnd,
                                       SatVisibilityMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4012)
    {
        node->log(log_level::ERROR, "SatVisibilityParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    msg.sat_info.resize(msg.n);
    for (auto & sat_info : msg.sat_info)
    {
        SatInfoParser(it, sat_info, msg.sb_length);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "SatVisibilityParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * NtripClientConnectionParser
 * @brief Qi based parser for the SBF sub-block "NtripClientConnection"
 */
template <typename It>
void NtripClientConnectionParser(It& it, NtripClientConnectionMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.cd_index);
    qiLittleEndianParser(it, msg.status);
    qiLittleEndianParser(it, msg.error_code);
    qiLittleEndianParser(it, msg.info);
    std::advance(it, sb_length - 4); // skip padding
};

/**
 * NtripClientStatusParser
 * @brief Qi based parser for the SBF block "NtripClientStatus"
 */
template <typename It>
[[nodiscard]] bool NtripClientStatusParser(ROSaicNodeBase* node, It it, It itEnd,
                                           NtripClientStatusMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4053)
    {
        node->log(log_level::ERROR, "NtripClientStatusParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    msg.ntrip_connections.resize(msg.n);
    for (auto & ntrip_connection : msg.ntrip_connections)
    {
        NtripClientConnectionParser(it, ntrip_connection, msg.sb_length);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "NtripClientStatusParser error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * AuxAntPositionParser
 * @brief Qi based parser for the SBF sub-block "AuxAntPositionP"
 */
template <typename It>
void AuxAntPositionParser(It& it, AuxAntPositionMsg& msg, uint8_t sb_length)
{
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.ambiguity_type);
    qiLittleEndianParser(it, msg.auxant_id);
    qiLittleEndianParser(it, msg.delta_east);
    qiLittleEndianParser(it, msg.delta_north);
    qiLittleEndianParser(it, msg.delta_up);
    qiLittleEndianParser(it, msg.east_vel);
    qiLittleEndianParser(it, msg.north_vel);
    qiLittleEndianParser(it, msg.up_vel);
    std::advance(it, sb_length - 52); // skip padding
};

/**
 * AuxAntPositionsParser
 * @brief Qi based parser for the SBF block "AuxAntPositions"
 */
template <typename It>
[[nodiscard]] bool AuxAntPositionsParser(ROSaicNodeBase* node, It it, It itEnd,
                                         AuxAntPositionsMsg& msg)
{
    if (!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5942)
    {
        node->log(log_level::ERROR, "AuxAntPositionsParser error: Wrong header ID " +
                                        std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    msg.auxant_positions.resize(msg.n);
    for (auto & auxant_position : msg.auxant_positions)
    {
        AuxAntPositionParser(it, auxant_position, msg.sb_length);
    }

    if (it > itEnd)
    {
        node->log(log_level::ERROR, "AuxAntPositionsParser error: iterator past end.");
        return false;
    }
    return true;
}
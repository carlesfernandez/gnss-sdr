/*!
 * \file osnma_msg_receiver.h
 * \brief GNU Radio block that processes Galileo OSNMA data received from
 * Galileo E1B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2023-2024. cfernandez(at)cttc.es
 * Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_OSNMA_MSG_RECEIVER_H
#define GNSS_SDR_OSNMA_MSG_RECEIVER_H

#define FRIEND_TEST(test_case_name, test_name) \
    friend class test_case_name##_##test_name##_Test

#include "galileo_inav_message.h"  // for OSNMA_msg
#include "gnss_block_interface.h"  // for gnss_shared_ptr
#include "gnss_sdr_make_unique.h"  // for std::make:unique in C++11
#include "osnma_data.h"            // for OSNMA_data structures
#include "osnma_nav_data_manager.h"
#include <gnuradio/block.h>        // for gr::block
#include <pmt/pmt.h>               // for pmt::pmt_t
#include <array>                   // for std::array
#include <cstdint>                 // for uint8_t
#include <ctime>                   // for std::time_t
#include <map>                     // for std::map, std::multimap
#include <memory>                  // for std::shared_ptr
#include <string>                  // for std::string
#include <vector>                  // for std::vector

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class OSNMA_DSM_Reader;
class Gnss_Crypto;
class Osnma_Helper;
class osnma_msg_receiver;

using osnma_msg_receiver_sptr = gnss_shared_ptr<osnma_msg_receiver>;

osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath, const std::string& rootKeyFilePath);

/*!
 * \brief GNU Radio block that receives asynchronous OSNMA messages
 * from the telemetry blocks, stores them in memory, and decodes OSNMA info
 * when enough data have been received.
 * The decoded OSNMA data is sent to the PVT block.
 */
class osnma_msg_receiver : public gr::block
{
public:
    ~osnma_msg_receiver() = default;  //!< Default destructor
private:
    friend osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath, const std::string& rootKeyFilePath);
    osnma_msg_receiver(const std::string& crtFilePath, const std::string& merkleFilePath, const std::string& rootKeyFilePath);

    void msg_handler_osnma(const pmt::pmt_t& msg);
    void process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_nma_header(uint8_t nma_header);
    void read_dsm_header(uint8_t dsm_header);
    void read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void local_time_verification(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_message(const std::vector<uint8_t>& dsm_msg, const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_and_process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_mack_header();
    void read_mack_body();
    void process_mack_message();
    void remove_verified_tags();
    void control_tags_awaiting_verify_size();
    void display_data();
    void send_data_to_pvt(std::vector<OSNMA_NavData>);

    bool verify_tesla_key(std::vector<uint8_t>& key, uint32_t TOW);
    bool verify_tag(Tag& tag) const;
    bool tag_has_nav_data_available(const Tag& t) const;
    bool tag_has_key_available(const Tag& t) const;
    bool verify_macseq(const MACK_message& mack);
    bool verify_dsm_pkr(const DSM_PKR_message& message) const;

    std::vector<uint8_t> get_merkle_tree_leaves(const DSM_PKR_message& dsm_pkr_message) const;
    std::vector<uint8_t> compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i) const;
    std::vector<uint8_t> build_message(Tag& tag) const;
    std::vector<uint8_t> hash_chain(uint32_t num_of_hashes_needed, const std::vector<uint8_t>& key, uint32_t GST_SFi, const uint8_t lk_bytes) const;
    std::vector<MACK_tag_and_info> verify_macseq_new(const MACK_message& mack);

    std::map<uint32_t, std::map<uint32_t, OSNMA_NavData>> d_satellite_nav_data;  // map holding OSNMA_NavData sorted by SVID (first key) and TOW (second key).
    std::map<uint32_t, std::vector<uint8_t>> d_tesla_keys;                       // tesla keys over time, sorted by TOW
    std::multimap<uint32_t, Tag> d_tags_awaiting_verify;                         // container with tags to verify from arbitrary SVIDs, sorted by TOW

    std::vector<uint8_t> d_kroot;  // last available stored root key
    std::vector<uint8_t> d_tags_to_verify{0, 4, 12};
    std::vector<MACK_message> d_macks_awaiting_MACSEQ_verification;

    std::array<std::array<uint8_t, 256>, 16> d_dsm_message{};  // structure for recording DSM blocks, when filled it sends them to parse and resets itself.
    std::array<std::array<uint8_t, 16>, 16> d_dsm_id_received{};
    std::array<uint16_t, 16> d_number_of_blocks{};
    std::array<uint8_t, 60> d_mack_message{};  // C: 480 b

    std::unique_ptr<OSNMA_DSM_Reader> d_dsm_reader;  // osnma parameters parser
    std::unique_ptr<Gnss_Crypto> d_crypto;           // access to cryptographic functions
    std::unique_ptr<Osnma_Helper> d_helper;          // helper class with auxiliary functions
    std::unique_ptr<OSNMA_nav_data_Manager> d_nav_data_manager;  // refactor for holding and processing navigation data

    OSNMA_data d_osnma_data{};

    enum tags_to_verify
    {
        all,
        utc,
        slow_eph,
        eph,
        none
    };
    tags_to_verify d_tags_allowed{tags_to_verify::all};
    std::time_t d_receiver_time{0};

    uint32_t d_GST_Sf{};  // C: used for MACSEQ and Tesla Key verification TODO need really to be global var?
    uint32_t d_last_verified_key_GST{0};
    uint32_t d_GST_0{};
    uint32_t d_GST_SIS{};

    uint8_t d_Lt_min{};             // minimum equivalent tag length
    uint8_t d_Lt_verified_eph{0};   // verified tag bits - ephemeris
    uint8_t d_Lt_verified_utc{0};   // verified tag bits - timing
    uint8_t const d_T_L{30};        // s RG Section 2.1
    uint8_t const d_delta_COP{30};  // s SIS ICD Table 14

    bool d_new_data{false};
    bool d_public_key_verified{false};
    bool d_kroot_verified{false};
    bool d_tesla_key_verified{false};
    bool d_flag_debug{false};

    // Provide access to inner functions to Gtest
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaKeyVerification);
    FRIEND_TEST(OsnmaMsgReceiverTest, OsnmaTestVectorsSimulation);
    FRIEND_TEST(OsnmaMsgReceiverTest, TagVerification);
    FRIEND_TEST(OsnmaMsgReceiverTest, BuildTagMessageM0);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifyPublicKey);
    FRIEND_TEST(OsnmaMsgReceiverTest, ComputeBaseLeaf);
    FRIEND_TEST(OsnmaMsgReceiverTest, ComputeMerkleRoot);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_MSG_RECEIVER_H

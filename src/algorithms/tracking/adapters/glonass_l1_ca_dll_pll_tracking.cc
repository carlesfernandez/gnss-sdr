/*!
 * \file glonass_l1_ca_dll_pll_tracking.cc
 * \brief  Adapter of a DLL+PLL tracking loop block for GLONASS L1 C/A signals.
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "glonass_l1_ca_dll_pll_tracking.h"
#include "GLONASS_L1_L2_CA.h"
#include "configuration_interface.h"
#include <algorithm>
#include <array>
#include <cmath>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GlonassL1CaDllPllTracking::GlonassL1CaDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GlonassL1CaDllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration [[maybe_unused]])
{
    if (config_params().glonass_nominal_carrier_hz <= 0.0)
        {
            config_params().glonass_nominal_carrier_hz = GLONASS_L1_CA_FREQ_HZ;
        }
    if (config_params().glonass_carrier_spacing_hz == 0.0)
        {
            config_params().glonass_carrier_spacing_hz = GLONASS_L1_CA_DFREQ_HZ;
        }
    if (config_params().glonass_code_rate_cps <= 0.0)
        {
            config_params().glonass_code_rate_cps = GLONASS_L1_CA_CODE_RATE_CPS;
        }
    if (config_params().glonass_code_length_chips <= 0.0)
        {
            config_params().glonass_code_length_chips = GLONASS_L1_CA_CODE_LENGTH_CHIPS;
        }

    const auto vector_length = static_cast<int>(std::round(
        config_params().fs_in /
        (config_params().glonass_code_rate_cps / config_params().glonass_code_length_chips)));
    config_params().vector_length = static_cast<uint32_t>(vector_length);
    config_params().system = 'R';
    const std::array<char, 3> sig{'1', 'G', '\0'};
    std::copy_n(sig.begin(), 3, config_params().signal);
    config_params().track_pilot = false;
}


void GlonassL1CaDllPllTracking::create_tracking_block()
{
    if (config_params().item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(config_params());
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            set_item_size(0);
            tracking_sptr_ = nullptr;
            LOG(WARNING) << config_params().item_type << " unknown tracking item type.";
        }
}

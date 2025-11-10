/*!
 * \file glonass_l2_ca_dll_pll_c_aid_tracking.cc
 * \brief  Adapter of a DLL+PLL tracking loop block with carrier aiding for GLONASS L2 C/A signals.
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
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

#include "glonass_l2_ca_dll_pll_c_aid_tracking.h"
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

GlonassL2CaDllPllCAidTracking::GlonassL2CaDllPllCAidTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GlonassL2CaDllPllCAidTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration)
{
    if (config_params().glonass_nominal_carrier_hz <= 0.0)
        {
            config_params().glonass_nominal_carrier_hz = GLONASS_L2_CA_FREQ_HZ;
        }
    if (config_params().glonass_carrier_spacing_hz == 0.0)
        {
            config_params().glonass_carrier_spacing_hz = GLONASS_L2_CA_DFREQ_HZ;
        }
    if (config_params().glonass_code_rate_cps <= 0.0)
        {
            config_params().glonass_code_rate_cps = GLONASS_L2_CA_CODE_RATE_CPS;
        }
    if (config_params().glonass_code_length_chips <= 0.0)
        {
            config_params().glonass_code_length_chips = GLONASS_L2_CA_CODE_LENGTH_CHIPS;
        }

    const auto vector_length = static_cast<int>(std::round(
        config_params().fs_in /
        (config_params().glonass_code_rate_cps / config_params().glonass_code_length_chips)));
    config_params().vector_length = static_cast<uint32_t>(vector_length);
    config_params().system = 'R';
    const std::array<char, 3> sig{'2', 'G', '\0'};
    std::copy_n(sig.begin(), 3, config_params().signal);
    config_params().track_pilot = false;

    if (configuration != nullptr)
        {
            const int extend_ms = configuration->property(role() + ".extend_correlation_ms",
                static_cast<int>(config_params().extend_correlation_symbols));
            if (extend_ms > 0)
                {
                    config_params().extend_correlation_symbols = extend_ms;
                }
        }
}


void GlonassL2CaDllPllCAidTracking::create_tracking_block()
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
            LOG(WARNING) << config_params().item_type
                          << " item type is not supported by dll_pll_veml_tracking."
                          << " Please use gr_complex inputs.";
        }
}

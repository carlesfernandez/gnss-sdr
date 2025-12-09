/*!
 * \file glonass_l2_ca_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Glonass L2 C/A to a TrackingInterface
 * \author Damian Miralles, 2018, dmiralles2009(at)gmail.com *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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

#include "glonass_l2_ca_dll_pll_tracking.h"
#include "GLONASS_L1_L2_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <algorithm>
#include <array>
#include <cmath>

GlonassL2CaDllPllTracking::GlonassL2CaDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GlonassL2CaDllPllTracking::configure_tracking_parameters(const ConfigurationInterface* configuration)
{
    const auto vector_length = static_cast<int>(std::round(static_cast<double>(config_params().fs_in) /
        (static_cast<double>(GLONASS_L2_CA_CODE_RATE_CPS) / static_cast<double>(GLONASS_L2_CA_CODE_LENGTH_CHIPS))));
    config_params().vector_length = vector_length;
    if (config_params().extend_correlation_symbols != 1)
        {
            config_params().extend_correlation_symbols = 1;
        }
    config_params().early_late_space_chips = configuration->property(role() + ".early_late_space_chips", static_cast<float>(0.5F));
    config_params().pll_bw_hz = configuration->property(role() + ".pll_bw_hz", static_cast<float>(50.0));
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_pll_bw_hz != 0.0)
        {
            config_params().pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
        }
#else
    if (absl::GetFlag(FLAGS_pll_bw_hz) != 0.0)
        {
            config_params().pll_bw_hz = static_cast<float>(absl::GetFlag(FLAGS_pll_bw_hz));
        }
#endif
    config_params().dll_bw_hz = configuration->property(role() + ".dll_bw_hz", static_cast<float>(2.0));
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_dll_bw_hz != 0.0)
        {
            config_params().dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
        }
#else
    if (absl::GetFlag(FLAGS_dll_bw_hz) != 0.0)
        {
            config_params().dll_bw_hz = static_cast<float>(absl::GetFlag(FLAGS_dll_bw_hz));
        }
#endif
    config_params().track_pilot = false;
    config_params().system = 'R';
    const std::array<char, 3> sig{'2', 'G', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
    config_params().enable_fll_steady_state = configuration->property(role() + ".enable_fll_steady_state", false);
}


void GlonassL2CaDllPllTracking::create_tracking_block()
{
    if (config_params().item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(config_params());
        }
    else
        {
            set_item_size(0);
            tracking_sptr_ = nullptr;
        }
}

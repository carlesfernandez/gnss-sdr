/*!
 * \file glonass_l1_ca_dll_pll_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Glonass L1 C/A to a TrackingInterface
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 *
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

#ifndef GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H
#define GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H

#include "base_dll_pll_tracking.h"
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GlonassL1CaDllPllTracking : public BaseDllPllTracking
{
public:
    GlonassL1CaDllPllTracking(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GlonassL1CaDllPllTracking() override = default;

    //! Returns "GLONASS_L1_CA_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "GLONASS_L1_CA_DLL_PLL_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H

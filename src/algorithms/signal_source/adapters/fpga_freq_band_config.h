/*!
 * \file fpga_freq_band_config.h
 * \brief Helpers to derive the default frequency band selection of the FPGA
 * signal sources from the configured channels.
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_FREQ_BAND_CONFIG_H
#define GNSS_SDR_FPGA_FREQ_BAND_CONFIG_H

#include "configuration_interface.h"
#include <array>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


/*!
 * \brief Returns true if any channel is configured for a signal that the FPGA
 * processes in frequency band 1 (acquisition buffer 0: GPS L1 C/A, Galileo E1).
 *
 * Used as the default value of the `rx1_enable` property of the FPGA signal
 * sources when it is not explicitly set in the configuration.
 */
inline bool fpga_freq_band_1_in_use(const ConfigurationInterface* configuration)
{
    const std::array<std::string, 2> signals{"1C", "1B"};
    for (const auto& signal : signals)
        {
            if (configuration->property("Channels_" + signal + ".count", 0) > 0)
                {
                    return true;
                }
        }
    return false;
}


/*!
 * \brief Returns true if any channel is configured for a signal that the FPGA
 * processes in frequency band 2 (acquisition buffer 1: GPS L2C, GPS L5,
 * Galileo E5a, Galileo E5b, Galileo E6).
 *
 * Used as the default value of the `rx2_enable` property of the FPGA signal
 * sources when it is not explicitly set in the configuration.
 */
inline bool fpga_freq_band_2_in_use(const ConfigurationInterface* configuration)
{
    const std::array<std::string, 5> signals{"2S", "L5", "5X", "7X", "E6"};
    for (const auto& signal : signals)
        {
            if (configuration->property("Channels_" + signal + ".count", 0) > 0)
                {
                    return true;
                }
        }
    return false;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_FREQ_BAND_CONFIG_H

/*!
 * \file galileo_e1b_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Galileo INAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas 2013. jarribas(at)cttc.es,
 * Mara Branzanti 2013. mara.branzanti(at)gmail.com
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

#include "galileo_e1b_telemetry_decoder.h"

GalileoE1BTelemetryDecoder::GalileoE1BTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : TelemetryDecoderAdapterBase(configuration, role, in_streams, out_streams,
                                    [](const Gnss_Satellite& satellite, const Tlm_Conf& tlm_parameters)
                                    { return galileo_make_telemetry_decoder_gs(satellite, tlm_parameters, 1); },
                                    [](Tlm_Conf& parameters, const ConfigurationInterface* config, const std::string& role_name)
                                    {
                                        parameters.enable_reed_solomon = config->property(role_name + ".enable_reed_solomon", false);
                                        parameters.use_ced = config->property(role_name + ".use_reduced_ced", false);
                                    })
{
}

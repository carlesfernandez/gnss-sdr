/*!
 * \file galileo_e6_telemetry_decoder.cc
 * \brief Interface of an adapter of a GALILEO E6 CNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Carles Fernandez-Prades, 2020 cfernandez@cttc.es
 *
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

#include "galileo_e6_telemetry_decoder.h"

GalileoE6TelemetryDecoder::GalileoE6TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : TelemetryDecoderAdapterBase(configuration, role, in_streams, out_streams,
                                    [](const Gnss_Satellite& satellite, const Tlm_Conf& tlm_parameters)
                                    { return galileo_make_telemetry_decoder_gs(satellite, tlm_parameters, 3); })
{
}

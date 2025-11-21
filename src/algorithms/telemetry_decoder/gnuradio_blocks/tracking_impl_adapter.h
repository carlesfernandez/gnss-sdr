/*!
 * \file tracking_impl_adapter.h
 * \brief Base class for telemetry decoder GNU Radio blocks.
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

#ifndef GNSS_SDR_TELEMETRY_DECODER_TRACKING_IMPL_ADAPTER_H
#define GNSS_SDR_TELEMETRY_DECODER_TRACKING_IMPL_ADAPTER_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>
#include <string>
#include <utility>

/** \addtogroup Telemetry_Decoder
 * \{
 */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks telemetry_decoder_gr_blocks
 * \{
 */

class tracking_impl_adapter;
using tracking_impl_adapter_sptr = gnss_shared_ptr<tracking_impl_adapter>;

/*!
 * \brief Common base class for telemetry decoder GNU Radio implementations.
 */
class tracking_impl_adapter : public gr::block
{
public:
    tracking_impl_adapter(const std::string& name,
        gr::io_signature::sptr input_signature,
        gr::io_signature::sptr output_signature) : gr::block(name, std::move(input_signature), std::move(output_signature)) {}

    ~tracking_impl_adapter() override = default;

    virtual void set_satellite(const Gnss_Satellite& satellite) = 0;
    virtual void set_channel(int channel) = 0;
    virtual void reset() = 0;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_DECODER_TRACKING_IMPL_ADAPTER_H

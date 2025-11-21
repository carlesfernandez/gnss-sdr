/*!
 * \file telemetry_decoder_adapter_base.h
 * \brief Common functionality for telemetry decoder adapters
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

#ifndef GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H
#define GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H

#include "configuration_interface.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"
#include "../gnuradio_blocks/telemetry_impl_base.h"
#include <gnuradio/runtime_types.h>
#include <cstddef>
#include <string>
#include <utility>

class ConfigurationInterface;

namespace telemetry_decoder_adapter_detail
{
void LogRole(const std::string& role);
void LogDecoderInstance(const telemetry_impl_base_sptr& decoder);
void LogInputStreamsError(unsigned int in_streams);
void LogOutputStreamsError(unsigned int out_streams);
void LogConnect();
void LogSatelliteChange(const Gnss_Satellite& satellite);
}  // namespace telemetry_decoder_adapter_detail

/** \addtogroup Telemetry_Decoder
 * \{
 */
/** \addtogroup Telemetry_Decoder_adapters
 * \{
 */

class TelemetryDecoderAdapterBase : public TelemetryDecoderInterface
{
public:
    TelemetryDecoderAdapterBase(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~TelemetryDecoderAdapterBase() override = default;

    void connect(gr::top_block_sptr top_block) override;

    void disconnect(gr::top_block_sptr top_block) override;

    gr::basic_block_sptr get_left_block() override;

    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    std::string role() override;

    void set_channel(int channel) override;

    void reset() override;

    size_t item_size() override;

protected:
    void InitializeDecoder(telemetry_impl_base_sptr decoder);
    // Adapters can tweak telemetry parameters directly before calling
    // InitializeDecoder(); no separate callback mechanism is required.
    Tlm_Conf& tlm_parameters();
    const Gnss_Satellite& satellite() const;

    telemetry_impl_base_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    Tlm_Conf tlm_parameters_;
    const ConfigurationInterface* configuration_ = nullptr;
    std::string role_;
    unsigned int in_streams_ = 0;
    unsigned int out_streams_ = 0;
};

inline void TelemetryDecoderAdapterBase::InitializeDecoder(telemetry_impl_base_sptr decoder)
{
    telemetry_decoder_ = std::move(decoder);
    telemetry_decoder_adapter_detail::LogDecoderInstance(telemetry_decoder_);
    if (in_streams_ > 1)
        {
            telemetry_decoder_adapter_detail::LogInputStreamsError(in_streams_);
        }
    if (out_streams_ > 1)
        {
            telemetry_decoder_adapter_detail::LogOutputStreamsError(out_streams_);
        }
}

inline void TelemetryDecoderAdapterBase::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        }
    telemetry_decoder_adapter_detail::LogConnect();
}

inline void TelemetryDecoderAdapterBase::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        }
}

inline gr::basic_block_sptr TelemetryDecoderAdapterBase::get_left_block()
{
    return telemetry_decoder_;
}

inline gr::basic_block_sptr TelemetryDecoderAdapterBase::get_right_block()
{
    return telemetry_decoder_;
}

inline void TelemetryDecoderAdapterBase::set_satellite(const Gnss_Satellite& satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    if (telemetry_decoder_)
        {
            telemetry_decoder_->set_satellite(satellite_);
        }
    telemetry_decoder_adapter_detail::LogSatelliteChange(satellite_);
}

inline std::string TelemetryDecoderAdapterBase::role()
{
    return role_;
}

inline void TelemetryDecoderAdapterBase::set_channel(int channel)
{
    if (telemetry_decoder_)
        {
            telemetry_decoder_->set_channel(channel);
        }
}

inline void TelemetryDecoderAdapterBase::reset()
{
    if (telemetry_decoder_)
        {
            telemetry_decoder_->reset();
        }
}

inline size_t TelemetryDecoderAdapterBase::item_size()
{
    return sizeof(Gnss_Synchro);
}

inline Tlm_Conf& TelemetryDecoderAdapterBase::tlm_parameters()
{
    return tlm_parameters_;
}

inline const Gnss_Satellite& TelemetryDecoderAdapterBase::satellite() const
{
    return satellite_;
}

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H

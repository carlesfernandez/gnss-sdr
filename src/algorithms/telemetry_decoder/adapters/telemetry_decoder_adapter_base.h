/*!
 * \file telemetry_decoder_adapter_base.h
 * \brief Common functionality for telemetry decoder adapters
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
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
#include "tracking_impl_adapter.h"
#include <gnuradio/runtime_types.h>
#include <cstddef>
#include <string>
#include <type_traits>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

class ConfigurationInterface;

/** \addtogroup Telemetry_Decoder
 * \{
 */
/** \addtogroup Telemetry_Decoder_adapters
 * \{
 */

struct TelemetryParametersCallbackNone
{
    void operator()(Tlm_Conf&, const ConfigurationInterface*, const std::string&) const {}
};

template <typename DecoderSptr>
class TelemetryDecoderAdapterBase : public TelemetryDecoderInterface
{
public:
    template <typename DecoderFactory, typename ParametersCallback = TelemetryParametersCallbackNone>
    TelemetryDecoderAdapterBase(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        DecoderFactory&& decoder_factory,
        ParametersCallback&& parameters_callback = ParametersCallback{})
        : role_(role),
          in_streams_(in_streams),
          out_streams_(out_streams)
    {
        DLOG(INFO) << "role " << role;
        if (configuration != nullptr)
            {
                tlm_parameters_.SetFromConfiguration(configuration, role);
                std::forward<ParametersCallback>(parameters_callback)(tlm_parameters_, configuration, role);
            }
        telemetry_decoder_ = std::forward<DecoderFactory>(decoder_factory)(satellite_, tlm_parameters_);

        static_assert(std::is_base_of<tracking_impl_adapter, typename DecoderSptr::element_type>::value,
            "Telemetry decoders must inherit from tracking_impl_adapter");

        DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";

        if (in_streams_ > 1)
            {
                LOG(ERROR) << "This implementation only supports one input stream";
            }
        if (out_streams_ > 1)
            {
                LOG(ERROR) << "This implementation only supports one output stream";
            }
    }

    void connect(gr::top_block_sptr top_block) override
    {
        if (top_block)
            {
                /* top_block is not null */
            }
        DLOG(INFO) << "nothing to connect internally";
    }

    void disconnect(gr::top_block_sptr top_block) override
    {
        if (top_block)
            {
                /* top_block is not null */
            }
        DLOG(INFO) << "nothing to connect internally";
    }

    gr::basic_block_sptr get_left_block() override
    {
        return telemetry_decoder_;
    }

    gr::basic_block_sptr get_right_block() override
    {
        return telemetry_decoder_;
    }

    void set_satellite(const Gnss_Satellite& satellite) override
    {
        satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
        telemetry_decoder_->set_satellite(satellite_);
        DLOG(INFO) << satellite.get_system() << " telemetry decoder: satellite set to " << satellite_;
    }

    inline std::string role() override
    {
        return role_;
    }

    void set_channel(int channel) override
    {
        telemetry_decoder_->set_channel(channel);
    }

    void reset() override
    {
        telemetry_decoder_->reset();
    }

    inline size_t item_size() override
    {
        return sizeof(Gnss_Synchro);
    }

protected:
    DecoderSptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    Tlm_Conf tlm_parameters_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H

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

 #include "telemetry_decoder_adapter_base.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

TelemetryDecoderAdapterBase::TelemetryDecoderAdapterBase(const ConfigurationInterface* configuration,
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
/*!
 * \file signal_conditioner.cc
 * \brief It holds blocks to change data type, filter and resample input data.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "signal_conditioner.h"
#include <gnuradio/io_signature.h>
#include <initializer_list>
#include <stdexcept>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

// Constructor
SignalConditioner::SignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
    std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res,
    std::string role) : data_type_adapt_(std::move(data_type_adapt)),
                        in_filt_(std::move(in_filt)),
                        res_(std::move(res)),
                        role_(std::move(role)),
                        connected_(false)
{
}


void SignalConditioner::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "Signal conditioner already connected internally";
            return;
        }
    if (data_type_adapt_ == nullptr)
        {
            throw std::invalid_argument("DataTypeAdapter implementation not defined");
        }
    if (in_filt_ == nullptr)
        {
            throw std::invalid_argument("InputFilter implementation not defined");
        }
    if (res_ == nullptr)
        {
            throw std::invalid_argument("Resampler implementation not defined");
        }
    data_type_adapt_->connect(top_block);
    in_filt_->connect(top_block);
    res_->connect(top_block);

    if (in_filt_->item_size() == 0)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/output data type configuration for the InputFilter");
        }

    // Item sizes are validated for every stage, including the identity stages
    // that are bypassed below, so that configuration errors are still reported.
    const size_t data_type_adapter_output_size = output_item_size(data_type_adapt_);
    const size_t input_filter_input_size = input_item_size(in_filt_);
    const size_t input_filter_output_size = output_item_size(in_filt_);
    const size_t resampler_input_size = input_item_size(res_);

    if (data_type_adapter_output_size != input_filter_input_size)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/output data type configuration for the DataTypeAdapter/InputFilter connection");
        }

    if (input_filter_output_size != resampler_input_size)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/output data type configuration for the Input Filter/Resampler connection");
        }

    // Chain the stages that actually process samples; identity stages are left out
    const auto stages = processing_stages();
    for (size_t i = 1; i < stages.size(); i++)
        {
            top_block->connect(stages[i - 1]->get_right_block(), 0, stages[i]->get_left_block(), 0);
            DLOG(INFO) << stages[i - 1]->role() << " -> " << stages[i]->role();
        }
    if (stages.size() < 3)
        {
            LOG(INFO) << role_ << ": " << (3 - stages.size()) << " identity stage(s) bypassed";
        }
    connected_ = true;
}


void SignalConditioner::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Signal conditioner already disconnected internally";
            return;
        }

    const auto stages = processing_stages();
    for (size_t i = 1; i < stages.size(); i++)
        {
            top_block->disconnect(stages[i - 1]->get_right_block(), 0, stages[i]->get_left_block(), 0);
        }

    data_type_adapt_->disconnect(top_block);
    in_filt_->disconnect(top_block);
    res_->disconnect(std::move(top_block));

    connected_ = false;
}


gr::basic_block_sptr SignalConditioner::get_left_block()
{
    const auto stages = processing_stages();
    if (stages.empty())
        {
            // Entirely identity: a single fallback endpoint serves both sides
            return res_->get_left_block();
        }
    return stages.front()->get_left_block();
}


gr::basic_block_sptr SignalConditioner::get_right_block()
{
    const auto stages = processing_stages();
    if (stages.empty())
        {
            return res_->get_right_block();
        }
    return stages.back()->get_right_block();
}


bool SignalConditioner::is_identity() const
{
    return (data_type_adapt_ != nullptr) && data_type_adapt_->is_identity() &&
           (in_filt_ != nullptr) && in_filt_->is_identity() &&
           (res_ != nullptr) && res_->is_identity();
}


std::vector<std::shared_ptr<GNSSBlockInterface>> SignalConditioner::processing_stages() const
{
    std::vector<std::shared_ptr<GNSSBlockInterface>> stages;
    for (const auto& stage : {data_type_adapt_, in_filt_, res_})
        {
            if ((stage != nullptr) && !stage->is_identity())
                {
                    stages.push_back(stage);
                }
        }
    return stages;
}


size_t SignalConditioner::input_item_size(const std::shared_ptr<GNSSBlockInterface>& stage)
{
    if (stage->is_identity())
        {
            return stage->item_size();
        }
    return stage->get_left_block()->input_signature()->sizeof_stream_item(0);
}


size_t SignalConditioner::output_item_size(const std::shared_ptr<GNSSBlockInterface>& stage)
{
    if (stage->is_identity())
        {
            return stage->item_size();
        }
    return stage->get_right_block()->output_signature()->sizeof_stream_item(0);
}

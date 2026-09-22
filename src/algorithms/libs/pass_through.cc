/*!
 * \file pass_through.cc
 * \brief Implementation of a block that just puts its input in its
 *        output.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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

#include "pass_through.h"
#include "configuration_interface.h"
#include <gnuradio/gr_complex.h>
#include <volk/volk_complex.h>
#include <cstdint>  // for int8_t
#include <ostream>  // for operator<<
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

Pass_Through::Pass_Through(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      max_source_buffer_samples_(configuration->property("GNSS-SDR.max_source_buffer_samples", uint64_t(0))),
      in_streams_(in_streams),
      out_streams_(out_streams),
      inverted_spectrum(configuration->property(role + ".inverted_spectrum", false))
{
    const std::string default_item_type("gr_complex");
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    if (item_type_ == "float")
        {
            item_size_ = sizeof(float);
        }
    else if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            if (inverted_spectrum)
                {
                    conjugate_cc_ = make_conjugate_cc();
                }
        }
    else if ((item_type_ == "short") || (item_type_ == "ishort"))
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "cshort")
        {
            item_size_ = sizeof(lv_16sc_t);
            if (inverted_spectrum)
                {
                    conjugate_sc_ = make_conjugate_sc();
                }
        }
    else if ((item_type_ == "byte") || (item_type_ == "ibyte"))
        {
            item_size_ = sizeof(int8_t);
        }
    else if (item_type_ == "cbyte")
        {
            item_size_ = sizeof(lv_8sc_t);
            if (inverted_spectrum)
                {
                    conjugate_ic_ = make_conjugate_ic();
                }
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using float";
            item_size_ = sizeof(float);
        }

    if (inverted_spectrum && is_identity())
        {
            LOG(WARNING) << "Setting inverted_spectrum to true with item_type "
                         << item_type_ << " is not defined and has no effect.";
        }

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream but it is set to " << in_streams_;
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream but it is set to " << out_streams_;
        }
}


bool Pass_Through::is_identity() const
{
    return conjugate_block() == nullptr;
}


void Pass_Through::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "nothing to connect internally";
}


void Pass_Through::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr Pass_Through::get_left_block()
{
    const gr::basic_block_sptr conjugate = conjugate_block();
    if (conjugate)
        {
            return conjugate;
        }
    return copy_block();
}


gr::basic_block_sptr Pass_Through::get_right_block()
{
    const gr::basic_block_sptr conjugate = conjugate_block();
    if (conjugate)
        {
            return conjugate;
        }
    return copy_block();
}


gr::basic_block_sptr Pass_Through::conjugate_block() const
{
    if (conjugate_cc_)
        {
            return conjugate_cc_;
        }
    if (conjugate_sc_)
        {
            return conjugate_sc_;
        }
    if (conjugate_ic_)
        {
            return conjugate_ic_;
        }
    return nullptr;
}


gr::basic_block_sptr Pass_Through::copy_block()
{
    // Fallback endpoint for callers that connect an identity block instead
    // of bypassing it. It is created on demand so that bypassed paths never
    // instantiate (nor schedule) a copy block.
    if (!kludge_copy_)
        {
            kludge_copy_ = gr::blocks::copy::make(item_size_);
            if (max_source_buffer_samples_ > 0)
                {
                    kludge_copy_->set_max_output_buffer(max_source_buffer_samples_);
                    LOG(INFO) << "Set signal conditioner max output buffer to " << max_source_buffer_samples_;
                }
            DLOG(INFO) << "kludge_copy(" << kludge_copy_->unique_id() << ")";
        }
    return kludge_copy_;
}

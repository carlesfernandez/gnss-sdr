/*!
 * \file signal_conditioner.h
 * \brief It wraps blocks to change data type, filter and resample input data.
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

#ifndef GNSS_SDR_SIGNAL_CONDITIONER_H
#define GNSS_SDR_SIGNAL_CONDITIONER_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Signal_Conditioner Signal Conditioner
 * Signal Conditioner wrapper block
 * \{ */
/** \addtogroup Signal_Conditioner_adapters conditioner_adapters
 * Wrap a Signal Conditioner with a GNSSBlockInterface
 * \{ */


/*!
 * \brief This class wraps blocks to change data_type_adapter, input_filter and resampler
 * to be applied to the input flow of sampled signal.
 *
 * Stages reporting is_identity() are not connected: the remaining stages are
 * chained directly, and get_left_block() / get_right_block() resolve to the
 * first and last stages that actually process samples. When every stage is an
 * identity, the whole conditioner reports is_identity() and the flow graph is
 * expected to bypass it.
 */
class SignalConditioner : public GNSSBlockInterface
{
public:
    //! Constructor
    SignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
        std::shared_ptr<GNSSBlockInterface> in_filt,
        std::shared_ptr<GNSSBlockInterface> res,
        std::string role);

    //! Destructor
    ~SignalConditioner() = default;

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    //! True when the data type adapter, the input filter and the resampler are all identities
    bool is_identity() const override;

    inline std::string role() override { return role_; }

    inline std::string implementation() override { return "Signal_Conditioner"; }  //!< Returns "Signal_Conditioner"

    inline size_t item_size() override { return data_type_adapt_->item_size(); }

    inline std::shared_ptr<GNSSBlockInterface> data_type_adapter() { return data_type_adapt_; }
    inline std::shared_ptr<GNSSBlockInterface> input_filter() { return in_filt_; }
    inline std::shared_ptr<GNSSBlockInterface> resampler() { return res_; }

private:
    std::vector<std::shared_ptr<GNSSBlockInterface>> processing_stages() const;
    static size_t input_item_size(const std::shared_ptr<GNSSBlockInterface>& stage);
    static size_t output_item_size(const std::shared_ptr<GNSSBlockInterface>& stage);

    std::shared_ptr<GNSSBlockInterface> data_type_adapt_;
    std::shared_ptr<GNSSBlockInterface> in_filt_;
    std::shared_ptr<GNSSBlockInterface> res_;
    std::string role_;
    bool connected_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SIGNAL_CONDITIONER_H

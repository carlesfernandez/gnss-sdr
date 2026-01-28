/*!
 * \file dll_pll_veml_tracking.h
 * \brief Implementation of a code DLL + carrier PLL tracking block.
 * \author Javier Arribas, 2018-2025. jarribas(at)cttc.es
 * \author Carles Fernandez-Prades, 2018-2025 carles.fernandez(at)cttc.es
 * \author Antonio Ramos, 2018 antonio.ramosdet(at)gmail.com
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

#ifndef GNSS_SDR_DLL_PLL_VEML_TRACKING_H
#define GNSS_SDR_DLL_PLL_VEML_TRACKING_H

#include "cpu_multicorrelator_real_codes.h"
#include "dll_pll_conf.h"
#include "exponential_smoother.h"
#include "gnss_block_interface.h"
#include "gnss_time.h"  // for timetags produced by File_Timestamp_Signal_Source
#include "tow_to_trk.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>                   // for block
#include <gnuradio/gr_complex.h>              // for gr_complex
#include <gnuradio/types.h>                   // for gr_vector_int, gr_vector...
#include <pmt/pmt.h>                          // for pmt_t
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <algorithm>                          // for fill
#include <cmath>                              // for abs
#include <complex>                            // for complex
#include <cstddef>                            // for size_t
#include <cstdint>                            // for int32_t
#include <fstream>                            // for ofstream
#include <string>                             // for string
#include <typeinfo>                           // for typeid
#include <utility>                            // for pair
#include <vector>                             // for vector

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_gnuradio_blocks tracking_gr_blocks
 * GNU Radio blocks for GNSS signal tracking.
 * \{ */


class Gnss_Synchro;
class dll_pll_veml_tracking;

using dll_pll_veml_tracking_sptr = gnss_shared_ptr<dll_pll_veml_tracking>;

dll_pll_veml_tracking_sptr dll_pll_veml_make_tracking(const Dll_Pll_Conf &conf_);

class HistogramBitSynchronizer
{
public:
    struct Config
    {
        int bit_period_ms;
        int epoch_ms;
        int min_events_for_lock;
        double dominance_ratio;
        int stable_best_required;
        float min_prompt_mag;
        bool use_phase_dot_detector;

        Config()
            : bit_period_ms(20),
              epoch_ms(1),
              min_events_for_lock(30),
              dominance_ratio(0.55),
              stable_best_required(5),
              min_prompt_mag(0.0f),
              use_phase_dot_detector(true)
        {
        }
    };

    explicit HistogramBitSynchronizer(const Config &cfg)
        : cfg_(cfg),
          total_events_(0),
          epoch_count_(0),
          locked_(false),
          edge_phase_(-1),
          has_last_prompt_(false),
          last_prompt_(0.0f, 0.0f),
          has_last_sign_(false),
          last_sign_(+1),
          has_last_best_bin_(false),
          last_best_bin_(0),
          stable_best_count_(0)
    {
        hist_.assign(bins(), 0);
    }

    void reset()
    {
        std::fill(hist_.begin(), hist_.end(), 0);
        total_events_ = 0;
        epoch_count_ = 0;
        locked_ = false;
        edge_phase_ = -1;

        has_last_prompt_ = false;
        last_prompt_ = std::complex<float>(0.0f, 0.0f);

        has_last_sign_ = false;
        last_sign_ = +1;

        has_last_best_bin_ = false;
        last_best_bin_ = 0;
        stable_best_count_ = 0;
    }

    bool update(const std::complex<float> &prompt, bool tracking_quality_ok)
    {
        const int N = bins();
        const int phase = (N > 0) ? static_cast<int>(epoch_count_ % N) : 0;

        ++epoch_count_;

        if (!tracking_quality_ok || (std::abs(prompt) < cfg_.min_prompt_mag))
            {
                last_prompt_ = prompt;
                has_last_prompt_ = true;
                return false;
            }

        bool edge_event = false;

        if (cfg_.use_phase_dot_detector)
            {
                if (has_last_prompt_)
                    {
                        const double dot = static_cast<double>(std::real(prompt * std::conj(last_prompt_)));
                        edge_event = (dot < 0.0);
                    }
                last_prompt_ = prompt;
                has_last_prompt_ = true;
            }
        else
            {
                const int s = (std::real(prompt) >= 0.0f) ? +1 : -1;
                if (has_last_sign_)
                    {
                        edge_event = (s != last_sign_);
                    }
                last_sign_ = s;
                has_last_sign_ = true;
            }

        if (edge_event && N > 0)
            {
                ++hist_[phase];
                ++total_events_;
            }

        if (!locked_ && (total_events_ >= cfg_.min_events_for_lock))
            {
                int best_bin = 0;
                int best_count = 0;
                best_bin_and_count(best_bin, best_count);

                const double ratio = (total_events_ > 0)
                                         ? (static_cast<double>(best_count) / static_cast<double>(total_events_))
                                         : 0.0;

                if (!has_last_best_bin_ || (best_bin != last_best_bin_))
                    {
                        last_best_bin_ = best_bin;
                        has_last_best_bin_ = true;
                        stable_best_count_ = 1;
                    }
                else
                    {
                        ++stable_best_count_;
                    }

                if ((ratio >= cfg_.dominance_ratio) &&
                    (stable_best_count_ >= cfg_.stable_best_required))
                    {
                        locked_ = true;
                        edge_phase_ = best_bin;
                        return true;
                    }
            }

        return false;
    }

    bool locked() const { return locked_; }
    int edge_phase() const { return edge_phase_; }

    bool is_edge_epoch(std::int64_t k) const
    {
        if (!locked_ || edge_phase_ < 0) return false;
        const int N = bins();
        if (N <= 0) return false;
        return (static_cast<int>(k % N) == edge_phase_);
    }

    int bins() const
    {
        const int N = (cfg_.epoch_ms > 0) ? (cfg_.bit_period_ms / cfg_.epoch_ms) : 0;
        return (N > 0) ? N : 0;
    }

    const std::vector<int> &histogram() const { return hist_; }
    std::int64_t total_events() const { return total_events_; }
    std::int64_t epoch_count() const { return epoch_count_; }

private:
    void best_bin_and_count(int &best_bin, int &best_count) const
    {
        best_bin = 0;
        best_count = (hist_.empty() ? 0 : hist_[0]);
        for (int i = 1; i < static_cast<int>(hist_.size()); ++i)
            {
                if (hist_[i] > best_count)
                    {
                        best_count = hist_[i];
                        best_bin = i;
                    }
            }
    }

    Config cfg_;
    std::vector<int> hist_;
    std::int64_t total_events_;
    std::int64_t epoch_count_;

    bool locked_;
    int edge_phase_;

    bool has_last_prompt_;
    std::complex<float> last_prompt_;

    bool has_last_sign_;
    int last_sign_;

    bool has_last_best_bin_;
    int last_best_bin_;
    int stable_best_count_;
};

/*!
 * \brief This class implements a code DLL + carrier PLL tracking block.
 */
class dll_pll_veml_tracking : public gr::block
{
public:
    ~dll_pll_veml_tracking() override;

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();
    void stop_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

    void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

private:
    friend dll_pll_veml_tracking_sptr dll_pll_veml_make_tracking(const Dll_Pll_Conf &conf_);
    explicit dll_pll_veml_tracking(const Dll_Pll_Conf &conf_);

    void msg_handler_telemetry_to_trk(const pmt::pmt_t &msg);
    void do_correlation_step(const gr_complex *input_samples);
    void run_dll_pll();
    void check_carrier_phase_coherent_initialization();
    void update_tracking_vars();
    void clear_tracking_vars();
    void save_correlation_results();
    void log_data();
    bool cn0_and_tracking_lock_status(double coh_integration_time_s);
    bool acquire_secondary();
    void configure_bit_synchronizer();
    int64_t uint64diff(uint64_t first, uint64_t second);
    int32_t save_matfile() const;

    Cpu_Multicorrelator_Real_Codes d_multicorrelator_cpu;
    Cpu_Multicorrelator_Real_Codes d_correlator_data_cpu;  // for data channel

    Dll_Pll_Conf d_trk_parameters;

    Exponential_Smoother d_cn0_smoother;
    Exponential_Smoother d_carrier_lock_test_smoother;

    Tracking_loop_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    Gnss_Synchro *d_acquisition_gnss_synchro;

    volk_gnsssdr::vector<float> d_tracking_code;
    volk_gnsssdr::vector<float> d_data_code;
    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<gr_complex> d_correlator_outs;
    volk_gnsssdr::vector<gr_complex> d_Prompt_Data;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;

    boost::circular_buffer<float> d_dll_filt_history;
    boost::circular_buffer<std::pair<double, double>> d_code_ph_history;
    boost::circular_buffer<std::pair<double, double>> d_carr_ph_history;
    boost::circular_buffer<gr_complex> d_Prompt_circular_buffer;

    const size_t d_int_type_hash_code = typeid(int).hash_code();
    const size_t d_tow_to_trk_type_hash_code = typeid(std::shared_ptr<TOW_to_trk>).hash_code();

    double d_signal_carrier_freq;
    double d_code_period;
    double d_code_chip_rate;
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    double d_current_correlation_time_s;
    double d_carr_phase_error_hz;
    double d_carr_freq_error_hz;
    double d_carr_error_filt_hz;
    double d_code_error_chips;
    double d_code_error_filt_chips;
    double d_code_freq_chips;
    double d_cfo_frequency_hz;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_rem_code_phase_chips;
    double d_T_chip_seconds;
    double d_T_prn_seconds;
    double d_T_prn_samples;
    double d_K_blk_samples;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_rem_code_phase_samples;

    gr_complex *d_Very_Early;
    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Very_Late;

    gr_complex d_VE_accu;
    gr_complex d_E_accu;
    gr_complex d_P_accu;
    gr_complex d_P_accu_old;
    gr_complex d_L_accu;
    gr_complex d_VL_accu;
    gr_complex d_P_data_accu;

    std::string d_secondary_code_string;
    std::string d_data_secondary_code_string;
    std::string d_systemName;
    std::string d_signal_type;
    std::string d_signal_pretty_name;
    std::string d_dump_filename;

    std::ofstream d_dump_file;

    // uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;
    GnssTime d_last_timetag{};
    std::shared_ptr<TOW_to_trk> d_last_tow_received;
    uint64_t d_last_timetag_samplecounter;
    bool d_timetag_waiting;

    float *d_prompt_data_shift;
    float d_rem_carr_phase_rad;

    uint64_t d_tow_from_telemetry_ms{};
    int32_t d_wn_from_telemetry{};

    int32_t d_symbols_per_bit;
    int32_t d_state;
    int32_t d_correlation_length_ms;
    int32_t d_n_correlator_taps;
    int32_t d_current_prn_length_samples;
    int32_t d_extend_correlation_symbols_count;
    int32_t d_extend_correlation_symbols;
    int32_t d_current_symbol;
    int32_t d_current_data_symbol;
    int32_t d_cn0_estimation_counter;
    int32_t d_carrier_lock_fail_counter;
    int32_t d_code_lock_fail_counter;
    int32_t d_code_samples_per_chip;  // All signals have 1 sample per chip code except Gal. E1 which has 2 (CBOC disabled) or 12 (CBOC enabled)
    int32_t d_code_length_chips;

    uint32_t d_channel;
    uint32_t d_secondary_code_length;
    uint32_t d_data_secondary_code_length;

    bool d_pull_in_transitory;
    bool d_corrected_doppler;
    bool d_interchange_iq;
    bool d_veml;
    bool d_cloop;
    bool d_secondary;
    bool d_dump;
    bool d_dump_mat;
    bool d_acc_carrier_phase_initialized;
    bool d_enable_extended_integration;
    bool d_Flag_PLL_180_deg_phase_locked;
    bool d_use_histogram_bit_sync;
    bool d_wait_for_bit_edge;
    std::int64_t d_bit_sync_lock_epoch;
    HistogramBitSynchronizer d_bit_sync;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_VEML_TRACKING_H

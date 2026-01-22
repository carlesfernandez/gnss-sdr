/*!
 * \file bit_synchronizer.cc
 * \brief Histogram-based bit-edge synchronizer for GNSS prompt correlator outputs.
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "bit_synchronizer.h"
#include <algorithm>

void HistogramBitSynchronizer::reset()
{
    if (cfg_.scheme == HistogramBitSynchronizer::Scheme::kGlonassBiphase)
        {
            glonass_sync_.reset();
            return;
        }

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


bool HistogramBitSynchronizer::update(const std::complex<float>& prompt, bool tracking_quality_ok)
{
    if (cfg_.scheme == HistogramBitSynchronizer::Scheme::kGlonassBiphase)
        {
            return glonass_sync_.update(prompt, tracking_quality_ok);
        }

    const int N = bins();
    const int phase = (N > 0) ? static_cast<int>(epoch_count_ % N) : 0;

    // Always advance epoch counter; even if gated out we keep phase consistent.
    ++epoch_count_;

    // Gate on tracking status and magnitude
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
                    // dot = Re( Pk * conj(Pk-1) ); negative suggests polarity inversion
                    const double dot = static_cast<double>(std::real(prompt * std::conj(last_prompt_)));
                    edge_event = (dot < 0.0);
                }
            // update last prompt after using it
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

    // Evaluate lock condition
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
                    return true;  // lock event
                }
        }

    return false;
}


bool HistogramBitSynchronizer::is_histogram_edge_epoch(std::int64_t k) const
{
    if (!locked_ || edge_phase_ < 0) return false;
    const int N = bins();
    if (N <= 0) return false;
    return (static_cast<int>(k % N) == edge_phase_);
}


int HistogramBitSynchronizer::histogram_bins() const
{
    const int N = (cfg_.epoch_ms > 0) ? (cfg_.bit_period_ms / cfg_.epoch_ms) : 0;
    return (N > 0) ? N : 0;
}


void HistogramBitSynchronizer::best_bin_and_count(int& best_bin, int& best_count) const
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


GlonassBiphaseSymbolSynchronizer::Config HistogramBitSynchronizer::build_glonass_config(const Config& cfg)
{
    GlonassBiphaseSymbolSynchronizer::Config glonass_cfg;
    glonass_cfg.symbol_period_ms = cfg.bit_period_ms;
    glonass_cfg.epoch_ms = cfg.epoch_ms;
    glonass_cfg.min_windows_for_lock = cfg.min_events_for_lock;
    glonass_cfg.stable_best_required = cfg.stable_best_required;
    glonass_cfg.min_prompt_mag = cfg.min_prompt_mag;
    glonass_cfg.min_norm_score = cfg.dominance_ratio;
    glonass_cfg.use_phase_dot_sign = cfg.use_phase_dot_detector;
    return glonass_cfg;
}


GlonassBiphaseSymbolSynchronizer::GlonassBiphaseSymbolSynchronizer(const Config& cfg)
    : cfg_(cfg),
      N_(compute_bins(cfg)),
      s_(),
      template_(),
      fill_(0),
      epoch_count_(0),
      windows_(0),
      locked_(false),
      symbol_phase_(-1),
      has_last_prompt_(false),
      last_prompt_(0.0f, 0.0f),
      has_last_sign_(false),
      last_sign_(+1),
      has_last_best_(false),
      last_best_(0),
      stable_count_(0)
{
    s_.assign(N_, 0);
    template_.assign(N_, 0);
    for (int i = 0; i < N_; ++i)
        {
            template_[i] = (i < (N_ / 2)) ? +1 : -1;
        }
}


void GlonassBiphaseSymbolSynchronizer::reset()
{
    std::fill(s_.begin(), s_.end(), 0);
    fill_ = 0;
    epoch_count_ = 0;
    windows_ = 0;
    locked_ = false;
    symbol_phase_ = -1;

    has_last_prompt_ = false;
    last_prompt_ = std::complex<float>(0.0f, 0.0f);

    has_last_sign_ = false;
    last_sign_ = +1;

    has_last_best_ = false;
    last_best_ = 0;
    stable_count_ = 0;
}


bool GlonassBiphaseSymbolSynchronizer::update(const std::complex<float>& prompt, bool tracking_quality_ok)
{
    if (N_ <= 0)
        {
            ++epoch_count_;
            return false;
        }

    ++epoch_count_;

    if (!tracking_quality_ok || (std::abs(prompt) < cfg_.min_prompt_mag))
        {
            last_prompt_ = prompt;
            has_last_prompt_ = true;
            return false;
        }

    const int sign = compute_sign(prompt);
    push_sign(sign);

    if (!buffer_full())
        {
            return false;
        }

    int best_p = 0;
    int best_score = -1;

    for (int p = 0; p < N_; ++p)
        {
            const int score = std::abs(correlation_score(p));
            if (score > best_score)
                {
                    best_score = score;
                    best_p = p;
                }
        }

    ++windows_;

    const double norm_score = static_cast<double>(best_score) / static_cast<double>(N_);

    if (!has_last_best_ || (best_p != last_best_))
        {
            last_best_ = best_p;
            has_last_best_ = true;
            stable_count_ = 1;
        }
    else
        {
            ++stable_count_;
        }

    if (!locked_ &&
        (windows_ >= cfg_.min_windows_for_lock) &&
        (norm_score >= cfg_.min_norm_score) &&
        (stable_count_ >= cfg_.stable_best_required))
        {
            locked_ = true;
            symbol_phase_ = best_p;
            return true;
        }

    return false;
}


bool GlonassBiphaseSymbolSynchronizer::is_symbol_epoch(std::int64_t k) const
{
    if (!locked_ || symbol_phase_ < 0 || N_ <= 0) return false;
    return (static_cast<int>(k % N_) == symbol_phase_);
}


int GlonassBiphaseSymbolSynchronizer::compute_bins(const Config& cfg)
{
    if (cfg.epoch_ms <= 0) return 0;
    const int N = cfg.symbol_period_ms / cfg.epoch_ms;
    return (N > 0) ? N : 0;
}


int GlonassBiphaseSymbolSynchronizer::compute_sign(const std::complex<float>& prompt)
{
    if (cfg_.use_phase_dot_sign)
        {
            int s = +1;
            if (has_last_prompt_)
                {
                    const double dot = static_cast<double>(std::real(prompt * std::conj(last_prompt_)));
                    s = (dot >= 0.0) ? +1 : -1;
                }
            last_prompt_ = prompt;
            has_last_prompt_ = true;
            return s;
        }

    const int s = (std::real(prompt) >= 0.0f) ? +1 : -1;
    has_last_sign_ = true;
    last_sign_ = s;
    return s;
}


void GlonassBiphaseSymbolSynchronizer::push_sign(int s)
{
    if (N_ <= 0)
        {
            return;
        }

    for (int i = 0; i < N_ - 1; ++i)
        {
            s_[i] = s_[i + 1];
        }
    s_[N_ - 1] = s;

    if (fill_ < N_) ++fill_;
}


int GlonassBiphaseSymbolSynchronizer::correlation_score(int phase) const
{
    int sum = 0;
    for (int i = 0; i < N_; ++i)
        {
            const int t = template_[(i + phase) % N_];
            sum += s_[i] * t;
        }
    return sum;
}

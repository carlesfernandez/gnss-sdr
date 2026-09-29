/*!
 * \file fpga_dynamic_bit_selection.cc
 * \brief Dynamic Bit Selection in the received signal.
 * \authors <ul>
 *    <li> Marc Majoral, 2023. mmajoral(at)cttc.es
 * </ul>
 *
 * Class that controls the Dynamic Bit Selection in the FPGA.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fpga_dynamic_bit_selection.h"
#include "uio_fpga.h"
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout
#include <sys/mman.h>  // for mmap
#include <unistd.h>    // for close

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

Fpga_dynamic_bit_selection::Fpga_dynamic_bit_selection(bool enable_rx1_band, bool enable_rx2_band)
    : d_map_base_freq_band_1(nullptr),
      d_map_base_freq_band_2(nullptr),
      d_dev_descr_freq_band_1(0),
      d_dev_descr_freq_band_2(0),
      d_shift_out_bits_freq_band_1(0),
      d_shift_out_bits_freq_band_2(0),
      d_enable_rx1_band(enable_rx1_band),
      d_enable_rx2_band(enable_rx2_band)
{
    if (d_enable_rx1_band)
        {
            if (open_device(&d_map_base_freq_band_1, d_dev_descr_freq_band_1, 0))
                {
                    // init bit selection corresponding to frequency band 1
                    d_shift_out_bits_freq_band_1 = shift_out_bits_default;
                    d_map_base_freq_band_1[0] = d_shift_out_bits_freq_band_1;
                }
            else
                {
                    // the FPGA does not provide dynamic bit selection for this band: disable it
                    d_enable_rx1_band = false;
                    LOG(WARNING) << "Dynamic bit selection disabled in frequency band 1";
                    std::cout << "Dynamic bit selection disabled in frequency band 1\n";
                }
        }
    if (d_enable_rx2_band)
        {
            if (open_device(&d_map_base_freq_band_2, d_dev_descr_freq_band_2, 1))
                {
                    // init bit selection corresponding to frequency band 2
                    d_shift_out_bits_freq_band_2 = shift_out_bits_default;
                    d_map_base_freq_band_2[0] = d_shift_out_bits_freq_band_2;
                }
            else
                {
                    // the FPGA does not provide dynamic bit selection for this band: disable it
                    d_enable_rx2_band = false;
                    LOG(WARNING) << "Dynamic bit selection disabled in frequency band 2";
                    std::cout << "Dynamic bit selection disabled in frequency band 2\n";
                }
        }
    DLOG(INFO) << "Dynamic bit selection FPGA class created";
}


Fpga_dynamic_bit_selection::~Fpga_dynamic_bit_selection()
{
    if (d_enable_rx1_band)
        {
            close_device(d_map_base_freq_band_1, d_dev_descr_freq_band_1);
        }
    if (d_enable_rx2_band)
        {
            close_device(d_map_base_freq_band_2, d_dev_descr_freq_band_2);
        }
}


void Fpga_dynamic_bit_selection::bit_selection()
{
    if (d_enable_rx1_band)
        {
            bit_selection_per_rf_band(d_map_base_freq_band_1, d_shift_out_bits_freq_band_1);
        }

    if (d_enable_rx2_band)
        {
            bit_selection_per_rf_band(d_map_base_freq_band_2, d_shift_out_bits_freq_band_2);
        }
}


bool Fpga_dynamic_bit_selection::open_device(volatile unsigned **d_map_base, int &d_dev_descr, int freq_band)
{
    *d_map_base = nullptr;
    d_dev_descr = -1;
    const int freq_band_number = freq_band + 1;

    // find the uio device file corresponding to the dynamic bit selector module of this frequency band.
    std::string device_name;
    if (find_uio_dev_file_name(device_name, dyn_bit_sel_device_name, freq_band) < 0)
        {
            std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << dyn_bit_sel_device_name << " in frequency band " << freq_band_number << '\n';
            LOG(WARNING) << "Cannot find the FPGA uio device file corresponding to device name " << dyn_bit_sel_device_name << " in frequency band " << freq_band_number;
            return false;
        }
    if ((d_dev_descr = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio " << device_name;
            std::cout << "Cannot open deviceio " << device_name << std::endl;
            return false;
        }
    void *map_base = mmap(nullptr, FPGA_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, d_dev_descr, 0);
    if (map_base == MAP_FAILED)
        {
            LOG(WARNING) << "Cannot map the FPGA dynamic bit selection module in frequency band " << freq_band_number << " into tracking memory";
            std::cout << "Could not map dynamic bit selection memory corresponding to frequency band " << freq_band_number << ".\n";
            close(d_dev_descr);
            d_dev_descr = -1;
            return false;
        }
    *d_map_base = reinterpret_cast<volatile unsigned *>(map_base);
    return true;
}


void Fpga_dynamic_bit_selection::bit_selection_per_rf_band(volatile unsigned *d_map_base, uint32_t &shift_out_bits)
{
    // estimated signal power
    uint32_t rx_signal_power = d_map_base[1];

    // dynamic bit selection
    if (rx_signal_power > Power_Threshold_High)
        {
            if (shift_out_bits < shift_out_bit_max)
                {
                    shift_out_bits = shift_out_bits + 1;
                }
        }
    else if (rx_signal_power < Power_Threshold_Low)
        {
            if (shift_out_bits > shift_out_bits_min)
                {
                    shift_out_bits = shift_out_bits - 1;
                }
        }

    // update bit selection corresponding to frequency band 1
    d_map_base[0] = shift_out_bits;
}


void Fpga_dynamic_bit_selection::close_device(volatile unsigned *d_map_base, int &d_dev_descr)
{
    auto *aux = const_cast<unsigned *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_dev_descr);
}
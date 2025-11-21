#include "telemetry_decoder_adapter_base.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
constexpr const char* kTelemetryDecoderLogPrefix = "TELEMETRY DECODER";
}

namespace telemetry_decoder_adapter_detail
{

void LogRole(const std::string& role)
{
    DLOG(INFO) << kTelemetryDecoderLogPrefix << ": role " << role;
}

void LogDecoderInstance(const tracking_impl_adapter_sptr& decoder)
{
    if (decoder)
        {
            DLOG(INFO) << kTelemetryDecoderLogPrefix << "(" << decoder->unique_id() << ")";
        }
}

void LogInputStreamsError(unsigned int in_streams)
{
    LOG(ERROR) << kTelemetryDecoderLogPrefix << ": this implementation only supports one input stream ("
                << in_streams << " provided)";
}

void LogOutputStreamsError(unsigned int out_streams)
{
    LOG(ERROR) << kTelemetryDecoderLogPrefix << ": this implementation only supports one output stream ("
                << out_streams << " provided)";
}

void LogConnect()
{
    DLOG(INFO) << kTelemetryDecoderLogPrefix << ": nothing to connect internally";
}

void LogSatelliteChange(const Gnss_Satellite& satellite)
{
    DLOG(INFO) << kTelemetryDecoderLogPrefix << ": satellite set to " << satellite;
}

}  // namespace telemetry_decoder_adapter_detail

TelemetryDecoderAdapterBase::TelemetryDecoderAdapterBase(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : configuration_(configuration),
                                role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    telemetry_decoder_adapter_detail::LogRole(role_);
    if (configuration_ != nullptr)
        {
            tlm_parameters_.SetFromConfiguration(configuration_, role_);
        }
}


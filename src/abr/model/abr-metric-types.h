#ifndef ABR_METRIC_TYPES_H
#define ABR_METRIC_TYPES_H

#include "ns3/ipv4-address.h"

#include <vector>

namespace ns3
{
namespace abr
{

struct NeighborTick
{
    Ipv4Address neighbor;
    uint32_t tick;
};

struct MetricBlock
{
    Ipv4Address owner;
    std::vector<NeighborTick> ticks;
};

} // namespace abr
} // namespace ns3

#endif // ABR_METRIC_TYPES_H
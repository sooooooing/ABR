#include "abr-ntable.h"

#include "abr-packet.h"

#include <algorithm>
#include <iomanip>
#include <limits>

namespace ns3
{
namespace abr
{

NeighborTableEntry::NeighborTableEntry(Ipv4Address neighbor, uint32_t assocTick)
    : m_neighbor(neighbor),
      m_assocTick(assocTick)
{
}

NeighborTableEntry::~NeighborTableEntry() = default;

NeighborTable::NeighborTable() = default;

bool
NeighborTable::HasNeighbor(Ipv4Address neighbor)
{
    return m_neighborTable.find(neighbor) != m_neighborTable.end();
}

// 해당 이웃과의 tick 반환
uint32_t
NeighborTable::GetAssocTick(Ipv4Address neighbor)
{
    auto it = m_neighborTable.find(neighbor);
    if (it == m_neighborTable.end())
    {
        return 0; // unknown neighbor
    }
    return it->second.GetAssocTick();
}

// 해당 이웃의 tick 삽입 (처음 본 이웃이면 tick=1로 시작, 이미 있으면 tick++)
void
NeighborTable::IncreaseTick(Ipv4Address sender, Ipv4Address receiver)
{
    Ipv4Address neighbor = sender;
    auto it = m_neighborTable.find(neighbor);

    // 처음 본 이웃이면 tick=1로 시작
    if (it == m_neighborTable.end())
    {
        m_neighborTable.emplace(neighbor, NeighborTableEntry(neighbor, 1));
        // NS_LOG_UNCOND("TICK-NEW  : me=" << receiver << " neighbor=" << neighbor << " tick=1"
        //                                 << " nt_size=" << m_neighborTable.size());
        return;
    }

    // 이미 있으면 tick++
    uint32_t t = it->second.GetAssocTick();
    if (t != std::numeric_limits<uint32_t>::max())
    {
        it->second.SetAssocTick(t + 1);
    }

    // NS_LOG_UNCOND("TICK-INC  : me=" << receiver << " neighbor=" << neighbor << " tick " << t
    //                                 << " -> " << it->second.GetAssocTick()
    //                                 << " nt_size=" << m_neighborTable.size());
}

// 해당 이웃 삭제
bool
NeighborTable::DeleteNeighbor(Ipv4Address neighbor)
{
    return m_neighborTable.erase(neighbor) > 0;
}

// 존재하는 모든 이웃 반환
std::vector<std::pair<Ipv4Address, uint32_t>>
NeighborTable::GetAllNeighbors() const
{
    std::vector<std::pair<Ipv4Address, uint32_t>> neighbors;
    for (const auto& entry : m_neighborTable)
    {
        neighbors.push_back(std::make_pair(entry.first, entry.second.GetAssocTick()));
    }
    return neighbors;
}

bool
NeighborTable::ResetTick(Ipv4Address neighbor)
{
    auto it = m_neighborTable.find(neighbor);
    if (it == m_neighborTable.end())
    {
        return false; // unknown neighbor
    }
    it->second.SetAssocTick(0);
    return true;
}

std::vector<NeighborTick>
NeighborTable::GetAllNeighborTicks() const
{
    std::vector<NeighborTick> ticks;
    ticks.reserve(m_neighborTable.size());

    for (const auto& entry : m_neighborTable)
    {
        NeighborTick nt;
        nt.neighbor = entry.first;
        nt.tick = entry.second.GetAssocTick();
        ticks.push_back(nt);
    }
    return ticks;
}

void
NeighborTable::Print(Ipv4Address address) const
{
    std::cout << "Neighbor" << std::setw(16) << "Tick" << std::endl;
    for (auto& entry : m_neighborTable)
    {
        std::cout << entry.first << std::setw(16) << entry.second.GetAssocTick() << std::endl;
    }
}

} // namespace abr
} // namespace ns3
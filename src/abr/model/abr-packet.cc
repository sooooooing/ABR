/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Based on
 *      NS-2 ABR model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 *
 *      ABR-UU implementation by Erik Nordström of Uppsala University
 *      https://web.archive.org/web/20100527072022/http://core.it.uu.se/core/index.php/ABR-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#include "abr-packet.h"

#include "ns3/address-utils.h"
#include "ns3/packet.h"

namespace ns3
{
namespace abr
{

NS_OBJECT_ENSURE_REGISTERED(TypeHeader);

TypeHeader::TypeHeader(MessageType t)
    : m_type(t),
      m_valid(true)
{
}

TypeId
TypeHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::abr::TypeHeader")
                            .SetParent<Header>()
                            .SetGroupName("Abr")
                            .AddConstructor<TypeHeader>();
    return tid;
}

TypeId
TypeHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
TypeHeader::GetSerializedSize() const
{
    return 1;
}

void
TypeHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8((uint8_t)m_type);
}

uint32_t
TypeHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    uint8_t type = i.ReadU8();
    m_valid = true;
    switch (type)
    {
    case ABRTYPE_RREQ:
    case ABRTYPE_RREP:
    case ABRTYPE_RERR:
    case ABRTYPE_RREP_ACK: {
        m_type = (MessageType)type;
        break;
    }
    default:
        m_valid = false;
    }
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
TypeHeader::Print(std::ostream& os) const
{
    switch (m_type)
    {
    case ABRTYPE_RREQ: {
        os << "RREQ";
        break;
    }
    case ABRTYPE_RREP: {
        os << "RREP";
        break;
    }
    case ABRTYPE_RERR: {
        os << "RERR";
        break;
    }
    case ABRTYPE_RREP_ACK: {
        os << "RREP_ACK";
        break;
    }
    default:
        os << "UNKNOWN_TYPE";
    }
}

bool
TypeHeader::operator==(const TypeHeader& o) const
{
    return (m_type == o.m_type && m_valid == o.m_valid);
}

std::ostream&
operator<<(std::ostream& os, const TypeHeader& h)
{
    h.Print(os);
    return os;
}

//-----------------------------------------------------------------------------
// RREQ
//-----------------------------------------------------------------------------

RreqHeader::RreqHeader(uint8_t flags,
                       uint8_t reserved,
                       uint8_t hopCount,
                       uint32_t requestID,
                       Ipv4Address dst,
                       uint32_t dstSeqNo,
                       Ipv4Address origin,
                       uint32_t originSeqNo,
                       const std::vector<Ipv4Address>& inIds,
                       const std::vector<MetricBlock>& metricBlocks)
    : m_flags(flags),
      m_reserved(reserved),
      m_hopCount(hopCount),
      m_requestID(requestID),
      m_dst(dst),
      m_dstSeqNo(dstSeqNo),
      m_origin(origin),
      m_originSeqNo(originSeqNo),
      m_inIds(inIds),
      m_metricBlocks(metricBlocks)

{
}

NS_OBJECT_ENSURE_REGISTERED(RreqHeader);

TypeId
RreqHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::abr::RreqHeader")
                            .SetParent<Header>()
                            .SetGroupName("Abr")
                            .AddConstructor<RreqHeader>();
    return tid;
}

TypeId
RreqHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
RreqHeader::GetSerializedSize() const
{
    // 기본 필드
    uint32_t size = 23;

    // InIds 길이저장 + InIds
    size += 4;
    size += m_inIds.size() * 4;

    // MetricBlocks 길이저장
    size += 4;
    for (const auto& block : m_metricBlocks)
    {
        size += 4;                            // owner
        size += 4;                            // ticks 길이 저장
        size += block.ticks.size() * (4 + 4); // neighbor + tick
    }

    return size;
}

// 패킷 직렬화
void
RreqHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(m_flags);
    i.WriteU8(m_reserved);
    i.WriteU8(m_hopCount);
    i.WriteHtonU32(m_requestID);
    WriteTo(i, m_dst);
    i.WriteHtonU32(m_dstSeqNo);
    WriteTo(i, m_origin);
    i.WriteHtonU32(m_originSeqNo);

    // InIds
    uint32_t inLen = m_inIds.size();
    i.WriteHtonU32(inLen);
    for (uint32_t j = 0; j < inLen; j++)
    {
        WriteTo(i, m_inIds[j]);
    }

    // MetricBlocks
    uint32_t blockLen = m_metricBlocks.size();
    i.WriteHtonU32(blockLen);
    for (const auto& block : m_metricBlocks)
    {
        WriteTo(i, block.owner);

        uint32_t ticksLen = block.ticks.size();
        i.WriteHtonU32(ticksLen);

        for (const auto& tick : block.ticks)
        {
            WriteTo(i, tick.neighbor);
            i.WriteHtonU32(tick.tick);
        }
    }
}

// 패킷 역직렬화
uint32_t
RreqHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_flags = i.ReadU8();
    m_reserved = i.ReadU8();
    m_hopCount = i.ReadU8();
    m_requestID = i.ReadNtohU32();
    ReadFrom(i, m_dst);
    m_dstSeqNo = i.ReadNtohU32();
    ReadFrom(i, m_origin);
    m_originSeqNo = i.ReadNtohU32();

    uint32_t inLen = i.ReadNtohU32();
    m_inIds.clear();

    for (size_t j = 0; j < inLen; j++)
    {
        Ipv4Address inId;
        ReadFrom(i, inId);
        m_inIds.push_back(inId);
    }

    uint32_t blockLen = i.ReadNtohU32();
    m_metricBlocks.clear();
    for (size_t j = 0; j < blockLen; j++)
    {
        MetricBlock block;
        ReadFrom(i, block.owner);

        uint32_t ticksLen = i.ReadNtohU32();
        for (size_t k = 0; k < ticksLen; k++)
        {
            NeighborTick tick;
            ReadFrom(i, tick.neighbor);
            tick.tick = i.ReadNtohU32();
            block.ticks.push_back(tick);
        }
        m_metricBlocks.push_back(block);
    }

    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
RreqHeader::Print(std::ostream& os) const
{
    os << "RREQ ID " << m_requestID << " dst=" << m_dst << " src=" << m_origin
       << " hop=" << unsigned(m_hopCount);

    os << " | IN_IDS: ";
    for (const auto& id : m_inIds)
        os << id << " ";

    os << " | METRICS: ";
    for (const auto& block : m_metricBlocks)
    {
        os << "[owner=" << block.owner << " : ";
        for (const auto& t : block.ticks)
            os << "(" << t.neighbor << ", " << t.tick << ") ";
        os << "] ";
        os << "\n";
    }
}

std::ostream&
operator<<(std::ostream& os, const RreqHeader& h)
{
    h.Print(os);
    return os;
}

void
RreqHeader::SetGratuitousRrep(bool f)
{
    if (f)
    {
        m_flags |= (1 << 5);
    }
    else
    {
        m_flags &= ~(1 << 5);
    }
}

bool
RreqHeader::GetGratuitousRrep() const
{
    return (m_flags & (1 << 5));
}

void
RreqHeader::SetDestinationOnly(bool f)
{
    if (f)
    {
        m_flags |= (1 << 4);
    }
    else
    {
        m_flags &= ~(1 << 4);
    }
}

bool
RreqHeader::GetDestinationOnly() const
{
    return (m_flags & (1 << 4));
}

void
RreqHeader::SetUnknownSeqno(bool f)
{
    if (f)
    {
        m_flags |= (1 << 3);
    }
    else
    {
        m_flags &= ~(1 << 3);
    }
}

bool
RreqHeader::GetUnknownSeqno() const
{
    return (m_flags & (1 << 3));
}

void
RreqHeader::AppendInId(Ipv4Address inId)
{
    m_inIds.push_back(inId);
}

void
RreqHeader::AppendMetricBlock(Ipv4Address owner, const std::vector<NeighborTick>& ticks)
{
    if (ticks.empty())
    {
        return;
    }

    MetricBlock block;
    block.owner = owner;
    block.ticks = ticks;
    m_metricBlocks.push_back(block);
}

bool
RreqHeader::PruneLastMetricBlock(Ipv4Address me)
{
    if (m_metricBlocks.empty())
    {
        return false;
    }
    MetricBlock& lastBlock = m_metricBlocks.back(); // 마지막 metricblock 가져오기
    if (lastBlock.ticks.empty())
    {
        return false;
    }

    // 나와 upstream 사이 tick만 유지
    std::vector<NeighborTick> prunedTicks;

    for (const auto& tick : lastBlock.ticks)
    {
        if (tick.neighbor != me)
        {
            continue;
        }
        else
        {
            prunedTicks.push_back(tick);
            break;
        }
    }

    // prunedTicks에 나와 upstream 사이 tick만 저장되어 있음
    if (prunedTicks.empty())
    {
        return false;
    }

    // lastBlock의 ticks를 prunedTicks로 업데이트
    lastBlock.ticks = prunedTicks;
    return true;
}

bool
RreqHeader::PruneUpstreamMetricBlock(Ipv4Address upstreamOwner, Ipv4Address me)
{
    if (m_metricBlocks.empty())
    {
        return false;
    }

    for (auto it = m_metricBlocks.rbegin(); it != m_metricBlocks.rend(); ++it)
    {
        if (it->owner != upstreamOwner)
            continue;

        if (it->ticks.empty())
            return false;

        for (const auto& t : it->ticks)
        {
            if (t.neighbor == me)
            {
                it->ticks = {t};
                return true;
            }
        }

        return false;
    }

    return false;
}

bool
RreqHeader::operator==(const RreqHeader& o) const
{
    if (!(m_flags == o.m_flags && m_reserved == o.m_reserved && m_hopCount == o.m_hopCount &&
          m_requestID == o.m_requestID && m_dst == o.m_dst && m_dstSeqNo == o.m_dstSeqNo &&
          m_origin == o.m_origin && m_originSeqNo == o.m_originSeqNo && m_inIds == o.m_inIds))
    {
        return false;
    }

    if (m_metricBlocks.size() != o.m_metricBlocks.size())
        return false;

    for (size_t i = 0; i < m_metricBlocks.size(); ++i)
    {
        if (m_metricBlocks[i].owner != o.m_metricBlocks[i].owner)
            return false;

        if (m_metricBlocks[i].ticks.size() != o.m_metricBlocks[i].ticks.size())
            return false;

        for (size_t j = 0; j < m_metricBlocks[i].ticks.size(); ++j)
        {
            if (m_metricBlocks[i].ticks[j].neighbor != o.m_metricBlocks[i].ticks[j].neighbor ||
                m_metricBlocks[i].ticks[j].tick != o.m_metricBlocks[i].ticks[j].tick)
                return false;
        }
    }

    return true;
}

void
RreqHeader::ForceSetUpstreamTick(Ipv4Address upstreamOwner, Ipv4Address me, uint32_t tick)
{
    NeighborTick nt;
    nt.neighbor = me;
    nt.tick = tick;

    for (auto it = m_metricBlocks.rbegin(); it != m_metricBlocks.rend(); ++it)
    {
        if (it->owner == upstreamOwner)
        {
            it->ticks.clear();
            it->ticks.push_back(nt);
            return;
        }
    }

    MetricBlock mb;
    mb.owner = upstreamOwner;
    mb.ticks = {nt};
    m_metricBlocks.push_back(mb);
}

//-----------------------------------------------------------------------------
// RREP
//-----------------------------------------------------------------------------

RrepHeader::RrepHeader(uint8_t prefixSize,
                       uint8_t hopCount,
                       Ipv4Address dst,
                       uint32_t dstSeqNo,
                       Ipv4Address origin,
                       Time lifeTime)
    : m_flags(0),
      m_prefixSize(prefixSize),
      m_hopCount(hopCount),
      m_dst(dst),
      m_dstSeqNo(dstSeqNo),
      m_origin(origin)
{
    m_lifeTime = uint32_t(lifeTime.GetMilliSeconds());
}

NS_OBJECT_ENSURE_REGISTERED(RrepHeader);

TypeId
RrepHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::abr::RrepHeader")
                            .SetParent<Header>()
                            .SetGroupName("Abr")
                            .AddConstructor<RrepHeader>();
    return tid;
}

TypeId
RrepHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
RrepHeader::GetSerializedSize() const
{
    return 19;
}

void
RrepHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(m_flags);
    i.WriteU8(m_prefixSize);
    i.WriteU8(m_hopCount);
    WriteTo(i, m_dst);
    i.WriteHtonU32(m_dstSeqNo);
    WriteTo(i, m_origin);
    i.WriteHtonU32(m_lifeTime);
}

uint32_t
RrepHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;

    m_flags = i.ReadU8();
    m_prefixSize = i.ReadU8();
    m_hopCount = i.ReadU8();
    ReadFrom(i, m_dst);
    m_dstSeqNo = i.ReadNtohU32();
    ReadFrom(i, m_origin);
    m_lifeTime = i.ReadNtohU32();

    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
RrepHeader::Print(std::ostream& os) const
{
    os << "destination: ipv4 " << m_dst << " sequence number " << m_dstSeqNo;
    if (m_prefixSize != 0)
    {
        os << " prefix size " << m_prefixSize;
    }
    os << " source ipv4 " << m_origin << " lifetime " << m_lifeTime
       << " acknowledgment required flag " << (*this).GetAckRequired();
}

void
RrepHeader::SetLifeTime(Time t)
{
    m_lifeTime = t.GetMilliSeconds();
}

Time
RrepHeader::GetLifeTime() const
{
    Time t(MilliSeconds(m_lifeTime));
    return t;
}

void
RrepHeader::SetAckRequired(bool f)
{
    if (f)
    {
        m_flags |= (1 << 6);
    }
    else
    {
        m_flags &= ~(1 << 6);
    }
}

bool
RrepHeader::GetAckRequired() const
{
    return (m_flags & (1 << 6));
}

void
RrepHeader::SetPrefixSize(uint8_t sz)
{
    m_prefixSize = sz;
}

uint8_t
RrepHeader::GetPrefixSize() const
{
    return m_prefixSize;
}

bool
RrepHeader::operator==(const RrepHeader& o) const
{
    return (m_flags == o.m_flags && m_prefixSize == o.m_prefixSize && m_hopCount == o.m_hopCount &&
            m_dst == o.m_dst && m_dstSeqNo == o.m_dstSeqNo && m_origin == o.m_origin &&
            m_lifeTime == o.m_lifeTime);
}

void
RrepHeader::SetHello(Ipv4Address origin, uint32_t srcSeqNo, Time lifetime)
{
    m_flags = 0;
    m_prefixSize = 0;
    m_hopCount = 0;
    m_dst = origin;
    m_dstSeqNo = srcSeqNo;
    m_origin = origin;
    m_lifeTime = lifetime.GetMilliSeconds();
}

std::ostream&
operator<<(std::ostream& os, const RrepHeader& h)
{
    h.Print(os);
    return os;
}

//-----------------------------------------------------------------------------
// RREP-ACK
//-----------------------------------------------------------------------------

RrepAckHeader::RrepAckHeader()
    : m_reserved(0)
{
}

NS_OBJECT_ENSURE_REGISTERED(RrepAckHeader);

TypeId
RrepAckHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::abr::RrepAckHeader")
                            .SetParent<Header>()
                            .SetGroupName("Abr")
                            .AddConstructor<RrepAckHeader>();
    return tid;
}

TypeId
RrepAckHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
RrepAckHeader::GetSerializedSize() const
{
    return 1;
}

void
RrepAckHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(m_reserved);
}

uint32_t
RrepAckHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_reserved = i.ReadU8();
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
RrepAckHeader::Print(std::ostream& os) const
{
}

bool
RrepAckHeader::operator==(const RrepAckHeader& o) const
{
    return m_reserved == o.m_reserved;
}

std::ostream&
operator<<(std::ostream& os, const RrepAckHeader& h)
{
    h.Print(os);
    return os;
}

//-----------------------------------------------------------------------------
// RERR
//-----------------------------------------------------------------------------
RerrHeader::RerrHeader()
    : m_flag(0),
      m_reserved(0)
{
}

NS_OBJECT_ENSURE_REGISTERED(RerrHeader);

TypeId
RerrHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::abr::RerrHeader")
                            .SetParent<Header>()
                            .SetGroupName("Abr")
                            .AddConstructor<RerrHeader>();
    return tid;
}

TypeId
RerrHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
RerrHeader::GetSerializedSize() const
{
    return (3 + 8 * GetDestCount());
}

void
RerrHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(m_flag);
    i.WriteU8(m_reserved);
    i.WriteU8(GetDestCount());
    for (auto j = m_unreachableDstSeqNo.begin(); j != m_unreachableDstSeqNo.end(); ++j)
    {
        WriteTo(i, (*j).first);
        i.WriteHtonU32((*j).second);
    }
}

uint32_t
RerrHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_flag = i.ReadU8();
    m_reserved = i.ReadU8();
    uint8_t dest = i.ReadU8();
    m_unreachableDstSeqNo.clear();
    Ipv4Address address;
    uint32_t seqNo;
    for (uint8_t k = 0; k < dest; ++k)
    {
        ReadFrom(i, address);
        seqNo = i.ReadNtohU32();
        m_unreachableDstSeqNo.insert(std::make_pair(address, seqNo));
    }

    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
RerrHeader::Print(std::ostream& os) const
{
    os << "Unreachable destination (ipv4 address, seq. number):";
    for (auto j = m_unreachableDstSeqNo.begin(); j != m_unreachableDstSeqNo.end(); ++j)
    {
        os << (*j).first << ", " << (*j).second;
    }
    os << "No delete flag " << (*this).GetNoDelete();
}

void
RerrHeader::SetNoDelete(bool f)
{
    if (f)
    {
        m_flag |= (1 << 0);
    }
    else
    {
        m_flag &= ~(1 << 0);
    }
}

bool
RerrHeader::GetNoDelete() const
{
    return (m_flag & (1 << 0));
}

bool
RerrHeader::AddUnDestination(Ipv4Address dst, uint32_t seqNo)
{
    if (m_unreachableDstSeqNo.find(dst) != m_unreachableDstSeqNo.end())
    {
        return true;
    }

    NS_ASSERT(GetDestCount() < 255); // can't support more than 255 destinations in single RERR
    m_unreachableDstSeqNo.insert(std::make_pair(dst, seqNo));
    return true;
}

bool
RerrHeader::RemoveUnDestination(std::pair<Ipv4Address, uint32_t>& un)
{
    if (m_unreachableDstSeqNo.empty())
    {
        return false;
    }
    auto i = m_unreachableDstSeqNo.begin();
    un = *i;
    m_unreachableDstSeqNo.erase(i);
    return true;
}

void
RerrHeader::Clear()
{
    m_unreachableDstSeqNo.clear();
    m_flag = 0;
    m_reserved = 0;
}

bool
RerrHeader::operator==(const RerrHeader& o) const
{
    if (m_flag != o.m_flag || m_reserved != o.m_reserved || GetDestCount() != o.GetDestCount())
    {
        return false;
    }

    auto j = m_unreachableDstSeqNo.begin();
    auto k = o.m_unreachableDstSeqNo.begin();
    for (uint8_t i = 0; i < GetDestCount(); ++i)
    {
        if ((j->first != k->first) || (j->second != k->second))
        {
            return false;
        }

        j++;
        k++;
    }
    return true;
}

std::ostream&
operator<<(std::ostream& os, const RerrHeader& h)
{
    h.Print(os);
    return os;
}
} // namespace abr
} // namespace ns3

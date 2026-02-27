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
 * This is an example script for ABR manet routing protocol.
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */

#include "ns3/abr-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/ping-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/yans-wifi-helper.h"

#include <cmath>
#include <iostream>

using namespace ns3;

/**
 * \ingroup abr-examples
 * \ingroup examples
 * \brief Test script.
 *
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 *
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 *
 * ping 10.0.0.4
 *
 * When 1/3 of simulation time has elapsed, one of the nodes is moved out of
 * range, thereby breaking the topology.  By default, this will result in
 * stopping ping replies reception after sequence number 33. If the step size is reduced
 * to cover the gap, then also the following pings can be received.
 */
class AbrExample
{
  public:
    AbrExample();
    /**
     * \brief Configure script parameters
     * \param argc is the command line argument count
     * \param argv is the command line arguments
     * \return true on successful configuration
     */
    bool Configure(int argc, char** argv);
    /// Run simulation
    void Run();
    /**
     * Report results
     * \param os the output stream
     */
    void Report(std::ostream& os);

  private:
    // parameters
    /// Number of nodes
    uint32_t size;
    /// Distance between nodes, meters
    double step;
    /// Simulation time, seconds
    double totalTime;
    /// Write per-device PCAP traces if true
    bool pcap;
    /// Print routes if true
    bool printRoutes;

    // network
    /// nodes used in the example
    NodeContainer nodes;
    /// devices used in the example
    NetDeviceContainer devices;
    /// interfaces used in the example
    Ipv4InterfaceContainer interfaces;

  private:
    /// Create the nodes
    void CreateNodes();
    /// Create the devices
    void CreateDevices();
    /// Create the network
    void InstallInternetStack();
    /// Create the simulation applications
    void InstallApplications();
};

int
main(int argc, char** argv)
{
    AbrExample test;
    if (!test.Configure(argc, argv))
    {
        NS_FATAL_ERROR("Configuration failed. Aborted.");
    }

    test.Run();
    test.Report(std::cout);
    return 0;
}

//-----------------------------------------------------------------------------
AbrExample::AbrExample()
    : size(10),
      step(100),
      totalTime(100),
      pcap(true),
      printRoutes(true)
{
}

bool
AbrExample::Configure(int argc, char** argv)
{
    // +) Enable per-node ABR logs for AT/BQ debugging in this scenario
    LogComponentEnable("AbrRoutingProtocol", LOG_LEVEL_INFO);
    LogComponentEnable("AbrNeighbors", LOG_LEVEL_INFO);

    SeedManager::SetSeed(12345);
    CommandLine cmd(__FILE__);

    cmd.AddValue("pcap", "Write PCAP traces.", pcap);
    cmd.AddValue("printRoutes", "Print routing table dumps.", printRoutes);
    cmd.AddValue("size", "Number of nodes.", size);
    cmd.AddValue("time", "Simulation time, s.", totalTime);
    cmd.AddValue("step", "Grid step, m", step);

    cmd.Parse(argc, argv);
    return true;
}

void
AbrExample::Run()
{
    //  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); //
    //  enable rts cts all the time.
    CreateNodes();
    CreateDevices();
    InstallInternetStack();
    InstallApplications();

    std::cout << "Starting simulation for " << totalTime << " s ...\n";

    Simulator::Stop(Seconds(totalTime));

    Ptr<FlowMonitor> flowmon;
    FlowMonitorHelper flow;
    flowmon = flow.InstallAll();
    AnimationInterface anim("abr_test.xml");
    anim.EnablePacketMetadata();
    anim.EnableIpv4RouteTracking("abr_route.xml", Seconds(1), Seconds(totalTime));
    anim.EnableQueueCounters(Seconds(0), Seconds(totalTime));

    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        anim.UpdateNodeDescription(i, "node " + std::to_string(i));
        anim.UpdateNodeSize(i, 1, 1);
    }

    Simulator::Run();
    Simulator::Destroy();
}

void
AbrExample::Report(std::ostream&)
{
}

void
AbrExample::CreateNodes()
{
    std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
    nodes.Create(size);
    // Name nodes
    for (uint32_t i = 0; i < size; ++i)
    {
        std::ostringstream os;
        os << "node-" << i;
        Names::Add(os.str(), nodes.Get(i));
    }

    // ABR) 이동 및 배치 설정
    MobilityHelper mobility;
    ObjectFactory userPosAlloc;
    userPosAlloc.SetTypeId("ns3::RandomRectanglePositionAllocator");
    userPosAlloc.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"));
    userPosAlloc.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"));

    Ptr<PositionAllocator> waypos = userPosAlloc.Create()->GetObject<PositionAllocator>();

    mobility.SetMobilityModel(
        "ns3::RandomWaypointMobilityModel",
        "Speed",
        StringValue("ns3::UniformRandomVariable[Min=2.0|Max=6.0]"), // 2-6m/s 속도로 이동
        "Pause",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
        "PositionAllocator",
        PointerValue(waypos));
    mobility.SetPositionAllocator(waypos);

    mobility.Install(nodes);
}

void
AbrExample::CreateDevices()
{
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                   "MaxRange",
                                   DoubleValue(50.0)); // 10m

    wifiPhy.SetChannel(wifiChannel.Create());
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("OfdmRate6Mbps"),
                                 "RtsCtsThreshold",
                                 UintegerValue(0));
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    if (pcap)
    {
        wifiPhy.EnablePcapAll(std::string("abr"));
    }
}

void
AbrExample::InstallInternetStack()
{
    AbrHelper abr;
    // you can configure ABR attributes here using abr.Set(name, value)
    abr.Set("DestinationOnly", BooleanValue(true));
    InternetStackHelper stack;
    stack.SetRoutingHelper(abr); // has effect on the next Install ()
    stack.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.0.0.0");
    interfaces = address.Assign(devices);

    if (printRoutes)
    {
        Ptr<OutputStreamWrapper> routingStream =
            Create<OutputStreamWrapper>("abr.routes", std::ios::out);
        Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(8), routingStream);
    }
}

void
AbrExample::InstallApplications()
{
    PingHelper ping(interfaces.GetAddress(size - 1));
    ping.SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::VERBOSE));

    ApplicationContainer p = ping.Install(nodes.Get(0));
    p.Start(Seconds(10));
    p.Stop(Seconds(totalTime) - Seconds(0.001));

    // move node away
    /*
    Ptr<Node> node = nodes.Get(size / 2);
    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(totalTime / 3),
                        &MobilityModel::SetPosition,
                        mob,
                        Vector(1e5, 1e5, 1e5));
    */
}
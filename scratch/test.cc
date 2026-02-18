#include "ns3/animation-interface.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/applications-module.h"
#include "ns3/bridge-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

static void
getStaPosition(std::string context, Ptr<const MobilityModel> mobility)
{
    Vector pos = mobility->GetPosition();
    std::cout << "station position: x=" << pos.x << ", y=" << pos.y << std::endl;
}

void
getBeaconInterval(NetDeviceContainer devices)
{
    for (uint32_t i = 0; i < devices.GetN(); i++)
    {
        PointerValue ptr;
        devices.Get(i)->GetAttribute("Mac", ptr);
        Ptr<WifiMac> mac = ptr.Get<WifiMac>();
        Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac>(mac);

        if (!apMac)
        {
            continue;
        }

        TimeValue value;
        apMac->GetAttribute("BeaconInterval", value);
        std::cout << Simulator::Now().GetSeconds() << " AP Device " << i
                  << " Beacon Interval: " << value.Get().GetSeconds() << " seconds" << std::endl;
    }

    Simulator::Schedule(Seconds(1.0), &getBeaconInterval, devices);
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    int nWifi = 1;
    bool tracing = true;

    if (verbose)
    {
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    }

    NodeContainer p2pNodes;
    p2pNodes.Create(2);

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer p2pDevices = pointToPoint.Install(p2pNodes);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nWifi);

    NodeContainer wifiApNodes;
    wifiApNodes.Create(3);
    wifiApNodes.Add(p2pNodes.Get(1));

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));
    NetDeviceContainer csmaDevices = csma.Install(wifiApNodes);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    staDevices = wifi.Install(phy, mac, wifiStaNodes);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i)
    {
        apDevices.Add(wifi.Install(phy, mac, wifiApNodes.Get(i)));
    }

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(100.0, 100.0, 0.0));
    positionAlloc->Add(Vector(300.0, 100.0, 0.0));
    positionAlloc->Add(Vector(100.0, 300.0, 0.0));
    positionAlloc->Add(Vector(300.0, 300.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes);

    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    staPositionAlloc->Add(Vector(120.0, 120.0, 0.0));
    mobility.SetPositionAllocator(staPositionAlloc);
    // Faster, longer random walk so the STA roams across the full grid
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds",
                              RectangleValue(Rectangle(0, 500, 0, 500)),
                              "Mode",
                              StringValue("Distance"),
                              "Distance",
                              DoubleValue(50.0),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=6.0]"));
    mobility.Install(wifiStaNodes);

    BridgeHelper bridge;
    NetDeviceContainer bridgeDevices;
    for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i)
    {
        NetDeviceContainer ports;
        ports.Add(apDevices.Get(i));
        ports.Add(csmaDevices.Get(i));
        bridgeDevices.Add(bridge.Install(wifiApNodes.Get(i), ports));
    }

    InternetStackHelper stack;
    stack.Install(p2pNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address1;
    address1.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pIfs = address1.Assign(p2pDevices);

    Ipv4AddressHelper address2;
    address2.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer staIfs = address2.Assign(staDevices);

    Ptr<Node> gw = p2pNodes.Get(1);
    stack.Install(gw);

    NetDeviceContainer gwBridgeDev;
    gwBridgeDev.Add(bridgeDevices.Get(3));
    Ipv4InterfaceContainer gwWifiIf = address2.Assign(gwBridgeDev);

    gw->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    Ipv4StaticRoutingHelper sr;

    Ptr<Ipv4StaticRouting> staRt = sr.GetStaticRouting(wifiStaNodes.Get(0)->GetObject<Ipv4>());
    staRt->SetDefaultRoute(gwWifiIf.GetAddress(0), 1);

    Ptr<Ipv4StaticRouting> n0Rt = sr.GetStaticRouting(p2pNodes.Get(0)->GetObject<Ipv4>());
    n0Rt->AddNetworkRouteTo(Ipv4Address("10.1.2.0"),
                            Ipv4Mask("255.255.255.0"),
                            p2pIfs.GetAddress(1),
                            1);

    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(p2pNodes.Get(0));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));

    UdpEchoClientHelper echoClient(p2pIfs.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(100));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(20.0));

    Simulator::Stop(Seconds(20.0));

    if (tracing)
    {
        phy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
        pointToPoint.EnablePcapAll("p2p");
        phy.EnablePcap("ap", apDevices.Get(3), true);
        csma.EnablePcapAll("csma", true);
    }

    std::ostringstream context;
    uint32_t staId = wifiStaNodes.Get(0)->GetId();
    context << "/NodeList/" << staId << "/$ns3::MobilityModel/CourseChange";
    Config::Connect(context.str(), MakeCallback(&getStaPosition));

    Simulator::Schedule(Seconds(1.0), &getBeaconInterval, apDevices);

    AnimationInterface anim("wireless-animation.xml");

    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
    {
        anim.UpdateNodeDescription(wifiStaNodes.Get(i), "STA"); // Optional
        anim.UpdateNodeColor(wifiStaNodes.Get(i), 255, 0, 0);   // Optional
        anim.UpdateNodeSize(i, 3, 3);
    }
    for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i)
    {
        anim.UpdateNodeDescription(wifiApNodes.Get(i), "AP"); // Optional
        anim.UpdateNodeColor(wifiApNodes.Get(i), 0, 255, 0);  // Optional
        anim.UpdateNodeSize(i, 3, 3);
    }
    for (uint32_t i = 0; i < p2pNodes.GetN(); i++)
    {
        anim.UpdateNodeDescription(wifiApNodes.Get(i), "AP"); // Optional
        anim.UpdateNodeColor(wifiApNodes.Get(i), 0, 255, 0);  // Optional
        anim.UpdateNodeSize(i, 3, 3);
    }

    anim.EnablePacketMetadata(); // Optional
    anim.EnableIpv4RouteTracking("routingtable-wireless.xml",
                                 Seconds(0),
                                 Seconds(5),
                                 Seconds(0.25));         // Optional
    anim.EnableWifiMacCounters(Seconds(0), Seconds(10)); // Optional
    anim.EnableWifiPhyCounters(Seconds(0), Seconds(10)); // Optional

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}

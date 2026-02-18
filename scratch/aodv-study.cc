/*
[10.0.0.1] <– step --> [10.0.0.2] <– step --> [10.0.0.3] <– step --> [10.0.0.4]
  Node A            Node B            Node C            Node D

  ping 10.0.0.4

When 1/3 of simulation time has elapsed, one of the nodes is moved out of range, thereby breaking
the topology. By default, this will result in stopping ping replies reception after sequence
number 33. If the step size is reduced to cover the gap, then also the following pings can be
received.
*/
#include "ns3/animation-interface.h"
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ping-helper.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

class AodvExample
{
  public:
    AodvExample();
    bool Configure(int argc, char** argv);
    void Report(std::ostream& os); // 결과 보고

    void Run(); // 시뮬레이션 실행

  private:
    // parameters
    uint32_t size;    // 노드 수
    double step;      // 노드간 간격(m)
    double totalTime; // 시뮬레이션 시간(s)
    bool pcap;        // pcap 생성 여부
    bool printRoutes; // 라우팅 테이블 출력 여부

    // network
    NodeContainer nodes;
    NetDeviceContainer devices;
    Ipv4InterfaceContainer interfaces;

  private:
    void CreateNodes();
    void CreateDevices();
    void InstallInternetStack();
    void InstallApplications();
};

int
main(int argc, char** argv)
{
    AodvExample test;
    PacketMetadata::Enable();
    if (!test.Configure(argc, argv))
    {
        NS_FATAL_ERROR("Configuration failed. Aborting.");
    }

    test.Run();
    test.Report(std::cout);
    return 0;
}

AodvExample::AodvExample()
    : size(4),
      step(50),
      totalTime(30),
      pcap(true),
      printRoutes(true)
{
}

void
AodvExample::Run()
{
    CreateNodes();
    CreateDevices();
    InstallInternetStack();
    InstallApplications();

    std::cout << "Starting simulation for " << totalTime << " s ...\n";
    AnimationInterface anim("aodv-animation.xml");

    for (uint32_t i = 0; i < size; ++i)
    {
        std::ostringstream os;
        os << "node-" << i;
        anim.UpdateNodeDescription(nodes.Get(i), os.str());
        anim.UpdateNodeColor(nodes.Get(i), 255, 0, 0); // 빨간색
    }

    anim.UpdateNodeColor(nodes.Get(0), 0, 255, 0);
    anim.UpdateNodeDescription(nodes.Get(0), "Ping Source");
    anim.UpdateNodeColor(nodes.Get(size - 1), 0, 0, 255);
    anim.UpdateNodeDescription(nodes.Get(size - 1), "Ping Destination");

    anim.UpdateNodeSize(nodes.Get(size / 2), 3.0, 3.0);
    anim.EnablePacketMetadata(); // Optional
    anim.EnableIpv4RouteTracking("routingtable-aodv.xml", Seconds(0), Seconds(5), Seconds(0.25));

    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();
    Simulator::Destroy();
}

void
AodvExample::Report(std::ostream&)
{
}

bool
AodvExample::Configure(int argc, char** argv)
{
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
AodvExample::CreateNodes()
{
    nodes.Create(size);
    // 노드에 이름붙이기
    for (uint32_t i = 0; i < size; ++i)
    {
        std::ostringstream os;
        os << "node-" << i;
        Names::Add(os.str(), nodes.Get(i));
    }
    // mobility 설정
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(0.0),
                                  "MinY",
                                  DoubleValue(0.0),
                                  "DeltaX",
                                  DoubleValue(step),
                                  "DeltaY",
                                  DoubleValue(step),
                                  "GridWidth",
                                  UintegerValue(size),
                                  "LayoutType",
                                  StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
}

void
AodvExample::CreateDevices()
{
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiHelper wifi;
    wifi.SetRemoteStationManager(
        "ns3::ConstantRateWifiManager", // 무선전송속도와 재전송정책을 결정하는 모듈
        "DataMode",
        StringValue("OfdmRate6Mbps"),
        "RtsCtsThreshold",
        UintegerValue(0));
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    if (pcap)
    {
        wifiPhy.EnablePcapAll("aodv");
    }
}

void
AodvExample::InstallInternetStack()
{
    // AODV 라우팅 프로토콜 헬퍼 생성
    AodvHelper aodv;
    InternetStackHelper stack;

    // AODV 라우팅 프로토콜은 네트워크 스택에 사용되니까 여기다가 설치함
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.0.0.0");
    interfaces = address.Assign(devices);

    if (printRoutes)
    {
        Ptr<OutputStreamWrapper> routingStream =
            Create<OutputStreamWrapper>("aodv-routingtable.xml", std::ios::out);
    }
}

void
AodvExample::InstallApplications()
{
    PingHelper ping(interfaces.GetAddress(size - 1)); // 목적지는 마지막 노드
    ping.SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::VERBOSE));

    ApplicationContainer p = ping.Install(nodes.Get(0)); // 첫번째 노드에 ping 애플리케이션 설치
    p.Start(Seconds(1.0));
    p.Stop(Seconds(totalTime) - Seconds(0.001));

    // 시뮬레이션 시간의 1/3이 경과했을 때, 중간 노드를 멀리 이동시켜 토폴로지 분리
    Ptr<Node> node = nodes.Get(size / 2);
    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(totalTime / 3),
                        &MobilityModel::SetPosition,
                        mob,
                        Vector(1000, 0, 0));
}
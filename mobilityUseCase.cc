/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

 //./waf --run "mobilityUseCase.cc --traceFile=mobilityTraces/scenarii/colmar/colmar800m_0_20nd.ns2mobility.xml --numberOfUes=10"



#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/data-rate.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/radio-environment-map-helper.h"
#include "ns3/flow-monitor-module.h"
#include <vector>
#include <stdlib.h> 
#include <sstream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/stats-module.h"
#include <iostream>
#include <fstream>
#include "ns3/ns2-mobility-helper.h"

#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/lte-global-pathloss-database.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CatMsingleUEtoremoteHost");


int
main (int argc, char *argv[])
{

  uint16_t numberOfUes = 60;
  uint16_t numberOfEnbs = 4;
  uint16_t numBearersPerUe = 1;
  double simTime = 25;
  std::string animFile = "CatMUe.xml";
  std::string traceFile;

  Config::SetDefault ("ns3::UdpClient::PacketSize", UintegerValue (20));
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
  //Config::SetDefault ("ns3::RadioBearerStatsCalculator::EpochDuration", TimeValue (Seconds(10)));
  Config::SetDefault ("ns3::LteEnbPhy::MacToChannelDelay", UintegerValue (1));


  //142857ns for 2 sym and 499999 for 7sym. subtract one nanosecond from it to avoid simulator overlapping
  Config::SetDefault ("ns3::LtePhy::TxTimeInterval", DoubleValue (0.000142857));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (46));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (5));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (9));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (640));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (6));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (6));
  //Config::SetDefault ("ns3::LteHelper::Scheduler", StringValue ("ns3::RrFfMacScheduler"));
  


  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numberOfUes", "Number of UEs", numberOfUes);
  cmd.AddValue ("numberOfEnbs", "Number of eNodeBs", numberOfEnbs);
  cmd.AddValue ("animFile",  "File Name for Animation Output", animFile);
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
  cmd.Parse (argc, argv);


  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  //lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));
  //lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::TwoRayGroundPropagationLossModel"));
  //lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ThreeLogDistancePropagationLossModel"));

  Config::SetDefault ("ns3::Cost231PropagationLossModel::Frequency", DoubleValue (0.824e9));
  Config::SetDefault ("ns3::Cost231PropagationLossModel::Lambda", DoubleValue (0.3641));
  Config::SetDefault ("ns3::Cost231PropagationLossModel::SSAntennaHeight", DoubleValue (3));
  Config::SetDefault ("ns3::Cost231PropagationLossModel::BSAntennaHeight", DoubleValue (50));
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::Cost231PropagationLossModel"));

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  lteHelper->SetSchedulerType ("ns3::RrSpsFfMacScheduler");
  //Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled", BooleanValue (false));

  //Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));


  //lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover

  lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (2400));
  lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (2400 + 18000));
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (6));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (6));


  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  NodeContainer ueNodes;
  ueNodes.Create (numberOfUes);
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  //Ns2MobilityHelper ns2 = Ns2MobilityHelper ("mobilityTraces/map/offenburg_test.ns2mobility.tcl");
  ns2.Install ();

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


/*
 // Install Mobility Model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
	    positionAlloc->Add (Vector (i, 0, 10));
    }
  positionAlloc->Add (Vector (0, 0, 0));
  MobilityHelper mobility;
  MobilityHelper mobilityUe;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  //Config::SetDefault ("ns3::RandomWaypointMobilityModel::MinX", DoubleValue (macroUeBox.xMin));

  mobility.Install (enbNodes);
  mobility.Install (remoteHostContainer);

  Ptr<PositionAllocator> positionAllocUe = CreateObject<RandomBoxPositionAllocator> ();

  Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable> ();
  xVal->SetAttribute ("Min", DoubleValue (0));
  xVal->SetAttribute ("Max", DoubleValue (1000));
  positionAllocUe->SetAttribute ("X", PointerValue (xVal));

  Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable> ();
  yVal->SetAttribute ("Min", DoubleValue (0));
  yVal->SetAttribute ("Max", DoubleValue (1000));
  positionAllocUe->SetAttribute ("Y", PointerValue (yVal));

  Ptr<UniformRandomVariable> zVal = CreateObject<UniformRandomVariable> ();
  zVal->SetAttribute ("Min", DoubleValue (1));
  zVal->SetAttribute ("Max", DoubleValue (1));
  positionAllocUe->SetAttribute ("Z", PointerValue (zVal));

  mobilityUe.SetPositionAllocator (positionAllocUe);
  mobilityUe.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle(0, 1000, 0, 1000)));
  //mobilityUe.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe.Install (ueNodes);
*/
  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  lteHelper->AttachToClosestEnb (ueLteDevs, enbLteDevs);

  // Attach all UEs to the first eNodeB
  /*for (uint16_t i = 0; i < numberOfUes; i++)
    {
      //lteHelper->Attach (ueLteDevs.Get (i), enbLteDevs.Get (1)); // ptr < NetDevice > ueDevice
      lteHelper->Attach (ueLteDevs.Get (i)); //Enable auto attach to the best cell available. Alternatively, it can also be done through AttachToClosestEnb function
    }*/


  NS_LOG_LOGIC ("setting up applications");

  // Install and start applications on UEs and remote host
  //uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.000));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (1.000));

  Ptr<UniformRandomVariable> stopTimeSeconds = CreateObject<UniformRandomVariable> ();
  stopTimeSeconds->SetAttribute ("Min", DoubleValue (20.0));
  stopTimeSeconds->SetAttribute ("Max", DoubleValue (24.1));

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
	  //++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          //NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
          //UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          //clientApps.Add (dlClientHelper.Install (remoteHost));
          //PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
          //                                    InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          //serverApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          //EpcTft::PacketFilter dlpf;
          //dlpf.localPortStart = dlPort;
          //dlpf.localPortEnd = dlPort;
          //tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
          Time stopTime = Seconds (stopTimeSeconds->GetValue ());

          serverApps.Start (startTime);
          clientApps.Start (startTime);
          clientApps.Stop (stopTime);

        } // end for b
    }


  // Add X2 inteface
  //lteHelper->AddX2Interface (enbNodes);

  // X2-based Handover
  //lteHelper->HandoverRequest (Seconds (0.100), ueLteDevs.Get (0), enbLteDevs.Get (0), enbLteDevs.Get (1));

  // Uncomment to enable PCAP tracing
  //p2ph.EnablePcapAll("lena-x2-handover");

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  //Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  //rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.10)));
  //Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  //pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.10)));

  // Set the bounding box for animation
  //star.BoundingBox (1, 1, 400, 400);

  // Create the animation object and configure for specified output
  AnimationInterface anim (animFile);

  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  //flowMonitor = flowHelper.InstallAll();
  flowMonitor = flowHelper.Install (ueNodes);
  flowMonitor = flowHelper.Install (remoteHostContainer);
  //Simulator::Schedule(Seconds(i*t_diff), &ThroughputMonitor, &flowHelper, flowMonitor);

//  int i;
 // for (i = 1; i <= n_dist; i++) {
//	Simulator::Schedule (Seconds (4), &RelocateUE, ueNodes, pos_diff2);
	//Simulator::Schedule (Seconds (i*t_diff), &ThroughputMonitor, &flowHelper, flowMonitor);
  //}
  //Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful",
  //                 MakeCallback (&NotifyRandomAccessSuccessful));

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  flowMonitor->SerializeToXmlFile("UeRhUl.xml", true, true);
  // GtkConfigStore config;
  // config.ConfigureAttributes ();

  Simulator::Destroy ();
  return 0;

}

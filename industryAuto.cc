/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Jaume Nin <jnin@cttc.es>
 *
 * Original File: lena-profiling.cc
 * Modified by: Zubair Amjad <zubair.amjad@hs-offenburg.de>
 * Investigation into sTTI and SPS for industry automation use case
 */

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

#include <ns3/buildings-module.h>
#include <iomanip>
#include <string>
//#include "ns3/gtk-config-store.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CatMBuilding_sTTI-SPS");

int
main (int argc, char *argv[])
{
  uint32_t nEnbPerFloor = 1;
  uint32_t nUe = 30;
  uint32_t nUep = 27;
  uint32_t nFloors = 1;
  double simTime = 10.0;
  uint16_t numBearersPerUe = 1;
  std::string animFile = "CatMUe.xml";
  std::string traceFile;

  CommandLine cmd;

  cmd.AddValue ("nEnb", "Number of eNodeBs per floor", nEnbPerFloor);
  cmd.AddValue ("numberOfUes", "Number of UEs", nUe);
  cmd.AddValue ("nFloors", "Number of floors, 0 for Friis propagation model",
                nFloors);
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)",
                simTime);
  cmd.Parse (argc, argv);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::UdpClient::PacketSize", UintegerValue (12));
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (1000)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::EpochDuration", TimeValue (Seconds(10)));
  Config::SetDefault ("ns3::LteEnbPhy::MacToChannelDelay", UintegerValue (1));

  //142857ns for 2 sym and 499999 for 7sym. subtract one nanosecond from it to avoid simulator overlapping
  Config::SetDefault ("ns3::LtePhy::TxTimeInterval", DoubleValue (0.000142857));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (46));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (5));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (9));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (80));
  Config::SetDefault ("ns3::RrHybridFfMacScheduler::nSpsUe", UintegerValue (3));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (6));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (6));

  // Geometry of the scenario (in meters)
  // Assume squared building
  double nodeHeight = 1.5;
  double roomHeight = 3;
  double roomLength = 200;
  uint32_t nRooms = std::ceil (std::sqrt (nEnbPerFloor));
  uint32_t nEnb;

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  //lteHelper->EnableLogComponents ();
  //LogComponentEnable ("BuildingsPropagationLossModel", LOG_LEVEL_ALL);
  if (nFloors == 0)
    {
      lteHelper->SetAttribute ("PathlossModel",
                               StringValue ("ns3::FriisPropagationLossModel"));
      nEnb = nEnbPerFloor;
    }
  else
    {
      lteHelper->SetAttribute ("PathlossModel",
                               StringValue ("ns3::HybridBuildingsPropagationLossModel"));
      nEnb = nFloors * nEnbPerFloor;
    }

  // Set scheduler type for the simulation
  lteHelper->SetSchedulerType ("ns3::RrHybridFfMacScheduler");

  // Set bandwidth of LTE system
  lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (100));
  lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (100 + 18000));
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (6));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (6));

  // Create EPC Helper and PGW
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

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

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  std::vector<NodeContainer> ueNodes;

  enbNodes.Create (nEnb);
  for (uint32_t i = 0; i < nEnb; i++)
    {
      NodeContainer ueNode;
      ueNode.Create (nUe);
      ueNodes.push_back (ueNode);
    }

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  std::vector<Vector> enbPosition;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<Building> building;

  if (nFloors == 0)
    {
      // Position of eNBs
      uint32_t plantedEnb = 0;
      for (uint32_t row = 0; row < nRooms; row++)
        {
          for (uint32_t column = 0; column < nRooms && plantedEnb < nEnbPerFloor; column++, plantedEnb++)
            {
              Vector v (roomLength * (column + 0.5), roomLength * (row + 0.5), nodeHeight);
              positionAlloc->Add (v);
              enbPosition.push_back (v);
              mobility.Install (ueNodes.at(plantedEnb));
            }
        }
      mobility.SetPositionAllocator (positionAlloc);
      mobility.Install (enbNodes);
      BuildingsHelper::Install (enbNodes);

      // Position of UEs attached to eNB
      for (uint32_t i = 0; i < nEnb; i++)
        {
          Ptr<UniformRandomVariable> posX = CreateObject<UniformRandomVariable> ();
          posX->SetAttribute ("Min", DoubleValue (enbPosition.at(i).x - roomLength * 0.5));
          posX->SetAttribute ("Max", DoubleValue (enbPosition.at(i).x + roomLength * 0.5));
          Ptr<UniformRandomVariable> posY = CreateObject<UniformRandomVariable> ();
          posY->SetAttribute ("Min", DoubleValue (enbPosition.at(i).y - roomLength * 0.5));
          posY->SetAttribute ("Max", DoubleValue (enbPosition.at(i).y + roomLength * 0.5));
          positionAlloc = CreateObject<ListPositionAllocator> ();
          for (uint32_t j = 0; j < nUe; j++)
            {
              positionAlloc->Add (Vector (posX->GetValue (), posY->GetValue (), nodeHeight));
              mobility.SetPositionAllocator (positionAlloc);
            }
          mobility.Install (ueNodes.at(i));
          BuildingsHelper::Install (ueNodes.at(i));
        }

    }
  else
    {
      building = CreateObject<Building> ();
      building->SetBoundaries (Box (0.0, nRooms * roomLength,
                                    0.0, nRooms * roomLength,
                                    0.0, nFloors* roomHeight));
      building->SetBuildingType (Building::Residential);
      building->SetExtWallsType (Building::ConcreteWithWindows);
      building->SetNFloors (nFloors);
      building->SetNRoomsX (nRooms);
      building->SetNRoomsY (nRooms);
      mobility.Install (enbNodes);
      BuildingsHelper::Install (enbNodes);
      uint32_t plantedEnb = 0;
      for (uint32_t floor = 0; floor < nFloors; floor++)
        {
          uint32_t plantedEnbPerFloor = 0;
          for (uint32_t row = 0; row < nRooms; row++)
            {
              for (uint32_t column = 0; column < nRooms && plantedEnbPerFloor < nEnbPerFloor; column++, plantedEnb++, plantedEnbPerFloor++)
                {
                  Vector v (roomLength * (column + 0.5),
                            roomLength * (row + 0.5),
                            nodeHeight + roomHeight * floor);
                  positionAlloc->Add (v);
                  enbPosition.push_back (v);
                  Ptr<MobilityModel> mmEnb = enbNodes.Get (plantedEnb)->GetObject<MobilityModel> ();
                  mmEnb->SetPosition (v);

                  // Positioning UEs attached to eNB
                  mobility.Install (ueNodes.at(plantedEnb));
                  BuildingsHelper::Install (ueNodes.at(plantedEnb));
                  for (uint32_t ue = 0; ue < nUe; ue++)
                    {
                      Ptr<MobilityModel> mmUe = ueNodes.at(plantedEnb).Get (ue)->GetObject<MobilityModel> ();
                      Vector vUe (v.x, v.y, v.z);
                      mmUe->SetPosition (vUe);
                    }
                }
            }
        }
    }


  // Create Devices and install them in the Nodes (eNB and UE)
  NetDeviceContainer enbDevs;
  std::vector<NetDeviceContainer> ueDevs;
  enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  for (uint32_t i = 0; i < nEnb; i++)
    {
      NetDeviceContainer ueDev = lteHelper->InstallUeDevice (ueNodes.at(i));

      internet.Install (ueNodes.at(i));
      Ipv4InterfaceContainer ueIpIfaces;
      ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDev));

      ueDevs.push_back (ueDev);
      lteHelper->Attach (ueDev, enbDevs.Get (i));
    }


        
  NS_LOG_LOGIC ("setting up applications");
  // Install and start applications on UEs and remote host
  uint16_t ulPort = 20000;
  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.000));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.100));
  Ptr<UniformRandomVariable> stopTimeSeconds = CreateObject<UniformRandomVariable> ();
  stopTimeSeconds->SetAttribute ("Min", DoubleValue (10.000));
  stopTimeSeconds->SetAttribute ("Max", DoubleValue (10.100));

  Ptr<UniformRandomVariable> ETstartTimeSeconds = CreateObject<UniformRandomVariable> ();
  ETstartTimeSeconds->SetAttribute ("Min", DoubleValue (7.200));
  ETstartTimeSeconds->SetAttribute ("Max", DoubleValue (7.300));
  Ptr<UniformRandomVariable> ETstopTimeSeconds = CreateObject<UniformRandomVariable> ();
  ETstopTimeSeconds->SetAttribute ("Min", DoubleValue (7.400));
  ETstopTimeSeconds->SetAttribute ("Max", DoubleValue (7.500));

  for (uint32_t i = 0; i < nEnb; i++)
  {
  /**  for (uint32_t u = 0; u < nUe; ++u)
    {
      Ptr<Node> ue = ueNodes.at(i).Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);**/
      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
           for(uint32_t u = 0; u < nUep; ++u)
          {
          Ptr<Node> ue = ueNodes.at(i).Get (u);
      // Set the default gateway for the UE
          Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

          ++ulPort;
          ApplicationContainer clientApps;
          ApplicationContainer serverApps;
          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueDevs.at (i).Get (u), bearer, tft);
          Time startTime = Seconds (startTimeSeconds->GetValue ());
          Time stopTime = Seconds (stopTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);
         // clientApps.Stop (Seconds(7.4));
        }
         for(uint32_t u = nUep; u < nUe; ++u)
         {
          Ptr<Node> ue = ueNodes.at(i).Get (u);
      // Set the default gateway for the UE
          Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

           ++ulPort;
          ApplicationContainer clientApps;
          ApplicationContainer serverApps;
          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueDevs.at (i).Get (u), bearer, tft);
          Time startTime = Seconds (ETstartTimeSeconds->GetValue ());
          Time stopTime = Seconds (ETstopTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);
          clientApps.Stop (stopTime);
         

        } // end for b
     
    }
  }
       

  BuildingsHelper::MakeMobilityModelConsistent ();

  Simulator::Stop (Seconds (simTime));

  lteHelper->EnableTraces ();

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}

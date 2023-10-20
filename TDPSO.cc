#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <valarray>
#include <cmath>
#include <ctime>
#include <math.h>


using namespace std;

mt19937 gen( time( 0 ) );
uniform_real_distribution<double> dist( 0.0, 1.0 );

// include header we will use for the packets header & payload
// DEfine logging component
#define myUAV 60 //jumlah UAV (0 - myUAV-1)
#define myGround 34 //jumlah UAV (myUAV - myGround-1)
#define WormNode 2 //jumlah worm node
#define SourceNode 1 //jumlah source node
#define DestNode 3 // jumlah destination node
#define MaxEnergy 99 // in percentage
#define MaxEnergyJoule 0.099 // in Joule = 9.9 mJoule
//#define numNodes 23

NS_LOG_COMPONENT_DEFINE("WifiSimpleAdhoc");


using namespace ns3;
double distance = 30; // m
static const uint32_t UAV = myUAV;
static const uint32_t GNode = myGround;
static const uint32_t numWorm = WormNode;
static const uint32_t numSource = SourceNode;
static const uint32_t numDest = DestNode;
static const uint32_t numInfo = myUAV+WormNode;
static const uint32_t numEXDes = UAV + numWorm + GNode;
static const uint32_t numNodes = UAV + numWorm + GNode + numSource + numDest ; // total all nodes in the network
int JumNodeInfo = numInfo;
static const uint32_t myNumber = UAV + GNode + numDest;
int numnodes = numNodes;
static ofstream outFiles[numNodes];
static NodeContainer c; //node variable
//parameter every node
int Speed[myNumber];
int Direction[myNumber];//={90,92,88,90,90};
float CosineSim[myNumber];//={0,0,0,0,0};
int IDCosim[myNumber];//={0,0,0,0,0};
int REnergy[myNumber]; // remaining energy
int PosX[myNumber]; // X position
int PosY[myNumber]; // Y Position
int PosZ[myNumber]; // Z Position
int TypeNode[myNumber]; // type of node


int CountInfo[myNumber];//={0,0,0,0,0}; // count for how many INFO packet received
int CountCosim[myNumber];//={0,0,0,0,0}; // count for how many CHI packet received
int CountJC[myNumber];//={0,0,0,0,0}; // count for how many JC packet received
int CountAC[myNumber];//={0,0,0,0,0}; // count for how many AC packet Sending
uint32_t transData;

//count control overhead
float CountControlOver=0;
float ControlROuting=0;
float delayROuting=0;

//status every node
int StatCH[myNumber];//={0,0,0,0,0}; // CH status -> if 0=CM and 1=CH
int DefStatCH[myNumber];//={0,0,0,0,0}; // Default status CH
int StatGW[myNumber];

//Table in the node
int CH[myNumber][myNumber][myNumber]; // CH Table
int CM[myNumber][myNumber]; // CM Table

//For routing
int StatRREQinS;	// STatus if the S received RREQ packet
int StatRREQinD;	// STatus if the D received RREQ packet
int StatRREPinS;	// Status if the S received RREP packet
int StatRREPinD;	// Status if the D received RREP packet
int StatRecForward[myNumber][10][2];	// Status for the RREQ packet received and already forward or not
int StatRecForRREP[myNumber][10][2];	// Status for the RREP packet received and already forward or not
int RREQ[myNumber][myNumber][6];	// RREQ Table
int RREP[myNumber][myNumber][7]; // RREP Table
int CountRREQ[myNumber];	// Count received RREQ packet
int CountRREP[myNumber];	// Count received RREQ packet
int NextHop[myNumber]; // result of 2nd price auction
int NextHopVal[myNumber]; // result value of 2nd price auction
int EP;

int StatDataRec[myNumber][1];	// Status for the Transmission Data received and already forward or not
int StatDataFor[myNumber][1];	// Status for the Transmission Data received and already forward or not
int TrData[myNumber][2]; // identity data trans
string DataTrans[myNumber]; //data

int DataReceived=0;
int SInkReceived=0;

float ETx0 = (330*11)/2000000;
float ETx1 = (330*11)/2000000;
float ETx2 = (330*14)/2000000;
float ETx3 = (330*8)/2000000;
float ETx4 = (330*21)/2000000;
float ETx5 = (330*25)/2000000;
float ETx6 = (330*16)/2000000;

float RTx0 = (23.*11)/2000000;
float RTx1 = (230*11)/2000000;
float RTx2 = (230*14)/2000000;
float RTx3 = (230*8)/2000000;
float RTx4 = (230*21)/2000000;
float RTx5 = (230*25)/2000000;
float RTx6 = (230*16)/2000000;

//Temp for PSO
double E;
double Dist;
float Cosim;
float Delta;
double MaxValue;

//Resut PSO
double YO_PSO[myNumber][5];

double fholder( const valarray<double> &pos )               // Holder's Table function
{
   double x1 = pos[0], x2 = pos[1], x3 = pos[2], x4 = pos[3] ;
   /*
   double E= 99;
   double Dist = 20;
   float Cosim = 0.9;
   float Delta = 0.1;
   */
   return  (Delta*pow((1-x1),2)*exp((pow(-x1,2)) - (pow((x1+1),2))) + Cosim*pow((1-x2),2)*exp((pow(-x2,2)) - (pow((x2+1),2))) +  Dist*pow((1-x2),2)*exp((pow(-x3,2)) - (pow((x3+1),2))) +  E*pow((1-x4),2)*exp((pow(-x4,2)) - (pow((x4+1),2))));
}

//====================================================================


double urand( double lower, double upper )                 // Returns uniform on (lower,upper)
{
   return lower + ( upper - lower ) * dist( gen );
}


//====================================================================


valarray<double> urand( double lower, double upper, int dim )        // Returns array uniform on (lower,upper)
{
   valarray<double> result( dim );
   for ( auto &x : result ) x = lower + ( upper - lower ) * dist( gen );
   return result;
}


//====================================================================


class particle                                              // Individual particle class
{
public:
   valarray<double> position, velocity, localg;
   double localMin;

   particle( int dim, double lower, double upper )
   {
      position = urand( lower, upper, dim );
      velocity = urand( lower, upper, dim );
      localg   = position;
   }
};


//====================================================================


class swarm                                                // Swarm class
{
   vector<particle> particles;
   valarray<double> globalg;
   double globalMax;
   int popsize, dim;
   double w, deltag, deltap, lrt, lower, upper;
   double (*func)( const valarray<double> & );

public:

   swarm( int n, int d, double w_, double dg, double dp, double lrt_, double l, double u, double (*f)( const valarray<double> & ) )
   : popsize( n ), dim( d ), w( w_ ), deltag( dg ), deltap( dp ), lrt( lrt_ ), lower( l ), upper( u ), func( f )
   {
      globalg = urand( lower, upper, dim );
      globalMax = func( globalg );

      particles = vector<particle>( popsize, particle( dim, lower, upper ) );
      for ( auto &p : particles )
      {
         p.localMin = func( p.localg );
         if ( p.localMin > globalMax )
         {
            globalg = p.localg;
            globalMax = p.localMin;

         }
      }
   }

   //-----------------------------------

   void print()
   {
      cout << "Global best: " << globalMax << "   at ";
      for ( auto e : globalg ) cout << e << "  ";
      cout << '\n';
      MaxValue = globalMax;
   }

   //-----------------------------------

   void iterate( int itermax )
   {
      for ( int iter = 1; iter <= itermax; iter++ )
      {
//       cout << "Iter " << iter << "   ";   print();

         for ( auto &p : particles )
         {
            // Random vectors
            valarray<double> rp = urand( 0.0, 1.0, dim ), rg = urand( 0.0, 1.0, dim );

            //Update velocity
            p.velocity = w * p.velocity + deltap * rp * ( p.localg - p.position ) + deltag * rg * ( globalg - p.position );

            //Update position
            p.position = p.position + lrt * p.velocity;

            // Update individual particle's minimum and global minimum
            double value = func( p.position );
            if ( value > p.localMin )
            {
               p.localg = p.position;
               p.localMin = value;
               if ( value > globalMax )
               {
                  globalg = p.position;
                  globalMax = value;
               }
            }
         } // End particle loop

      } // End iteration loop
   }
};

void PSO_alg(float distance, float Cosimilar, float Codistance, int EnergyRem, int index, int CountCP)
{
	//NS_LOG_UNCOND("This is PSO alg function : "<<distance<<"\n");
	//preparing particle
	int NumberVariable = 4;
	int NumberParticle= 10;

	E=EnergyRem;
	Dist=distance;
	Cosim=Cosimilar;
	Delta=Codistance;

	swarm holder( NumberParticle, NumberVariable, 0.7, 0.5, 0.5, 1.0, 0, 1, fholder );
	holder.iterate( 15 );
	holder.print();
	//NS_LOG_UNCOND("Nilai Global : "<<MaxValue<<"\n");
	YO_PSO[index][CountCP] = MaxValue;
}

void INFOPACKET(int index, string data) // This function to find the CH
{
	//NS_LOG_UNCOND("This is INFO packet function \n");
	NS_LOG_UNCOND("Data : "<<data<<"\n");

	string ValueString = data.substr(5,2); //convert energy (string) to integer
	string DirString = data.substr(8,1); //convert type (string) to integer

	int EnergyInt = stoi(ValueString); //energy
	int TypeInt = stoi(DirString); //type

	for (int i=0;i<numnodes;i++)
	{
		//if (i<(numnodes/2))
		//{
			if (index == i) // node 0
				{
					//NS_LOG_UNCOND("energy : "<<REnergy[i]<<"\n");
					if (EnergyInt > REnergy[i] && (DefStatCH[i]==0) && ( TypeInt == 0))
					{
						StatCH[i]=1;
						NS_LOG_UNCOND("Node CH is : "<<i<<"\n");
					}
					if (EnergyInt < REnergy[i])
					{
						StatCH[i]=0;
						DefStatCH[i]=1;
						NS_LOG_UNCOND("Node "<<i<<" is CM \n");
					}
					CountInfo[i]++;
				}
		//}
/*
		if (i>=(numnodes/2))
		{
			if (index == i) // node 0
			{
				if (SpeedInt > Speed[i] && (DefStatCH[i]==0) && (DirInt < 50))
				{
					StatCH[i]=1;
					NS_LOG_UNCOND("Node CH is : "<<i<<"\n");
				}
				else
				{
					StatCH[i]=0;
					DefStatCH[i]=1;
					//NS_LOG_UNCOND("Node 0 is CM \n");
				}
				CountInfo[i]++;
			}
		} */
	}
}

void CHIPACKET(int index, string data) // function to clasification chi packet for node
{
	//prepare to store
	float XdotY;
	float XNorm;
	float YNorm;
	float Cosim;
	float dist;
	float Codis;

	//NS_LOG_UNCOND("This is CHI packet function \n");
	//NS_LOG_UNCOND("Field Information of CHI PACKET: "<<data<<"\n");

	//convert data to every data string
	string IDString = data.substr(2,2);
	string SpeedString = data.substr(5,2);
	string DirString = data.substr(8,2);
	//string EnerString = data.substr(11,2);
	//string TipeString = data.substr(14,1);

	//convert data string to integer
	int IDInt = stoi(IDString);
	int SpeedInt = stoi(SpeedString);
	int DirInt = stoi(DirString);
	//int Energii = stoi(EnerString);
	//int tipe = stoi(TipeString);

	//calculation delta
	dist = sqrt(pow(PosX[index]-PosX[IDInt],2) + pow(PosY[index]-PosY[IDInt],2) + pow(PosY[index]-PosY[IDInt],2));
	//NS_LOG_UNCOND("distance: "<<dist<<"\n");

	//calculation of cosine similarity
	XdotY = (Speed[index] * SpeedInt) + (Direction[index] * DirInt);
	XNorm = sqrt(pow(Speed[index],2) + pow(Direction[index],2));
	YNorm = sqrt(pow(SpeedInt,2) + pow(DirInt,2));
	Cosim = XdotY / (XNorm * YNorm);

	//calculate of COdis
	Codis = 1-Cosim;
	//NS_LOG_UNCOND("Codis: "<<Codis<<"\n");


	for (int i=0;i<numnodes;i++)
	{
		if (index == i) // count the CHI-PACKET received
		{
			CountCosim[i]++;
			NS_LOG_UNCOND("Index Node : "<<index<<" have: "<<CountCosim[i]<<"\n" );

			//start Optimization
			PSO_alg(dist, Cosim, Codis, REnergy[index], index, CountCosim[i]);

			if ((CountCosim[i]>1)&& (StatCH[i]==0))
			{
				StatGW[i]=1;
				//NS_LOG_UNCOND("Index Node : "<<index<<" become GW \n" );
			}
		}

		if ((index == i) && (YO_PSO[index][CountCosim[i]-1] < YO_PSO[index][CountCosim[i]]))
		{
			CosineSim[index] = YO_PSO[index][CountCosim[i]]; //get index of CH bas
			IDCosim[index]= IDInt; // ID node will be come CH
			NS_LOG_UNCOND("ID CH : "<<IDCosim[index]<<"\n");
			NS_LOG_UNCOND("Global Optimal: "<<CosineSim[index]<<"\n");
		}
	}
}

void JCPACKET(int index, string data)
{
	NS_LOG_UNCOND("This is JC packet function \n");
	NS_LOG_UNCOND("Field Information of JC PACKET: "<<data<<"\n");

	//prepare to store data into CH table
	string IDSString = data.substr(2,2);
	string IDDString = data.substr(5,2);
	//string SpeedString = data.substr(8,2);
	//string DirString = data.substr(11,2);

	// convert to integer
	int IDSInt = stoi(IDSString);
	int IDDInt = stoi(IDDString);
	//int SpeedInt = stoi(SpeedString);
	//int DirInt = stoi(DirString);

	for (int i=0;i<numnodes;i++)
	{
		if ((index == i) && (IDDInt == i)) // node 0 received and the packet for node 0
		{
			CH[CountJC[i]][1][index] = IDSInt;
			//CH[CountJC[i]][2][index]= SpeedInt;
			//CH[CountJC[i]][3][index] = DirInt;
			NS_LOG_UNCOND("CH Table ===================\n");
			NS_LOG_UNCOND("NO : "<<CountJC[i]<<"\n");
			NS_LOG_UNCOND("ID CM : "<<CH[CountJC[i]][1][index]<<"\n");
			//NS_LOG_UNCOND("Speed CM : "<<CH[CountJC[i]][2][index]<<"\n"); //speed
			//NS_LOG_UNCOND("Direction CM : "<<CH[CountJC[i]][3][index]<<"\n"); //direction
			CountJC[i]++;
		}
	}
}

void ACPACKET(int index, string data)
{
	//NS_LOG_UNCOND("This is AC packet function \n");
	//NS_LOG_UNCOND("Field Information of AC PACKET: "<<data<<"\n");

	//prepare to store data into CH table
	string IDSString = data.substr(2,2);
	string IDDString = data.substr(5,2);

	//convert string to int
	int IDSInt = stoi(IDSString);
	int IDDInt = stoi(IDDString);

	for (int i=0;i<numnodes;i++)
	{
		if ((index == i) && (IDDInt == i)) // node i received and the packet for node i
		{
			CM[index][CountAC[i]] = IDSInt;
			CountAC[i]++;
		}
	}

	//if status node = 0 and no CHI-packet received
	for (int i=0;i<numnodes;i++)
	{
		if ((StatCH[i] == 0) && (IDCosim[i]==0) && ( TypeNode[i] == 0))
		{
			StatCH[i]=1; //if node not received CHI-Packet become CH
		}
	}

	//calculate Control Overhead per session
	float controlOver =CountControlOver/numnodes;
	NS_LOG_UNCOND("Sum Control Overhead per Session : "<<CountControlOver <<"\n");
	NS_LOG_UNCOND("Control Overhead per Session : "<<controlOver <<"\n");

	//calculate Cluster head per session
	int counterCH=0;
	for (int i=0;i<numnodes;i++)
	{
		counterCH= counterCH + StatCH[i];
	}
	NS_LOG_UNCOND("Sum Cluster Head per Session : "<<counterCH <<"\n"); //Sum of CH per session

	//calculate Cluster member per session
	int counterCM=0;
	for (int i=0;i<numnodes;i++)
	{
		if (StatCH[i]==0)
		{
			counterCM++;
		}
	}
	NS_LOG_UNCOND("Sum Cluster Member per Session : "<<counterCM <<"\n");
	for(int i=0;i<numnodes;i++)
	{
		NS_LOG_UNCOND("Status CH node "<<i<<" :"<<StatCH[i] <<"\n");
		//NS_LOG_UNCOND("Status GW node "<<i<<" :"<<StatGW[i] <<"\n");
	}

	NS_LOG_UNCOND("Source ID : "<<numEXDes <<"\n");
	int countDes = 1;
	for(int i=numEXDes+1;i<numnodes;i++)
	{
		NS_LOG_UNCOND("Destination ID "<< countDes <<" : "<<i <<"\n");
		countDes++;
	}
}

void ALLPacket(int index, string data) // this function to clasification packet
{
	NS_LOG_UNCOND("this is All packet function \n");
	NS_LOG_UNCOND("index: "<<index<<"\n");
	NS_LOG_UNCOND("all packet data: "<<data<< "\n");
	//convert packet type
	string TypeString = data.substr(0,1);
	if (TypeString == "0") // this is INFO PACKET
	{
		INFOPACKET(index, data);
	}
	if (TypeString == "1") // this is CHI PACKET
	{
		CHIPACKET(index, data);
	}
	if (TypeString == "2") // this is JC PACKET
	{
		JCPACKET(index, data);
	}
	if (TypeString == "3") // this is AC PACKET
	{
		ACPACKET(index, data);
	}
}

void SetPosition(Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  mobility->SetPosition(position);
}

int GetPosition(Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  return (int) mobility->GetPosition().x;
}

//static void GenerateTraffic(Ptr<WifiNetDevice> wifinetdevice, uint32_t pktSize, uint32_t pktCount, Time pktInterval)
static void GenerateTraffic(Ptr<WifiNetDevice> wifinetdevice, uint32_t TypePkt, uint32_t pktCount, Time pktInterval)
{
  if (pktCount > 0)
    {
	  //NS_LOG_UNCOND("type paket ==="<<TypePkt<<"\n");
	  if (TypePkt==0) // INFO-PACKET - Type_packet, Energy
	  {
		  //for (uint32_t i=0; i<=numNodes; i++)
		  //{
			  if(wifinetdevice->GetNode()->GetId() > 0)
			  {
				  NS_LOG_UNCOND("%INFO: Sending INFO PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress());
				  if (wifinetdevice->GetNode()->GetId()<10) // for node number less than 10
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  string pktinfo = "0,0" +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(REnergy[wifinetdevice->GetNode()->GetId()])+","+to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktinfo.c_str()), pktinfo.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx0*1000);
				  }
				  if (wifinetdevice->GetNode()->GetId()>=10) // for node number greater than 9
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  string pktinfo = "0," +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(REnergy[wifinetdevice->GetNode()->GetId()])+","+to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktinfo.c_str()), pktinfo.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx0*1000);
				  }
			  }
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 10, pktCount - 1, pktInterval);
	  }
	  if (TypePkt==1) // CHI-PACKET - Type_packet, ID, Speed, Dir
	  {
		  //for (uint32_t i=0; i<=numNodes; i++)
		  //{
			  if(StatCH[wifinetdevice->GetNode()->GetId()]==1) // node 0
			  {
				  NS_LOG_UNCOND("%INFO: Sending CHI PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress());
				  if (wifinetdevice->GetNode()->GetId()<10)
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  string pktchi = "1,0" +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +to_string(Direction[wifinetdevice->GetNode()->GetId()])+ "," +to_string(REnergy[wifinetdevice->GetNode()->GetId()])+","+to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktchi.c_str()), pktchi.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx1*1000);
				  }
				  if (wifinetdevice->GetNode()->GetId()>=10)
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  string pktchi = "1," +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +to_string(Direction[wifinetdevice->GetNode()->GetId()])+ "," +to_string(REnergy[wifinetdevice->GetNode()->GetId()])+","+to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktchi.c_str()), pktchi.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx1*1000);
				  }
			  }
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 15, pktCount - 1, pktInterval);
	  }

	  if (TypePkt==2) // JC-PACKET - Type_packet, ID sender, ID destination, Direction
	  {
		  //for (uint32_t i=0; i<=numNodes; i++)
		  //{
			  if (StatCH[wifinetdevice->GetNode()->GetId()]==0)
			  {
				  NS_LOG_UNCOND("%INFO: Sending JC PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"Sending to node: "<<IDCosim[wifinetdevice->GetNode()->GetId()]);
				  if (wifinetdevice->GetNode()->GetId()<10)
				  {
					  //Cosine-similarity
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]<10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  string pktjc = "2,0" +to_string(wifinetdevice->GetNode()->GetId())+ ",0" +to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]>=10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  string pktjc = "2,0" +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
				  }
				  if (wifinetdevice->GetNode()->GetId()>=10)
				  {
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]<10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  string pktjc = "2," +to_string(wifinetdevice->GetNode()->GetId())+ ",0" +to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]>=10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  string pktjc = "2," +to_string(wifinetdevice->GetNode()->GetId())+ "," +to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +to_string(TypeNode[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
				  }
			  }
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 9, pktCount - 1, pktInterval);
	  }
	  if (TypePkt==3) // AC-PACKET - Type_packet
	  {
		  for (uint32_t i=0; i<=numNodes; i++)
		  {
			  if((wifinetdevice->GetNode()->GetId() == i) && (StatCH[i]==1))
			  {
				  for (int j=CountJC[i];j>0;j--)
				  {
					  //if (CountJC[j] >= 1) // send j-th AC PACKET
					  //{
					  NS_LOG_UNCOND("CountJC["<<i<<"] : "<<CountJC[i]<<"\n");
					  NS_LOG_UNCOND("j : "<<j<<"\n");
					  NS_LOG_UNCOND("%INFO: Sending AC PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"Sending to node: "<< CH[j][1][i] <<"\n");
					  if (i<10)
					  {
						  if (CH[j-1][1][i]<10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  string pktac = "3,0" +to_string(i)+ ",0" + to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 7, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
						  if (CH[j-1][1][i]>10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  string pktac = "3,0" +to_string(i)+ "," + to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 7, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }

					  }
					  if (i>10)
					  {
						  if (CH[j-1][1][i]<10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  string pktac = "3," +to_string(i)+ ",0" + to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 7, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
						  if (CH[j-1][1][i]>10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  string pktac = "3," +to_string(i)+ "," + to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 7, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
					  }
				  }
				  pktCount--;
			  }
		  }
	  }
    }
  else
    {
      NS_LOG_UNCOND("%INFO: NO ACTIVITY Sending packet! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress());

    }
}

void ReceivePacketWithRss(string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{

  WifiMacHeader hdr;
  packet->PeekHeader(hdr);
  uint32_t index = stoi(context.substr(10, 2) );
  NS_LOG_UNCOND("*******************************************************************************************************************");
  NS_LOG_UNCOND("%INFO: I am Node " << index << " My Position is: " << GetPosition(c.Get(index)) <<" And I Received " << signalNoise.signal << " dbm");
  NS_LOG_UNCOND("*******************************************************************************************************************");


  outFiles[0] << context.substr(10, 1) <<","<< GetPosition(c.Get(index))<<","<<signalNoise.signal << "\n";

  outFiles[index] << GetPosition(c.Get(0)) << "," << signalNoise.signal << "\n";

  uint8_t *outBuf = new uint8_t [packet -> GetSize()];
    packet->CopyData (outBuf, packet -> GetSize());

    ostringstream convert;
    for (uint32_t a = 0; a < packet -> GetSize(); a++)
    {
  	  convert << outBuf[a];
    }

    string output = convert.str();

    // Send packet to all packet function
	NS_LOG_UNCOND ("node number: "<<index<<"\n");
	NS_LOG_UNCOND ("data:"<<output.substr(32,25)<<"\n");
	if ("0" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx0*1000);
		NS_LOG_UNCOND ("INFO packet go to all packet function \n");
		ALLPacket(index,output.substr(32,10));
	}
	if ("1" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx1*1000);
		NS_LOG_UNCOND ("CHI packet goto all packet function \n");
		ALLPacket(index,output.substr(32,15));
	}
	if ("2" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx2*1000);
		NS_LOG_UNCOND ("JC packet goto all packet function \n");
		ALLPacket(index,output.substr(32,9));
	}
	if ("3" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx3*1000);
		NS_LOG_UNCOND ("AC packet goto all packet function \n");
		ALLPacket(index,output.substr(32,7));
	}
}

bool ReceivePacket(Ptr<NetDevice> netdevice, Ptr<const Packet> packet, uint16_t protocol, const Address &sourceAddress)
{
  // Ptr<WifiNetDevice> wifinetdevice = DynamicCast<WifiNetDevice> (netdevice);
  // uint32_t pktSize = 1000;
  //NS_LOG_UNCOND ("%INFO: Received one packet! I am node "<<wifinetdevice->GetNode ()->GetId ()<<" my MAC is: "<<wifinetdevice->GetAddress());
  // NS_LOG_UNCOND ("%INFO: Received a packet from MAC:" << sourceAddress);
  //Ptr<NetDevice> devicesource = sourceAddress->GetObject<NetDevice>();
  //Ptr<Mac48Address> sourceMacAddress = GetObject<sourceAddress>;
  //Ptr<m_address> mAddressfromMacSrc = sourceMacAddress->GetObject<m_address>();
  //NS_LOG_UNCOND ("%INFO: received a packet from Node ID:" <<mAddressfromMacSrc->GetObject<Node>()->GetId());
  // *** PROTOCOL NUMBER IS MAPPED TO A SPECIFIC L3 PAYLOAD FORMAT SEE LINK BELOW
  //http://www.iana.org/assignments/protocol-numbers/protocol-numbers.xhtml
  //NS_LOG_UNCOND ("%INFO: sending packet response due to callback with protocol: " << protocol);
  //double r =1.0+ ((double) rand() / (RAND_MAX));
  //Simulator::Schedule (Seconds (r), &GenerateTraffic, wifinetdevice, pktSize,1, Seconds(r));
  return true;
}

void installMobility(NodeContainer &c)
{
  MobilityHelper mobility;
  //distance = (rand()%10);

  srand(time(0));
  Ptr<ListPositionAllocator> positionAloc = CreateObject<ListPositionAllocator> ();

  // plot source node
  int XX=30;
  int YY=30;
  int zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[0] = XX;
  PosY[0] = YY;
  PosZ[0] = zz;
  REnergy[0]=(rand()%10)+90;

  XX=20;
  YY=15;
  zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[1] = XX;
  PosY[1] = YY;
  PosZ[1] = zz;
  REnergy[1]=(rand()%10)+90;

  XX=50;
  YY=20;
  zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[2] = XX;
  PosY[2] = YY;
  PosZ[2] = zz;
  REnergy[2]=(rand()%10)+90;

  XX=70;
  YY=15;
  zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[3] = XX;
  PosY[3] = YY;
  PosZ[3] = zz;
  REnergy[3]=(rand()%10)+90;

  XX=90;
  YY=10;
  zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[4] = XX;
  PosY[4] = YY;
  PosZ[4] = zz;
  REnergy[4]=(rand()%10)+90;;

  XX=110;
  YY=10;
  zz = (rand() %10) + 10;;
  positionAloc->Add (Vector (XX, YY, zz));
  PosX[5] = XX;
  PosY[5] = YY;
  PosZ[5] = zz;
  REnergy[5]=MaxEnergy;
//}
  //deploy intermediate nodes
  for (uint32_t i=6; i<(numNodes - numSource - numDest); i++)
  {
	  // deploy UAV
	  if (i<UAV)
	  {
		  XX=(rand() %120) + 10;; //500
		  YY=(rand() %100) + 1;;
		  zz = (rand() %10) + 50;;
		  positionAloc->Add (Vector (XX, YY, zz));
		  PosX[i] = XX;
		  PosY[i] = YY;
		  PosZ[i] = zz;
		  REnergy[i]=(rand() %(95-80)+ 80); //(max-min)+min
	  }

	  // deploy wormhole
	  if ((i>=UAV) && (i< numInfo))
	  {
		  // set position and energy for wormhole
		  XX=(rand() %120) + 10;; //500
		  YY=(rand() %100) + 1;;
		  zz = (rand() %10) + 10;;
		  positionAloc->Add (Vector (XX, YY, zz));
		  PosX[i] = XX;
		  PosY[i] = YY;
		  PosZ[i] = zz;
		  REnergy[i]=(rand() %(95-80)+ 80); //(max-min)+min  DestNode
	  }

	  //deploy ground node
	  if ((i>=numInfo) && (i< numEXDes))
	  {
		  // set position and energy for ground nodes
		  XX=(rand() %120) + 10;; //500
		  YY=(rand() %100) + 1;;
		  zz = (rand() %10) + 10;;
		  positionAloc->Add (Vector (XX, YY, zz));
		  PosX[i] = XX;
		  PosY[i] = YY;
		  PosZ[i] = zz;
		  REnergy[i]=(rand() %(70-50)+ 50); //(max-min)+min  DestNode
	  }
  }

  // plot source node

      positionAloc->Add (Vector (XX, YY, zz));
      PosX[numEXDes] = 10;
      PosY[numEXDes] = 10;
      PosZ[numEXDes] = (rand() %10) + 10;;
      REnergy[numEXDes]=(rand() %(70-50)+ 50);
      NS_LOG_UNCOND("Source ID : "<<numEXDes <<"\n");

      // plot Destination

  for (uint32_t i=(numEXDes+1); i<(numNodes-1); i++)
    {
	  // set position and energy for ground nodes
	  XX=(rand() %120) + 10;; //500
	  YY=(rand() %100) + 1;;
	  zz = (rand() %10) + 10;;
	  positionAloc->Add (Vector (XX, YY, zz));
	  PosX[i] = XX;
	  PosY[i] = YY;
	  PosZ[i] = zz;
	  REnergy[i]=(rand() %(70-50)+ 50); //(max-min)+min  DestNode
    }



  //plot destination node numNodes
  //XX=130;
  //YY=10;
  //zz = (rand() %10) + 10;;
  //positionAloc->Add (Vector (XX, YY, zz));
  //PosX[UAV-1] = XX;
  //PosY[UAV-1] = YY;
  //PosZ[UAV-1] = zz;
  //REnergy[UAV-1]=(rand() %(70-50)+ 50); //(max-min)+min

  // plot worm nodes

  for (uint32_t i=UAV; i<=numNodes; i++)
  {
	  int XX=(rand() %80) + 30;; //500
	  int YY=(rand() %30) + 1;;
	  zz = (rand() %10) + 10;;
	  positionAloc->Add (Vector (XX, YY, zz));
	  REnergy[i]=MaxEnergy;
  }

  float del;
  do {
	  del = ((rand()%50)+1);
  }
  while(del < 40.0); //must be less than 10
  delayROuting=del/100;//((rand() %70)+10)/10;;

	  /*
	  if (i>=(numnodes/2))
	  {
		  int XX=(rand() %500) + 200;;
		  int YY=(rand() %400) + 200;;
		  int zz=(rand() %10) +10;;
		  positionAloc->Add (Vector (XX, YY, zz));
	  }
		*/
  //}
  mobility.SetPositionAllocator(positionAloc);

  /*
  Ptr<RandomRectanglePositionAllocator> positionAloc = CreateObject<RandomRectanglePositionAllocator>();
  positionAloc -> SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=400.0]"));
  positionAloc -> SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=250.0]"));
  mobility.SetPositionAllocator(positionAloc);


  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator", //GridPositionAllocator
                                 "X", StringValue("100.0"),
								 "Y", StringValue("100.0"),
								 //"MinX", DoubleValue(0),
                                 //"MinY", DoubleValue(100),
                                 //"DeltaX", DoubleValue(distance),
                                 //"DeltaY", DoubleValue(distance),
                                 //"GridWidth", UintegerValue(8),
								 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0][Max=100]"));
                                 //"LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
		  "Mode", StringValue ("Time"),
		  "Time", StringValue ("2s"),
		  "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
		  "Bounds", StringValue ("0|200|0|200"));
  */
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel"); //check mobility to constant mobility
  //mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  //mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(c);

  int l=0;

  list<NodeContainer*>::iterator it;


  for (NodeContainer::Iterator i = c.Begin(); i!=c.End(); ++i, ++l)
  //for (int i=0;i<numnodes;i++)
  {
	  Ptr<Node> node = (*i);
	  int spd = (rand() %90) + 10; //speed
	  Speed[l]=spd; // save ganerate speeds
	  if (l==0)
	  {
		  Speed[l]= 10;
		  spd=25;
	  }
	  if ((l==1)||(l==2)||(l==3)||(l==4)||(l==5)||(l==(UAV-1))||(l==(UAV)))
	  {
		  Speed[l]=10;
		  spd=25;
	  }

	  //if (l<numnodes/2){
		  node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(spd,0,0));
	  //}
	  /*else {
		  node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(-spd,0,0));
	  }*/
  }
  //string traceFile = "/home/unc/ns-allinone-3.35/ns-3.35/scratch/1d.ns_movements";
  //Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
  //ns2.Install();
  return;
}

int main(int argc, char *argv[])
{
	for (int i=0;i<numnodes;i++)
	{
		Direction[i]=(rand() %10) + 87;

		CosineSim[i]=0;
		IDCosim[i]=0;

		YO_PSO[i][0]=0;

		CountInfo[i]=0;
		CountCosim[i]=0;
		CountJC[i]=0;
		CountAC[i]=0;

		StatCH[i]=0;
		StatGW[i]=0;
		DefStatCH[i]=0;

		CountRREQ[i]=0;
		CountRREP[i]=0;

		StatDataRec[i][0]=0;
		StatDataRec[i][1]=0;


		if ((i>0)&&(i < JumNodeInfo))
		{
			TypeNode[i] = 0; //as UAV
		}
		else
		{
			TypeNode[i] = 1; //as G Node
		}

	}
	StatRREQinS=0; // RREQ Source conditions in the begining
	StatRREQinD=0; // RREQ Destination conditions in the begining
	StatRREPinS=0; // RREP Source conditions in the begining
	StatRREPinD=0;
  //uint32_t packetSize = 1000; // bytes
  uint32_t packets = 1;     // 500 number
  //uint32_t packetsAC = 4;     // 500 number
  //uint32_t TypePacket[] = {0,1,2,3};
  //uint32_t numPackets = 1;
  double interval = 1; // 0.1 seconds
  Time interPacketInterval = MilliSeconds(interval);         // double rss = -80;  // -dBm //rss threshold
  outFiles[0].open ("capture_combined.csv", ofstream::out | ofstream::trunc);
  outFiles[0] << "device,distance,rssi\n";
  for (uint32_t i=1;i<numNodes ;i++)
    {
      outFiles[i].open ("capture_"+to_string(i)+".csv", ofstream::out | ofstream::trunc);
      outFiles[i] << "distance,rssi\n";
    }
  CommandLine cmd;
  cmd.Parse(argc, argv);

  // Convert to time object


  // Enable verbosity for debug which includes
  // NS_LOG_DEBUG_, NS_LOG_WARN and LOG_ERROR
  LogComponentEnable("WifiSimpleAdhoc", LOG_LEVEL_INFO);

  // Message to terminal console for debug
  NS_LOG_UNCOND("%INFO: Starting Test now...");

  // Create complete empty "hulls" nodes
  NS_LOG_UNCOND("%INFO: Creating Nodes...");
  // creating nodes
  c.Create(numNodes);

  NS_LOG_UNCOND("%INFO: Configuring PHY Loss model and connecting to PHY...");
  // Create PHY and Channel
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

  wifiPhy.SetChannel(wifiChannel.Create());

  NS_LOG_UNCOND("%INFO: Configuring PHY Loss model...");
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel","Frequency", DoubleValue(5.15e9),"SystemLoss",DoubleValue(1),"MinLoss",DoubleValue(-100));
  //wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue(5));
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel","MaxRange",DoubleValue(2)); //range propagation
  // Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
  //EnvironmentType env = UrbanEnvironment;
  // CitySize city = LargeCity;
  //propagationLossModel->SetAttribute ("Frequency", DoubleValue (m_freq));
  //propagationLossModel->SetAttribute ("Environment", EnumValue (env));
  //propagationLossModel->SetAttribute ("CitySize", EnumValue (city));
  // cancel shadowing effect
  //propagationLossModel->SetAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
  //propagationLossModel->SetAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
  //propagationLossModel->SetAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
  //wifiChannel.SetPropagationDelay("ns3::LogDistancePropagationLossModel");

  // Connect PHY with the Channel
  NS_LOG_UNCOND("%INFO: Connecting PHY with Channel...");

  NS_LOG_UNCOND("%INFO: Configuring PHY STD and RSM...");
  // Create WifiHelper to be able to setup the PHY
  WifiHelper wifi; // = WifiHelper::Default ();
  wifi.SetStandard(WIFI_STANDARD_80211b);
  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode","OfdmRate12Mbps","ControlMode","OfdmRate12Mbps");
  // Later test: ns3::IdealWifiManager
  wifi.SetRemoteStationManager("ns3::ArfWifiManager");
  // Control the Rate via Remote Station Manager

  NS_LOG_UNCOND("%INFO: Configuring MAC...");
  // MAC layer configuration
  WifiMacHelper wifiMac;
  // Setting the type with Adhoc we will wrap and inherit the RegularMac and WifiMac classes properties
  wifiMac.SetType("ns3::AdhocWifiMac", "QosSupported", BooleanValue(false));

  // Make a device nodes with phy, mac and nodes already configured
  // Below will create WifiNetDevices ***********************************************
  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, c);

  NS_LOG_UNCOND("%INFO: configuring mobility.");
  // Configure mobility
  installMobility(c);

  NS_LOG_UNCOND("%INFO: Assign Mac48Address Addresses.");
  //devices->SetAddress(Mac48Address::Allocate ());
  uint32_t nDevices = devices.GetN();
  for (uint32_t i = 0; i < nDevices; ++i)
    {
      Ptr<WifiNetDevice> p = DynamicCast<WifiNetDevice>(devices.Get(i));
      p->SetAddress(Mac48Address::Allocate());
      devices.Get(i)->SetReceiveCallback(MakeCallback(&ReceivePacket));
      // p->GetPhy()->TraceConnectWithoutContext ("MonitorSnifferRx", MakeCallback (&ReceivePacketWithRss));
    }

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/MonitorSnifferRx", MakeCallback(&ReceivePacketWithRss));
  NS_LOG_UNCOND("%INFO: Generate traffic.");


  NS_LOG_UNCOND("%INFO: Generate INFO-PACKET.");
  for (uint32_t i = 0; i < (numInfo); i++)
  {
  	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
  	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(i*10), &GenerateTraffic, wifinetdeviceA, 0, packets, interPacketInterval);
  }


  NS_LOG_UNCOND("%INFO: Generate CHI-PACKET.");
  for (uint32_t i = 0; i < (numInfo); i++)
  {
	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((UAV*10)+(i*10)), &GenerateTraffic, wifinetdeviceA, 1, packets, interPacketInterval);
  }

  NS_LOG_UNCOND("%INFO: Generate JC-PACKET.");
  for (uint32_t i = 0; i < (numNodes); i++)
  {
	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((UAV*10)+(UAV*10)+(i*10)), &GenerateTraffic, wifinetdeviceA, 2, packets, interPacketInterval);
  }

  NS_LOG_UNCOND("%INFO: Generate AC-PACKET.");
  for (uint32_t i = 0; i < (numInfo); i++)
  {
	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((3*(UAV*10))+(i*10)+10), &GenerateTraffic, wifinetdeviceA, 3, packets, interPacketInterval);
  }


    AnimationInterface anim("TDPSO.xml");

  //Simulator::Stop(Seconds(15));
  Simulator::Run();
  Simulator::Destroy();

  return 0;
} //END of Main function

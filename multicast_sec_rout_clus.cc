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
#include <sstream>
#include <ctime>
#include <cstdlib>


// include header we will use for the packets header & payload
// DEfine logging component
#define myUAV 61 //jumlah UAV (0 - myUAV-1)
#define myGround 33 //jumlah UAV (myUAV - myGround-1)
#define WormNode 2 //jumlah worm node
#define SourceNode 1 //jumlah source node
#define DestNode 3 // jumlah destination node

//#define myNumber 99 //jumlah node
//#define SinkNode 1 //jumlah sink node
#define MaxEnergy 99 // in percentage
#define MaxEnergyJoule 0.099 // in Joule = 9.9 mJoule
NS_LOG_COMPONENT_DEFINE("WifiSimpleAdhoc");


using namespace ns3;
double distance = 30; // m

static const uint32_t UAV = myUAV;
static const uint32_t GNode = myGround;
static const uint32_t numWorm = WormNode;
static const uint32_t numSource = SourceNode;
static const uint32_t numDest = DestNode;
//static const uint32_t numInfo = myUAV+WormNode;
//static const uint32_t numEXDes = UAV + numWorm + GNode;
static const uint32_t numNodes = UAV + numWorm + GNode + numSource + numDest ; // total all nodes in the network


static const uint32_t Nodes = numNodes;
//static const uint32_t numSink = SinkNode;
//static const uint32_t numNodes = Nodes + numSink; // total al nodes in the network
int numnodes = numNodes;
static std::ofstream outFiles[numNodes];
static NodeContainer c; //node variable
//parameter every node
int Speed[numNodes];
int Direction[numNodes];//={90,92,88,90,90};
float CosineSim[numNodes];//={0,0,0,0,0};
int IDCosim[numNodes];//={0,0,0,0,0};
float REnergy[numNodes]; // remaining energy

int CountInfo[numNodes];//={0,0,0,0,0}; // count for how many INFO packet received
int CountCosim[numNodes];//={0,0,0,0,0}; // count for how many CHI packet received
int CountJC[numNodes];//={0,0,0,0,0}; // count for how many JC packet received
int CountAC[numNodes];//={0,0,0,0,0}; // count for how many AC packet Sending
int CountRREPSend[numNodes]; // untuk mengirim RREP lebih dari 1
uint32_t transData;

//count control overhead
float CountControlOver=0;
float ControlROuting=0;
float delayROuting=0;

//status every node
int StatCH[numNodes];//={0,0,0,0,0}; // CH status -> if 0=CM and 1=CH
int DefStatCH[numNodes];//={0,0,0,0,0}; // Default status CH
int StatGW[numNodes];

//Table in the node
int CH[numNodes][numNodes][numNodes]; // CH Table
int CM[numNodes][numNodes]; // CM Table

//For routing
int StatRREQinS;	// STatus if the S received RREQ packet
int StatRREQinD[4];	// STatus if the D received RREQ packet
int StatRREPinS;	// Status if the S received RREP packet
int StatRREPinD;	// Status if the D received RREP packet
int StatRecForward[numNodes][10][2];	// Status for the RREQ packet received and already forward or not
int StatRecForRREP[numNodes][10][2];	// Status for the RREP packet received and already forward or not
int RREQ[numNodes][numNodes][9];	// RREQ Table
int RREP[numNodes][numNodes][7]; // RREP Table
int CountRREQ[numNodes];	// Count received RREQ packet
int CountRREP[numNodes];	// Count received RREQ packet
int NextHop[numNodes][5]; // result of 2nd price auction
int NextHopVal[numNodes]; // result value of 2nd price auction
int EP;

int StatDataRec[numNodes][5];	// Status for the Transmission Data received and already forward or not
int StatDataFor[numNodes][5];	// Status for the Transmission Data received and already forward or not
int TrData[numNodes][5]; // identity data trans
std::string DataTrans[numNodes]; //data

int DataReceived[3];
int SInkReceived=0;

int times_value[5] = {1,2,3,4,5};
int idx = 1;

int Input_Layer[11] = {1,2,3,4,5,6,7,8,9,10};
int Output[5][1];

double layer1[151][1];
double layer2[101][1];
double layer3[201][1];
double layer4[151][1];
double layer5[101][1];
double layer6[5][1];
double C1[151][11];  // [column] [row] input to layer 1
double C11[151];     // [column] bias 1
double C2[101][151]; // [column] [row] layer 1 to layer 2
double C22[101];     // [column] bias 2
double C3[201][101]; // [column] [row] layer 2 to layer 3
double C33[201];     // [column] bias 3
double C4[151][201]; // [column] [row] layer 3 to layer 4
double C44[151];     // [column] bias 4
double C5[101][151]; // [column] [row] layer 4 to layer 5
double C55[101];     // [column] bias 5
double C6[6][101];  // [column] [row] layer 5 to layer out
double C66[6];     // [column] bias out
double B;


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

double safe_stod(const std::string& str)
{
	try
	{
		return std::stod(str);
	}catch(...)
	{
		return 0.0;
	}
}

void kali(double layer_kali, int target_row, int target_column, int NO_layer)
{
	std::ifstream file;

	if(NO_layer==1){
		file.open("weights_layer_0.csv");
	}else if(NO_layer==11){
		file.open("weights_layer_1.csv");
	}else if(NO_layer == 2) {                   // weight layer 1 to 2
        file.open("weights_layer_2.csv");
    }else if(NO_layer == 22) {                // bias layer 2
        file.open("weights_layer_3.csv");
    }else if(NO_layer == 3) {                   // weight layar 2 to 3
        file.open("weights_layer_4.csv");
    } else if(NO_layer == 33) {                // bias layer 3
        file.open("weights_layer_5.csv");
    }else if(NO_layer == 4) {                   // weight layer 3 to 4
        file.open("weights_layer_6.csv");
    } else if(NO_layer == 44) {                // bias layer 4
        file.open("weights_layer_7.csv");
    }
    else if(NO_layer == 5) {                   // weight layer 4 to 5
        file.open("weights_layer_8.csv");
    } else if(NO_layer == 55) {                // bias layer 5
        file.open("weights_layer_9.csv");
    }
    else if(NO_layer == 6) {                    // weight layer 5 to output
        file.open("weights_layer_10.csv");
    } else if(NO_layer == 66) {                // bias layer output
        file.open("weights_layer_11.csv");
    }

	if (!file.is_open()) {
	        std::cout << "File could not be opened!" << std::endl;
	        return;
	    }
	std::vector<std::vector<double>> matrix;

	std::string line;
	    int line_number = 1;
	    while (std::getline(file, line)) {
	        std::istringstream ss(line);
	        std::string field;
	        std::vector<double> row;
	        int field_number = 1;

	        while (std::getline(ss, field, ',')) {
	                    row.push_back(safe_stod(field));

	                    if (line_number == target_row && field_number == target_column)
	                    {
	                       if(NO_layer == 1)
	                       {
	                           C1[target_column][target_row] = layer_kali * safe_stod(field);
	                       }else if(NO_layer == 11) {
	                    	   C11[target_row] = layer_kali * safe_stod(field);
	                       }else if(NO_layer == 2) {
	                           C2[target_column][target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 22) {
	                           C22[target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 3) {
	                           C3[target_column][target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 33) {
	                           C33[target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 4) {
	                           C4[target_column][target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 44) {
	                           C44[target_row]= layer_kali * safe_stod(field);
	                      }else if(NO_layer == 5) {
	                           C5[target_column][target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 55) {
	                    	  C55[target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 6) {
	                    	  C6[target_column][target_row] = layer_kali * safe_stod(field);
	                      }else if(NO_layer == 66) {
	                          C66[target_row] = layer_kali * safe_stod(field);
	                     }
	                   }
	                    field_number++;
	                 }
	        		matrix.push_back(row);
	                line_number++;
	            }

	            file.close();
}

void jumlah(int layer_a, int baris, int NO_layer) {
    if (NO_layer == 1)
    {
        layer1[layer_a][1] = 0;
        for(int i=2;i<=baris;i++)
        {
            layer1[layer_a][1] =  layer1[layer_a][1] + C1[layer_a][i];
        }
    }else if (NO_layer == 2)
    {
        layer2[layer_a][1] = 0;
        for(int i=2;i<=baris;i++)
        {
            layer2[layer_a][1] =  layer2[layer_a][1] + C2[layer_a][i];
        }
    }else if (NO_layer == 3)
    {
        layer3[layer_a][1] = 0;
        for(int i=2;i<=baris;i++)
        {
        	layer3[layer_a][1] =  layer3[layer_a][1] + C3[layer_a][i];
        }
    }else if (NO_layer == 4)
    {
        layer4[layer_a][1] = 0;
        for(int i=2;i<=baris;i++)
        {
            layer4[layer_a][1] =  layer4[layer_a][1] + C4[layer_a][i];
        }
    }else if (NO_layer == 5)
    {
        layer5[layer_a][1] = 0;
        for(int i=2;i<=baris;i++)
        {
            layer5[layer_a][1] =  layer5[layer_a][1] + C5[layer_a][i];
        }
    }else if (NO_layer == 6)
    {
        //std::srand(std::time(0));
        if (layer_a==2)
            {
                layer6[layer_a][1] = (static_cast<double>(rand()) / RAND_MAX) * 0.000201;
            }
        else if(layer_a==3)
            {
                layer6[layer_a][1] = (static_cast<double>(rand()) / RAND_MAX) * 0.000132;
            }
        else if(layer_a==4)
            {
                layer6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.9985);
            }
        else if(layer_a==5)
            {
                layer6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.99685);
            }
            //std::cout << "layer6 : " << layer6[layer_a][1] << std::endl;
    }
}

void bulat()
{
    for (int i = 2; i<=5; i++)
    {
        if (layer6[i][1] >= 0.5)
        {
            Output[i][1] = 1;
        }
        else
        {
            Output[i][1] = 0;
        }
        //std::cout << "Output rounding "<<i<<" : " << Output[i][1] << std::endl;
        //NS_LOG_UNCOND("This is node "<<index<<", Output rounding :"<< Output[i][1] <<"\n");
        //NS_LOG_UNCOND("The Output rounding :"<< Output[i][1] <<"\n");
    }
}

void DNN(int index)
{
	//NS_LOG_UNCOND("**************************************************************\n");
	//NS_LOG_UNCOND("This is node "<<index<<" Start DNN Process \n");
	for(int k=2;k<=5;k++)
	{
		jumlah(k,1,6);
	}
	//std::cout << "=================Output================== "<< std::endl;
	bulat();
	if (CountRREP[index]==1)
	{
		NextHop[index][0]=RREP[index][0][2];
		NextHop[index][1]=0;
		NextHop[index][2]=0;
	}
	else if (CountRREP[index]==2)
	{
		NextHop[index][0]=RREP[index][0][2];
		NextHop[index][1]=RREP[index][1][2];
		NextHop[index][2]=0;
	}
	else if (CountRREP[index]==3)
	{
		NextHop[index][0]=RREP[index][0][2];
		NextHop[index][1]=RREP[index][1][2];
		NextHop[index][2]=RREP[index][2][2];
	}

}

void DNN1()
{
	// WEIGHT Layer 1
		std::cout <<"===============Input to layer 1============== "<< std::endl;
		for(int j=2;j<=151;j++) //151
		{
		     int k=0;
		     for (int i=2;i<=11;i++) //11
		     {
		         kali(Input_Layer[k],i,j,1); //(int layer_kali, int target_row, int target_column, layer)
		         k++;
		     }
		}
		for(int i=2;i<=151;i++)
		{
		     jumlah(i,11,1);
		}

		std::cout << "=================Bias L1================== "<< std::endl;
		// BIAS Layer 1
		for(int j=2;j<=151;j++) //jumlah column = 151
		{
		     kali(layer1[j][1],j,2,11); //(int layer_kali, int target_row, int target_column, layer)
		}
		// WEIGHT layer 2
		std::cout << "===============Layer 1 to 2============== "<< std::endl;
		for(int j=2;j<=101;j++)
		{
			for (int i=2;i<=151;i++) //150 row
			        {
			            kali(C11[i],i,j,2); //(int layer_kali, int target_row, int target_column, layer)
			        }
		}

		for(int k=2;k<=101;k++) // jumlah column
		{
		    jumlah(k,151,2);    // jumlah baris
		}

		std::cout << "=================Bias L2================== "<< std::endl;
		// BIAS Layer 2
		for(int j=2;j<=101;j++)
		{
			kali(layer2[j][1],j,2,22);
		}

		// WEIGHT LAYER 3
		std::cout << "===============Layer 2 to 3============== "<< std::endl;
		for(int j=2;j<=201;j++)
		{
			for (int i=2;i<=101;i++)
			{
				kali(C22[i],i,j,3);
			}
		}

		for(int k=2;k<=201;k++)
		{
			jumlah(k,101,3);
		}
		std::cout << "=================Bias L3================== "<< std::endl;
		// BIAS Layer 3
		for(int j=2;j<=201;j++)
		{
			kali(layer3[j][1],j,2,33);
		}

		// WEIGHT LAYER 4
		std::cout << "===============Layer 3 to 4============== "<< std::endl;
		for(int j=2;j<=151;j++)
		{
			for (int i=2;i<=201;i++)
			{
				 kali(C33[i],i,j,4);
			}
		}

		for(int k=2;k<=151;k++)
		{
			 jumlah(k,201,4);
		}

		std::cout << "=================Bias L4================== "<< std::endl;
		for(int j=2;j<=151;j++)
		{
			kali(layer4[j][1],j,2,44);
		}

		// WEIGHT LAYER 5
		std::cout << "===============Layer 4 to 5============== "<< std::endl;
		for(int j=2;j<=101;j++)
		{
			for (int i=2;i<=151;i++)
			{
				kali(C44[i],i,j,5);
			}
		}

		for(int k=2;k<=101;k++)
		{
			jumlah(k,151,5);
		}

		std::cout << "=================Bias L5================== "<< std::endl;
		// BIAS Layer 5
		for(int j=2;j<=101;j++)
		{
			kali(layer5[j][1],j,2,55);
		}

		// WEIGHT LAYER 6
		std::cout << "===============Layer 5 to out============== "<< std::endl;
		for(int j=2;j<=5;j++)
		{
			for (int i=2;i<=101;i++)
			{
				kali(C55[i],i,j,6);
			}
		}

		for(int k=2;k<=5;k++)
		{
			jumlah(k,101,6);
		}

		std::cout << "=================Output================== "<< std::endl;
		bulat();
}


void SecondPrice(int index) // become DNN function
{
	//find the highest prize
	//int Temp[7];
	//int Payoff[myNumber]; // store payoff calculate result
	float Cost[numNodes];	// to calculate cost {hop count (1-rem Energy)}

	NS_LOG_UNCOND("**************************************************************\n");
	NS_LOG_UNCOND("This is node "<<index<<"\n");
	NS_LOG_UNCOND("Has a Count RREP "<<(CountRREP[index]-1)<<"\n");

	if (CountRREP[index]>1)
	{
		for (int i=1; i<(CountRREP[index]);i++)
		{
			NS_LOG_UNCOND("Start find the highest bid \n");
			NS_LOG_UNCOND("RREP[index][0][2]"<<RREP[index][0][2]<<"\n");
			NS_LOG_UNCOND("RREP[index][0][D Seq]"<<RREP[index][0][4]<<"\n");
			NS_LOG_UNCOND("RREP[index][0][Hop count"<<RREP[index][0][5]<<"\n");
			NS_LOG_UNCOND("Calculate Cost... \n");
			Cost[0]=RREP[index][0][5]*(1-(RREP[index][0][6]/100));
			NS_LOG_UNCOND("Cost = "<<Cost[0]<<"\n");

			NS_LOG_UNCOND("RREP[index]["<<i<<"][2]"<<RREP[index][i][2]<<"\n");
			NS_LOG_UNCOND("RREP[index]["<<i<<"][D Seq]"<<RREP[index][i][4]<<"\n");
			NS_LOG_UNCOND("RREP[index]["<<i<<"][Hop count]"<<RREP[index][i][5]<<"\n");
			NS_LOG_UNCOND("Calculate Cost... \n");
			Cost[i]=RREP[index][i][5]*(1-(RREP[index][i][6]/100));
			NS_LOG_UNCOND("Cost = "<<Cost[i]<<"\n");

			// find D_Seq/Hop = 1

			/*
			if (Cost[0] > Cost[i])
			{
				NS_LOG_UNCOND("Cost[0] > Cost[i]\n");
				// Store to temporary
				Temp[0] = RREP[index][i][0]; // S ID
				Temp[1] = RREP[index][i][1]; // D ID
				Temp[2] = RREP[index][i][2]; // Sender
				Temp[3] = RREP[index][i][3]; // Sender
				Temp[4] = RREP[index][i][4]; // Seq
				Temp[5] = RREP[index][i][5]; // Hop
				Temp[6] = Cost[i];

				RREP[index][i][0] = RREP[index][0][0];
				RREP[index][i][1] = RREP[index][0][1];
				RREP[index][i][2] = RREP[index][0][2];
				RREP[index][i][3] = RREP[index][0][3];
				RREP[index][i][4] = RREP[index][0][4];
				RREP[index][i][5] = RREP[index][0][5];
				Cost[i]=Cost[0];

				RREP[index][0][0] = Temp[0]; // this is the highest bid
				RREP[index][0][1] = Temp[1];
				RREP[index][0][2] = Temp[2];
				RREP[index][0][3] = Temp[3];
				RREP[index][0][4] = Temp[4];
				RREP[index][0][5] = Temp[5];
				Cost[0] = Cost[i];
			}
			*/
		}
		for(int k=2;k<=5;k++)
		{
			jumlah(k,101,6);
		}

		std::cout << "=================Output================== "<< std::endl;
		bulat();
/*
		//NS_LOG_UNCOND("Highest Bid ID Source: "<<RREP[index][0][0]<<"\n");
		//NS_LOG_UNCOND("Highest Bid Packet ID Destination: "<<RREP[index][0][1]<<"\n");
		NS_LOG_UNCOND("Highest Bid Packet ID Sender : "<<RREP[index][0][2]<<"\n");
		//NS_LOG_UNCOND("Highest Bid Packet Sequence: "<<RREP[index][0][3]<<"\n");
		NS_LOG_UNCOND("Highest Bid Packet cost : "<<Cost[0]<<"\n");

		// Calculate payoff
		for (int i=1; i<(CountRREP[index]);i++)
		{
			Payoff[i]= Cost[i] - Cost[0];
			NS_LOG_UNCOND("Payoff "<<i<<" : "<<Payoff[i]<<"\n");
		}

		// find the minimal payoff *********************************** pay off minimal belum jalan
		int TempNextHop[2];

		if (CountRREP[index]==2)
		{
			NextHop[index] = RREP[index][1][2];
			NextHopVal[index] = RREP[index][1][5];
		}

		if (CountRREP[index]>2)
		{
			for (int i=2; i<(CountRREP[index]);i++)
			{
				if (Payoff[1] < Payoff[i])
				{
					TempNextHop[0] = RREP[index][i][2];
					TempNextHop[1] = Cost[i];

					RREP[index][i][2] = RREP[index][1][2];
					Cost[i] = Cost[1];

					RREP[index][1][2] = TempNextHop[0];
					Cost[1] = TempNextHop[1];

					NextHop[index] = RREP[index][1][2];
					NextHopVal[index] = RREP[index][1][5];

				}
				else
				{
					NextHop[index] = RREP[index][1][2];
					NextHopVal[index] = Cost[1];
				}
			}
		}

		if (RREP[index][0][2]==(Nodes-1))
		{
			NextHop[index] = RREP[index][0][2];
			NextHopVal[index] = RREP[index][0][5];
		}
		*/
		NS_LOG_UNCOND("%INFO: The 2nd price bid is : "<< NextHop[index] <<"\n");
		NS_LOG_UNCOND("%INFO: The value bid is : "<< NextHopVal[index] <<"\n");
	}
	if (CountRREP[index]==1)
	{
		Cost[0]=RREP[index][0][5]*(1-(RREP[index][0][6]/100));
		NextHop[index][0] = RREP[index][0][2];
		NextHopVal[index] = Cost[0];
		NS_LOG_UNCOND("%INFO: The 1st price bid is : "<< NextHop[index] <<"\n");
		NS_LOG_UNCOND("%INFO: The value bid is : "<< NextHopVal[index] <<"\n");
	}
	else
	{
		NS_LOG_UNCOND("%Do Nothing\n");
	}

}

void INFOPACKET(int index, std::string data) // This function to find the CH
{
	//NS_LOG_UNCOND("This is INFO packet function \n");
	//NS_LOG_UNCOND("Data : "<<data<<"\n");
	std::string ValueString = data.substr(5,2); //convert speed (string) to integer
	std::string DirString = data.substr(8,2); //convert direction (string) to integer
	int SpeedInt = stoi(ValueString); //speed
	int DirInt = stoi(DirString); //direction

	for (int i=0;i<numnodes;i++)
	{
		//if (i<(numnodes/2))
		//{
			if (index == i) // node 0
				{
					if (SpeedInt > Speed[i] && (DefStatCH[i]==0) && (DirInt > 50))
					{
						StatCH[i]=1;
						NS_LOG_UNCOND("Node CH is : "<<i<<"\n");
					}
					if ((SpeedInt < Speed[i]) && (DirInt > 50))
					{
						StatCH[i]=0;
						DefStatCH[i]=1;
						NS_LOG_UNCOND("Node "<<i<<" is CM \n");
					}
					CountInfo[i]++;
				}
		//}
		/*if (i>=(numnodes/2))
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
		}*/
	}

}

void CHIPACKET(int index, std::string data) // function to clasification chi packet for node
{
	//prepare to store
	float XdotY;
	float XNorm;
	float YNorm;
	float Cosim;

	//NS_LOG_UNCOND("This is CHI packet function \n");
	//NS_LOG_UNCOND("Field Information of CHI PACKET: "<<data<<"\n");

	//convert data to every data string
	std::string IDString = data.substr(2,2);
	std::string SpeedString = data.substr(5,2);
	std::string DirString = data.substr(8,2);

	//convert data string to integer
	int IDInt = stoi(IDString);
	int SpeedInt = stoi(SpeedString);
	int DirInt = stoi(DirString);

	//calculation of cosine similarity
	XdotY = (Speed[index] * SpeedInt) + (Direction[index] * DirInt);
	XNorm = sqrt(pow(Speed[index],2) + pow(Direction[index],2));
	YNorm = sqrt(pow(SpeedInt,2) + pow(DirInt,2));
	Cosim = XdotY / (XNorm * YNorm);

	for (int i=0;i<numnodes;i++)
	{
		if (index == i) // count the CHI-PACKET received
		{
			CountCosim[i]++;
			NS_LOG_UNCOND("Index Node : "<<index<<" have: "<<CountCosim[i]<<"\n" );
			if ((CountCosim[i]>1)&& (StatCH[i]==0))
			{
				StatGW[i]=1;
				NS_LOG_UNCOND("Index Node : "<<index<<" become GW \n" );
			}
		}

		if ((index == i) && (CosineSim[i] < Cosim))
		{
			CosineSim[index] = Cosim;
			IDCosim[index]= IDInt; // ID node will be come CH
			NS_LOG_UNCOND("ID Cosine Sim : "<<IDCosim[index]<<"\n");
			NS_LOG_UNCOND("Cosine Sim : "<<CosineSim[index]<<"\n");
		}
	}
}

void JCPACKET(int index, std::string data)
{
	NS_LOG_UNCOND("This is JC packet function \n");
	NS_LOG_UNCOND("Field Information of JC PACKET: "<<data<<"\n");

	//prepare to store data into CH table
	std::string IDSString = data.substr(2,2);
	std::string IDDString = data.substr(5,2);
	std::string SpeedString = data.substr(8,2);
	std::string DirString = data.substr(11,2);

	// convert to integer
	int IDSInt = stoi(IDSString);
	int IDDInt = stoi(IDDString);
	int SpeedInt = stoi(SpeedString);
	int DirInt = stoi(DirString);

	for (int i=0;i<numnodes;i++)
	{
		if ((index == i) && (IDDInt == i)) // node 0 received and the packet for node 0
		{
			CH[CountJC[i]][1][index] = IDSInt;
			CH[CountJC[i]][2][index]= SpeedInt;
			CH[CountJC[i]][3][index] = DirInt;
			NS_LOG_UNCOND("CH Table ===================\n");
			NS_LOG_UNCOND("NO : "<<CountJC[i]<<"\n");
			NS_LOG_UNCOND("ID CM : "<<CH[CountJC[i]][1][index]<<"\n");
			NS_LOG_UNCOND("Speed CM : "<<CH[CountJC[i]][2][index]<<"\n"); //speed
			NS_LOG_UNCOND("Direction CM : "<<CH[CountJC[i]][3][index]<<"\n"); //direction
			CountJC[i]++;
		}
	}
}

void ACPACKET(int index, std::string data)
{
	//NS_LOG_UNCOND("This is AC packet function \n");
	//NS_LOG_UNCOND("Field Information of AC PACKET: "<<data<<"\n");

	//prepare to store data into CH table
	std::string IDSString = data.substr(2,2);
	std::string IDDString = data.substr(5,2);

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
		if ((StatCH[i] == 0) && (IDCosim[i]==0))
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
		NS_LOG_UNCOND("Status GW node "<<i<<" :"<<StatGW[i] <<"\n");
	}
}

void RREQ_PACKET(int index, std::string data)
{
	//NS_LOG_UNCOND("This is RREQ packet function \n");
	//NS_LOG_UNCOND("Field Information of RREQ PACKET: "<<data<<"\n"); //Type+Sid+Did+NodeSender+Sseq+Dseq+BCid+Hop

	//prepare to store data into RREQ table
	std::string IDSString = data.substr(2,2);
	std::string IDD_1_String = data.substr(5,2);
	std::string IDD_2_String = data.substr(8,2);
	std::string IDD_3_String = data.substr(11,2);

	std::string IDSenderString = data.substr(14,2);
	std::string SeqString = data.substr(17,1); //source sequence
	std::string HopCString = data.substr(23,2); // hop count

	//convert string to int
	int IDSInt = stoi(IDSString);
	int IDD_1_Int = stoi(IDD_1_String);
	int IDD_2_Int = stoi(IDD_2_String);
	int IDD_3_Int = stoi(IDD_3_String);

	int IDSenderInt = stoi(IDSenderString);
	int Seq = stoi(SeqString);
	int HopC = stoi(HopCString);

	//Search if already save or not?
	//for (uint32_t i=0;i<=CountRREQ[index];i++)
	if (CountRREQ[index]==1) //is this first received first RREQ packet at node(index)?
	{
/*
		RREQ[index][CountRREQ[index]][0]=IDSInt;
		RREQ[index][CountRREQ[index]][1]=IDD_1_Int;
		RREQ[index][CountRREQ[index]][2]=IDD_2_Int;
		RREQ[index][CountRREQ[index]][3]=IDD_3_Int;

		RREQ[index][CountRREQ[index]][4]=IDSenderInt; // harus dirubah
		RREQ[index][CountRREQ[index]][5]=Seq;
		RREQ[index][CountRREQ[index]][6]=HopC;

		StatRecForward[index][CountRREQ[index]][0]=1; //status for received RREQ packet
		StatRecForward[index][CountRREQ[index]][1]=0; //status for forward RREQ packet

		//NS_LOG_UNCOND("This is Second RREQ packet\n");

		if ((RREQ[index][0][0]!=IDSInt) && ((RREQ[index][0][1]!=IDD_1_Int) || (RREQ[index][0][1]!=IDD_2_Int) || (RREQ[index][0][1]!=IDD_3_Int))) // if the first RREQ packet is different source ID and Des ID, if diff save it
		{
			//Save the data to RREQ Table
			RREQ[index][CountRREQ[index]][0]=IDSInt;
			RREQ[index][CountRREQ[index]][1]=IDD_1_Int;
			RREQ[index][CountRREQ[index]][2]=IDD_2_Int;
			RREQ[index][CountRREQ[index]][3]=IDD_3_Int;

			RREQ[index][CountRREQ[index]][4]=IDSenderInt; // harus dirubah
			RREQ[index][CountRREQ[index]][5]=Seq;
			RREQ[index][CountRREQ[index]][6]=HopC;

			StatRecForward[index][CountRREQ[index]][0]=1; //status for received RREQ packet
			StatRecForward[index][CountRREQ[index]][1]=0; //status for forward RREQ packet

			// Debug
			NS_LOG_UNCOND("This is second RREQ packet\n");
			NS_LOG_UNCOND("Information from RREQ Packet ID Source: "<<RREQ[index][CountRREQ[index]][0]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet ID Destination 1: "<<RREQ[index][CountRREQ[index]][1]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet ID Destination 2: "<<RREQ[index][CountRREQ[index]][2]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet ID Destination 3: "<<RREQ[index][CountRREQ[index]][3]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet ID Sender: "<<RREQ[index][CountRREQ[index]][4]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet Sequence: "<<RREQ[index][CountRREQ[index]][5]<<"\n");
			NS_LOG_UNCOND("Information from RREQ Packet Hop Count: "<<RREQ[index][CountRREQ[index]][6]<<"\n");


			//if (RREQ[numNodes][CountRREQ[index]][1]==(numNodes-1))
			//{
			//	StatRREQinD=1;
			//	NS_LOG_UNCOND("Destination received RREQ packet from "<<RREQ[index][CountRREQ[index]][0]<<"\n");
			//}
			//CountRREQ[index]++;
		}
		*/
	}

	else // this is for first RREQ packet
	{
		//Save the data to RREQ Table
		RREQ[index][CountRREQ[index]][0]=IDSInt;
		RREQ[index][CountRREQ[index]][1]=IDD_1_Int;
		RREQ[index][CountRREQ[index]][2]=IDD_2_Int;
		RREQ[index][CountRREQ[index]][3]=IDD_3_Int;

		if(index==10)
		{
			RREQ[index][CountRREQ[index]][4]=0;
		}
		else if(index == 16)
		{
			RREQ[index][CountRREQ[index]][4]=38;
		}
		else if(index == 38)
		{
			RREQ[index][CountRREQ[index]][4]=0;
		}
		else if(index == 19)
		{
			RREQ[index][CountRREQ[index]][4]=6;
		}
		else if(index == 6)
		{
			RREQ[index][CountRREQ[index]][4]=0;
		}
		else
		{
			RREQ[index][CountRREQ[index]][4]=IDSenderInt; // harus dirubah
		}
		RREQ[index][CountRREQ[index]][5]=Seq;
		RREQ[index][CountRREQ[index]][6]=HopC;

		StatRecForward[index][CountRREQ[index]][0]=1; //status for received RREQ packet
		StatRecForward[index][CountRREQ[index]][1]=0; //status for forward RREQ packet

		// Debug
		//NS_LOG_UNCOND("This is First RREQ packet\n");
		//NS_LOG_UNCOND("Information from RREQ Packet ID Destination 1: "<<RREQ[index][CountRREQ[index]][1]<<"\n");
		//NS_LOG_UNCOND("Information from RREQ Packet ID Destination 2: "<<RREQ[index][CountRREQ[index]][2]<<"\n");
		//NS_LOG_UNCOND("Information from RREQ Packet ID Destination 3: "<<RREQ[index][CountRREQ[index]][3]<<"\n");
		//NS_LOG_UNCOND("Information from RREQ Packet ID Sender: "<<RREQ[index][CountRREQ[index]][4]<<"\n");
		//NS_LOG_UNCOND("Information from RREQ Packet Sequence: "<<RREQ[index][CountRREQ[index]][5]<<"\n");
		//NS_LOG_UNCOND("Information from RREQ Packet Hop Count: "<<RREQ[index][CountRREQ[index]][6]<<"\n");
		// if (RREQ[2][CountRREQ[index]][1]==(numNodes-1))
		//	{
		//		StatRREQinD=1;
		//		NS_LOG_UNCOND("Destination received RREQ packet from "<<RREQ[index][CountRREQ[index]][0]<<"\n");
		//	}
		//CountRREQ[index]++; 	//increase the number of RREQ
	}

	if (index == 97)
	{
		if ((RREQ[index][CountRREQ[index]][1]==97) && (CountRREQ[index]<2))
		{
			StatRREQinD[0]=1;
			NS_LOG_UNCOND("count rreq: "<<CountRREQ[index]<<"\n");
			NS_LOG_UNCOND("index: "<<index<<"\n");
			NS_LOG_UNCOND("data: "<<data<< "\n");
			RREQ[index][CountRREQ[index]][4]=19;
			NS_LOG_UNCOND("Destination 1 received RREQ packet from "<<RREQ[index][CountRREQ[index]][4]<<"\n");
			CountRREQ[index]++;
		}
	}

	else if (index == 98)
	{
		if ((RREQ[index][CountRREQ[index]][2]==98) && (CountRREQ[index]<2))
		{
			StatRREQinD[1]=1;
			NS_LOG_UNCOND("count rreq: "<<CountRREQ[index]<<"\n");
			NS_LOG_UNCOND("index: "<<index<<"\n");
			NS_LOG_UNCOND("data: "<<data<< "\n");
			RREQ[index][CountRREQ[index]][4]=16;
			NS_LOG_UNCOND("Destination 2 received RREQ packet from "<<RREQ[index][CountRREQ[index]][4]<<"\n");
			CountRREQ[index]++;
		}
	}
	else if (index == 99)
	{
		if ((RREQ[index][CountRREQ[index]][3]==99) && (CountRREQ[index]<2))
		{
			StatRREQinD[2]=1;
			NS_LOG_UNCOND("count rreq: "<<CountRREQ[index]<<"\n");
			NS_LOG_UNCOND("index: "<<index<<"\n");
			NS_LOG_UNCOND("data: "<<data<< "\n");
			RREQ[index][CountRREQ[index]][4]=10;
			NS_LOG_UNCOND("Destination 3 received RREQ packet from "<<RREQ[index][CountRREQ[index]][4]<<"\n");
			CountRREQ[index]++;
		}
	}

	//CountRREQ[index]++;

}

void RREP_PACKET(int index, std::string data)
{
	//NS_LOG_UNCOND("This is RREP packet function \n");
	//NS_LOG_UNCOND("Field Information of RREP PACKET: "<<data<<"\n"); //Type+Sid+Did+NodeSender+Sseq+Dseq+BCid+Hop


	//prepare to store data into RREP table
	std::string IDSString = data.substr(2,2); // SOurce ID
	std::string IDDString = data.substr(5,2); // Dest ID
	std::string IDSenderString = data.substr(8,2);	// Sender ID
	std::string IDDesString = data.substr(11,2);	// Next Node ID/ yang menerima
	std::string SeqString = data.substr(14,1); //source sequence
	std::string DeqString = data.substr(16,1); //Destination sequence
	std::string HopCString = data.substr(20,2); // hop count
	std::string RemEnergy = data.substr(23,2); // remaining energy
	//std::string PosString = data.substr(26,2); // position

	//convert string to int
	int IDSInt = stoi(IDSString);
	int IDDInt = stoi(IDDString);
	int IDSenderInt = stoi(IDSenderString);
	int IDDesInt = stoi(IDDesString);
	int Seq = stoi(SeqString);
	int Deq = stoi(DeqString);
	int HopC = stoi(HopCString);
	int RemEnergyInt = stoi(RemEnergy);
	//int PosInt = stoi(PosString);


	//if((index == 0)&&(EP==0))
	//{
	//	CountRREP[0]=0;
	//	EP=1;
	//}
	//NS_LOG_UNCOND("index: "<<index<<"\n");
	if (index == IDDesInt)
	{
		NS_LOG_UNCOND("tujuan dan penerima sama \n");
		NS_LOG_UNCOND("index: "<<index<<"\n");
		NS_LOG_UNCOND("data: "<<data<< "\n");
		//Search if already save or not?
		//for (uint32_t i=0;i<=CountRREQ[index];i++)

		if (CountRREP[index]>0) //is this first received first RREP packet at node(index)?
		{

			if (RREQ[index][CountRREQ[index]-1][2]!=IDSenderInt)// if RREQ sender and RREP sender different?
			{
				if (RREP[index][(CountRREP[index]-1)][2]!=IDSenderInt)// ((RREP[index][0][0]!=IDSInt) && (RREP[index][0][1]!=IDDInt)) // record all RREP packet from different sender
				{
					//Save the data to RREP Table
					RREP[index][CountRREP[index]][0]=IDSInt;
					RREP[index][CountRREP[index]][1]=IDDInt;
					RREP[index][CountRREP[index]][2]=IDSenderInt;
					RREP[index][CountRREP[index]][3]=IDDesInt;
					RREP[index][CountRREP[index]][4]=Seq;
					RREP[index][CountRREP[index]][5]=Deq;
					RREP[index][CountRREP[index]][6]=HopC;
					RREP[index][CountRREP[index]][7]=RemEnergyInt;

					StatRecForRREP[index][CountRREP[index]][0]=1; //status for received RREP packet
					StatRecForRREP[index][CountRREP[index]][1]=0; //status for forward RREP packet

					// Debug

					//NS_LOG_UNCOND("Count RREP Source : "<<CountRREP[0]<<"\n");
					NS_LOG_UNCOND("Count RREP ID "<< index<<" : "<<CountRREP[index]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet ID Source: "<<RREP[index][CountRREP[index]][0]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet ID Destination: "<<RREP[index][CountRREP[index]][1]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet ID Sender: "<<RREP[index][CountRREP[index]][2]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet to Node: "<<RREP[index][CountRREP[index]][3]<<"\n");
					NS_LOG_UNCOND("Information old RREP Packet ID Sender: "<<RREQ[index][CountRREQ[index]-1][2]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet Source Sequence: "<<RREP[index][CountRREP[index]][4]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet Destination Sequence: "<<RREP[index][CountRREP[index]][5]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet Hop Count: "<<RREP[index][CountRREP[index]][6]<<"\n");
					NS_LOG_UNCOND("Information from RREP Packet Remaining Energy: "<<RREP[index][CountRREP[index]][7]<<"\n");

					CountRREP[index]++;

					//if (RREQ[numNodes][CountRREQ[index]][1]==(numNodes-1))
					//{
					//	StatRREQinD=1;
					//	NS_LOG_UNCOND("Destination received RREQ packet from "<<RREQ[index][CountRREQ[index]][0]<<"\n");
					//}
					//CountRREQ[index]++;
				}
			}
		}
		if (CountRREP[index]<1) // this is for first RREP packet
		{
			// if the sender RREP is sender RREQ must be drop
			if (RREQ[index][CountRREQ[index]-1][2]!=IDSenderInt)
			{
				//Save the data to RREP Table
				RREP[index][CountRREP[index]][0]=IDSInt;
				RREP[index][CountRREP[index]][1]=IDDInt;
				RREP[index][CountRREP[index]][2]=IDSenderInt;
				RREP[index][CountRREP[index]][3]=IDDesInt;
				RREP[index][CountRREP[index]][4]=Seq;
				RREP[index][CountRREP[index]][5]=Deq;
				RREP[index][CountRREP[index]][6]=HopC;
				RREP[index][CountRREP[index]][7]=RemEnergyInt;

				StatRecForRREP[index][CountRREP[index]][0]=1; //status for received RREP packet
				StatRecForRREP[index][CountRREP[index]][1]=0; //status for forward RREP packet

				/// Debug


				//NS_LOG_UNCOND("Count RREP Source : "<<CountRREP[0]<<"\n");
				NS_LOG_UNCOND("Count RREP ID "<< index<<" : "<<CountRREP[index]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet ID Source: "<<RREP[index][CountRREP[index]][0]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet ID Destination: "<<RREP[index][CountRREP[index]][1]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet ID Sender: "<<RREP[index][CountRREP[index]][2]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet to Node: "<<RREP[index][CountRREP[index]-1][3]<<"\n");
				NS_LOG_UNCOND("Information old RREP Packet ID Sender: "<<RREQ[index][CountRREQ[index]][2]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet Source Sequence: "<<RREP[index][CountRREP[index]][4]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet Destination Sequence: "<<RREP[index][CountRREP[index]][5]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet Hop Count: "<<RREP[index][CountRREP[index]][6]<<"\n");
				NS_LOG_UNCOND("Information from RREP Packet Remaining Energy: "<<RREP[index][CountRREP[index]][7]<<"\n");

				CountRREP[index]++;
			}
			// if (RREQ[2][CountRREQ[index]][1]==(numNodes-1))
			//	{
			//		StatRREQinD=1;
			//		NS_LOG_UNCOND("Destination received RREQ packet from "<<RREQ[index][CountRREQ[index]][0]<<"\n");
		}

			//CountRREQ[index]++; 	//increase the number of RREQ
		//}

	}

	if (index==0) // if the index is source
	{
		//if (StatRecForRREP[index][CountRREP[index]-1][0]==1) // if source received the RREP packet
		//	{
				NS_LOG_UNCOND("Source received RREP packet \n");
				if (RREP[index][CountRREP[index]-1][1]==0)
				{
					StatRREPinD=1;
					NS_LOG_UNCOND("Source received RREP packet from "<<RREP[index][CountRREP[index]-1][2]<<"\n");
					//NS_LOG_UNCOND("Count RREP ID Source: "<<(CountRREP[index]-1)<<"\n");


					//for (uint32_t k = 0; k < (Nodes-1); k++)
					//{
					//	SecondPrice(k);
					//	NS_LOG_UNCOND("==============================\n");
					//	NS_LOG_UNCOND("Hybrid Price auction for: "<<k<<"\n");
					//}

				}
		//	}
	}


}

void DATATR(int index, std::string data)
{
	//NS_LOG_UNCOND("This is Transmission Data Function \n");
	//NS_LOG_UNCOND("Field Information of Transmission Data: "<<data<<"\n");
	//NS_LOG_UNCOND("%index: "<<index<<"\n");
	//prepare to Processing data into Trans Data
	std::string IDSString = data.substr(2,2);
	std::string IDDString1 = data.substr(5,2);
	std::string IDDString2 = data.substr(8,2);
	std::string IDDString3 = data.substr(11,2);
	std::string This1 = data.substr(14,2);
	std::string This2 = data.substr(17,2);
	std::string This3 = data.substr(20,2);
	std::string Data = data.substr(23,4);
	//std::string IDThisNode = data.substr(14,2);


	//convert string to int
	int IDSInt = stoi(IDSString);
	int IDDInt1 = stoi(IDDString1);
	int IDDInt2 = stoi(IDDString2);
	int IDDInt3 = stoi(IDDString3);
	int ThisInt1 = stoi(This1);
	int ThisInt2 = stoi(This2);
	int ThisInt3 = stoi(This3);
	//int IDThisNodeInt = stoi(IDThisNode);
	int IDThisNodeInt;

	if (ThisInt1 == index)
	{
		IDThisNodeInt = ThisInt1;
	}
	else if (ThisInt2 == index)
	{
		IDThisNodeInt = ThisInt2;
	}
	else if (ThisInt3 == index)
	{
		IDThisNodeInt = ThisInt3;
	}



	if ((index == IDThisNodeInt) && (StatDataRec[index][1]==0))
	{
		if (RREP[index][0][0] == IDDInt1)
		{
			//NS_LOG_UNCOND("IDDInt1: "<<IDDInt1<<"\n");
			TrData[index][1] = IDDInt1;
		}
		else if (RREP[index][0][0] == IDDInt2)
		{
			//NS_LOG_UNCOND("IDDInt2: "<<IDDInt2<<"\n");
			TrData[index][1] = IDDInt2;
		}
		else if (RREP[index][0][0] == IDDInt3)
		{
			//NS_LOG_UNCOND("IDDInt3: "<<IDDInt3<<"\n");
			TrData[index][1] = IDDInt3;
		}

		// set status received Trans Data
		StatDataRec[index][0]=1;
		StatDataRec[index][1]=0;

		// Store data identity
		TrData[index][0] = IDSInt;
		//TrData[index][1] = IDDInt1;
		//TrData[index][2] = IDDInt2;
		//TrData[index][3] = IDDInt3;

		// Store Data
		DataTrans[index] = Data;

		//NS_LOG_UNCOND("index: "<<index<<"\n");
		//NS_LOG_UNCOND("This is node: "<<IDThisNodeInt<<"\n");
		//NS_LOG_UNCOND("Status Rec 0 : "<<StatDataRec[index][0]<<"\n");
		//NS_LOG_UNCOND("Status Rec 1 : "<<StatDataRec[index][1]<<"\n");
		//NS_LOG_UNCOND("Information from Trans Data ID Source: "<<TrData[index][0]<<"\n");
		//NS_LOG_UNCOND("Information from Trans Data ID D: "<<TrData[index][1]<<"\n");
		//NS_LOG_UNCOND("Information from Trans Data ID D2: "<<TrData[index][2]<<"\n");
		//NS_LOG_UNCOND("Information from Trans Data ID D3: "<<TrData[index][3]<<"\n");
		//NS_LOG_UNCOND("Information from Trans Data : "<<DataTrans[index]<<"\n");
		//NS_LOG_UNCOND("Sum of Data Received: "<<DataReceived<<"\n");
		//NS_LOG_UNCOND("Sum of worm Hole Received: "<<SInkReceived<<"\n");
		//NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
	}
	else if ((index != IDThisNodeInt) && (StatDataRec[index][1]!=0))
	{
		//NS_LOG_UNCOND("Im not the next node \n");
		StatDataRec[index][0]=0;
		StatDataRec[index][1]=0;
		//NS_LOG_UNCOND("Sum of Data Received: "<<DataReceived<<"\n");
		//NS_LOG_UNCOND("Sum of worm Hole Received: "<<SInkReceived<<"\n");
		//NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
	}
	if ((index==97)||(index==98)||(index==99))
	{
		if ((index == (97)) && ((97)== IDThisNodeInt))
		{
			NS_LOG_UNCOND("D1 Receive Data Trans \n");
			DataReceived[0]++;
			NS_LOG_UNCOND("Sum of Data Received D1: "<<DataReceived[0]<<"\n");
			NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
			NS_LOG_UNCOND("Control overhead: "<<ControlROuting/100<<"\n");
			NS_LOG_UNCOND("Sum of Data Received D1 + D2 + D3: "<<DataReceived[0]+DataReceived[1]+DataReceived[2]<<"\n");

		}
		if ((index == (98)) && ((98)== IDThisNodeInt))
		{
			NS_LOG_UNCOND("D2 Receive Data Trans \n");
			DataReceived[1]++;
			NS_LOG_UNCOND("Sum of Data Received D2: "<<DataReceived[1]<<"\n");
			NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
			NS_LOG_UNCOND("Control overhead: "<<ControlROuting/100<<"\n");
			NS_LOG_UNCOND("Sum of Data Received D1 + D2 + D3: "<<DataReceived[0]+DataReceived[1]+DataReceived[2]<<"\n");
		}
		if ((index == (99)) && ((99)== IDThisNodeInt))
		{
			NS_LOG_UNCOND("D3 Receive Data Trans \n");
			DataReceived[2]++;
			NS_LOG_UNCOND("Sum of Data Received D3: "<<DataReceived[2]<<"\n");
			NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
			NS_LOG_UNCOND("Control overhead: "<<ControlROuting/100<<"\n");
			NS_LOG_UNCOND("Sum of Data Received D1 + D2 + D3: "<<DataReceived[0]+DataReceived[1]+DataReceived[2]<<"\n");
		}
	}
	/*
	if ((index == Nodes)&&(Nodes == IDThisNodeInt))
	{
		NS_LOG_UNCOND("Sink hole Receive Data Trans \n");
		SInkReceived=(transData-20);
		NS_LOG_UNCOND("Sum of Sink Hole Received: "<<SInkReceived<<"\n");
		NS_LOG_UNCOND("Routing delay: "<<delayROuting<<"\n");
	}
	*/


	//float controlOver =ControlROuting/numNodes;
	//NS_LOG_UNCOND("Sum Control Overhead Routing per Session : "<<ControlROuting <<"\n");
	//NS_LOG_UNCOND("Control Overhead ROuting per Session : "<<controlOver <<"\n");
}

void ALLPacket(int index, std::string data) // this function to clasification packet
{
	//NS_LOG_UNCOND("this is All packet function \n");
	//NS_LOG_UNCOND("index: "<<index<<"\n");
	//NS_LOG_UNCOND("data: "<<data<< "\n");
	//convert packet type
	std::string TypeString = data.substr(0,1);
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
	if (TypeString == "4") // this is RREQ PACKET
		{
			if(index!=0)
			{
				if(index!=0)
				{
					RREQ_PACKET(index, data);
				}
			}
			else
			{
				//NS_LOG_UNCOND("Node S drop the packet \n");
				//StatRREQinS=1;
			}
		}
	if (TypeString == "5") // this is RREP PACKET
	{
		if((index!=(97))&&(index!=(98))&&(index!=(99))) // this is not destination
		{
			RREP_PACKET(index, data);
		}
		else
		{
			NS_LOG_UNCOND("Node D received and drop the packet \n");
			//StatRREPinD=1;
		}
	}
	if (TypeString == "6") // this is Transmission Data
	{
		if(index!=0)
		{
			DATATR(index, data);
		}
		else
		{
			NS_LOG_UNCOND("Node S received and drop the packet \n");
			//StatRREQinS=1;
		}
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
	  if (TypePkt==0) // INFO-PACKET - Type_packet, Speed
	  {
		  //for (uint32_t i=0; i<=numNodes; i++)
		  //{
			  //if(wifinetdevice->GetNode()->GetId() == i)
			  //{
				  NS_LOG_UNCOND("%INFO: Sending INFO PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress());
				  if (wifinetdevice->GetNode()->GetId()<10)
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  std::string pktinfo = "0,0" +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+","+std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktinfo.c_str()), pktinfo.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx0*1000);
				  }
				  if (wifinetdevice->GetNode()->GetId()>=10)
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  std::string pktinfo = "0," +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+","+std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktinfo.c_str()), pktinfo.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx0*1000);
				  }
			  //}
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 11, pktCount - 1, pktInterval);
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
					  std::string pktchi = "1,0" +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktchi.c_str()), pktchi.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx1*1000);
				  }
				  if (wifinetdevice->GetNode()->GetId()>=10)
				  {
					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
					  std::string pktchi = "1," +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktchi.c_str()), pktchi.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
					  CountControlOver++; //counter control overhead in info packet
					  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx1*1000);
				  }
			  }
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 10, pktCount - 1, pktInterval);
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
						  std::string pktjc = "2,0" +std::to_string(wifinetdevice->GetNode()->GetId())+ ",0" +std::to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]>=10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  std::string pktjc = "2,0" +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
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
						  std::string pktjc = "2," +std::to_string(wifinetdevice->GetNode()->GetId())+ ",0" +std::to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
					  if (IDCosim[wifinetdevice->GetNode()->GetId()]>=10)
					  {
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  std::string pktjc = "2," +std::to_string(wifinetdevice->GetNode()->GetId())+ "," +std::to_string(IDCosim[wifinetdevice->GetNode()->GetId()]) +"," +std::to_string(Speed[wifinetdevice->GetNode()->GetId()])+ "," +std::to_string(Direction[wifinetdevice->GetNode()->GetId()]);
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktjc.c_str()), pktjc.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  CountControlOver++; //counter control overhead in info packet
						  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx2*1000);
					  }
				  }
			  }
		  //}
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 14, pktCount - 1, pktInterval);
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
							  std::string pktac = "3,0" +std::to_string(i)+ ",0" + std::to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 8, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
						  if (CH[j-1][1][i]>10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string pktac = "3,0" +std::to_string(i)+ "," + std::to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 8, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }

					  }
					  if (i>10)
					  {
						  if (CH[j-1][1][i]<10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string pktac = "3," +std::to_string(i)+ ",0" + std::to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 8, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
						  if (CH[j-1][1][i]>10)
						  {
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string pktac = "3," +std::to_string(i)+ "," + std::to_string(CH[j-1][1][i]);
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (pktac.c_str()), pktac.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 8, pktCount, pktInterval);
							  CountControlOver++; //counter control overhead in info packet
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx3*1000);
						  }
					  }
				  }
				  pktCount--;
			  }
		  }
	  }
	  if (TypePkt==4) // RREQ-PACKET - Type_packet, ID, Speed, Dire
	  {
		  //if(StatRREQinD==0)
		  //{
			  //for (uint32_t i=0; i<=numNodes; i++)
			  //{
				  if ((wifinetdevice->GetNode()->GetId()==0))// && (JValue==0))
				  {
					  if (StatRREQinS==0)
					  {
						  ControlROuting=0;
						  //ControlROuting++;
						  if (Nodes<=10) //must be add 0
						  {
							  //NS_LOG_UNCOND("%INFO: Sending RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREQ = "4,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,00"; //Type+Sid+Did+NodeSender+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQ.c_str()), RREQ.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  ControlROuting++;
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
						  }
						  if (Nodes>10)
						  {
							  //NS_LOG_UNCOND("%INFO: Sending RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREQ = "4,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,00"; //Type+Sid+Did+NodeSender+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQ.c_str()), RREQ.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  ControlROuting++;
							  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
						  }

					  }
				  }
				  if (wifinetdevice->GetNode()->GetId()>0) // forward
				  {
					  //for (uint32_t i=0; i<=CountRREQ[wifinetdevice->GetNode()->GetId()]; i++)
					  //{
						  if (StatRecForward[wifinetdevice->GetNode()->GetId()][0][0]==1)
						  {
							  if(StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]==0)
							  {
								  //ControlROuting++;
								  if (Nodes<=10) // destination < 10 or just one digit
								  {
									  if (wifinetdevice->GetNode()->GetId()<10) // sender < 10 or just one digit
									  {
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) < 10) // hop count < 10 or just one digit
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,0"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) >= 10) // hop count >= 10 or two digits
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
									  }
									  if (wifinetdevice->GetNode()->GetId()>=10) // sender >= 10 or two digit
									  {
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) < 10) // hop count < 10 or just one digit
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0,1,0"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) >= 10) // hop count >= 10 or two digits
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0,1,"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
									  }
								  }
								  if (Nodes>10) // destination >= 10 or just one digit
								  {
									  if (wifinetdevice->GetNode()->GetId()<10) // sender < 10 or just one digit
									  {
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) < 10) // hop count < 10 or just one digit
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,0"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) >= 10) // hop count >= 10 or two digits
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
									  }
									  if (wifinetdevice->GetNode()->GetId()>=10) // sender >= 10 or two digit
									  {
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) < 10) // hop count < 10 or just one digit
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,0"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
										  if ((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4]) >= 10) // hop count >= 10 or two digits
											{
												//NS_LOG_UNCOND("%INFO: Forward RREQ PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												std::string RREQSent = "4,00,97-98-99,"+std::to_string(wifinetdevice->GetNode()->GetId())+",1,0,1,"+std::to_string((RREQ[wifinetdevice->GetNode()->GetId()][CountRREQ[wifinetdevice->GetNode()->GetId()]-1][4])+1);//std::to_string(RREQ[1][2][i]); //Type+Sid+Did+Sseq+Dseq+NodeSender+BCid+Hop
												wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREQSent.c_str()), RREQSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												StatRecForward[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREQ already forwarded
												ControlROuting++;
												REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx4*1000);
											}
									  }
								  }
							  }
						  }
					  //}
				  }
				  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 27, pktCount, pktInterval);
				  //StatRREQinS++;
			  //}
		 //}
	  }
	  if (TypePkt==5) // RREP-PACKET - Type_packet, ID, Speed, Dire
	  {
		  if ((StatRREQinD[0]==1) && (StatRREQinD[1]==1) && (StatRREQinD[2]==1))
		  {
			  //for (uint32_t i=0; i<=numNodes; i++)
			  //{
				  if ((wifinetdevice->GetNode()->GetId()==(97)) || (wifinetdevice->GetNode()->GetId()==(98)) || (wifinetdevice->GetNode()->GetId()==(99)))// this id Destination
				  {
					  if (StatRREPinD==0) // destination send RREP packet
					  {
						  //ControlROuting++;
						  /*
						  if (Nodes<=10) // destination and sender less than 10
						  {
							  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]<10) // next node less than 10
							  {
								  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
								  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
								  std::string RREP = "5,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
								  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//								  ControlROuting++;
								  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;
								  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
								  //StatRREQinS=1;
							  }
							  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]>=10) // next node more than 10
							  {
								  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
								  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
								  std::string RREP = "5,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
								  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//								  ControlROuting++;

								  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
								  //StatRREQinS=1;
							  }
						  }
							*/
						  if (Nodes>10) // destination and sender more than 10
						  {
							  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]<10) // next node less than 10
							  {
								  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " send to: " << RREQ[wifinetdevice->GetNode()->GetId()][0+CountRREPSend[wifinetdevice->GetNode()->GetId()]][4]<<"\n");
								  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
								  std::string RREP = "5,"+std::to_string(wifinetdevice->GetNode()->GetId())+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0+CountRREPSend[wifinetdevice->GetNode()->GetId()]][4])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
								  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
								  ControlROuting++;
								  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
								  //StatRREQinS=1;
							  }
							  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]>=10) // next node more than 10
							  {
								  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " send to: " << RREQ[wifinetdevice->GetNode()->GetId()][0+CountRREPSend[wifinetdevice->GetNode()->GetId()]][4]<<"\n");
								  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
								  std::string RREP = "5,"+std::to_string(wifinetdevice->GetNode()->GetId())+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0+CountRREPSend[wifinetdevice->GetNode()->GetId()]][4])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
								  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
								  ControlROuting++;
								  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
								  //StatRREQinS=1;
							  }
						  }
						  CountRREPSend[wifinetdevice->GetNode()->GetId()]=1;
					  }
				  }
				  //================================================================================================ batas D generate RREP packet
				  // need check forward packet

				  if (wifinetdevice->GetNode()->GetId()<97)// this is inter node
				  {
					  //for (uint32_t i=0; i<=CountRREQ[wifinetdevice->GetNode()->GetId()]; i++)
					  //{
						  if (StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][0]==1) // if this node received the RREP packet
						  {
							  if(StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]==0) // not forward yet
							  {
								  //ControlROuting++;
								  /*
								  if (Nodes<=10) // destination less than 10
								  {
									  if (wifinetdevice->GetNode()->GetId()<10) // sender less than 10
									  {
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]<10) // next node less than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]>=10) // next node more than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] < 9) // hop less than 9
											  	{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
											  	}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
									  }
									  if (wifinetdevice->GetNode()->GetId()>=10) // sender greater than equal 10
									  {
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]<10) // next node less than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]>=10) // next node more than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,0"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
//												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
									  }
								  }
								  */
								  if (Nodes>10) // total node more than 10
								  {
									  if (wifinetdevice->GetNode()->GetId()<10) // sender less than 10
									  {
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]<10) // next node less than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop+energy
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] >= 9) // hop more than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop+energy
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]>=10) // next node more than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
									  }
									  if (wifinetdevice->GetNode()->GetId()>=10) // sender greater than equal 10
									  {
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]<10) // next node less than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
										  if (RREQ[wifinetdevice->GetNode()->GetId()][0][4]>=10) // next node more than 10
										  {
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] < 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,0"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
											  if (RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6] >= 9) // hop less than 9
												{
												  NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
												  //NS_LOG_UNCOND("%next node :" << RREQ[wifinetdevice->GetNode()->GetId()][0][4]<<"\n");
												  //NS_LOG_UNCOND("%source node :" << RREP[wifinetdevice->GetNode()->GetId()][0][0]<<"\n");
												  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
												  std::string RREPSent = "5,"+std::to_string(RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][0])+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][4])+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1)+",1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][6])+1)+","+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
												  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
												  StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded
												  ControlROuting++;
												  REnergy[wifinetdevice->GetNode()->GetId()] = REnergy[wifinetdevice->GetNode()->GetId()] - (ETx5*1000);
												}
										  }
									  }
								  }

								  //NS_LOG_UNCOND("%INFO: Forward RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
								  //static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
								  //std::string RREPSent = "5,"+std::to_string(Nodes-1)+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][2])+",1,1,1,"+std::to_string((RREP[wifinetdevice->GetNode()->GetId()][CountRREP[wifinetdevice->GetNode()->GetId()]-1][5])+1);//Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
								  //wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREPSent.c_str()), RREPSent.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
								  //StatRecForRREP[wifinetdevice->GetNode()->GetId()][0][1]=1;	// mark this RREP already forwarded

							  }
						  }
					  //}
				  }


				/*
				if (wifinetdevice->GetNode()->GetId()>(Nodes-1)) // this is sink hole node
				  {
					ControlROuting++;
					if (wifinetdevice->GetNode()->GetId()<=10) // destination and sender less than 10
					  {
						  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]<10) // next node less than 10
						  {
							  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREP = "5,0"+std::to_string(Nodes-1)+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][2])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//							  ControlROuting++;
							  //StatRREQinS=1;
						  }
						  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]>=10) // next node more than 10
						  {
							  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREP = "5,0"+std::to_string(Nodes-1)+",00,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][2])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//							  ControlROuting++;
							  //StatRREQinS=1;
						  }
					  }
					  if (wifinetdevice->GetNode()->GetId()>10) // destination and sender more than 10
					  {
						  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]<10) // next node less than 10
						  {
							  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREP = "5,"+std::to_string(Nodes-1)+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+",0"+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][2])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//							  ControlROuting++;
							  //StatRREQinS=1;
						  }
						  if (RREQ[wifinetdevice->GetNode()->GetId()][0][2]>=10) // next node more than 10
						  {
							  NS_LOG_UNCOND("%INFO: Sending RREP PACKET! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string RREP = "5,"+std::to_string(Nodes-1)+",00,"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(RREQ[wifinetdevice->GetNode()->GetId()][0][2])+",1,1,1,00,"+std::to_string(REnergy[wifinetdevice->GetNode()->GetId()]); //Type+Sid+Did+NodeSender+NextNode+Sseq+Dseq+BCid+Hop
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (RREP.c_str()), RREP.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//							  ControlROuting++;
							  //StatRREQinS=1;
						  }
					  }
				  }*/

				  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 25, pktCount, pktInterval);
			  //}
		 }
	  }
	  if (TypePkt==6) // Transmission data
	  {
		  //NS_LOG_UNCOND("%If==6 node " << wifinetdevice->GetNode()->GetId());
		  if (StatRREPinD==1)
		  {
			  //NS_LOG_UNCOND("%If==1 node " << wifinetdevice->GetNode()->GetId());
			  if ((wifinetdevice->GetNode()->GetId()==0))// Source
			  {
				  DNN(wifinetdevice->GetNode()->GetId()); // must change to DNN

				  /*
				  if(Nodes<=10) // if nodes less than 10
				  {
					  if(NextHop[wifinetdevice->GetNode()->GetId()] < 10) // if next hope less than 10
					  {
						  NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  std::string DATA = "6,00,97-98-99,0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if(NextHop[wifinetdevice->GetNode()->GetId()] >= 10) // if next hop more than 10
					  {
						  NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
				  }
				  */
				  if(Nodes>10) // if node more than 10
				  {
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] < 10) ) // if next hop less than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  NS_LOG_UNCOND("%Next node 0-0-0\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+",0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] >= 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  NS_LOG_UNCOND("%Next node 0-0-1\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+",0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] < 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  NS_LOG_UNCOND("%Next node 0-1-0\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+",0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] >= 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  NS_LOG_UNCOND("%Next node 0-1-1\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+",0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] < 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  NS_LOG_UNCOND("%Next node 1-0-0\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] < 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] >= 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  //NS_LOG_UNCOND("%Next node 1-0-1\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] < 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  //NS_LOG_UNCOND("%Next node 1-1-0\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
					  if( (NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][1] >= 10) && (NextHop[wifinetdevice->GetNode()->GetId()][2] >= 10) ) // if next hop more than 10
					  {
						  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
						  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
						  //NS_LOG_UNCOND("%Next node 1-1-1\n");
						  //NS_LOG_UNCOND("%Next node 1 "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
						  //NS_LOG_UNCOND("%Next node 2 "<<NextHop[wifinetdevice->GetNode()->GetId()][1]<<"\n");
						  //NS_LOG_UNCOND("%Next node 3 "<<NextHop[wifinetdevice->GetNode()->GetId()][2]<<"\n");
						  std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
						  //std::string DATA = "6,00,97-98-99,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
						  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
						  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
					  }
				  }

				  //SecondPrice(wifinetdevice->GetNode()->GetId()); // run the 2nd price auction
				  //if (StatRREQinS==0)
				 //{
				  //NS_LOG_UNCOND("%INFO: 2nd Price Auction! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
				  //NS_LOG_UNCOND("%INFO: The Highest bid is : "<< RREP[wifinetdevice->GetNode()->GetId()][0][2] <<"\n");
				  //NS_LOG_UNCOND("%INFO: The value bid is : "<< RREP[wifinetdevice->GetNode()->GetId()][0][4] <<"\n");
				  //NS_LOG_UNCOND("%INFO: The 2nd price bid is : "<< NextHop[wifinetdevice->GetNode()->GetId()] <<"\n");
				  //NS_LOG_UNCOND("%INFO: The value bid is : "<< NextHopVal[wifinetdevice->GetNode()->GetId()] <<"\n");

				  //NS_LOG_UNCOND("%INFO: Sending Data Trans ! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
				  //static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
				  //std::string DATA = "6,0"+std::to_string(wifinetdevice->GetNode()->GetId())+","+std::to_string(Nodes-1)+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+",Data"; //Type+Sid+Did+NextNode+Data
				  //wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
				  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
				  //}
			  }
			  // inter-node
			  if ((wifinetdevice->GetNode()->GetId() > 0) || (wifinetdevice->GetNode()->GetId() < 97))// Inter Node
			  {
				  //NS_LOG_UNCOND("%If==inter node " << wifinetdevice->GetNode()->GetId());
				  //NS_LOG_UNCOND("%If==rec =  " << StatDataRec[wifinetdevice->GetNode()->GetId()][0]);
				  //NS_LOG_UNCOND("%If==forw =  " << StatDataRec[wifinetdevice->GetNode()->GetId()][1]);

				  if ((StatDataRec[wifinetdevice->GetNode()->GetId()][0]==1) && (StatDataRec[wifinetdevice->GetNode()->GetId()][1]==0)) // if node received and not forward yet
				  {
					  //NS_LOG_UNCOND("%If==rec or not " << wifinetdevice->GetNode()->GetId());

					  DNN(wifinetdevice->GetNode()->GetId());
					  if(Nodes<=10) // nodes less than 10
					  {
						  if(NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) // if next hop less than 10
						  {
							  //NS_LOG_UNCOND("%INFO: Forward Data Trans! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  //std::string DATA = "6,00,"+std::to_string(RREP[0][0][0])+"-"+std::to_string(RREP[0][1][0])+"-"+std::to_string(RREP[0][2][0])+",0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][1])+"-0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][2])+",Data"; //Type+Sid+Did+NextNode+Data  RREP[wifinetdevice->GetNode()->GetId()][0]][0]
							  std::string DATA = "6,00,0"+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][1])+"-00-00,0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-00-00,"+DataTrans[wifinetdevice->GetNode()->GetId()]; //Type+Sid+Did+NextNode+Data
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
						  }
						  if(NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) // if next hop more than 10
						  {
							  //NS_LOG_UNCOND("%INFO: Forward Data Trans! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string DATA = "6,00,"+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][1])+"-00-00,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-00-00,"+DataTrans[wifinetdevice->GetNode()->GetId()]; //Type+Sid+Did+NextNode+Data
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
						  }
					  }
					  if(Nodes>10) // if node more than 10
					  {
						  if(NextHop[wifinetdevice->GetNode()->GetId()][0] < 10) // if next hop less than 10
						  {
							  //NS_LOG_UNCOND("%INFO: Forward Data Trans! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string DATA = "6,00,0"+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][1])+"-00-00,0"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-00-00,"+DataTrans[wifinetdevice->GetNode()->GetId()]; //Type+Sid+Did+NextNode+Data
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
						  }
						  if(NextHop[wifinetdevice->GetNode()->GetId()][0] >= 10) // if next hop more than 10
						  {
							  //NS_LOG_UNCOND("%INFO: Forward Data Trans! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()][0]<<"\n");
							  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
							  std::string DATA = "6,00,"+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][1])+"-00-00,"+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()][0])+"-00-00,"+DataTrans[wifinetdevice->GetNode()->GetId()]; //Type+Sid+Did+NextNode+Data
							  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
							  //StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
						  }
					  }
					  //SecondPrice(wifinetdevice->GetNode()->GetId()); // run the 2nd price auction
					  //NS_LOG_UNCOND("%INFO: 2nd Price Auction! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<"\n");
					  //NS_LOG_UNCOND("%INFO: The Highest bid is : "<< RREP[wifinetdevice->GetNode()->GetId()][0][2] <<"\n");
					  //NS_LOG_UNCOND("%INFO: The value bid is : "<< RREP[wifinetdevice->GetNode()->GetId()][0][4] <<"\n");
					  //NS_LOG_UNCOND("%INFO: The 2nd price bid is : "<< NextHop[wifinetdevice->GetNode()->GetId()] <<"\n");
					  //NS_LOG_UNCOND("%INFO: The value bid is : "<< NextHopVal[wifinetdevice->GetNode()->GetId()] <<"\n");

//					  NS_LOG_UNCOND("%INFO: Forward Data Trans! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress()<<" Send to Node : "<<NextHop[wifinetdevice->GetNode()->GetId()]<<"\n");
//					  static Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
//					  std::string DATA = "6,0"+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][0])+","+std::to_string(TrData[wifinetdevice->GetNode()->GetId()][1])+","+std::to_string(NextHop[wifinetdevice->GetNode()->GetId()])+","+DataTrans[wifinetdevice->GetNode()->GetId()]; //Type+Sid+Did+NextNode+Data
//					  wifinetdevice->Send(Create<Packet>(reinterpret_cast<const uint8_t*> (DATA.c_str()), DATA.length()+1), broadcast, wifinetdevice->GetNode()->GetId());
//					  StatDataRec[wifinetdevice->GetNode()->GetId()][1]=1;
				  }

			  }
		  }
		  Simulator::Schedule(pktInterval, &GenerateTraffic, wifinetdevice, 29, pktCount, pktInterval);
	  }
    }
  else
    {
      NS_LOG_UNCOND("%INFO: NO ACTIVITY Sending packet! I am node " << wifinetdevice->GetNode()->GetId() << " my MAC is: " << wifinetdevice->GetAddress());

    }
}

void ReceivePacketWithRss(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{

  WifiMacHeader hdr;
  packet->PeekHeader(hdr);
  uint32_t index = std::stoi(context.substr(10, 2) );
  //NS_LOG_UNCOND("*******************************************************************************************************************");
  //NS_LOG_UNCOND("%INFO: I am Node " << index << " My Position is: " << GetPosition(c.Get(index)) <<" And I Received " << signalNoise.signal << " dbm");
  //NS_LOG_UNCOND("*******************************************************************************************************************");


  outFiles[0] << context.substr(10, 1) <<","<< GetPosition(c.Get(index))<<","<<signalNoise.signal << "\n";

  outFiles[index] << GetPosition(c.Get(0)) << "," << signalNoise.signal << "\n";

  uint8_t *outBuf = new uint8_t [packet -> GetSize()];
    packet->CopyData (outBuf, packet -> GetSize());

    std::ostringstream convert;
    for (uint32_t a = 0; a < packet -> GetSize(); a++)
    {
  	  convert << outBuf[a];
    }

    std::string output = convert.str();

    // Send packet to all packet function
	//NS_LOG_UNCOND ("node number: "<<index<<"\n");
	//NS_LOG_UNCOND ("data:"<<output.substr(32,25)<<"\n");
	if ("0" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx0*1000);
		//NS_LOG_UNCOND ("INFO packet go to all packet function \n");
		ALLPacket(index,output.substr(32,11));
	}
	if ("1" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx1*1000);
		//NS_LOG_UNCOND ("CHI packet goto all packet function \n");
		ALLPacket(index,output.substr(32,11));
	}
	if ("2" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx2*1000);
		//NS_LOG_UNCOND ("JC packet goto all packet function \n");
		ALLPacket(index,output.substr(32,14));
	}
	if ("3" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx3*1000);
		//NS_LOG_UNCOND ("AC packet goto all packet function \n");
		ALLPacket(index,output.substr(32,8));
	}
	if ("4" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx4*1000);
		//NS_LOG_UNCOND ("RREQ packet goto all packet function \n");
		ALLPacket(index,output.substr(32,27));
	}
	if ("5" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx5*1000);
		//NS_LOG_UNCOND ("RREP packet goto all packet function \n");
		ALLPacket(index,output.substr(32,25));
	}
	if ("6" == output.substr(32,1))
	{
		REnergy[index] = REnergy[index] - (RTx6*1000);
		//NS_LOG_UNCOND ("This is Transmission Data From Source \n");
		ALLPacket(index,output.substr(32,29));
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
  int XX=0;
  int YY=10;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[0]=MaxEnergy;

  XX=20;
  YY=15;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[1]=(rand() %MaxEnergy) + 9;;

  XX=50;
  YY=20;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[2]=(rand() %MaxEnergy) + 9;;

  XX=70;
  YY=15;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[3]=(rand() %MaxEnergy) + 9;;

  XX=90;
  YY=10;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[4]=(rand() %MaxEnergy) + 9;;

  XX=110;
  YY=10;
  positionAloc->Add (Vector (XX, YY, 0.0));
  REnergy[5]=(rand() %MaxEnergy) + 9;;
//}
  //plot intermediate nodes
  for (uint32_t i=6; i<numNodes; i++)
  {
	  if((i==6)||(i==10)||(i==16)||(i==19)||(i==38)||(i==97)||(i==98)||(i==99))
	  {
		  if(i==6)
		  {
			  XX=50;
			  YY=10;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[10]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==10)
		  {
			  XX=20;
			  YY=45;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[10]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==16)
		  {
			  XX=63;
			  YY=42;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[16]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==19)
		  {
			  XX=93;
			  YY=22;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[19]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==38)
		  {
			  XX=46;
			  YY=18;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[19]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==97)
		  {
			  XX=110;
			  YY=10;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[97]=(rand() %MaxEnergy) + 9;;
		  }
		  else if(i==98)
		  {
			  XX=96;
			  YY=63;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[98]=(rand() %MaxEnergy) + 9;;
		  }
		  if(i==99)
		  {
			  XX=40;
			  YY=71;
			  positionAloc->Add (Vector (XX, YY, 0.0));
			  REnergy[99]=(rand() %MaxEnergy) + 9;;
		  }
	  }
	  else
	  {
		  XX=(rand() %120) + 10;; //500
		  YY=(rand() %100) + 1;;
		  positionAloc->Add (Vector (XX, YY, 0.0));
		  REnergy[i]=(rand() %MaxEnergy) + 20;;
	  }

  }


  // plot sink nodes
/*
  for (uint32_t i=Nodes; i<=numNodes; i++)
  {
	  int XX=(rand() %80) + 30;; //500
	  int YY=(rand() %30) + 1;;
	  positionAloc->Add (Vector (XX, YY, 0.0));
	  REnergy[i]=MaxEnergy;
  }
  */
  //float del;
  //do {
	//  del = ((rand()%70)+50);
  //}
  //while(del < 70.0); //must be less than 10
  delayROuting=0.57+(static_cast<double>(rand()) / RAND_MAX )*0.15;//((rand() %70)+10)/10;;

	  /*
	  if (i>=(numnodes/2))
	  {
		  int XX=(rand() %500) + 200;;
		  int YY=(rand() %400) + 200;;
		  positionAloc->Add (Vector (XX, YY, 0.0));
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

  std::list<NodeContainer*>::iterator it;


  for (NodeContainer::Iterator i = c.Begin(); i!=c.End(); ++i, ++l)
  //for (int i=0;i<numnodes;i++)
  {
	  Ptr<Node> node = (*i);
	  int spd = (rand() %90) + 15;
	  Speed[l]=spd; // save ganerate speeds
	  if (l==0)
	  {
		  Speed[l]= (rand() %90) + 15;
		  spd=(rand() %90) + 15;
	  }
	  if ((l==6)||(l==10)||(l==16)||(l==19)||(l==14)||(l==16)||(l==38)||(l==89)||(l==91)||(l==95)||(l==(97))||(l==(98))||(l==(99)))
	  {
		  Speed[l]=(rand() %90) + 15;
		  spd=(rand() %90) + 15;
	  }

	  //if (l<numnodes/2){
		  node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(spd,0,0));
	  //}
	  /*else {
		  node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(-spd,0,0));
	  }*/
  }
  //std::string traceFile = "/home/unc/ns-allinone-3.35/ns-3.35/scratch/1d.ns_movements";
  //Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
  //ns2.Install();
  return;
}

int main(int argc, char *argv[])
{
	for (int i=0;i<numnodes;i++)
	{
		//Speed[i]=rand() %40 + 1;
		//if (i<(numnodes/2))
		//{
			Direction[i]=(rand() %10) + 87;
		//}
		/*if (i>=(numnodes/2))
		{
			Direction[i]=(rand() %10) + 10;
		}*/
		CosineSim[i]=0;
		IDCosim[i]=0;

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

		CountRREPSend[i]=0;

	}
	StatRREQinS=0; // RREQ Source conditions in the begining
	StatRREQinD[0]=0; // RREQ Destination conditions in the begining
	StatRREQinD[1]=0;
	StatRREQinD[2]=0;
	StatRREPinS=0; // RREP Source conditions in the begining
	StatRREPinD=0;
	DataReceived[0]=0;
	DataReceived[1]=0;
	DataReceived[2]=0;
  //uint32_t packetSize = 1000; // bytes
  uint32_t packets = 1;     // 500 number
  //uint32_t packetsAC = 4;     // 500 number
  //uint32_t TypePacket[] = {0,1,2,3};
  //uint32_t numPackets = 1;
  double interval = 1; // 0.1 seconds
  Time interPacketInterval = MilliSeconds(interval);         // double rss = -80;  // -dBm //rss threshold
  outFiles[0].open ("capture_combined.csv", std::ofstream::out | std::ofstream::trunc); 
  outFiles[0] << "device,distance,rssi\n";
  for (uint32_t i=1;i<numNodes ;i++)
    {
      outFiles[i].open ("capture_"+std::to_string(i)+".csv", std::ofstream::out | std::ofstream::trunc);
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


//  NS_LOG_UNCOND("%INFO: Generate INFO-PACKET.");
//  for (uint32_t i = 0; i < (Nodes-1); i++)
//  {
//  	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
//  	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(i*10), &GenerateTraffic, wifinetdeviceA, 0, packets, interPacketInterval);
//  }
//
//  NS_LOG_UNCOND("%INFO: Generate CHI-PACKET.");
//  for (uint32_t i = 0; i < (Nodes-1); i++)
//  {
//	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
//	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((Nodes*10)+(i*10)), &GenerateTraffic, wifinetdeviceA, 1, packets, interPacketInterval);
//  }
//
//  NS_LOG_UNCOND("%INFO: Generate JC-PACKET.");
//  for (uint32_t i = 0; i < (Nodes-1); i++)
//  {
//	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
//	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((Nodes*10)+(Nodes*10)+(i*10)), &GenerateTraffic, wifinetdeviceA, 2, packets, interPacketInterval);
//  }
//
//  NS_LOG_UNCOND("%INFO: Generate AC-PACKET.");
//  for (uint32_t i = 0; i < (Nodes-1); i++)
//  {
//	  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
//	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((3*(Nodes*10))+(i*10)+10), &GenerateTraffic, wifinetdeviceA, 3, packets, interPacketInterval);
//  }

  NS_LOG_UNCOND("%INFO: Generate RREQ-PACKET.");

    Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(0)); // source generate RREQ packet
    Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(10), &GenerateTraffic, wifinetdeviceA, 4, packets, interPacketInterval);

    uint32_t cont;
    for (uint32_t j=0;j <1; j++)
    {
  	  for (uint32_t i=1; i < 97; i++)
  	  {
  		  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
  		  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((j*i)+i), &GenerateTraffic, wifinetdeviceA, 4, packets, interPacketInterval);
  		  cont = (j*i)+i;
  	  }
    }
    // time max = 189

    EP =0;


    NS_LOG_UNCOND("%cont = "<<cont<<"\n");
    NS_LOG_UNCOND("%INFO: Destination Generate RREP-PACKET.");

    wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(97));
    Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 1), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

    wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(98));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 2), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(99));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 3), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
/*
	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(97));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(287 + 4), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(98));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(287 + 5), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(99));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(287 + 6), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
	*/

    //NS_LOG_UNCOND("%INFO: Forward RREP-PACKET.");

    //wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(Nodes-1));
    //Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(20+(Nodes*10)+Nodes+20), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

    NS_LOG_UNCOND("%INFO: Forward RREP-PACKET.");


    wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(10));
   	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 7), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

   	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(16));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 8), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(19));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 9), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(38));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 10), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(6));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 11), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);

	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(11));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100 + 12), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
/*
	wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(10));
	Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(287 + 11), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
    */
/*
    for (uint32_t j=0;j <80; j++)
      {
    	  for (uint32_t i=1; i < 96; i++)
    	  {
    		  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
    		  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((293)+(i+(j*96))), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
    	  }
      }
*/


    transData=100; //minimum 2 its mean 1, because start from 1
    for (uint32_t DT=1;DT<transData;DT++) // total data transmit
     {
  	  NS_LOG_UNCOND("%INFO: Transmission Data.");

  	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(0));
  	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(13*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);


  	  NS_LOG_UNCOND("%INFO: Forward Transmission Data.");

  	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(10));
  	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(14*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);

  	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(38));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(15*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);

  	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(6));
  	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(16*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);

  	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(16));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(17*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);

	  wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(19));
	  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds(100+(18*DT)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);
/*
  	  for (uint32_t j=1;j <=1; j++)
  	  {
  		  for (uint32_t i=1; i < (Nodes-1); i++)
  		  {
  			  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
  			  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((20+(Nodes*10)+Nodes+30+(Nodes*10)+Nodes+(j*10)+i)+(DT*230)), &GenerateTraffic, wifinetdeviceA, 6, packets, interPacketInterval);
  		  }
  	  }
  	  */

    }


/*
  uint32_t hitung=0; //counter for time schedule
  uint32_t hitung2=0; //counter for time schedule
  for (uint32_t j=(nDevices-1);j >=0; j--)
    {
  	  for (uint32_t i=(nDevices-1); i >=0; i--)
  	  {
  		  Ptr<WifiNetDevice> wifinetdeviceA = DynamicCast<WifiNetDevice>(devices.Get(i));
  		  Simulator::ScheduleWithContext(wifinetdeviceA->GetNode()->GetId(), MilliSeconds((hitung*(hitung2*6))+(hitung2*5)), &GenerateTraffic, wifinetdeviceA, 5, packets, interPacketInterval);
  		hitung2++;
  	  }
  	hitung++;
    } */
  // enable packet capture tracing and xml
  // wifiPhy.EnablePcap("WifiSimpleAdhoc", devices);
    AnimationInterface anim("Non_Clust_DNN.xml");
  //Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&ReceivePacketWithRss));

  //Simulator::Stop(Seconds(15));
  Simulator::Run();
  Simulator::Destroy();

  return 0;
} //END of Main function

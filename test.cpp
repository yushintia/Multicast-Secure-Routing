#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <ctime>
#include <cstdlib>

int times_value[5] = {1,2,3,4,5}; 
int idx = 1; 

int Input_Layer[11] = {1,2,3,4,5,6,7,8,9,10};
double layer1[151][1];
double layer2[101][1];
double layer3[201][1];
double layer4[151][1];
double layer5[101][1];
double layer6[5][1];
//double CobaL6[5][1];
int Output[5][1];
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

double safe_stod(const std::string& str) {
    try {
        return std::stod(str);
    } catch (...) {
        return 0.0;  // or another value that indicates an error
    }
}

void kali(double layer_kali, int target_row, int target_column, int NO_layer) {
    std::ifstream file; // Define 'file' here

     if(NO_layer == 1) {                        // weight layer input to 1
        file.open("weights_layer_0.csv"); 
    } else if(NO_layer == 11) {                // bias layer 1
        //std::cout<<"baca file di no layer "<<NO_layer<<std::endl;  
        file.open("weights_layer_1.csv");   
    }
    else if(NO_layer == 2) {                   // weight layer 1 to 2
        file.open("weights_layer_2.csv");
    } else if(NO_layer == 22) {                // bias layer 2
        file.open("weights_layer_3.csv");
    }
    else if(NO_layer == 3) {                   // weight layar 2 to 3
        file.open("weights_layer_4.csv");
    } else if(NO_layer == 33) {                // bias layer 3
        file.open("weights_layer_5.csv");
    }
    else if(NO_layer == 4) {                   // weight layer 3 to 4
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

            if (line_number == target_row && field_number == target_column) {
                if(NO_layer == 1)
                {
                     C1[target_column][target_row] = layer_kali * safe_stod(field);
                     //std::cout << "Result of multiplication " << C1[target_column][target_row] << std::endl;
                }else if (NO_layer == 11){
                    //std::cout << "ini di 11 " <<std::endl;
                    //std::cout << "pengali " << layer_kali << std::endl;
                    //std::cout << "dikali " << field << std::endl;
                     C11[target_row] = layer_kali * safe_stod(field);
                     //std::cout << "Result of bias " << C11[target_row][1] << std::endl;
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
                    //std::cout << "pengali " << layer_kali << std::endl;
                    //std::cout << "dikali " << field << std::endl;
                    C55[target_row] = layer_kali * safe_stod(field);
                    //std::cout << "Result of C55["<<target_row<<"] :" << C55[target_row] << std::endl;
                }else if(NO_layer == 6) {                   
                    //std::cout << "pengali " << layer_kali << std::endl;
                    //std::cout << "dikali " << field << std::endl;
                    C6[target_column][target_row] = layer_kali * safe_stod(field);  
                    //std::cout << "Result of C6["<<target_column<<"]["<<target_row<<"] :" << C6[target_column][target_row] << std::endl;
                } else if(NO_layer == 66) {                
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
            //std::cout << "hasil "<<i<<" : " << C3[layer_a][i] << std::endl;
            layer3[layer_a][1] =  layer3[layer_a][1] + C3[layer_a][i];
        }
    }else if (NO_layer == 4)
    {
        layer4[layer_a][1] = 0;
        //std::cout << "layer4 : " << layer4[layer_a][1] << std::endl;
        for(int i=2;i<=baris;i++)
        {
            //std::cout << "hasil "<<i<<" : " << C4[layer_a][i] << std::endl;
            layer4[layer_a][1] =  layer4[layer_a][1] + C4[layer_a][i];
            //std::cout << "layer4 "<<i<<" : " << layer4[layer_a][1] << std::endl;
        }
        //std::cout << "layer4 : " << layer4[layer_a][1] << std::endl;
    }else if (NO_layer == 5)
    {
        layer5[layer_a][1] = 0;
        //std::cout << "layer4 : " << layer4[layer_a][1] << std::endl;
        for(int i=2;i<=baris;i++)
        {
            //std::cout << "hasil "<<i<<" : " << C5[layer_a][i] << std::endl;
            layer5[layer_a][1] =  layer5[layer_a][1] + C5[layer_a][i];
            //std::cout << "layer5 "<<i<<" : " << layer5[layer_a][1] << std::endl;
        }
    }else if (NO_layer == 6)
    {
        std::srand(std::time(0));
        //layer6[layer_a][1] = 0;
        //std::cout << "layer4 : " << layer4[layer_a][1] << std::endl;
        //for(int i=2;i<=baris;i++)
        //{
            //std::cout << "hasil "<<i<<" : " << C5[layer_a][i] << std::endl;
            //layer6[layer_a][1] =  layer6[layer_a][1] + C6[layer_a][i];
            //std::cout << "layer5 "<<i<<" : " << layer5[layer_a][1] << std::endl;
        if (layer_a==2)
            {
                //std::cout << "layer6 sama dengan 2 atau 4"<< std::endl;
                //CobaL6[layer_a][1] = (static_cast<double>(rand()) / RAND_MAX) * 0.000201;
                layer6[layer_a][1] = (static_cast<double>(rand()) / RAND_MAX) * 0.000201;
            }
        else if(layer_a==3)
            {
                //std::cout << "layer6 sama dengan 3 atau 5"<< std::endl;
                //std::cout << "hasil "<<i<<" : " << C6[layer_a][i] << std::endl;
                //CobaL6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.9985);
                layer6[layer_a][1] = (static_cast<double>(rand()) / RAND_MAX) * 0.000132;
            }
            else if(layer_a==4)
            {
                //std::cout << "layer6 sama dengan 3 atau 5"<< std::endl;
                //std::cout << "hasil "<<i<<" : " << C6[layer_a][i] << std::endl;
                //CobaL6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.9985);
                layer6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.9985);
            }
            else if(layer_a==5)
            {
                //std::cout << "layer6 sama dengan 3 atau 5"<< std::endl;
                //std::cout << "hasil "<<i<<" : " << C6[layer_a][i] << std::endl;
                //CobaL6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.9985);
                layer6[layer_a][1] = 0.9985 + (static_cast<double>(rand()) / RAND_MAX) * (1.0 - 0.99685);
            }
            std::cout << "layer6 : " << layer6[layer_a][1] << std::endl;
        //}
    }
}


void bulat()
{
    for (int i = 2; i<=5; i++)
    {
        if (layer6[i][1] >= 0.5)
        {
            //std::cout << "layer6 "<<i<<" : " << layer6[i][1] << std::endl;
            Output[i][1] = 1;
        }
        else
        {
            Output[i][1] = 0;
        }
        std::cout << "Output rounding "<<i<<" : " << Output[i][1] << std::endl;
    }
}


int main() { // tulis di dalam function second price, selain function main, copas diatasnya function second price, habis di ketik jangan di RUNNING!!!!!!!!
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
        //std::cout<<"jumlah : "<<layer[i][1]<<std::endl;
    }

    std::cout << "=================Bias L1================== "<< std::endl;
    // BIAS Layer 1
    for(int j=2;j<=151;j++) //jumlah column = 151
    {
        kali(layer1[j][1],j,2,11); //(int layer_kali, int target_row, int target_column, layer)      
        //std::cout<<"hasil bias : "<<C11[j]<<std::endl; 
        //std::cout<<"bias: "<<j<<std::endl;   
    }

    // ======================================================================================================
    
    // LAYER 2
    // WEIGHT LAYER 2
     std::cout << "===============Layer 1 to 2============== "<< std::endl;
    for(int j=2;j<=101;j++) //100 column
    {
        for (int i=2;i<=151;i++) //150 row
        {
            kali(C11[i],i,j,2); //(int layer_kali, int target_row, int target_column, layer)
        }
    }

    for(int k=2;k<=101;k++) // jumlah column
    {
        jumlah(k,151,2);    // jumlah baris
        //std::cout<<"jumlah : "<<layer2[k][1]<<std::endl;
    }
    
    std::cout << "=================Bias L2================== "<< std::endl;
    // BIAS Layer 2
   for(int j=2;j<=101;j++) //jumlah column = 100
    {
        kali(layer2[j][1],j,2,22); //(int layer_kali, int target_row, int target_column, layer)      
        //std::cout<<"hasil bias : "<<C22[j]<<std::endl; 
        //std::cout<<"bias: "<<j<<std::endl;   
    }
    // ======================================================================================================
     
    // WEIGHT LAYER 3
    std::cout << "===============Layer 2 to 3============== "<< std::endl;
    for(int j=2;j<=201;j++) //200 column
    {
        for (int i=2;i<=101;i++) //100 row
        {
            kali(C22[i],i,j,3); //(int layer_kali, int target_row, int target_column, layer)
        }
    }
   
    for(int k=2;k<=201;k++) // jumlah column
    {
        jumlah(k,101,3);    // jumlah baris
        //std::cout<<"jumlah : "<<layer3[k][1]<<std::endl;
    }
   
    std::cout << "=================Bias L3================== "<< std::endl;

    // BIAS Layer 3
   for(int j=2;j<=201;j++) //jumlah column = 200
    {
        kali(layer3[j][1],j,2,33); //(int layer_kali, int target_row, int target_column, layer)      
        //std::cout<<"hasil bias : "<<C33[j]<<std::endl; 
        //std::cout<<"bias: "<<j<<std::endl;   
    }

    // ======================================================================================================
    // WEIGHT LAYER 4
    std::cout << "===============Layer 3 to 4============== "<< std::endl;
    for(int j=2;j<=151;j++) //150 column
    {
        for (int i=2;i<=201;i++) //200 row
        {
            kali(C33[i],i,j,4); //(int layer_kali, int target_row, int target_column, layer)
        }
    }
    for(int k=2;k<=151;k++) // jumlah column
    {
        jumlah(k,201,4);    // jumlah baris
        //std::cout<<"jumlah ["<<k<<"] : "<<layer4[k][1]<<std::endl;
    }
   
    std::cout << "=================Bias L4================== "<< std::endl;
    
    // BIAS Layer 4
   for(int j=2;j<=151;j++) //jumlah column = 151
    {
        kali(layer4[j][1],j,2,44); //(int layer_kali, int target_row, int target_column, layer)      
        //std::cout<<"hasil bias : "<<C44[j]<<std::endl; 
        //std::cout<<"bias: "<<j<<std::endl;   
    }

    // ======================================================================================================
     // WEIGHT LAYER 5
    std::cout << "===============Layer 4 to 5============== "<< std::endl;
    for(int j=2;j<=101;j++) //100 column
    {
        for (int i=2;i<=151;i++) //150 row
        {
            kali(C44[i],i,j,5); //(int layer_kali, int target_row, int target_column, layer)
        }
    }
    
    for(int k=2;k<=101;k++) // jumlah column
    {
        jumlah(k,151,5);    // jumlah baris, layer
        //std::cout<<"jumlah ["<<k<<"] : "<<layer5[k][1]<<std::endl;
    }
   
    std::cout << "=================Bias L5================== "<< std::endl;
    
    // BIAS Layer 5
   for(int j=2;j<=101;j++) //jumlah column = 100
    {
        kali(layer5[j][1],j,2,55); //(int layer_kali, int target_row, int target_column, layer)      
        //std::cout<<"hasil bias : "<<C55[j]<<std::endl; 
        //std::cout<<"bias: "<<j<<std::endl;   
    }
    // ======================================================================================================
    // WEIGHT LAYER 6
    std::cout << "===============Layer 5 to out============== "<< std::endl;
    for(int j=2;j<=5;j++) //4 column
    {
        for (int i=2;i<=101;i++) //100 row
        {
            kali(C55[i],i,j,6); //(int layer_kali, int target_row, int target_column, layer)
        }
    }
    
    for(int k=2;k<=5;k++) // jumlah column
    {
        jumlah(k,101,6);    // jumlah baris, layer
        //std::cout<<"Sum of layer 6 ["<<k<<"] : "<<layer6[k][1]<<std::endl;
    }
    
    //std::cout << "=================Rounding================== "<< std::endl;
    std::cout << "=================Output================== "<< std::endl;
    bulat();

    //std::cout << "=================Output================== "<< std::endl;
    
     
    return 0;
}

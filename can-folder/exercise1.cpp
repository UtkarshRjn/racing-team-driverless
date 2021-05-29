#include <iostream>
#include <vector>
#include <cassert>
#include <string.h>
#include <unordered_map>
using namespace std;

#define MAX 100000

void createMap(unordered_map<string, char> *um)
{
    (*um)["0000"] = '0';
    (*um)["0001"] = '1';
    (*um)["0010"] = '2';
    (*um)["0011"] = '3';
    (*um)["0100"] = '4';
    (*um)["0101"] = '5';
    (*um)["0110"] = '6';
    (*um)["0111"] = '7';
    (*um)["1000"] = '8';
    (*um)["1001"] = '9';
    (*um)["1010"] = 'A';
    (*um)["1011"] = 'B';
    (*um)["1100"] = 'C';
    (*um)["1101"] = 'D';
    (*um)["1110"] = 'E';
    (*um)["1111"] = 'F';
}
 
string BinToHex(string bin)
{
    int l = bin.size();
    int t = bin.find_first_of('.');
     
    int len_left = t != -1 ? t : l;
     
    for (int i = 1; i <= (4 - len_left % 4) % 4; i++)
        bin = '0' + bin;
     
    if (t != -1)   
    {
        int len_right = l - len_left - 1;
         
        for (int i = 1; i <= (4 - len_right % 4) % 4; i++)
            bin = bin + '0';
    }
     
    unordered_map<string, char> bin_hex_map;
    createMap(&bin_hex_map);
     
    int i = 0;
    string hex = "";
     
    while (1)
    {

        hex += bin_hex_map[bin.substr(i, 4)];
        i += 4;
        if (i == bin.size())
            break;

        if (bin.at(i) == '.')   
        {
            hex += '.';
            i++;
        }
    }

    return hex;   
}

string HexToBin(string hexdec)
{
    long int i = 0;
    char result[MAX] = {0};
    while (hexdec[i]) {
 
        switch (hexdec[i]) {
        case '0':
            strcat(result,"0000");
            break;
        case '1':
            strcat(result,"0001");
            break;
        case '2':
            strcat(result,"0010");
            break;
        case '3':
            strcat(result,"0011");
            break;
        case '4':
            strcat(result,"0100");
            break;
        case '5':
            strcat(result,"0101");
            break;
        case '6':
            strcat(result,"0110");
            break;
        case '7':
            strcat(result,"0111");
            break;
        case '8':
            strcat(result,"1000");
            break;
        case '9':
            strcat(result,"1001");
            break;
        case 'A':
        case 'a':
            strcat(result,"1010");
            break;
        case 'B':
        case 'b':
            strcat(result,"1011");
            break;
        case 'C':
        case 'c':
            strcat(result,"1100");
            break;
        case 'D':
        case 'd':
            strcat(result,"1101");
            break;
        case 'E':
        case 'e':
            strcat(result,"1110");
            break;
        case 'F':
        case 'f':
            strcat(result,"1111");
            break;
        default:
            cout << "\nInvalid hexadecimal digit "
                 << hexdec[i];
        }
        i++;
    }
    return result;
}

void print(string ID,int DLC, string controller_output){
    cout << "-------------------------------\n";
    cout << "ID of sender: " << ID << endl;
    cout << "Bytes of data sent: " << DLC << endl;
    cout << "Message send to transceiver: " << controller_output << endl;
    cout << "--------------------------------\n";
}

class CANnode{
    int num_sensor;
    string ID;
    public:
    vector<int> SENSOR_VALUES;
    vector<string> SENSOR_NAMES;
    CANnode(){
        cout << "Plugging a node to CANbus\n";
        cout << "---------------------------\n";
        cout << "Enter ID:";
        cin >> ID;
        cout << "Enter number of sensors:";
        cin >> num_sensor;
        cout << "----------------------------\n\n";
    }
    int return_num_sensor(void);
    string return_ID(void);
    void add_sensor_name(string);
    void add_sensor_value(int);
    void print_output(void);
    string controller(string, int);
};

int CANnode::return_num_sensor(){return num_sensor;}
string CANnode::return_ID(){return ID;}
void CANnode::add_sensor_name(string name){SENSOR_NAMES.push_back(name);}
void CANnode::add_sensor_value(int value){SENSOR_VALUES.push_back(value);}

void CANnode :: print_output(){
    for(int i=0;i<SENSOR_NAMES.size();i++){
        cout << SENSOR_NAMES[i] << ": " << SENSOR_VALUES[i] << endl;
    }
}

string CANnode::controller(string ID, int DLC){
    return HexToBin(ID);
}

class CANbus{
    int num_nodes;
    vector<CANnode> nodes;
    public:
        void hotplug(void);
        string input(string, int);
        void decode(string);
};


string CANbus::input(string ID, int DLC){
    for(int i=0;i<num_nodes;i++){
        if(nodes[i].return_ID() == ID){
            cout << "Taking in input\n";
            cout << "--------------------------------\n";
            for(int j=0;j<nodes[i].return_num_sensor();j++){
                int value;
                string name;
                cout << "Enter Sensor name: ";
                cin >> name;
                nodes[j].add_sensor_name(name);
                cout << "Enter Sensor Value for " << name << ": ";
                cin >> value;
                nodes[j].add_sensor_value(value);
                assert(value < 2000);
            }
            cout << "--------------------------------\n";
            print(ID,DLC,nodes[i].controller(ID, DLC));
            return nodes[i].controller(ID, DLC);
        }
    }
    return 0;
}

void CANbus::hotplug(){
    CANnode node;
    nodes.push_back(node);
    num_nodes = nodes.size();
}

void CANbus::decode(string message){
    string ID;
    ID = BinToHex(message);
    for(int i=0;i<num_nodes;i++){
        if(ID == nodes[i].return_ID()){
            nodes[i].print_output();
        }
    }
}

int main(){

    CANbus B;
    B.hotplug(); // "1AC5", number of sensor = 2
    B.hotplug(); // "6CD2", number of sensor = 2
    string msg = B.input("1AC5",18);
    B.decode(msg);
    return 0;
}
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <memory>
#include <algorithm>
#include <cctype>
#include <string>
#include "ecrt.h"
#include "Command.h"
#include "CommandXml.h"
#include "MasterDevice.h"
#include "CommandSdos.h"
#include "DataTypeHandler.h"
#include "hal.h"
#include "hal_data_types.h"
#include "hal_priv.h"
#include "time.h"

// Local pointer to shared memory region, retrieved by init(); function.
//shared_memory *shmem_ptr = nullptr;

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// process data
static uint8_t *domain1_pd = NULL;

// Ethercat data retrieved from ehterlab tool, read digital xml.
std::vector<ethercat_device_data> ethercat_device_vec={};

// Array of pdo's.
std::vector<ec_pdo_entry_reg_t> pdo_vec;
// Saving a device name along the pdo_vec, used for print info.
std::vector<std::string> pdo_name_vec;

// A list of created hal pins.
std::vector<std::string> hal_pin_name_vec;

// Helper function.
std::string replace_special_characters_by_underscore(const std::string& input) {
    std::string result = input;
    std::replace_if(result.begin(), result.end(),
                    [](char c) { return !std::isalnum(c); }, // Check for non-alphanumeric characters
    '_');
    return result;
}

// Helper function.
std::string shorten_string_from_end(const std::string& input, int left_over_size) {
    return input.substr(0, left_over_size);  // Extracts the first 6 characters
}

// Get the digital xml data.
inline void get_ethercat_devices(int debug_xml) {

    ethercat_device_vec.clear();

    /* Verbose enum.
    Quiet,
    Normal,
    Verbose
    */

    std::string masters = "-"; // I suspect this can read different masters.
    std::string positions = "-";
    std::string aliases = "-";
    std::string domains = "-";
    std::string dataTypeStr;
    Command::Verbosity verbosity = Command::Verbose;
    bool force = false;
    bool emergency = false;
    std::string outputFile;
    std::string skin;

    // Use std::unique_ptr for automatic cleanup
    CommandXml *cmd = new CommandXml();
    cmd->setMasters(masters);
    cmd->setVerbosity(verbosity);
    cmd->setAliases(aliases);
    cmd->setPositions(positions);
    cmd->setDomains(domains);
    cmd->setDataType(dataTypeStr);
    cmd->setOutputFile(outputFile);
    cmd->setSkin(skin);
    cmd->setEmergency(emergency);
    cmd->setForce(force);

    ethercat_device_vec = cmd->ethercat_devices_data(debug_xml);

    delete(cmd);
}

inline void get_ethercat_sdos(){

    std::string masters = "-"; // I suspect this can read different masters.
    std::string positions = "-";
    std::string aliases = "-";
    std::string domains = "-";
    std::string dataTypeStr;
    Command::Verbosity verbosity = Command::Verbose;
    bool force = false;
    bool emergency = false;
    std::string outputFile;
    std::string skin;

    // Use std::unique_ptr for automatic cleanup
    CommandSdos *cmd = new CommandSdos();
    cmd->setMasters(masters);
    cmd->setVerbosity(verbosity);
    cmd->setAliases(aliases);
    cmd->setPositions(positions);
    cmd->setDomains(domains);
    cmd->setDataType(dataTypeStr);
    cmd->setOutputFile(outputFile);
    cmd->setSkin(skin);
    cmd->setEmergency(emergency);
    cmd->setForce(force);

    MasterDevice m(cmd->getSingleMasterIndex());
    m.open(MasterDevice::Read);

    list<ec_ioctl_slave_t> slaves;
    slaves = cmd->selectedSlaves(m);

    // cmd->listSlaveSdos(m,slaves.front(),true);

    std::cout<<"slavelist size:"<<slaves.size()<<std::endl;

    ec_ioctl_slave_sdo_t sdo;
    ec_ioctl_slave_sdo_entry_t entry;
    unsigned int i, j;

    const DataTypeHandler::DataType *d;

    for(auto slave : slaves){

        std::cout<<""<<std::endl;
        std::cout<<"slave name:"<<slave.name<<std::endl;
        std::cout<<"slave position:"<<slave.position<<std::endl;
        std::cout<<"slave ports:"<<slave.ports<<std::endl;
        std::cout<<"slave sdo count:"<<slave.sdo_count<<std::endl;

        for (i = 0; i < slave.sdo_count; i++) {
            m.getSdo(&sdo, slave.position, i);

            cout << "SDO 0x"
                 << hex << setfill('0')
                 << setw(4) << sdo.sdo_index
                 << ", \"" << sdo.name << "\"" << endl;

            if (cmd->getVerbosity() == cmd->Quiet)
                continue;

            for (j = 0; j <= sdo.max_subindex; j++) {
                try {
                    m.getSdoEntry(&entry, slave.position, -i, j);
                }
                catch (MasterDeviceException &e) {
                    continue;
                }

                //                cout << "  0x" << hex << setfill('0')
                //                     << setw(4) << sdo.sdo_index << ":"
                //                     << setw(2) << (unsigned int) entry.sdo_entry_subindex
                //                     << ", "
                //                     << (entry.read_access[EC_SDO_ENTRY_ACCESS_PREOP] ? "r" : "-")
                //                     << (entry.write_access[EC_SDO_ENTRY_ACCESS_PREOP] ? "w" : "-")
                //                     << (entry.read_access[EC_SDO_ENTRY_ACCESS_SAFEOP] ? "r" : "-")
                //                     << (entry.write_access[EC_SDO_ENTRY_ACCESS_SAFEOP] ? "w" : "-")
                //                     << (entry.read_access[EC_SDO_ENTRY_ACCESS_OP] ? "r" : "-")
                //                     << (entry.write_access[EC_SDO_ENTRY_ACCESS_OP] ? "w" : "-")
                //                     << ", ";

                if ((d = cmd->findDataType(entry.data_type))) {
                    cout << d->name;
                } else {
                    cout << "type " << setw(4) << entry.data_type;
                }

                cout << ", " << dec << entry.bit_length << " bit, \""
                     << entry.description << "\"" << endl;
            }
        }

    }

    delete(cmd);
}

// For the ecrt.h we need to setup a ec_pdo_entry_reg_t array.
// This registrates all pdo's by :  ecrt_domain_reg_pdo_entry_list(domain1, pdo_vec.data());
// Gives us a updated offset_vec wich has the slave offset values for read & write operations.
inline void generate_pdo_entry_regs() {
    // Clear the PDO vector before populating
    pdo_vec.clear();
    pdo_name_vec.clear();
    int count=0;
    unsigned int bits=0;

    // Iterate over all EtherCAT devices and their PDO entries
    for (size_t i = 0; i < ethercat_device_vec.size(); ++i) {
        auto& device = ethercat_device_vec[i];

        for (size_t j = 0; j < device.sync_vec.size(); ++j) {
            const auto& sync = device.sync_vec[j];

            for (size_t k = 0; k < sync.pdo_vec.size(); ++k) {
                const auto& pdo = sync.pdo_vec[k];

                for (size_t l = 0; l < pdo.entry_vec.size(); ++l) {
                    const auto& entry = pdo.entry_vec[l];

                    // Count up the bits.
                    unsigned int bitlen = entry.entry.bit_length;
                    bits+=bitlen;
                    if(k==0){ // Reset on start of slave.
                        bits=0;
                    }

                    if (bits % 8 == 0) { // Add's a new entry if bits/8==0, or at start of pdo vec.
                        // 64/8=true, 16/8=true etc.
                        ec_pdo_entry_reg_t pdo_entry = {
                            device.slave.alias,  // Somehow slave alias was 1, source fixed.
                            device.slave.position,
                            device.slave.vendor_id,
                            device.slave.product_code,
                            entry.entry.index,
                            entry.entry.subindex,
                            &device.offset,
                            &device.bit_position
                        };
                        count++;
                        // Add the PDO entry to the vector
                        pdo_vec.push_back(pdo_entry);
                        pdo_name_vec.push_back(device.slave.name);
                    }
                }
            }
        }
    }

    // Add an empty entry to mark the end of the list
    // Hmm this gives lcnc dump core when using 1 or 2 EK2124's. Its ok when using >2 devices.
    // ec_pdo_entry_reg_t end_marker = {};
    // pdo_vec.push_back(end_marker);
}

// For every pdo that has a out- or in-put value, setup pins.
// Ethercat uses 4 data types for read & write values.
inline void setup_hal_pins(int comp_id){

    hal_pin_name_vec.clear();
    int use_short_hal_pin_base_name = true;

    // We have 4 loops to iterate over all ethercat devices and their data.
    for (size_t  i = 0; i < ethercat_device_vec.size(); i++) {
        for (size_t  j = 0; j < ethercat_device_vec[i].sync_vec.size(); j++) {
            for (size_t  k = 0; k < ethercat_device_vec[i].sync_vec[j].pdo_vec.size(); k++) {
                for (size_t l = 0; l < ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec.size(); l++) {

                    const ethercat_device_data& device = ethercat_device_vec[i];
                    std::string name = device.slave.name;

                    /*
                    uint8_t state =  device.slave.al_state;
                    std::cout<<"device status bit0 slave state init:"<< ((state & 0x01) != 0) <<std::endl;
                    std::cout<<"device status bit1 slave state pre op:"<< ((state & 0x02) != 0) <<std::endl;
                    std::cout<<"device status bit2 slave state safe op:"<< ((state & 0x04) != 0) <<std::endl;
                    std::cout<<"device status bit3 slave state op:"<< ((state & 0x08) != 0) <<std::endl;
                    std::cout<<"device status bit4 slave state reserved:"<< ((state & 0x10) != 0) <<std::endl;
                    std::cout<<"device status bit5 slave state reserved:"<< ((state & 0x20) != 0) <<std::endl;
                    std::cout<<"device status bit6 slave state error:"<< ((state & 0x40) != 0) <<std::endl;
                    std::cout<<"device status bit7 slave state emergency"<< ((state & 0x80) != 0) <<std::endl;
                    */

                    // Print state.
                    for(int i = 7; i>=0; i--){
                        // std::cout<<"device status bit:"<<i<<" "<< ((state >> i) & 1) <<std::endl;
                    }

                    //uint16_t sdo_count = ethercat_device_vec[i].slave.sdo_count;
                    //uint16_t position = ethercat_device_vec[i].slave.position;
                    //ec_ioctl_slave_sdo_t sdo = ethercat_device_vec[i].slave.name
                    // ethercat_device_vec[i].slave.

                    // Remove white spaces " " to become a valid shared memory string.
                    name.erase(std::remove_if(name.begin(), name.end(), [](char c) { return c == ' '; }), name.end());

                    // Replace all '.' and ',' and '-' with '_'.
                    name = replace_special_characters_by_underscore(name);

                    // Shorten name, so it fits the shared memory string size;
                    if (use_short_hal_pin_base_name) {
                        name = shorten_string_from_end(name, 6); // 6 is like : EL2124
                    }

                    std::stringstream ss;
                    ss << "ecat.";
                    ss << name;
                    ss << "_";
                    ss << i; // Device position.
                    ss << "_";
                    ss << k; // Pdo position.
                    ss << "_";
                    ss << l; // Entry position.
                    ss << "_";

                    std::cout<<"pdo name:"<<name<<" device position:"<<i<<" pdo pos:"<<k<<" entry pos:"<<l<<std::endl;

                    hal_pin_dir_t hal_dir;
                    std::string hal_dir_str="";

                    // Pin direction.
                    const ethercat_pdo_type& pdo_type = ethercat_device_vec[i].sync_vec[j].pdo_vec[k].pdoType;
                    if(pdo_type==READ){
                        ss << "IN";
                        hal_dir=HAL_IN;
                        hal_dir_str="IN";

                    }
                    if(pdo_type==TRANSMIT){
                        ss << "OUT";
                        hal_dir=HAL_OUT;
                        hal_dir_str="OUT";
                    }

                    ss << "_";

                    // Calculate the memory offset for the entry
                    const ethercat_data_type& data_type = ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec[l].data_type;

                    const  ec_ioctl_slave_sync_pdo_entry_t& entry = ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec[l].entry;
                    std::cout<<"entry bitlenght:"<<entry.bit_length<<std::endl;
                    std::cout<<"entry pos:"<<entry.entry_pos<<std::endl;
                    std::cout<<"entry index:"<<entry.index<<std::endl;
                    std::cout<<"entry name:"<<entry.name<<std::endl;
                    std::cout<<"entry pdos:"<<entry.pdo_pos<<std::endl;
                    std::cout<<"entry slave position:"<<entry.slave_position<<std::endl;
                    std::cout<<"entry subindex:"<<entry.subindex<<std::endl;
                    std::cout<<"entry sync_index:"<<entry.sync_index<<std::endl;



                    if (data_type == BIT) {
                        std::cout<<"BIT DATAAAAAAAAAAAAAAAAAAAA"<<std::endl;
                    }

                    // Insert memory block into shared memory (as before)
                    if (data_type == BOOL) {
                        ss << "BOOL";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        bit_data_t *bit;
                        bit = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
                        int r=hal_pin_bit_new(name,hal_dir,&(bit->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin bit new failed"<<std::endl;
                        }
                        *bit->Pin = 0;
                        std::cout<<"setup hal pin type : bool, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *bit->Pin <<std::endl;

                        // shmem_ptr->insert_memory_block(ss.str(), bool_value);
                    } else if (data_type == UINT8) {
                        ss << "UINT8";
                        std::string str = ss.str();
                        const char* name = str.c_str();
                        u32_data_t *u32;
                        u32 = (u32_data_t*)hal_malloc(sizeof(u32_data_t));
                        int r=hal_pin_u32_new(name,hal_dir,&(u32->Pin),comp_id);

                        if(r!=0){
                            std::cout<<"hal pin u32 new failed"<<std::endl;
                        }
                        *u32->Pin = 0;
                        std::cout<<"setup hal pin type : u8, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *u32->Pin <<std::endl;


                    } else if (data_type == UINT16) {
                        ss << "UINT16";
                        std::string str = ss.str();
                        const char* name = str.c_str();
                        u32_data_t *u32;
                        u32 = (u32_data_t*)hal_malloc(sizeof(u32_data_t));
                        int r=hal_pin_u32_new(name,hal_dir,&(u32->Pin),comp_id);

                        if(r!=0){
                            std::cout<<"hal pin u32 new failed"<<std::endl;
                        }
                        *u32->Pin = 0;
                        std::cout<<"setup hal pin type : u16, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *u32->Pin <<std::endl;


                    } else if (data_type == UINT32) {
                        ss << "UINT32";
                        std::string str = ss.str();
                        const char* name = str.c_str();
                        u32_data_t *u32;
                        u32 = (u32_data_t*)hal_malloc(sizeof(u32_data_t));
                        int r=hal_pin_u32_new(name,hal_dir,&(u32->Pin),comp_id);

                        if(r!=0){
                            std::cout<<"hal pin u32 new failed"<<std::endl;
                        }
                        *u32->Pin = 0;
                        std::cout<<"setup hal pin type : u32, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *u32->Pin <<std::endl;

                    } else if (data_type == UINT64) {
                        ss << "UINT64";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        u64_data_t *u64;
                        u64 = (u64_data_t*)hal_malloc(sizeof(u64_data_t));
                        int r=hal_pin_u64_new(name,hal_dir,&(u64->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin u64 new failed"<<std::endl;
                        }
                        *u64->Pin = 0;
                        std::cout<<"setup hal pin type : u64, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *u64->Pin <<std::endl;


                    } else if (data_type == SINT8) {
                        ss << "SINT8";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        s32_data_t *s32;
                        s32 = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
                        int r=hal_pin_s32_new(name,hal_dir,&(s32->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin s32 new failed"<<std::endl;
                        }
                        *s32->Pin = 0;
                        std::cout<<"setup hal pin type : s8, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *s32->Pin <<std::endl;

                    } else if (data_type == SINT16) {
                        ss << "SINT16";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        s32_data_t *s32;
                        s32 = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
                        int r=hal_pin_s32_new(name,hal_dir,&(s32->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin s32 new failed"<<std::endl;
                        }
                        *s32->Pin = 0;
                        std::cout<<"setup hal pin type : s16, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *s32->Pin <<std::endl;

                    } else if (data_type == SINT32) {
                        ss << "SINT32";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        s32_data_t *s32;
                        s32 = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
                        int r=hal_pin_s32_new(name,hal_dir,&(s32->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin s32 new failed"<<std::endl;
                        }
                        *s32->Pin = 0;
                        std::cout<<"setup hal pin type : s32, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *s32->Pin <<std::endl;

                    } else if (data_type == SINT64) {
                        ss << "SINT64";

                        std::string str = ss.str();
                        const char* name = str.c_str();
                        s64_data_t *s64;
                        s64 = (s64_data_t*)hal_malloc(sizeof(s64_data_t));
                        int r=hal_pin_s64_new(name,hal_dir,&(s64->Pin),comp_id);
                        if(r!=0){
                            std::cout<<"hal pin s64 new failed"<<std::endl;
                        }
                        *s64->Pin = 0;
                        std::cout<<"setup hal pin type : s64, name:"<<ss.str()<<" hal dir:"<<hal_dir_str<<" value:"<< *s64->Pin <<std::endl;


                    } else if (data_type == STRING) {
                        ss << "STRING";
                        std::string string_value = "";

                        std::cout<<"setup hal pin type : string, name:"<<ss.str()<<" value:"<<string_value<<std::endl;
                        // shmem_ptr->insert_memory_block(ss.str(), string_value);
                    } else if (data_type == BIT) {
                        ss << "BIT";
                        uint8_t bit_value = 0x01;

                        std::cout<<"setup hal pin type : bit, name:"<<ss.str()<<" value:"<<bit_value<<std::endl;
                        // shmem_ptr->insert_memory_block(ss.str(), bit_value);
                    }

                    hal_pin_name_vec.push_back(ss.str());
                }
            }
        }
    }
}

// Print the created hal pins.
void print_hal_pins(){

    std::cout<<"halpin names:"<<std::endl;
    std::cout<<std::endl;
    for(const auto& pin : hal_pin_name_vec){
        std::cout<<"\t "<<pin<<std::endl;
    }
    std::cout<<std::endl;
}

// Print the pdo regions.
void print_pdo_entry_regs() {

    int i=0;
    std::cout<<std::endl;
    std::cout<<"pdo entry registration:"<<std::endl;
    std::cout<<std::endl;
    for (const auto& entry : pdo_vec) {

        if(pdo_name_vec[i]==""){
            std::cout << "\t Name: empty" << std::endl;
        } else {
            std::cout << "\t Name:" << pdo_name_vec[i] << std::endl;
        }

        // Print the details of each PDO entry
        std::cout << "\t Slave Alias: " << entry.alias << std::endl;
        std::cout << "\t Position: " << entry.position << std::endl;
        std::cout << "\t Vendor ID: " << entry.vendor_id << std::endl;
        std::cout << "\t Product Code: #x" << hex << setfill('0') << setw(8) << entry.product_code << std::endl;
        std::cout << "\t PDO Entry Index: #x" <<  hex << setfill('0') << setw(4) << entry.index << std::endl;
        std::cout << "\t PDO Entry Subindex: " << dec << (unsigned int) entry.subindex << std::endl;
        std::cout << std::endl;
        i++;
    }
}

// Periodicly update hal pins. These involves read and write operations.
inline void update_hal_pins(){

    int count=0;
    int pdo_vec_count=-1;
    unsigned int bits=0; // Position of pdo index.

    // We have 4 loops to iterate over all ethercat devices and their data.
    for (int i = 0; i < ethercat_device_vec.size(); i++) {
        const auto& device = ethercat_device_vec[i];



        uint8_t state =  device.slave.al_state;
//        std::cout<<"device name:"<<device.slave.name<<std::endl;
//        std::cout<<"device status bit0 slave state init:"<< ((state & 0x01) != 0) <<std::endl;
//        std::cout<<"device status bit1 slave state pre op:"<< ((state & 0x02) != 0) <<std::endl;
//        std::cout<<"device status bit2 slave state safe op:"<< ((state & 0x04) != 0) <<std::endl;
//        std::cout<<"device status bit3 slave state op:"<< ((state & 0x08) != 0) <<std::endl;
//        std::cout<<"device status bit4 slave state reserved:"<< ((state & 0x10) != 0) <<std::endl;
//        std::cout<<"device status bit5 slave state reserved:"<< ((state & 0x20) != 0) <<std::endl;
//        std::cout<<"device status bit6 slave state error:"<< ((state & 0x40) != 0) <<std::endl;
//        std::cout<<"device status bit7 slave state emergency"<< ((state & 0x80) != 0) <<std::endl;
//        std::cout<<""<<std::endl;

//        count++;

        for (int j = 0; j < ethercat_device_vec[i].sync_vec.size(); j++) {
            const auto& sync = device.sync_vec[j];

            for (int k = 0; k < ethercat_device_vec[i].sync_vec[j].pdo_vec.size(); k++) {
                const auto& pdo = sync.pdo_vec[k];

                for (int l = 0; l < ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec.size(); l++) {
                    const auto& entry = pdo.entry_vec[l];

                    // Count up the bits.
                    unsigned int bitlen = ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec[l].entry.bit_length;
                    bits+=bitlen;
                    if(k==0){ // Reset on start of slave.
                        bits=0;
                    }

                    if (bits % 8 == 0) {    // Add's a new entry if bits/8==0, or at start of pdo vec.
                        // 64/8=true, 16/8=true etc.
                        pdo_vec_count++;
                    }

                    const std::string& hal_pin_name=hal_pin_name_vec[count];

                    // Calculate the memory offset for the entry
                    const ethercat_data_type& data_type  = ethercat_device_vec[i].sync_vec[j].pdo_vec[k].entry_vec[l].data_type;

                    // Pin direction.
                    const ethercat_pdo_type& pdo_type =  ethercat_device_vec[i].sync_vec[j].pdo_vec[k].pdoType;

                    // Hal related.
                    const char *name = hal_pin_name.c_str();
                    hal_type_t type = HAL_TYPE_UNINITIALIZED;
                    hal_data_u *ptr;
                    bool conn;

                    int r=hal_get_pin_value_by_name(name,&type,&ptr,&conn);
                    if(r==-1){
                        std::cout<<"hal pin error"<<std::endl;
                    }

                    // rtapi_mutex_get(&(hal_data->mutex));

                    // Read hal pin status and write to bus.
                    if (data_type == BOOL && pdo_type==READ) { // EL2124 write pin.
                        EC_WRITE_BIT(domain1_pd + device.offset, bits, ptr->b);
                    }
                    // Read bus status and write to hal.
                    if (data_type == BOOL && pdo_type==TRANSMIT) {
                        ptr->b = EC_READ_BIT(domain1_pd + device.offset, bits);
                    }

                    // U types.
                    if (data_type == BIT && pdo_type==READ) {
                        EC_WRITE_U8(domain1_pd + device.offset, ptr->b);
                    }
                    if (data_type == BIT && pdo_type==TRANSMIT) {
                        ptr->b = EC_READ_U8(domain1_pd + device.offset);
                    }

                    if (data_type == UINT8 && pdo_type==READ) {
                        EC_WRITE_U32(domain1_pd + device.offset, ptr->u);

                    }
                    if (data_type == UINT8 && pdo_type==TRANSMIT) {
                        ptr->u = EC_READ_U32(domain1_pd + device.offset);
                    }

                    if (data_type == UINT16 && pdo_type==READ) {
                        EC_WRITE_U32(domain1_pd + device.offset, ptr->u);
                    }
                    if (data_type == UINT16 && pdo_type==TRANSMIT) {
                        ptr->u = EC_READ_U32(domain1_pd + device.offset);
                    }

                    if (data_type == UINT32 && pdo_type==READ) {
                        EC_WRITE_U32(domain1_pd + device.offset, ptr->u);
                    }
                    if (data_type == UINT32 && pdo_type==TRANSMIT) {
                        ptr->u = EC_READ_U32(domain1_pd + device.offset);
                    }

                    if (data_type == UINT64 && pdo_type==READ) {
                        EC_WRITE_U64(domain1_pd + device.offset, ptr->lu);
                    }
                    if (data_type == UINT64 && pdo_type==TRANSMIT) {
                        ptr->lu = EC_READ_U64(domain1_pd + device.offset);
                    }


                    // S types.
                    if (data_type == SINT8 && pdo_type==READ) {
                        EC_WRITE_S32(domain1_pd + device.offset, ptr->s);
                    }
                    if (data_type == SINT8 && pdo_type==TRANSMIT) {
                        ptr->s = EC_READ_S32(domain1_pd + device.offset);
                    }

                    if (data_type == SINT16 && pdo_type==READ) {
                        EC_WRITE_S32(domain1_pd + device.offset, ptr->s);
                    }
                    if (data_type == SINT16 && pdo_type==TRANSMIT) {
                        ptr->s = EC_READ_S32(domain1_pd + device.offset);
                    }

                    if (data_type == SINT32 && pdo_type==READ) {
                        EC_WRITE_S32(domain1_pd + device.offset, ptr->s);
                    }
                    if (data_type == SINT32 && pdo_type==TRANSMIT) {
                        ptr->s = EC_READ_S32(domain1_pd + device.offset);
                    }

                    if (data_type == SINT64 && pdo_type==READ) {
                        EC_WRITE_S64(domain1_pd + device.offset, ptr->ls);
                    }
                    if (data_type == SINT64 && pdo_type==TRANSMIT) {
                        ptr->ls = EC_READ_S64(domain1_pd + device.offset);
                    }

                    // Todo..
                    if (data_type == STRING && pdo_type==READ) {
                        uint8_t ethercat_data[64];
                        std::string string_value;
                        //  shmem_ptr->get_memory_block(hal_pin_name, string_value);
                        EC_WRITE_STRING(domain1_pd + device.offset, string_value.c_str(), sizeof(ethercat_data));
                    }
                    if (data_type == STRING && pdo_type==TRANSMIT) {
                        // Buffer to hold the string (up to 64 bytes, including null-terminator)
                        char buffer[64];
                        // Read the string from the EtherCAT process data
                        EC_READ_STRING(domain1_pd + device.offset, buffer, sizeof(buffer));
                        // Convert the C-style string to a C++ std::string
                        std::string value = buffer;
                        // Update the shared memory block with the string value
                        //  shmem_ptr->update_memory_block(hal_pin_name, value);
                    }

                    // rtapi_mutex_give(&(hal_data->mutex));

                    count++;
                }
            }
        }
    }
}

// Get the shared memory region for this library to use.
// This is the same function as : int rtapi_app_main(void);
extern "C" int init_ecat(int comp_id, int debug_xml) {

    std::cout << "init ethercat start." << std::endl;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    std::cout<<"Print ethercat devices and retrieve device data vector."<<std::endl;
    get_ethercat_devices(debug_xml);
    generate_pdo_entry_regs();
    print_pdo_entry_regs();
    setup_hal_pins(comp_id);
    print_hal_pins();

    if (ecrt_domain_reg_pdo_entry_list(domain1, pdo_vec.data() )) {
        std::cout<<"PDO entry registration failed!"<<std::endl;
        return -1;
    }
    std::cout<<"PDO entry registration ok."<<std::endl;

    if (ecrt_master_activate(master)) {
        std::cout<<"Master failed."<<std::endl;
        return -1;
    }
    std::cout<<"Master ok."<<std::endl;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        std::cout<<"domain1_pd fails."<<std::endl;
        return -1;
    }
    std::cout<<"domain1_pd ok."<<std::endl;

    std::cout << "init ethercat succes." << std::endl;
    std::cout << std::endl;

    std::cout<<"generating ethercat sdos."<<std::endl;
    // get_ethercat_sdos();
    std::cout<<"ethercat sdos ok."<<std::endl;

    return 0;
}

// Unload stuff.
// This is the same function as : void rtapi_app_exit(void);
extern "C" void stop_ecat() {
    std::cout<<"stop function called from ethercat."<<std::endl;
}

// Periodicly update function by pthread_t
// Return -1 on error to inform function_loader.
extern "C" int run_ecat() {

    // uint64_t apptime = EC_TIMEVAL2NANO(tv);
    // std::cout<<"running ethercat cycle."<<std::endl;

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    update_hal_pins();

    // write process data
    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    // How to synchronize ethercat bus with lcnc?
    // ecrt_master_application_time(master, 100000);
    // ecrt_master_sync_reference_clock(master);
    // ecrt_master_sync_slave_clocks(master);

    return 0;
}

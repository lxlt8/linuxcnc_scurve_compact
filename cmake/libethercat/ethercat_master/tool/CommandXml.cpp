/*****************************************************************************
 *
 *  Copyright (C) 2006-2014  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 ****************************************************************************/

#include <iostream>
#include <iomanip>
#include <string.h>
using namespace std;

#include "CommandXml.h"
#include "MasterDevice.h"

/****************************************************************************/

CommandXml::CommandXml():
    Command("xml", "Generate slave information XML.")
{
}

/****************************************************************************/

string CommandXml::helpString(const string &binaryBaseName) const
{
    stringstream str;

    str << binaryBaseName << " " << getName() << " [OPTIONS]" << endl
        << endl
        << getBriefDescription() << endl
        << endl
        << "Note that the PDO information can either originate" << endl
        << "from the SII or from the CoE communication area. For" << endl
        << "slaves, that support configuring PDO assignment and" << endl
        << "mapping, the output depends on the last configuration." << endl
        << endl
        << "Command-specific options:" << endl
        << "  --alias    -a <alias>" << endl
        << "  --position -p <pos>    Slave selection. See the help of" << endl
        << "                         the 'slaves' command." << endl
        << endl
        << numericInfo();

    return str.str();
}

/****************************************************************************/

void CommandXml::execute(const StringVector &args)
{
    SlaveList slaves;
    SlaveList::const_iterator si;

    if (args.size()) {
        stringstream err;
        err << "'" << getName() << "' takes no arguments!";
        throwInvalidUsageException(err);
    }

    MasterDevice m(getSingleMasterIndex());
    m.open(MasterDevice::Read);
    slaves = selectedSlaves(m);

    cout << "<?xml version=\"1.0\" ?>" << endl;

    bool is_multi_slave = slaves.size() > 1;
    if (is_multi_slave) {
        cout << "<EtherCATInfoList>" << endl;
    }

    // Align aliases and generate slave XML
    uint16_t alias = slaves.front().alias;
    for (auto& slave : slaves) {
        slave.alias = alias; // Ensure all slaves use the same alias
        generateSlaveXml(m, slave, is_multi_slave);
    }

    // XML Footer
    if (is_multi_slave) {
        cout << "</EtherCATInfoList>" << endl;
    }
}

std::vector<ethercat_device_data> CommandXml::ethercat_devices_data(int debug){

    SlaveList slaves;
    SlaveList::const_iterator si;

    MasterDevice m(getSingleMasterIndex());
    m.open(MasterDevice::Read);
    slaves = selectedSlaves(m);

    const ec_ioctl_slave_t &upper_slave=slaves.front();

    std::vector<ethercat_device_data> slave_data_vec;

    // Populate the vector with data from each slave
    bool is_multi_slave = slaves.size() > 1;

    for (auto& slave : slaves) {
        slave.alias = upper_slave.alias;  // Ensure alias matches upper_slave
        slave_data_vec.push_back(generateSlaveData(m, slave, is_multi_slave, debug));
    }

    // Debug output
    if (is_multi_slave && debug) {
        cout << "</EtherCATInfoList>" << endl;
    }

    return slave_data_vec;
}

/****************************************************************************/

ethercat_device_data CommandXml::generateSlaveData(
    MasterDevice &m,
    const ec_ioctl_slave_t &slave,
    unsigned int indent, int debug
    )
{
    ec_ioctl_slave_sync_t sync;
    ec_ioctl_slave_sync_pdo_t pdo;
    string pdoType, in;
    ec_ioctl_slave_sync_pdo_entry_t entry;
    unsigned int i, j, k;

    ethercat_device_data data;
    data.slave=slave;

    if(debug){
        for (i = 0; i < indent; i++) {
            in += "  ";
        }

        cout
            << in << "<EtherCATInfo>" << endl
            << in << "  <!-- Slave alias " << dec << slave.alias << " -->" << endl
            << in << "  <!-- Slave position " << dec << slave.position << " -->" << endl
            << in << "  <Vendor>" << endl
            << in << "    <Id>" << slave.vendor_id << "</Id>" << endl
            << in << "  </Vendor>" << endl
            << in << "  <Descriptions>" << endl
            << in << "    <Devices>" << endl
            << in << "      <Device>" << endl
            << in << "        <Type ProductCode=\"#x"
            << hex << setfill('0') << setw(8) << slave.product_code
            << "\" RevisionNo=\"#x"
            << hex << setfill('0') << setw(8) << slave.revision_number
            << "\">" << slave.order << "</Type>" << endl;


        if (strlen(slave.name)) {
            cout
                << in << "        <Name><![CDATA["
                << slave.name
                << "]]></Name>" << endl;
        }

        for (i = 0; i < slave.sync_count; i++) {
            m.getSync(&sync, slave.position, i);

            cout
                << in << "        <Sm Enable=\""
                << dec << (unsigned int) sync.enable
                << "\" StartAddress=\"#x" << hex << sync.physical_start_address
                << "\" ControlByte=\"#x"
                << hex << (unsigned int) sync.control_register
                << "\" DefaultSize=\"" << dec << sync.default_size
                << "\" />" << endl;
        }
    }

    for (i = 0; i < slave.sync_count; i++) {
        m.getSync(&sync, slave.position, i);

        // Add to vector the sync's, empty pdo data inserted here.
        data.sync_vec.push_back({{sync},{/*pdo_vec*/}});

        for (j = 0; j < sync.pdo_count; j++) {
            m.getPdo(&pdo, slave.position, i, j);

            // Add to the sync's vector, the pdo vec, empty pdoType, empty pdo entry's.
            data.sync_vec.back().pdo_vec.push_back({pdo,NONE_PDO,{/* empty entry vec */}});

            pdoType = (sync.control_register & 0x04 ? "R" : "T");
            // We don't need this:
            // pdoType += "xPdo"; // last 2 letters lowercase in XML!

            if(pdoType=="R"){
                // Add to the pdo vec, the pdo type.
                data.sync_vec.back().pdo_vec.back().pdoType=READ;
            }
            if(pdoType=="T"){
                // Add to the pdo vec, the pdo type.
                data.sync_vec.back().pdo_vec.back().pdoType=TRANSMIT;
            }

            if(debug){
                cout
                    << in << "        <" << pdoType
                    << " Sm=\"" << i << "\" Fixed=\"1\" Mandatory=\"1\">" << endl
                    << in << "          <Index>#x"
                    << hex << setfill('0') << setw(4) << pdo.index
                    << "</Index>" << endl
                    << in << "          <Name>" << pdo.name << "</Name>" << endl;
            }

            for (k = 0; k < pdo.entry_count; k++) {
                m.getPdoEntry(&entry, slave.position, i, j, k);

                // Add to pdo vec, the pdo entry's, add default 0 for data type.
                data.sync_vec.back().pdo_vec.back().entry_vec.push_back({entry,NONE_DATA});

                if(debug){
                    cout
                        << in << "          <Entry>" << endl
                        << in << "            <Index>#x"
                        << hex << setfill('0') << setw(4) << entry.index
                        << "</Index>" << endl;
                    if (entry.index)
                        cout
                            << in << "            <SubIndex>"
                            << dec << (unsigned int) entry.subindex
                            << "</SubIndex>" << endl;

                    cout
                        << in << "            <BitLen>"
                        << dec << (unsigned int) entry.bit_length
                        << "</BitLen>" << endl;
                }
                if (entry.index) {

                    if(debug){
                        cout
                            << in << "            <Name>" << entry.name
                            << "</Name>" << endl
                            << in << "            <DataType>";
                    }

                    if (entry.bit_length == 1) {
                        if(debug){
                            cout << "BOOL";
                        }

                        // Set data type.
                        data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=BOOL;

                    } else if (!(entry.bit_length % 8)) {
                        if (entry.bit_length <= 64) {
                            if(debug){
                                cout << "UINT" << (unsigned int) entry.bit_length;
                            }
                            // Set data type's for uint.
                            if(entry.bit_length==8){
                                data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=UINT8;
                            }
                            if(entry.bit_length==16){
                                data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=UINT16;
                            }
                            if(entry.bit_length==32){
                                data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=UINT32;
                            }
                            if(entry.bit_length==64){
                                data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=UINT64;
                            }

                        } else {
                            if(debug){
                                cout << "STRING("
                                     << (unsigned int) (entry.bit_length / 8)
                                     << ")";
                            }
                            // Set data type.
                            data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=STRING;
                        }
                    } else {
                        if(debug){
                            cout << "BIT" << (unsigned int) entry.bit_length;
                        }
                        // Set data type.
                        data.sync_vec.back().pdo_vec.back().entry_vec.back().data_type=BIT;
                    }
                    if(debug){
                        cout << "</DataType>" << endl;
                    }
                }
                if(debug){
                    cout << in << "          </Entry>" << endl;
                }
            }
            if(debug){
                cout
                    << in << "        </" << pdoType << ">" << endl;
            }
        }
    }

    if(debug){
        cout
            << in << "      </Device>" << endl
            << in << "    </Devices>" << endl
            << in << "  </Descriptions>" << endl
            << in << "</EtherCATInfo>" << endl;
    }



    return data;
}

/****************************************************************************/

void CommandXml::generateSlaveXml(
    MasterDevice &m,
    const ec_ioctl_slave_t &slave,
    unsigned int indent
    )
{
    ec_ioctl_slave_sync_t sync;
    ec_ioctl_slave_sync_pdo_t pdo;
    string pdoType, in;
    ec_ioctl_slave_sync_pdo_entry_t entry;
    unsigned int i, j, k;

    for (i = 0; i < indent; i++) {
        in += "  ";
    }

    cout
        << in << "<EtherCATInfo>" << endl
        << in << "  <!-- Slave alias " << dec << slave.alias << " -->" << endl
        << in << "  <!-- Slave position " << dec << slave.position << " -->" << endl
        << in << "  <Vendor>" << endl
        << in << "    <Id>" << slave.vendor_id << "</Id>" << endl
        << in << "  </Vendor>" << endl
        << in << "  <Descriptions>" << endl
        << in << "    <Devices>" << endl
        << in << "      <Device>" << endl
        << in << "        <Type ProductCode=\"#x"
        << hex << setfill('0') << setw(8) << slave.product_code
        << "\" RevisionNo=\"#x"
        << hex << setfill('0') << setw(8) << slave.revision_number
        << "\">" << slave.order << "</Type>" << endl;

    if (strlen(slave.name)) {
        cout
            << in << "        <Name><![CDATA["
            << slave.name
            << "]]></Name>" << endl;
    }

    for (i = 0; i < slave.sync_count; i++) {
        m.getSync(&sync, slave.position, i);

        cout
            << in << "        <Sm Enable=\""
            << dec << (unsigned int) sync.enable
            << "\" StartAddress=\"#x" << hex << sync.physical_start_address
            << "\" ControlByte=\"#x"
            << hex << (unsigned int) sync.control_register
            << "\" DefaultSize=\"" << dec << sync.default_size
            << "\" />" << endl;
    }

    for (i = 0; i < slave.sync_count; i++) {
        m.getSync(&sync, slave.position, i);

        for (j = 0; j < sync.pdo_count; j++) {
            m.getPdo(&pdo, slave.position, i, j);
            pdoType = (sync.control_register & 0x04 ? "R" : "T");
            pdoType += "xPdo"; // last 2 letters lowercase in XML!

            cout
                << in << "        <" << pdoType
                << " Sm=\"" << i << "\" Fixed=\"1\" Mandatory=\"1\">" << endl
                << in << "          <Index>#x"
                << hex << setfill('0') << setw(4) << pdo.index
                << "</Index>" << endl
                << in << "          <Name>" << pdo.name << "</Name>" << endl;

            for (k = 0; k < pdo.entry_count; k++) {
                m.getPdoEntry(&entry, slave.position, i, j, k);

                cout
                    << in << "          <Entry>" << endl
                    << in << "            <Index>#x"
                    << hex << setfill('0') << setw(4) << entry.index
                    << "</Index>" << endl;
                if (entry.index)
                    cout
                        << in << "            <SubIndex>"
                        << dec << (unsigned int) entry.subindex
                        << "</SubIndex>" << endl;

                cout
                    << in << "            <BitLen>"
                    << dec << (unsigned int) entry.bit_length
                    << "</BitLen>" << endl;

                if (entry.index) {
                    cout
                        << in << "            <Name>" << entry.name
                        << "</Name>" << endl
                        << in << "            <DataType>";

                    if (entry.bit_length == 1) {
                        cout << "BOOL";
                    } else if (!(entry.bit_length % 8)) {
                        if (entry.bit_length <= 64) {
                            cout << "UINT" << (unsigned int) entry.bit_length;
                        } else {
                            cout << "STRING("
                                 << (unsigned int) (entry.bit_length / 8)
                                 << ")";
                        }
                    } else {
                        cout << "BIT" << (unsigned int) entry.bit_length;
                    }

                    cout << "</DataType>" << endl;
                }

                cout << in << "          </Entry>" << endl;
            }

            cout
                << in << "        </" << pdoType << ">" << endl;
        }
    }

    cout
        << in << "      </Device>" << endl
        << in << "    </Devices>" << endl
        << in << "  </Descriptions>" << endl
        << in << "</EtherCATInfo>" << endl;
}

/****************************************************************************/

#include <iostream>
#include <iomanip>
#include <string.h>

#include "CommandXml.h"
#include "MasterDevice.h"

enum ethercat_pdo_type {
    NONE_PDO,
    READ,
    TRANSMIT
};

enum ethercat_data_type {
    NONE_DATA,
    BOOL,
    UINT,
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    SINT8,
    SINT16,
    SINT32,
    SINT64,
    STRING,
    BIT
};

struct ethercat_entry_data {
    ec_ioctl_slave_sync_pdo_entry_t entry;
    ethercat_data_type data_type;
};

// Pdo entry's of a pdo.
struct ethercat_pdo_data {
    ec_ioctl_slave_sync_pdo_t pdo;
    ethercat_pdo_type pdoType;
    std::vector<ethercat_entry_data> entry_vec;
};

// Pdo's of a sync.
struct ethercat_sync_data {
    ec_ioctl_slave_sync_t sync;
    std::vector<ethercat_pdo_data> pdo_vec;
};

// Device and sync's
struct ethercat_device_data {
    ec_ioctl_slave_t slave;
    std::vector<ethercat_sync_data> sync_vec;

    unsigned int offset;
    unsigned int bit_position;
};

/** Write a string up to 64 bytes to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param STR  Null-terminated string
 * \param MAX_LEN Maximum length of the string to write (up to 64 bytes)
 */
#define EC_WRITE_STRING(DATA, STR, MAX_LEN) \
do { \
        size_t _len = strnlen((STR), (MAX_LEN)); \
        memcpy((DATA), (STR), _len); \
        if (_len < (MAX_LEN)) { \
            *((char *)(DATA) + _len) = '\0'; /* Null-terminate */ \
    } \
} while (0)

/** Read a string up to 64 bytes from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param BUF  Buffer to store the string
 * \param MAX_LEN Maximum length of the buffer
 */
#define EC_READ_STRING(DATA, BUF, MAX_LEN) \
    do { \
        strncpy((BUF), (const char *)(DATA), (MAX_LEN)); \
        (BUF)[(MAX_LEN) - 1] = '\0'; /* Ensure null-termination */ \
} while (0)

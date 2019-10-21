/** 
   @file mailbox.h
   @brief don't know yet

Copyright (c) 2012, Broadcom Europe Ltd.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <linux/ioctl.h>
#include <stdint.h>
// Newer kernels (>= 4.1) use major 249, older ones major 100.
/** new kernel version is (>= 4.1) */
#define MAJOR_NUM_A 249
/** older kernel version */
#define MAJOR_NUM_B 100
/** value of the device dependent request code for the ioctl command */
#define IOCTL_MBOX_PROPERTY _IOWR(MAJOR_NUM_B, 0, char *)
/** name of the file sent to open command in mbox_open function */
#define DEVICE_FILE_NAME "/dev/vcio"
/** alternative file name used in mbox_open if DEVICE_FILE_NAME didn't work */
#define LOCAL_DEVICE_FILE_NAME "/tmp/mbox"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief open a char device file used for communicated with the kernel mbox driver. A value of -1 returned from this fuction means that it was unable to open/create the kernal mbox device file
 *  @return int - the file handle descriptor
 */
int mbox_open();
/** @brief close the file descriptor
 *  @param file_desc - the number returned by mbox_open
 *  @return Void.
 */
void mbox_close(int file_desc);

/** @brief there is no body for this function
 *  @param file_desc - the number returned by mbox_open
 *  @return uint32_t
 */
uint32_t get_version(int file_desc);
/** @brief write an array out to mbox_property. Calls exit if this fails
 *  @param file_desc - the number returned by mbox_open
 *  @param size - number of bytes or pages?
 *  @param align - alignment
 *  @param flags - MEM_FLAG_L1_NONALLOCATING?
 *  @return uint32_t - returns size
 */
uint32_t mem_alloc(int file_desc, uint32_t size, uint32_t align, uint32_t flags);
/** @brief write an array out to mbox_proptery.
 *  @param file_desc - the number returned by mbox_open
 *  @param handle - value written to mbox_property
 *  @return uint32_t - either value of handle for success or 0 for error
 */
uint32_t mem_free(int file_desc, uint32_t handle);
/** @brief write an array out to mbox_property. Calls exit if this fails
 *  @param file_desc - the number returned by mbox_open
 *  @param handle - value written to mbox_property
 *  @return uint32_t - value of handle 
 */
uint32_t mem_lock(int file_desc, uint32_t handle);
/** @brief write an array out to mbox_proptery.
 *  @param file_desc - the number returned by mbox_open
 *  @param handle - value written to mbox_property
 *  @return uint32_t - either value of handle for success or 0 for error
 */
uint32_t mem_unlock(int file_desc, uint32_t handle);
/** @brief open a read/write/sync file at the start of page where base is located and make the file size
 *  @param base - the base address of the file
 *  @param size - the size of the file
 *  @return (void *) - pointer to the spot in memory at the address base
 */
void *mapmem(uint32_t base, uint32_t size);
/** @brief call munmap on *addr and return NULL. Call exit if it fails
 *  @param addr - the address to pass to munmap
 *  @param size - the size to pass to munmap
 *  @return (void *) - always NULL
 */
void *unmapmem(void *addr, uint32_t size);

/** @brief write an array out to mbox_proptery.
 *  @param file_desc - the number returned by mbox_open
 *  @param code - value written to mbox_property
 *  @param r0
 *  @param r1
 *  @param r2
 *  @param r3
 *  @param r4
 *  @param r5
 *  @return uint32_t - either value of code for success or 0 for error
 */
uint32_t execute_code(int file_desc, uint32_t code, uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3, uint32_t r4, uint32_t r5);
/** @brief write an array out to mbox_proptery.
 *  @param file_desc - the number returned by mbox_open
 *  @param enable - value written to mbox_property
 *  @return uint32_t - either value of enable for success or 0 for error
 */
uint32_t qpu_enable(int file_desc, uint32_t enable);
/** @brief write an array out to mbox_proptery.
 *  @param file_desc - the number returned by mbox_open
 *  @param num_qpus
 *  @param control
 *  @param noflush
 *  @param timeout
 *  @return uint32_t - either value of num_qpus for success or 0 for error
 */
uint32_t execute_qpu(int file_desc, uint32_t num_qpus, uint32_t control, uint32_t noflush, uint32_t timeout);
#ifdef __cplusplus
};
#endif

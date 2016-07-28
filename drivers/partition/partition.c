/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <debug.h>
#include <io_storage.h>
#include <partition/partition.h>
#include <platform.h>
#include <string.h>

#define MAX_PARTITION_ENTRIES		128

static int part_num_entries;
static uint8_t mbr_sector[PARTITION_BLOCK_SIZE];

partition_entry_t entry_list[PLAT_PARTITION_MAX_ENTRIES];

static int verify_partition_mbr(uintptr_t image_handle)
{
	size_t bytes_read;
	mbr_entry_t *entry;
	int result;

	/* MBR partition table is in LBA0. */
	result = io_seek(image_handle, IO_SEEK_SET, MBR_OFFSET);
	if (result != 0) {
		WARN("Failed to seek (%i)\n", result);
		return result;
	}
	result = io_read(image_handle, (uintptr_t)&mbr_sector,
			 PARTITION_BLOCK_SIZE, &bytes_read);
	if (result != 0) {
		WARN("Failed to read data (%i)\n", result);
		return result;
	}

	/* Check MBR boot signature. */
	if ((mbr_sector[PARTITION_BLOCK_SIZE - 2] != MBR_SIGNATURE_FIRST) ||
	    (mbr_sector[PARTITION_BLOCK_SIZE - 1] != MBR_SIGNATURE_SECOND)) {
		return -EINVAL;
	}
	/*
	 * Check the first partition entry, since we only support a
	 * "protective" MBR on a disk with a GPT.
	 */
	entry = (mbr_entry_t *)((uintptr_t)&mbr_sector +
				MBR_PRIMARY_ENTRY_OFFSET);
	if (entry->type == PARTITION_TYPE_GPT)
		return -EAGAIN;
	return -EINVAL;
}

#if LOG_LEVEL >= LOG_LEVEL_INFO
static void dump_entries(int num)
{
	char name[EFI_NAMELEN];
	int i, j, len;

	INFO("Partition table with %d entries:\n", num);
	for (i = 0; i < num; i++) {
		len = snprintf(name, EFI_NAMELEN, "%s", entry_list[i].name);
		for (j = 0; j < EFI_NAMELEN - len; j++) {
			name[len + j] = ' ';
		}
		INFO("%d: %s %lx-%lx\n", i + 1, name, entry_list[i].start,
		     entry_list[i].start + entry_list[i].length - 4);
	}
}
#else
#define dump_entries(int num)	((void)0)
#endif

static int verify_partition_gpt(uintptr_t image_handle)
{
	gpt_header_t header;
	gpt_entry_t entry;
	size_t bytes_read;
	int result, i;

	result = io_seek(image_handle, IO_SEEK_SET, GPT_HEADER_OFFSET);
	if (result != 0) {
		return result;
	}
	result = io_read(image_handle, (uintptr_t)&header,
			 sizeof(gpt_header_t), &bytes_read);
	if ((result != 0) || (sizeof(gpt_header_t) != bytes_read)) {
		return result;
	}
	if (memcmp(header.signature, GPT_SIGNATURE,
		   sizeof(header.signature)) != 0) {
		return -EINVAL;
	}
	part_num_entries = header.part_num;
	if (part_num_entries > MAX_PARTITION_ENTRIES) {
		return -EINVAL;
	}
	if (part_num_entries > PLAT_PARTITION_MAX_ENTRIES) {
		part_num_entries = PLAT_PARTITION_MAX_ENTRIES;
	}

	result = io_seek(image_handle, IO_SEEK_SET, GPT_ENTRY_OFFSET);
	if (result != 0) {
		return result;
	}
	for (i = 0; i < part_num_entries; i++) {
		result = io_read(image_handle, (uintptr_t)&entry,
				 sizeof(gpt_entry_t), &bytes_read);
		if (result || (sizeof(gpt_entry_t) != bytes_read)) {
			return result;
		}
		result = parse_gpt_entry(&entry, i);
		if (result != 0) {
			break;
		}
	}
	if (i == 0) {
		return -EINVAL;
	}
	part_num_entries = i;
	dump_entries(part_num_entries);

	return 0;
}

int load_partition_table(unsigned int image_id)
{
	uintptr_t dev_handle, image_handle, image_spec = 0;
	int result;

	result = plat_get_image_source(image_id, &dev_handle, &image_spec);
	if (result != 0) {
		WARN("Failed to obtain reference to image id=%u (%i)\n",
			image_id, result);
		return result;
	}

	result = io_open(dev_handle, image_spec, &image_handle);
	if (result != 0) {
		WARN("Failed to access image id=%u (%i)\n", image_id, result);
		return result;
	}

	result = verify_partition_mbr(image_handle);
	if (result != 0) {
		INFO("Can't find MBR partition table (%i)\n", result);
		if (result != -EAGAIN)
			goto exit;
	} else {
		/*
		 * Find MBR signature. Need to check further for the GPT.
		 * Since we only support a "protective" MBR with a GPT.
		 */
		goto exit;
	}

	result = verify_partition_gpt(image_handle);
	if (result != 0) {
		INFO("Can't find GPT partition table (%i)\n", result);
	}
exit:
	io_close(image_handle);
	return result;
}

const partition_entry_t *get_partition_entry(const char *name)
{
	int i;

	for (i = 0; i < part_num_entries; i++) {
		if (strcmp(name, entry_list[i].name) == 0) {
			return &entry_list[i];
		}
	}
	return NULL;
}

const partition_entry_t *get_partition_entry_list(int *num_entries)
{
	assert(num_entries != NULL);
	*num_entries = part_num_entries;
	return entry_list;
}

void partition_init(unsigned int image_id)
{
	load_partition_table(image_id);
}

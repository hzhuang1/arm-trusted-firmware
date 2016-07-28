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

#ifndef __PARTITION_H__
#define __PARTITION_H__

#include <types.h>

#define MAX_PARTITION_NUM		128
#define EFI_NAMELEN			36
#define GUID_LEN			16
#define PARTITION_BLOCK_SIZE		512

#define MBR_OFFSET			0
#define GPT_HEADER_OFFSET		PARTITION_BLOCK_SIZE

#define PARTITION_TYPE_GPT		0xee

#define MBR_PRIMARY_ENTRY_OFFSET	0x1be
#define MBR_PRIMARY_ENTRY_SIZE		0x10
#define MBR_PRIMARY_ENTRY_NUMBER	4
#define MBR_CHS_ADDRESS_LEN		3

#define GPT_SIGNATURE			"EFI PART"
#define GPT_ENTRY_OFFSET		(GPT_HEADER_OFFSET +		\
					 PARTITION_BLOCK_SIZE)

typedef struct partition_entry {
	uint64_t		start;
	uint64_t		length;
	unsigned int		flags;
	unsigned int		load_addr;
	unsigned int		load_size;
	int			id;
	char			name[EFI_NAMELEN];
} part_entry_t;

typedef struct partition_ops {
	int (*handler)(const char *name, part_entry_t *entry);
} partition_ops_t;

typedef struct partition_mbr_entry {
	uint8_t			status;
	uint8_t			first_sector[MBR_CHS_ADDRESS_LEN];
	uint8_t			type;
	uint8_t			last_sector[MBR_CHS_ADDRESS_LEN];
	uint32_t		first_lba;
	uint32_t		sector_nums;
} mbr_entry_t;

typedef struct partition_gpt_entry {
	uint8_t			type_uuid[GUID_LEN];
	uint8_t			unique_uuid[GUID_LEN];
	uint64_t		first_lba;
	uint64_t		last_lba;
	uint64_t		attr;
	uint16_t		name[EFI_NAMELEN];
} gpt_entry_t;

typedef struct gpt_header {
	char			signature[8];
	uint32_t		revision;
	uint32_t		size;
	uint32_t		header_crc;
	uint32_t		reserved;
	uint64_t		current_lba;
	uint64_t		backup_lba;
	uint64_t		first_lba;
	uint64_t		last_lba;
	uint8_t			disk_uuid[16];
	/* starting LBA of array of partition entries */
	uint64_t		part_lba;
	/* number of partition entries in array */
	uint32_t		part_num;
	/* size of a single partition entry (usually 128) */
	uint32_t		part_size;
	uint32_t		part_crc;
} gpt_header_t;


int load_partition_table(unsigned int image_id);
int get_partition_entry(const char *name, part_entry_t *entry);
void get_partition_entry_list(part_entry_t **list, int *num_entries);
void partition_init(unsigned int image_id, const partition_ops_t *ops);

#endif	/* __PARTITION_GPT_H__ */

/*
 * (C) Copyright 2013
 * Max Nekludov <macscomp@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

/*
 * SHA1-RSA signature check
 */

#define DEBUG

#include <common.h>
#include <config.h>
#include <command.h>
#include <tomcrypt.h>

#define SIG_SIZE 128
#define HASH_SIZE 20
#define MAX_PK_SIZE 512

#ifdef DEBUG
inline void print_hex(char *src, int src_size)
{
	int x;
	for (x = 0; x < src_size; x++)
		printf("%.2x", (unsigned char)src[x] );
	printf("\n");
}
#endif

static __attribute__ ((noinline)) int check_signature(void* buf, int size, void *key, int key_size)
{
	void *signature;
	void *data;
	int data_size;
	int ret;
	int sha1_index;
	int result = 0;
	char hash_result[HASH_SIZE];
	rsa_key pub_key;
	hash_state hash;

#ifdef DEBUG
	printf("Checking signature [size = %i, key_size = %i]\n", size, key_size);
#endif

	if( size <= SIG_SIZE )
		goto err;

	signature = buf;
	data = (char*)buf + SIG_SIZE;
	data_size = size - SIG_SIZE;
	ltc_mp = ltm_desc;

	sha1_index = register_hash(&sha1_desc);

	ret = rsa_import(key, key_size, &pub_key);
	if( ret )
	{
#ifdef DEBUG
		printf("Fail to import public key %i\n", ret);
#endif
		goto err;
	}

	sha1_init(&hash);
	sha1_process(&hash, data, data_size);
	sha1_done(&hash, hash_result);

#ifdef DEBUG
	printf("Data (first 40 bytes of %i bytes):\n", data_size);
	print_hex(data, HASH_SIZE);

	printf("SHA1 hash:\n");
	print_hex(hash_result, HASH_SIZE);

	printf("Signature:\n");
	print_hex(signature, SIG_SIZE);
#endif

	ret = rsa_verify_hash_ex(signature, SIG_SIZE, hash_result, HASH_SIZE,
	                         LTC_LTC_PKCS_1_V1_5, sha1_index, 0, &result, &pub_key);
	if( ret )
	{
#ifdef DEBUG
		printf("Fail to check signature %i\n", ret);
#endif
		goto err;
	}

#ifdef DEBUG
	if( result )
		printf("Verify OK\n");
	else
		printf("Verify FAIL\n");
#endif

	if( !result )
		goto err;

	return 0;

err:
	memset(buf, 0, size);
	return -1;
}

static int fs_load_file(char *cmd, int addr, const char *name)
{
	cmd_tbl_t *cmdtp;
	char *argv[4];
	char saddr[16];
	int ret;

	cmdtp = find_cmd(cmd);
	if( cmdtp == 0 )
	{
#ifdef DEBUG
		printf("Fail to find command %s\n", cmd);
#endif
		return -1;
	}

	sprintf(saddr, "0x%X", addr);

	argv[0] = cmd;
	argv[1] = saddr;
	argv[2] = name;
	argv[3] = 0;

	ret = cmdtp->cmd(cmdtp, 0, 3, argv);
	if( ret )
		return -1;

	ret = simple_strtoul(getenv("filesize"), NULL, 16);

	return ret;
}

static int load_file(int addr, const char* name)
{
	if( strncmp(name, "ubifs://", 8) == 0 )
		return fs_load_file("ubifsload", addr, name + 8);
	else if( strncmp(name, "cramfs://", 9) == 0 )
		return fs_load_file("cramfsload", addr, name + 9);
	else
	{
#ifdef DEBUG
		printf("Unknown filesystem of file %s\n", name);
#endif
		return -1;
	}
}

int do_secure_load(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	u32 addr;
	char *endp;
	int ret;
	void *pub_key;
	int pub_key_size;

	if( argc != 4 )
		goto err;

	addr = simple_strtoul(argv[1], &endp, 16);
	if (endp == argv[1])
		goto err;

	ret = load_file(addr, argv[3]);
	if( ret < 0 )
	{
#ifdef DEBUG
		printf("Fail to load %s\n", argv[3]);
#endif
		goto err;
	}
	pub_key_size = ret;

	if( pub_key_size > MAX_PK_SIZE )
	{
#ifdef DEBUG
		printf("Too large public key %i\n", pub_key_size);
#endif
		goto err;
	}

	pub_key = malloc(pub_key_size);
	memcpy(pub_key, (void*)addr, pub_key_size);

	ret = load_file(addr, argv[2]);
	if( ret < 0 )
	{
		free(pub_key);
#ifdef DEBUG
		printf("Fail to load %s\n", argv[2]);
#endif
		goto err;
	}

	ret = check_signature((void*)addr, ret, pub_key, pub_key_size);
	if( ret )
		goto err;

	free(pub_key);

	return 0;

err:
#ifdef DEBUG
	printf("Tampering possible! Halt the system.\n");
#endif
	while(1) {} // software tampering DO NOT EXIT!
	return 0;
}

U_BOOT_CMD(
	secld, 4, 0, do_secure_load,
	"load file and check SHA1-RSA signature",
	"<addr> <file> <pub_key>\n"
	"    - load <file> to <addr> and check signature with <pub_key>."
);

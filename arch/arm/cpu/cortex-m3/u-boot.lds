/*
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

MEMORY
{
	ROM :   ORIGIN = 0,          LENGTH = 512K
	SRAM :  ORIGIN = 0x20000000, LENGTH = 96K
	SDRAM : ORIGIN = 0x60000000, LENGTH = 8M
}

REGION_ALIAS("REGION_TEXT", ROM);
REGION_ALIAS("REGION_RODATA", ROM);
REGION_ALIAS("REGION_DATA", SRAM);
REGION_ALIAS("REGION_BSS", SRAM);

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = ALIGN(4);
	.text	:
	{
		__u_boot_code_start = .;
		*(.text)
		__u_boot_code_end = .;
	} > REGION_TEXT

	. = ALIGN(4);
	.dynsym :
	{
		*(.dynsym)
	} > REGION_TEXT

	. = ALIGN(4);
	.rel :
	{
		*(.rel*)
	} > REGION_TEXT

	. = ALIGN(4);
	.got :
	{
		*(.got*)
	} > REGION_TEXT

	. = ALIGN(4);
	.hash :
	{
		*(.hash)
	} > REGION_TEXT

	. = ALIGN(4);
	.u_boot_cmd :
	{
		__u_boot_cmd_start = .;
		*(.u_boot_cmd)
		__u_boot_cmd_end = .;
	} > REGION_RODATA

	. = ALIGN(4);
	.rodata :
	{
		*(.rodata*)

		. = ALIGN(4);
		__u_boot_rodata_end = .;
	} > REGION_RODATA

	. = ALIGN(4);
	.data : AT (__u_boot_rodata_end)
	{
		__u_boot_data_start = .;
		*(.data*)
		. = ALIGN(4);
		__u_boot_data_end = .;
	} > REGION_DATA

	__u_boot_data_load_start = LOADADDR(.data);

	. = ALIGN(4);
	.bss :
	{
		__u_boot_bss_start = .;
		*(.bss)
		. = ALIGN(4);
		__u_boot_bss_end = .;
	} > REGION_BSS

	/DISCARD/ : { *(.dynstr*) }
	/DISCARD/ : { *(.dynamic*) }
	/DISCARD/ : { *(.plt*) }
	/DISCARD/ : { *(.interp*) }
	/DISCARD/ : { *(.gnu*) }
	/DISCARD/ : { *(.ARM*) }
}

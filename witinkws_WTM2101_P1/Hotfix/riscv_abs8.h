/* vim: set fileencoding=utf-8:
 *
 *                   GNU GENERAL PUBLIC LICENSE
 *                       Version 2, June 1991
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 * 
 *
 */
#ifndef RISCV_ABS8_H_16ED48D2_7E1B_5ACC_FE6A_A3EDEB4FB027_INCLUDED_
#define RISCV_ABS8_H_16ED48D2_7E1B_5ACC_FE6A_A3EDEB4FB027_INCLUDED_

// patch-2024.05.28
// yl, riscv optimized fast abs:8bits
// input range : -128 .. 127
// output range: 0..127
// begin
__STATIC_FORCEINLINE int8_t __abs8(int8_t v)
{
    int32_t  input = v;
    int32_t  result1;
    uint32_t result2;
    int8_t   result3;

    __ASM volatile("kslliw %0, %1, 24" : "=r"(result1) : "r"(input));
    __ASM volatile("kabsw  %0, %1"     : "=r"(result2) : "r"(result1));
    __ASM volatile("srli   %0, %1, 24" : "=r"(result3) : "r"(result2));
    return result3;
}
// end

#endif


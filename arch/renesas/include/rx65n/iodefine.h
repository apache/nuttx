/*****************************************************************************
 * arch/renesas/include/rx65n/iodefine.h
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __RX65NIODEFINE_HEADER__
#define __RX65NIODEFINE_HEADER__

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define IEN_BSC_BUSERR          IEN0
#define IEN_RAM_RAMERR          IEN2
#define IEN_FCU_FIFERR          IEN5
#define IEN_FCU_FRDYI           IEN7
#define IEN_ICU_SWINT2          IEN2
#define IEN_ICU_SWINT           IEN3
#define IEN_CMT0_CMI0           IEN4
#define IEN_CMT1_CMI1           IEN5
#define IEN_CMTW0_CMWI0         IEN6
#define IEN_CMTW1_CMWI1         IEN7
#define IEN_USB0_D0FIFO0        IEN2
#define IEN_USB0_D1FIFO0        IEN3
#define IEN_RSPI0_SPRI0         IEN6
#define IEN_RSPI0_SPTI0         IEN7
#define IEN_RSPI1_SPRI1         IEN0
#define IEN_RSPI1_SPTI1         IEN1
#define IEN_QSPI_SPRI           IEN2
#define IEN_QSPI_SPTI           IEN3
#define IEN_SDHI_SBFAI          IEN4
#define IEN_MMCIF_MBFAI         IEN5
#define IEN_RIIC1_RXI1          IEN2
#define IEN_RIIC1_TXI1          IEN3
#define IEN_RIIC0_RXI0          IEN4
#define IEN_RIIC0_TXI0          IEN5
#define IEN_RIIC2_RXI2          IEN6
#define IEN_RIIC2_TXI2          IEN7
#define IEN_SCI0_RXI0           IEN2
#define IEN_SCI0_TXI0           IEN3
#define IEN_SCI1_RXI1           IEN4
#define IEN_SCI1_TXI1           IEN5
#define IEN_SCI2_RXI2           IEN6
#define IEN_SCI2_TXI2           IEN7
#define IEN_ICU_IRQ0            IEN0
#define IEN_ICU_IRQ1            IEN1
#define IEN_ICU_IRQ2            IEN2
#define IEN_ICU_IRQ3            IEN3
#define IEN_ICU_IRQ4            IEN4
#define IEN_ICU_IRQ5            IEN5
#define IEN_ICU_IRQ6            IEN6
#define IEN_ICU_IRQ7            IEN7
#define IEN_ICU_IRQ8            IEN0
#define IEN_ICU_IRQ9            IEN1
#define IEN_ICU_IRQ10           IEN2
#define IEN_ICU_IRQ11           IEN3
#define IEN_ICU_IRQ12           IEN4
#define IEN_ICU_IRQ13           IEN5
#define IEN_ICU_IRQ14           IEN6
#define IEN_ICU_IRQ15           IEN7
#define IEN_SCI3_RXI3           IEN0
#define IEN_SCI3_TXI3           IEN1
#define IEN_SCI4_RXI4           IEN2
#define IEN_SCI4_TXI4           IEN3
#define IEN_SCI5_RXI5           IEN4
#define IEN_SCI5_TXI5           IEN5
#define IEN_SCI6_RXI6           IEN6
#define IEN_SCI6_TXI6           IEN7
#define IEN_LVD1_LVD1           IEN0
#define IEN_LVD2_LVD2           IEN1
#define IEN_USB0_USBR0          IEN2
#define IEN_RTC_ALM             IEN4
#define IEN_RTC_PRD             IEN5
#define IEN_IWDT_IWUNI          IEN7
#define IEN_WDT_WUNI            IEN0
#define IEN_PDC_PCDFI           IEN1
#define IEN_SCI7_RXI7           IEN2
#define IEN_SCI7_TXI7           IEN3
#define IEN_SCI8_RXI8           IEN4
#define IEN_SCI8_TXI8           IEN5
#define IEN_SCI9_RXI9           IEN6
#define IEN_SCI9_TXI9           IEN7
#define IEN_SCI10_RXI10         IEN0
#define IEN_SCI10_TXI10         IEN1
#define IEN_ICU_GROUPBE0        IEN2
#define IEN_ICU_GROUPBL2        IEN3
#define IEN_RSPI2_SPRI2         IEN4
#define IEN_RSPI2_SPTI2         IEN5
#define IEN_ICU_GROUPBL0        IEN6
#define IEN_ICU_GROUPBL1        IEN7
#define IEN_ICU_GROUPAL0        IEN0
#define IEN_ICU_GROUPAL1        IEN1
#define IEN_SCI11_RXI11         IEN2
#define IEN_SCI11_TXI11         IEN3
#define IEN_SCI12_RXI12         IEN4
#define IEN_SCI12_TXI12         IEN5
#define IEN_DMAC_DMAC0I         IEN0
#define IEN_DMAC_DMAC1I         IEN1
#define IEN_DMAC_DMAC2I         IEN2
#define IEN_DMAC_DMAC3I         IEN3
#define IEN_DMAC_DMAC74I        IEN4
#define IEN_OST_OSTDI           IEN5
#define IEN_EXDMAC_EXDMAC0I     IEN6
#define IEN_EXDMAC_EXDMAC1I     IEN7
#define IEN_PERIB_INTB128       IEN0
#define IEN_PERIB_INTB129       IEN1
#define IEN_PERIB_INTB130       IEN2
#define IEN_PERIB_INTB131       IEN3
#define IEN_PERIB_INTB132       IEN4
#define IEN_PERIB_INTB133       IEN5
#define IEN_PERIB_INTB134       IEN6
#define IEN_PERIB_INTB135       IEN7
#define IEN_PERIB_INTB136       IEN0
#define IEN_PERIB_INTB137       IEN1
#define IEN_PERIB_INTB138       IEN2
#define IEN_PERIB_INTB139       IEN3
#define IEN_PERIB_INTB140       IEN4
#define IEN_PERIB_INTB141       IEN5
#define IEN_PERIB_INTB142       IEN6
#define IEN_PERIB_INTB143       IEN7
#define IEN_PERIB_INTB144       IEN0
#define IEN_PERIB_INTB145       IEN1
#define IEN_PERIB_INTB146       IEN2
#define IEN_PERIB_INTB147       IEN3
#define IEN_PERIB_INTB148       IEN4
#define IEN_PERIB_INTB149       IEN5
#define IEN_PERIB_INTB150       IEN6
#define IEN_PERIB_INTB151       IEN7
#define IEN_PERIB_INTB152       IEN0
#define IEN_PERIB_INTB153       IEN1
#define IEN_PERIB_INTB154       IEN2
#define IEN_PERIB_INTB155       IEN3
#define IEN_PERIB_INTB156       IEN4
#define IEN_PERIB_INTB157       IEN5
#define IEN_PERIB_INTB158       IEN6
#define IEN_PERIB_INTB159       IEN7
#define IEN_PERIB_INTB160       IEN0
#define IEN_PERIB_INTB161       IEN1
#define IEN_PERIB_INTB162       IEN2
#define IEN_PERIB_INTB163       IEN3
#define IEN_PERIB_INTB164       IEN4
#define IEN_PERIB_INTB165       IEN5
#define IEN_PERIB_INTB166       IEN6
#define IEN_PERIB_INTB167       IEN7
#define IEN_PERIB_INTB168       IEN0
#define IEN_PERIB_INTB169       IEN1
#define IEN_PERIB_INTB170       IEN2
#define IEN_PERIB_INTB171       IEN3
#define IEN_PERIB_INTB172       IEN4
#define IEN_PERIB_INTB173       IEN5
#define IEN_PERIB_INTB174       IEN6
#define IEN_PERIB_INTB175       IEN7
#define IEN_PERIB_INTB176       IEN0
#define IEN_PERIB_INTB177       IEN1
#define IEN_PERIB_INTB178       IEN2
#define IEN_PERIB_INTB179       IEN3
#define IEN_PERIB_INTB180       IEN4
#define IEN_PERIB_INTB181       IEN5
#define IEN_PERIB_INTB182       IEN6
#define IEN_PERIB_INTB183       IEN7
#define IEN_PERIB_INTB184       IEN0
#define IEN_PERIB_INTB185       IEN1
#define IEN_PERIB_INTB186       IEN2
#define IEN_PERIB_INTB187       IEN3
#define IEN_PERIB_INTB188       IEN4
#define IEN_PERIB_INTB189       IEN5
#define IEN_PERIB_INTB190       IEN6
#define IEN_PERIB_INTB191       IEN7
#define IEN_PERIB_INTB192       IEN0
#define IEN_PERIB_INTB193       IEN1
#define IEN_PERIB_INTB194       IEN2
#define IEN_PERIB_INTB195       IEN3
#define IEN_PERIB_INTB196       IEN4
#define IEN_PERIB_INTB197       IEN5
#define IEN_PERIB_INTB198       IEN6
#define IEN_PERIB_INTB199       IEN7
#define IEN_PERIB_INTB200       IEN0
#define IEN_PERIB_INTB201       IEN1
#define IEN_PERIB_INTB202       IEN2
#define IEN_PERIB_INTB203       IEN3
#define IEN_PERIB_INTB204       IEN4
#define IEN_PERIB_INTB205       IEN5
#define IEN_PERIB_INTB206       IEN6
#define IEN_PERIB_INTB207       IEN7
#define IEN_PERIA_INTA208       IEN0
#define IEN_PERIA_INTA209       IEN1
#define IEN_PERIA_INTA210       IEN2
#define IEN_PERIA_INTA211       IEN3
#define IEN_PERIA_INTA212       IEN4
#define IEN_PERIA_INTA213       IEN5
#define IEN_PERIA_INTA214       IEN6
#define IEN_PERIA_INTA215       IEN7
#define IEN_PERIA_INTA216       IEN0
#define IEN_PERIA_INTA217       IEN1
#define IEN_PERIA_INTA218       IEN2
#define IEN_PERIA_INTA219       IEN3
#define IEN_PERIA_INTA220       IEN4
#define IEN_PERIA_INTA221       IEN5
#define IEN_PERIA_INTA222       IEN6
#define IEN_PERIA_INTA223       IEN7
#define IEN_PERIA_INTA224       IEN0
#define IEN_PERIA_INTA225       IEN1
#define IEN_PERIA_INTA226       IEN2
#define IEN_PERIA_INTA227       IEN3
#define IEN_PERIA_INTA228       IEN4
#define IEN_PERIA_INTA229       IEN5
#define IEN_PERIA_INTA230       IEN6
#define IEN_PERIA_INTA231       IEN7
#define IEN_PERIA_INTA232       IEN0
#define IEN_PERIA_INTA233       IEN1
#define IEN_PERIA_INTA234       IEN2
#define IEN_PERIA_INTA235       IEN3
#define IEN_PERIA_INTA236       IEN4
#define IEN_PERIA_INTA237       IEN5
#define IEN_PERIA_INTA238       IEN6
#define IEN_PERIA_INTA239       IEN7
#define IEN_PERIA_INTA240       IEN0
#define IEN_PERIA_INTA241       IEN1
#define IEN_PERIA_INTA242       IEN2
#define IEN_PERIA_INTA243       IEN3
#define IEN_PERIA_INTA244       IEN4
#define IEN_PERIA_INTA245       IEN5
#define IEN_PERIA_INTA246       IEN6
#define IEN_PERIA_INTA247       IEN7
#define IEN_PERIA_INTA248       IEN0
#define IEN_PERIA_INTA249       IEN1
#define IEN_PERIA_INTA250       IEN2
#define IEN_PERIA_INTA251       IEN3
#define IEN_PERIA_INTA252       IEN4
#define IEN_PERIA_INTA253       IEN5
#define IEN_PERIA_INTA254       IEN6
#define IEN_PERIA_INTA255       IEN7

#define VECT_BSC_BUSERR         16
#define VECT_RAM_RAMERR         18
#define VECT_FCU_FIFERR         21
#define VECT_FCU_FRDYI          23
#define VECT_ICU_SWINT2         26
#define VECT_ICU_SWINT          27
#define VECT_CMT0_CMI0          28
#define VECT_CMT1_CMI1          29
#define VECT_CMTW0_CMWI0        30
#define VECT_CMTW1_CMWI1        31
#define VECT_USB0_D0FIFO0       34
#define VECT_USB0_D1FIFO0       35
#define VECT_RSPI0_SPRI0        38
#define VECT_RSPI0_SPTI0        39
#define VECT_RSPI1_SPRI1        40
#define VECT_RSPI1_SPTI1        41
#define VECT_QSPI_SPRI          42
#define VECT_QSPI_SPTI          43
#define VECT_SDHI_SBFAI         44
#define VECT_MMCIF_MBFAI        45
#define VECT_RIIC1_RXI1         50
#define VECT_RIIC1_TXI1         51
#define VECT_RIIC0_RXI0         52
#define VECT_RIIC0_TXI0         53
#define VECT_RIIC2_RXI2         54
#define VECT_RIIC2_TXI2         55
#define VECT_SCI0_RXI0          58
#define VECT_SCI0_TXI0          59
#define VECT_SCI1_RXI1          60
#define VECT_SCI1_TXI1          61
#define VECT_SCI2_RXI2          62
#define VECT_SCI2_TXI2          63
#define VECT_ICU_IRQ0           64
#define VECT_ICU_IRQ1           65
#define VECT_ICU_IRQ2           66
#define VECT_ICU_IRQ3           67
#define VECT_ICU_IRQ4           68
#define VECT_ICU_IRQ5           69
#define VECT_ICU_IRQ6           70
#define VECT_ICU_IRQ7           71
#define VECT_ICU_IRQ8           72
#define VECT_ICU_IRQ9           73
#define VECT_ICU_IRQ10          74
#define VECT_ICU_IRQ11          75
#define VECT_ICU_IRQ12          76
#define VECT_ICU_IRQ13          77
#define VECT_ICU_IRQ14          78
#define VECT_ICU_IRQ15          79
#define VECT_SCI3_RXI3          80
#define VECT_SCI3_TXI3          81
#define VECT_SCI4_RXI4          82
#define VECT_SCI4_TXI4          83
#define VECT_SCI5_RXI5          84
#define VECT_SCI5_TXI5          85
#define VECT_SCI6_RXI6          86
#define VECT_SCI6_TXI6          87
#define VECT_LVD1_LVD1          88
#define VECT_LVD2_LVD2          89
#define VECT_USB0_USBR0         90
#define VECT_RTC_ALM            92
#define VECT_RTC_PRD            93
#define VECT_IWDT_IWUNI         95
#define VECT_WDT_WUNI           96
#define VECT_PDC_PCDFI          97
#define VECT_SCI7_RXI7          98
#define VECT_SCI7_TXI7          99
#define VECT_SCI8_RXI8          100
#define VECT_SCI8_TXI8          101
#define VECT_SCI9_RXI9          102
#define VECT_SCI9_TXI9          103
#define VECT_SCI10_RXI10        104
#define VECT_SCI10_TXI10        105
#define VECT_ICU_GROUPBE0       106
#define VECT_ICU_GROUPBL2       107
#define VECT_RSPI2_SPRI2        108
#define VECT_RSPI2_SPTI2        109
#define VECT_ICU_GROUPBL0       110
#define VECT_ICU_GROUPBL1       111
#define VECT_ICU_GROUPAL0       112
#define VECT_ICU_GROUPAL1       113
#define VECT_SCI11_RXI11        114
#define VECT_SCI11_TXI11        115
#define VECT_SCI12_RXI12        116
#define VECT_SCI12_TXI12        117
#define VECT_DMAC_DMAC0I        120
#define VECT_DMAC_DMAC1I        121
#define VECT_DMAC_DMAC2I        122
#define VECT_DMAC_DMAC3I        123
#define VECT_DMAC_DMAC74I       124
#define VECT_OST_OSTDI          125
#define VECT_EXDMAC_EXDMAC0I    126
#define VECT_EXDMAC_EXDMAC1I    127
#define VECT_PERIB_INTB128      128
#define VECT_PERIB_INTB129      129
#define VECT_PERIB_INTB130      130
#define VECT_PERIB_INTB131      131
#define VECT_PERIB_INTB132      132
#define VECT_PERIB_INTB133      133
#define VECT_PERIB_INTB134      134
#define VECT_PERIB_INTB135      135
#define VECT_PERIB_INTB136      136
#define VECT_PERIB_INTB137      137
#define VECT_PERIB_INTB138      138
#define VECT_PERIB_INTB139      139
#define VECT_PERIB_INTB140      140
#define VECT_PERIB_INTB141      141
#define VECT_PERIB_INTB142      142
#define VECT_PERIB_INTB143      143
#define VECT_PERIB_INTB144      144
#define VECT_PERIB_INTB145      145
#define VECT_PERIB_INTB146      146
#define VECT_PERIB_INTB147      147
#define VECT_PERIB_INTB148      148
#define VECT_PERIB_INTB149      149
#define VECT_PERIB_INTB150      150
#define VECT_PERIB_INTB151      151
#define VECT_PERIB_INTB152      152
#define VECT_PERIB_INTB153      153
#define VECT_PERIB_INTB154      154
#define VECT_PERIB_INTB155      155
#define VECT_PERIB_INTB156      156
#define VECT_PERIB_INTB157      157
#define VECT_PERIB_INTB158      158
#define VECT_PERIB_INTB159      159
#define VECT_PERIB_INTB160      160
#define VECT_PERIB_INTB161      161
#define VECT_PERIB_INTB162      162
#define VECT_PERIB_INTB163      163
#define VECT_PERIB_INTB164      164
#define VECT_PERIB_INTB165      165
#define VECT_PERIB_INTB166      166
#define VECT_PERIB_INTB167      167
#define VECT_PERIB_INTB168      168
#define VECT_PERIB_INTB169      169
#define VECT_PERIB_INTB170      170
#define VECT_PERIB_INTB171      171
#define VECT_PERIB_INTB172      172
#define VECT_PERIB_INTB173      173
#define VECT_PERIB_INTB174      174
#define VECT_PERIB_INTB175      175
#define VECT_PERIB_INTB176      176
#define VECT_PERIB_INTB177      177
#define VECT_PERIB_INTB178      178
#define VECT_PERIB_INTB179      179
#define VECT_PERIB_INTB180      180
#define VECT_PERIB_INTB181      181
#define VECT_PERIB_INTB182      182
#define VECT_PERIB_INTB183      183
#define VECT_PERIB_INTB184      184
#define VECT_PERIB_INTB185      185
#define VECT_PERIB_INTB186      186
#define VECT_PERIB_INTB187      187
#define VECT_PERIB_INTB188      188
#define VECT_PERIB_INTB189      189
#define VECT_PERIB_INTB190      190
#define VECT_PERIB_INTB191      191
#define VECT_PERIB_INTB192      192
#define VECT_PERIB_INTB193      193
#define VECT_PERIB_INTB194      194
#define VECT_PERIB_INTB195      195
#define VECT_PERIB_INTB196      196
#define VECT_PERIB_INTB197      197
#define VECT_PERIB_INTB198      198
#define VECT_PERIB_INTB199      199
#define VECT_PERIB_INTB200      200
#define VECT_PERIB_INTB201      201
#define VECT_PERIB_INTB202      202
#define VECT_PERIB_INTB203      203
#define VECT_PERIB_INTB204      204
#define VECT_PERIB_INTB205      205
#define VECT_PERIB_INTB206      206
#define VECT_PERIB_INTB207      207
#define VECT_PERIA_INTA208      208
#define VECT_PERIA_INTA209      209
#define VECT_PERIA_INTA210      210
#define VECT_PERIA_INTA211      211
#define VECT_PERIA_INTA212      212
#define VECT_PERIA_INTA213      213
#define VECT_PERIA_INTA214      214
#define VECT_PERIA_INTA215      215
#define VECT_PERIA_INTA216      216
#define VECT_PERIA_INTA217      217
#define VECT_PERIA_INTA218      218
#define VECT_PERIA_INTA219      219
#define VECT_PERIA_INTA220      220
#define VECT_PERIA_INTA221      221
#define VECT_PERIA_INTA222      222
#define VECT_PERIA_INTA223      223
#define VECT_PERIA_INTA224      224
#define VECT_PERIA_INTA225      225
#define VECT_PERIA_INTA226      226
#define VECT_PERIA_INTA227      227
#define VECT_PERIA_INTA228      228
#define VECT_PERIA_INTA229      229
#define VECT_PERIA_INTA230      230
#define VECT_PERIA_INTA231      231
#define VECT_PERIA_INTA232      232
#define VECT_PERIA_INTA233      233
#define VECT_PERIA_INTA234      234
#define VECT_PERIA_INTA235      235
#define VECT_PERIA_INTA236      236
#define VECT_PERIA_INTA237      237
#define VECT_PERIA_INTA238      238
#define VECT_PERIA_INTA239      239
#define VECT_PERIA_INTA240      240
#define VECT_PERIA_INTA241      241
#define VECT_PERIA_INTA242      242
#define VECT_PERIA_INTA243      243
#define VECT_PERIA_INTA244      244
#define VECT_PERIA_INTA245      245
#define VECT_PERIA_INTA246      246
#define VECT_PERIA_INTA247      247
#define VECT_PERIA_INTA248      248
#define VECT_PERIA_INTA249      249
#define VECT_PERIA_INTA250      250
#define VECT_PERIA_INTA251      251
#define VECT_PERIA_INTA252      252
#define VECT_PERIA_INTA253      253
#define VECT_PERIA_INTA254      254
#define VECT_PERIA_INTA255      255

#ifndef __ASSEMBLER__

#define MSTP_EXDMAC     SYSTEM.MSTPCRA.BIT.MSTPA29
#define MSTP_EXDMAC0    SYSTEM.MSTPCRA.BIT.MSTPA29
#define MSTP_EXDMAC1    SYSTEM.MSTPCRA.BIT.MSTPA29
#define MSTP_DMAC       SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC0      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC1      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC2      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC3      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC4      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC5      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC6      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DMAC7      SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DTC        SYSTEM.MSTPCRA.BIT.MSTPA28
#define MSTP_DA         SYSTEM.MSTPCRA.BIT.MSTPA19
#define MSTP_S12AD      SYSTEM.MSTPCRA.BIT.MSTPA17
#define MSTP_S12AD1     SYSTEM.MSTPCRA.BIT.MSTPA16
#define MSTP_CMT0       SYSTEM.MSTPCRA.BIT.MSTPA15
#define MSTP_CMT1       SYSTEM.MSTPCRA.BIT.MSTPA15
#define MSTP_CMT2       SYSTEM.MSTPCRA.BIT.MSTPA14
#define MSTP_CMT3       SYSTEM.MSTPCRA.BIT.MSTPA14
#define MSTP_TPU0       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPU1       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPU2       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPU3       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPU4       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPU5       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_TPUA       SYSTEM.MSTPCRA.BIT.MSTPA13
#define MSTP_PPG0       SYSTEM.MSTPCRA.BIT.MSTPA11
#define MSTP_PPG1       SYSTEM.MSTPCRA.BIT.MSTPA10
#define MSTP_MTU        SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU0       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU1       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU2       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU3       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU4       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU5       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU6       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU7       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_MTU8       SYSTEM.MSTPCRA.BIT.MSTPA9
#define MSTP_TMR0       SYSTEM.MSTPCRA.BIT.MSTPA5
#define MSTP_TMR1       SYSTEM.MSTPCRA.BIT.MSTPA5
#define MSTP_TMR01      SYSTEM.MSTPCRA.BIT.MSTPA5
#define MSTP_TMR2       SYSTEM.MSTPCRA.BIT.MSTPA4
#define MSTP_TMR3       SYSTEM.MSTPCRA.BIT.MSTPA4
#define MSTP_TMR23      SYSTEM.MSTPCRA.BIT.MSTPA4
#define MSTP_CMTW0      SYSTEM.MSTPCRA.BIT.MSTPA1
#define MSTP_CMTW1      SYSTEM.MSTPCRA.BIT.MSTPA0
#define MSTP_SCI0       SYSTEM.MSTPCRB.BIT.MSTPB31
#define MSTP_SMCI0      SYSTEM.MSTPCRB.BIT.MSTPB31
#define MSTP_SCI1       SYSTEM.MSTPCRB.BIT.MSTPB30
#define MSTP_SMCI1      SYSTEM.MSTPCRB.BIT.MSTPB30
#define MSTP_SCI2       SYSTEM.MSTPCRB.BIT.MSTPB29
#define MSTP_SMCI2      SYSTEM.MSTPCRB.BIT.MSTPB29
#define MSTP_SCI3       SYSTEM.MSTPCRB.BIT.MSTPB28
#define MSTP_SMCI3      SYSTEM.MSTPCRB.BIT.MSTPB28
#define MSTP_SCI4       SYSTEM.MSTPCRB.BIT.MSTPB27
#define MSTP_SMCI4      SYSTEM.MSTPCRB.BIT.MSTPB27
#define MSTP_SCI5       SYSTEM.MSTPCRB.BIT.MSTPB26
#define MSTP_SMCI5      SYSTEM.MSTPCRB.BIT.MSTPB26
#define MSTP_SCI6       SYSTEM.MSTPCRB.BIT.MSTPB25
#define MSTP_SMCI6      SYSTEM.MSTPCRB.BIT.MSTPB25
#define MSTP_SCI7       SYSTEM.MSTPCRB.BIT.MSTPB24
#define MSTP_SMCI7      SYSTEM.MSTPCRB.BIT.MSTPB24
#define MSTP_CRC        SYSTEM.MSTPCRB.BIT.MSTPB23
#define MSTP_PDC        SYSTEM.MSTPCRB.BIT.MSTPB22
#define MSTP_RIIC0      SYSTEM.MSTPCRB.BIT.MSTPB21
#define MSTP_RIIC1      SYSTEM.MSTPCRB.BIT.MSTPB20
#define MSTP_USB0       SYSTEM.MSTPCRB.BIT.MSTPB19
#define MSTP_RSPI0      SYSTEM.MSTPCRB.BIT.MSTPB17
#define MSTP_RSPI1      SYSTEM.MSTPCRB.BIT.MSTPB16
#define MSTP_EDMAC0     SYSTEM.MSTPCRB.BIT.MSTPB15
#define MSTP_ETHERC0    SYSTEM.MSTPCRB.BIT.MSTPB15
#define MSTP_ELC        SYSTEM.MSTPCRB.BIT.MSTPB9
#define MSTP_TEMPS      SYSTEM.MSTPCRB.BIT.MSTPB8
#define MSTP_DOC        SYSTEM.MSTPCRB.BIT.MSTPB6
#define MSTP_SCI12      SYSTEM.MSTPCRB.BIT.MSTPB4
#define MSTP_SMCI12     SYSTEM.MSTPCRB.BIT.MSTPB4
#define MSTP_CAN1       SYSTEM.MSTPCRB.BIT.MSTPB1
#define MSTP_CAN0       SYSTEM.MSTPCRB.BIT.MSTPB0
#define MSTP_GLCDC      SYSTEM.MSTPCRC.BIT.MSTPC29
#define MSTP_DRW2D      SYSTEM.MSTPCRC.BIT.MSTPC28
#define MSTP_SCI8       SYSTEM.MSTPCRC.BIT.MSTPC27
#define MSTP_SMCI8      SYSTEM.MSTPCRC.BIT.MSTPC27
#define MSTP_SCI9       SYSTEM.MSTPCRC.BIT.MSTPC26
#define MSTP_SMCI9      SYSTEM.MSTPCRC.BIT.MSTPC26
#define MSTP_SCI10      SYSTEM.MSTPCRC.BIT.MSTPC25
#define MSTP_SMCI10     SYSTEM.MSTPCRC.BIT.MSTPC25
#define MSTP_SCI11      SYSTEM.MSTPCRC.BIT.MSTPC24
#define MSTP_SMCI11     SYSTEM.MSTPCRC.BIT.MSTPC24
#define MSTP_QSPI       SYSTEM.MSTPCRC.BIT.MSTPC23
#define MSTP_RSPI2      SYSTEM.MSTPCRC.BIT.MSTPC22
#define MSTP_CAC        SYSTEM.MSTPCRC.BIT.MSTPC19
#define MSTP_RIIC2      SYSTEM.MSTPCRC.BIT.MSTPC17
#define MSTP_STBYRAM    SYSTEM.MSTPCRC.BIT.MSTPC7
#define MSTP_RAM2       SYSTEM.MSTPCRC.BIT.MSTPC2
#define MSTP_RAM0       SYSTEM.MSTPCRC.BIT.MSTPC0
#define MSTP_MMCIF      SYSTEM.MSTPCRD.BIT.MSTPD21
#define MSTP_SDHI       SYSTEM.MSTPCRD.BIT.MSTPD19
#define MSTP_SDSI       SYSTEM.MSTPCRD.BIT.MSTPD13

#define IS_CAN0_ERS0            IS0
#define IS_CAN1_ERS1            IS1
#define IS_SCI0_TEI0            IS0
#define IS_SCI0_ERI0            IS1
#define IS_SCI1_TEI1            IS2
#define IS_SCI1_ERI1            IS3
#define IS_SCI2_TEI2            IS4
#define IS_SCI2_ERI2            IS5
#define IS_SCI3_TEI3            IS6
#define IS_SCI3_ERI3            IS7
#define IS_SCI4_TEI4            IS8
#define IS_SCI4_ERI4            IS9
#define IS_SCI5_TEI5            IS10
#define IS_SCI5_ERI5            IS11
#define IS_SCI6_TEI6            IS12
#define IS_SCI6_ERI6            IS13
#define IS_SCI7_TEI7            IS14
#define IS_SCI7_ERI7            IS15
#define IS_SCI12_TEI12          IS16
#define IS_SCI12_ERI12          IS17
#define IS_SCI12_SCIX0          IS18
#define IS_SCI12_SCIX1          IS19
#define IS_SCI12_SCIX2          IS20
#define IS_SCI12_SCIX3          IS21
#define IS_QSPI_QSPSSLI         IS24
#define IS_CAC_FERRI            IS26
#define IS_CAC_MENDI            IS27
#define IS_CAC_OVFI             IS28
#define IS_DOC_DOPCI            IS29
#define IS_PDC_PCFEI            IS30
#define IS_PDC_PCERI            IS31
#define IS_SDHI_CDETI           IS3
#define IS_SDHI_CACI            IS4
#define IS_SDHI_SDACI           IS5
#define IS_MMCIF_CDETIO         IS6
#define IS_MMCIF_ERRIO          IS7
#define IS_MMCIF_ACCIO          IS8
#define IS_POE3_OEI1            IS9
#define IS_POE3_OEI2            IS10
#define IS_POE3_OEI3            IS11
#define IS_POE3_OEI4            IS12
#define IS_RIIC0_TEI0           IS13
#define IS_RIIC0_EEI0           IS14
#define IS_RIIC2_TEI2           IS15
#define IS_RIIC2_EEI2           IS16
#define IS_S12AD_S12CMPAI       IS20
#define IS_S12AD_S12CMPBI       IS21
#define IS_S12AD1_S12CMPAI1     IS22
#define IS_S12AD1_S12CMPBI1     IS23
#define IS_SCI8_TEI8            IS24
#define IS_SCI8_ERI8            IS25
#define IS_SCI9_TEI9            IS26
#define IS_SCI9_ERI9            IS27
#define IS_RIIC1_TEI1           IS28
#define IS_RIIC1_EEI1           IS29
#define IS_SDSI_SDIOI           IS0
#define IS_SCI10_TEI10          IS8
#define IS_SCI10_ERI10          IS9
#define IS_SCI11_TEI11          IS12
#define IS_SCI11_ERI11          IS13
#define IS_RSPI0_SPII0          IS16
#define IS_RSPI0_SPEI0          IS17
#define IS_RSPI1_SPII1          IS18
#define IS_RSPI1_SPEI1          IS19
#define IS_RSPI2_SPII2          IS20
#define IS_RSPI2_SPEI2          IS21
#define IS_EDMAC0_EINT0         IS4
#define IS_GLCDC_VPOS           IS8
#define IS_GLCDC_GR1UF          IS9
#define IS_GLCDC_GR2UF          IS10
#define IS_DRW2D_DRWIRQ         IS11

#define EN_CAN0_ERS0            EN0
#define EN_CAN1_ERS1            EN1
#define EN_SCI0_TEI0            EN0
#define EN_SCI0_ERI0            EN1
#define EN_SCI1_TEI1            EN2
#define EN_SCI1_ERI1            EN3
#define EN_SCI2_TEI2            EN4
#define EN_SCI2_ERI2            EN5
#define EN_SCI3_TEI3            EN6
#define EN_SCI3_ERI3            EN7
#define EN_SCI4_TEI4            EN8
#define EN_SCI4_ERI4            EN9
#define EN_SCI5_TEI5            EN10
#define EN_SCI5_ERI5            EN11
#define EN_SCI6_TEI6            EN12
#define EN_SCI6_ERI6            EN13
#define EN_SCI7_TEI7            EN14
#define EN_SCI7_ERI7            EN15
#define EN_SCI12_TEI12          EN16
#define EN_SCI12_ERI12          EN17
#define EN_SCI12_SCIX0          EN18
#define EN_SCI12_SCIX1          EN19
#define EN_SCI12_SCIX2          EN20
#define EN_SCI12_SCIX3          EN21
#define EN_QSPI_QSPSSLI         EN24
#define EN_CAC_FERRI            EN26
#define EN_CAC_MENDI            EN27
#define EN_CAC_OVFI             EN28
#define EN_DOC_DOPCI            EN29
#define EN_PDC_PCFEI            EN30
#define EN_PDC_PCERI            EN31
#define EN_SDHI_CDETI           EN3
#define EN_SDHI_CACI            EN4
#define EN_SDHI_SDACI           EN5
#define EN_MMCIF_CDETIO         EN6
#define EN_MMCIF_ERRIO          EN7
#define EN_MMCIF_ACCIO          EN8
#define EN_POE3_OEI1            EN9
#define EN_POE3_OEI2            EN10
#define EN_POE3_OEI3            EN11
#define EN_POE3_OEI4            EN12
#define EN_RIIC0_TEI0           EN13
#define EN_RIIC0_EEI0           EN14
#define EN_RIIC2_TEI2           EN15
#define EN_RIIC2_EEI2           EN16
#define EN_S12AD_S12CMPAI       EN20
#define EN_S12AD_S12CMPBI       EN21
#define EN_S12AD1_S12CMPAI1     EN22
#define EN_S12AD1_S12CMPBI1     EN23
#define EN_SCI8_TEI8            EN24
#define EN_SCI8_ERI8            EN25
#define EN_SCI9_TEI9            EN26
#define EN_SCI9_ERI9            EN27
#define EN_RIIC1_TEI1           EN28
#define EN_RIIC1_EEI1           EN29
#define EN_SDSI_SDIOI           EN0
#define EN_SCI10_TEI10          EN8
#define EN_SCI10_ERI10          EN9
#define EN_SCI11_TEI11          EN12
#define EN_SCI11_ERI11          EN13
#define EN_RSPI0_SPII0          EN16
#define EN_RSPI0_SPEI0          EN17
#define EN_RSPI1_SPII1          EN18
#define EN_RSPI1_SPEI1          EN19
#define EN_RSPI2_SPII2          EN20
#define EN_RSPI2_SPEI2          EN21
#define EN_EDMAC0_EINT0         EN4
#define EN_GLCDC_VPOS           EN8
#define EN_GLCDC_GR1UF          EN9
#define EN_GLCDC_GR2UF          EN10
#define EN_DRW2D_DRWIRQ         EN11

#define CLR_CAN0_ERS0           CLR0
#define CLR_CAN1_ERS1           CLR1

#define GEN_CAN0_ERS0           GENBE0
#define GEN_CAN1_ERS1           GENBE0
#define GEN_SCI0_TEI0           GENBL0
#define GEN_SCI0_ERI0           GENBL0
#define GEN_SCI1_TEI1           GENBL0
#define GEN_SCI1_ERI1           GENBL0
#define GEN_SCI2_TEI2           GENBL0
#define GEN_SCI2_ERI2           GENBL0
#define GEN_SCI3_TEI3           GENBL0
#define GEN_SCI3_ERI3           GENBL0
#define GEN_SCI4_TEI4           GENBL0
#define GEN_SCI4_ERI4           GENBL0
#define GEN_SCI5_TEI5           GENBL0
#define GEN_SCI5_ERI5           GENBL0
#define GEN_SCI6_TEI6           GENBL0
#define GEN_SCI6_ERI6           GENBL0
#define GEN_SCI7_TEI7           GENBL0
#define GEN_SCI7_ERI7           GENBL0
#define GEN_SCI12_TEI12         GENBL0
#define GEN_SCI12_ERI12         GENBL0
#define GEN_SCI12_SCIX0         GENBL0
#define GEN_SCI12_SCIX1         GENBL0
#define GEN_SCI12_SCIX2         GENBL0
#define GEN_SCI12_SCIX3         GENBL0
#define GEN_QSPI_QSPSSLI        GENBL0
#define GEN_CAC_FERRI           GENBL0
#define GEN_CAC_MENDI           GENBL0
#define GEN_CAC_OVFI            GENBL0
#define GEN_DOC_DOPCI           GENBL0
#define GEN_PDC_PCFEI           GENBL0
#define GEN_PDC_PCERI           GENBL0
#define GEN_SDHI_CDETI          GENBL1
#define GEN_SDHI_CACI           GENBL1
#define GEN_SDHI_SDACI          GENBL1
#define GEN_MMCIF_CDETIO        GENBL1
#define GEN_MMCIF_ERRIO         GENBL1
#define GEN_MMCIF_ACCIO         GENBL1
#define GEN_POE3_OEI1           GENBL1
#define GEN_POE3_OEI2           GENBL1
#define GEN_POE3_OEI3           GENBL1
#define GEN_POE3_OEI4           GENBL1
#define GEN_RIIC0_TEI0          GENBL1
#define GEN_RIIC0_EEI0          GENBL1
#define GEN_RIIC2_TEI2          GENBL1
#define GEN_RIIC2_EEI2          GENBL1
#define GEN_S12AD_S12CMPAI      GENBL1
#define GEN_S12AD_S12CMPBI      GENBL1
#define GEN_S12AD1_S12CMPAI1    GENBL1
#define GEN_S12AD1_S12CMPBI1    GENBL1
#define GEN_SCI8_TEI8           GENBL1
#define GEN_SCI8_ERI8           GENBL1
#define GEN_SCI9_TEI9           GENBL1
#define GEN_SCI9_ERI9           GENBL1
#define GEN_RIIC1_TEI1          GENBL1
#define GEN_RIIC1_EEI1          GENBL1
#define GEN_SDSI_SDIOI          GENBL2
#define GEN_SCI10_TEI10         GENAL0
#define GEN_SCI10_ERI10         GENAL0
#define GEN_SCI11_TEI11         GENAL0
#define GEN_SCI11_ERI11         GENAL0
#define GEN_RSPI0_SPII0         GENAL0
#define GEN_RSPI0_SPEI0         GENAL0
#define GEN_RSPI1_SPII1         GENAL0
#define GEN_RSPI1_SPEI1         GENAL0
#define GEN_RSPI2_SPII2         GENAL0
#define GEN_RSPI2_SPEI2         GENAL0
#define GEN_EDMAC0_EINT0        GENAL1
#define GEN_GLCDC_VPOS          GENAL1
#define GEN_GLCDC_GR1UF         GENAL1
#define GEN_GLCDC_GR2UF         GENAL1
#define GEN_DRW2D_DRWIRQ        GENAL1

#define GRP_CAN0_ERS0           GRPBE0
#define GRP_CAN1_ERS1           GRPBE0
#define GRP_SCI0_TEI0           GRPBL0
#define GRP_SCI0_ERI0           GRPBL0
#define GRP_SCI1_TEI1           GRPBL0
#define GRP_SCI1_ERI1           GRPBL0
#define GRP_SCI2_TEI2           GRPBL0
#define GRP_SCI2_ERI2           GRPBL0
#define GRP_SCI3_TEI3           GRPBL0
#define GRP_SCI3_ERI3           GRPBL0
#define GRP_SCI4_TEI4           GRPBL0
#define GRP_SCI4_ERI4           GRPBL0
#define GRP_SCI5_TEI5           GRPBL0
#define GRP_SCI5_ERI5           GRPBL0
#define GRP_SCI6_TEI6           GRPBL0
#define GRP_SCI6_ERI6           GRPBL0
#define GRP_SCI7_TEI7           GRPBL0
#define GRP_SCI7_ERI7           GRPBL0
#define GRP_SCI12_TEI12         GRPBL0
#define GRP_SCI12_ERI12         GRPBL0
#define GRP_SCI12_SCIX0         GRPBL0
#define GRP_SCI12_SCIX1         GRPBL0
#define GRP_SCI12_SCIX2         GRPBL0
#define GRP_SCI12_SCIX3         GRPBL0
#define GRP_QSPI_QSPSSLI        GRPBL0
#define GRP_CAC_FERRI           GRPBL0
#define GRP_CAC_MENDI           GRPBL0
#define GRP_CAC_OVFI            GRPBL0
#define GRP_DOC_DOPCI           GRPBL0
#define GRP_PDC_PCFEI           GRPBL0
#define GRP_PDC_PCERI           GRPBL0
#define GRP_SDHI_CDETI          GRPBL1
#define GRP_SDHI_CACI           GRPBL1
#define GRP_SDHI_SDACI          GRPBL1
#define GRP_MMCIF_CDETIO        GRPBL1
#define GRP_MMCIF_ERRIO         GRPBL1
#define GRP_MMCIF_ACCIO         GRPBL1
#define GRP_POE3_OEI1           GRPBL1
#define GRP_POE3_OEI2           GRPBL1
#define GRP_POE3_OEI3           GRPBL1
#define GRP_POE3_OEI4           GRPBL1
#define GRP_RIIC0_TEI0          GRPBL1
#define GRP_RIIC0_EEI0          GRPBL1
#define GRP_RIIC2_TEI2          GRPBL1
#define GRP_RIIC2_EEI2          GRPBL1
#define GRP_S12AD_S12CMPAI      GRPBL1
#define GRP_S12AD_S12CMPBI      GRPBL1
#define GRP_S12AD1_S12CMPAI1    GRPBL1
#define GRP_S12AD1_S12CMPBI1    GRPBL1
#define GRP_SCI8_TEI8           GRPBL1
#define GRP_SCI8_ERI8           GRPBL1
#define GRP_SCI9_TEI9           GRPBL1
#define GRP_SCI9_ERI9           GRPBL1
#define GRP_RIIC1_TEI1          GRPBL1
#define GRP_RIIC1_EEI1          GRPBL1
#define GRP_SDSI_SDIOI          GRPBL2
#define GRP_SCI10_TEI10         GRPAL0
#define GRP_SCI10_ERI10         GRPAL0
#define GRP_SCI11_TEI11         GRPAL0
#define GRP_SCI11_ERI11         GRPAL0
#define GRP_RSPI0_SPII0         GRPAL0
#define GRP_RSPI0_SPEI0         GRPAL0
#define GRP_RSPI1_SPII1         GRPAL0
#define GRP_RSPI1_SPEI1         GRPAL0
#define GRP_RSPI2_SPII2         GRPAL0
#define GRP_RSPI2_SPEI2         GRPAL0
#define GRP_EDMAC0_EINT0        GRPAL1
#define GRP_GLCDC_VPOS          GRPAL1
#define GRP_GLCDC_GR1UF         GRPAL1
#define GRP_GLCDC_GR2UF         GRPAL1
#define GRP_DRW2D_DRWIRQ        GRPAL1

#define GCR_CAN0_ERS0           GCRBE0
#define GCR_CAN1_ERS1           GCRBE0

#define __IR( x )               ICU.IR[ IR ## x ].BIT.IR
#define  _IR( x )               __IR( x )
#define   IR( x , y )   _IR( _ ## x ## _ ## y )
#define __DTCE( x )             ICU.DTCER[ DTCE ## x ].BIT.DTCE
#define  _DTCE( x )             __DTCE( x )
#define   DTCE( x , y ) _DTCE( _ ## x ## _ ## y )
#define __IEN( x )              ICU.IER[ IER ## x ].BIT.IEN ## x
#define  _IEN( x )              __IEN( x )
#define   IEN( x , y )  _IEN( _ ## x ## _ ## y )
#define __IPR( x )              ICU.IPR[ IPR ## x ].BIT.IPR
#define  _IPR( x )              __IPR( x )
#define   IPR( x , y )  _IPR( _ ## x ## _ ## y )
#define __VECT( x )             VECT ## x
#define  _VECT( x )             __VECT( x )
#define   VECT( x , y ) _VECT( _ ## x ## _ ## y )
#define __MSTP( x )             MSTP ## x
#define  _MSTP( x )             __MSTP( x )
#define   MSTP( x )             _MSTP( _ ## x )

#define __IS( x )               ICU.GRP ## x.BIT.IS ## x
#define  _IS( x )               __IS( x )
#define   IS( x , y )   _IS( _ ## x ## _ ## y )
#define __EN( x )               ICU.GEN ## x.BIT.EN ## x
#define  _EN( x )               __EN( x )
#define   EN( x , y )   _EN( _ ## x ## _ ## y )
#define __CLR( x )              ICU.GCR ## x.BIT.CLR ## x
#define  _CLR( x )              __CLR( x )
#define   CLR( x , y )  _CLR( _ ## x ## _ ## y )

#define BSC             (*(volatile struct st_bsc      *)0x81300)
#define CAC             (*(volatile struct st_cac      *)0x8b000)
#define CMT             (*(volatile struct st_cmt      *)0x88000)
#define CMT0    (*(volatile struct st_cmt0     *)0x88002)
#define CMT1    (*(volatile struct st_cmt0     *)0x88008)
#define CMT2    (*(volatile struct st_cmt0     *)0x88012)
#define CMT3    (*(volatile struct st_cmt0     *)0x88018)
#define CMTW0   (*(volatile struct st_cmtw     *)0x94200)
#define ICU     (*(volatile struct st_icu      *)0x87000)
#define MPC     (*(volatile struct st_mpc      *)0x8c100)
#define PORT0   (*(volatile struct st_port0    *)0x8c000)
#define PORT1   (*(volatile struct st_port1    *)0x8c001)
#define PORT2   (*(volatile struct st_port2    *)0x8c002)
#define PORT3   (*(volatile struct st_port3    *)0x8c003)
#define PORT5   (*(volatile struct st_port5    *)0x8c005)
#define PORT6   (*(volatile struct st_port6    *)0x8c006)
#define PORT7   (*(volatile struct st_port7    *)0x8c007)
#define PORT8   (*(volatile struct st_port8    *)0x8c008)
#define PORT9   (*(volatile struct st_port9    *)0x8c009)
#define PORTA   (*(volatile struct st_porta    *)0x8c00a)
#define PORTB   (*(volatile struct st_portb    *)0x8c00b)
#define PORTC   (*(volatile struct st_portc    *)0x8c00c)
#define PORTE   (*(volatile struct st_porte    *)0x8c00e)
#define PORTF   (*(volatile struct st_portf    *)0x8c00f)
#define PORTG   (*(volatile struct st_portg    *)0x8c010)
#define PORTJ   (*(volatile struct st_portj    *)0x8c012)
#define RTC     (*(volatile struct st_rtc      *)0x8c400)
#define SCI0    (*(volatile struct st_sci0     *)0x8a000)
#define SCI1    (*(volatile struct st_sci0     *)0x8a020)
#define SCI2    (*(volatile struct st_sci0     *)0x8a040)
#define SCI3    (*(volatile struct st_sci0     *)0x8a060)
#define SCI4    (*(volatile struct st_sci0     *)0x8a080)
#define SCI5    (*(volatile struct st_sci0     *)0x8a0a0)
#define SCI6    (*(volatile struct st_sci0     *)0x8a0c0)
#define SCI7    (*(volatile struct st_sci0     *)0x8a0e0)
#define SCI8    (*(volatile struct st_sci0     *)0x8a100)
#define SCI9    (*(volatile struct st_sci0     *)0x8a120)
#define SCI10   (*(volatile struct st_sci10    *)0xd0040)
#define SCI11   (*(volatile struct st_sci10    *)0xd0060)
#define SCI12   (*(volatile struct st_sci12    *)0x8b300)

#define SYSTEM  (*(volatile struct st_system   *)0x80000)

/****************************************************************************
 * Public Types
 ***************************************************************************/
typedef enum enum_ir
{
  IR_BSC_BUSERR = 16,
  IR_RAM_RAMERR = 18,
  IR_FCU_FIFERR = 21,
  IR_FCU_FRDYI = 23,
  IR_ICU_SWINT2 = 26,
  IR_ICU_SWINT,
  IR_CMT0_CMI0,
  IR_CMT1_CMI1,
  IR_CMTW0_CMWI0,
  IR_CMTW1_CMWI1,
  IR_USB0_D0FIFO0 = 34,
  IR_USB0_D1FIFO0,
  IR_RSPI0_SPRI0 = 38,
  IR_RSPI0_SPTI0,
  IR_RSPI1_SPRI1,
  IR_RSPI1_SPTI1,
  IR_QSPI_SPRI,
  IR_QSPI_SPTI,
  IR_SDHI_SBFAI,
  IR_MMCIF_MBFAI,
  IR_RIIC1_RXI1 = 50,
  IR_RIIC1_TXI1,
  IR_RIIC0_RXI0,
  IR_RIIC0_TXI0,
  IR_RIIC2_RXI2,
  IR_RIIC2_TXI2,
  IR_SCI0_RXI0 = 58,
  IR_SCI0_TXI0,
  IR_SCI1_RXI1,
  IR_SCI1_TXI1,
  IR_SCI2_RXI2,
  IR_SCI2_TXI2,
  IR_ICU_IRQ0,
  IR_ICU_IRQ1,
  IR_ICU_IRQ2,
  IR_ICU_IRQ3,
  IR_ICU_IRQ4,
  IR_ICU_IRQ5,
  IR_ICU_IRQ6,
  IR_ICU_IRQ7,
  IR_ICU_IRQ8,
  IR_ICU_IRQ9,
  IR_ICU_IRQ10,
  IR_ICU_IRQ11,
  IR_ICU_IRQ12,
  IR_ICU_IRQ13,
  IR_ICU_IRQ14,
  IR_ICU_IRQ15,
  IR_SCI3_RXI3,
  IR_SCI3_TXI3,
  IR_SCI4_RXI4,
  IR_SCI4_TXI4,
  IR_SCI5_RXI5,
  IR_SCI5_TXI5,
  IR_SCI6_RXI6,
  IR_SCI6_TXI6,
  IR_LVD1_LVD1,
  IR_LVD2_LVD2,
  IR_USB0_USBR0,
  IR_RTC_ALM = 92,
  IR_RTC_PRD,
  IR_IWDT_IWUNI = 95,
  IR_WDT_WUNI,
  IR_PDC_PCDFI,
  IR_SCI7_RXI7,
  IR_SCI7_TXI7,
  IR_SCI8_RXI8,
  IR_SCI8_TXI8,
  IR_SCI9_RXI9,
  IR_SCI9_TXI9,
  IR_SCI10_RXI10,
  IR_SCI10_TXI10,
  IR_ICU_GROUPBE0,
  IR_ICU_GROUPBL2,
  IR_RSPI2_SPRI2,
  IR_RSPI2_SPTI2,
  IR_ICU_GROUPBL0,
  IR_ICU_GROUPBL1,
  IR_ICU_GROUPAL0,
  IR_ICU_GROUPAL1,
  IR_SCI11_RXI11,
  IR_SCI11_TXI11,
  IR_SCI12_RXI12,
  IR_SCI12_TXI12,
  IR_DMAC_DMAC0I = 120,
  IR_DMAC_DMAC1I,
  IR_DMAC_DMAC2I,
  IR_DMAC_DMAC3I,
  IR_DMAC_DMAC74I,
  IR_OST_OSTDI,
  IR_EXDMAC_EXDMAC0I,
  IR_EXDMAC_EXDMAC1I,
  IR_PERIB_INTB128,
  IR_PERIB_INTB129,
  IR_PERIB_INTB130,
  IR_PERIB_INTB131,
  IR_PERIB_INTB132,
  IR_PERIB_INTB133,
  IR_PERIB_INTB134,
  IR_PERIB_INTB135,
  IR_PERIB_INTB136,
  IR_PERIB_INTB137,
  IR_PERIB_INTB138,
  IR_PERIB_INTB139,
  IR_PERIB_INTB140,
  IR_PERIB_INTB141,
  IR_PERIB_INTB142,
  IR_PERIB_INTB143,
  IR_PERIB_INTB144,
  IR_PERIB_INTB145,
  IR_PERIB_INTB146,
  IR_PERIB_INTB147,
  IR_PERIB_INTB148,
  IR_PERIB_INTB149,
  IR_PERIB_INTB150,
  IR_PERIB_INTB151,
  IR_PERIB_INTB152,
  IR_PERIB_INTB153,
  IR_PERIB_INTB154,
  IR_PERIB_INTB155,
  IR_PERIB_INTB156,
  IR_PERIB_INTB157,
  IR_PERIB_INTB158,
  IR_PERIB_INTB159,
  IR_PERIB_INTB160,
  IR_PERIB_INTB161,
  IR_PERIB_INTB162,
  IR_PERIB_INTB163,
  IR_PERIB_INTB164,
  IR_PERIB_INTB165,
  IR_PERIB_INTB166,
  IR_PERIB_INTB167,
  IR_PERIB_INTB168,
  IR_PERIB_INTB169,
  IR_PERIB_INTB170,
  IR_PERIB_INTB171,
  IR_PERIB_INTB172,
  IR_PERIB_INTB173,
  IR_PERIB_INTB174,
  IR_PERIB_INTB175,
  IR_PERIB_INTB176,
  IR_PERIB_INTB177,
  IR_PERIB_INTB178,
  IR_PERIB_INTB179,
  IR_PERIB_INTB180,
  IR_PERIB_INTB181,
  IR_PERIB_INTB182,
  IR_PERIB_INTB183,
  IR_PERIB_INTB184,
  IR_PERIB_INTB185,
  IR_PERIB_INTB186,
  IR_PERIB_INTB187,
  IR_PERIB_INTB188,
  IR_PERIB_INTB189,
  IR_PERIB_INTB190,
  IR_PERIB_INTB191,
  IR_PERIB_INTB192,
  IR_PERIB_INTB193,
  IR_PERIB_INTB194,
  IR_PERIB_INTB195,
  IR_PERIB_INTB196,
  IR_PERIB_INTB197,
  IR_PERIB_INTB198,
  IR_PERIB_INTB199,
  IR_PERIB_INTB200,
  IR_PERIB_INTB201,
  IR_PERIB_INTB202,
  IR_PERIB_INTB203,
  IR_PERIB_INTB204,
  IR_PERIB_INTB205,
  IR_PERIB_INTB206,
  IR_PERIB_INTB207,
  IR_PERIA_INTA208,
  IR_PERIA_INTA209,
  IR_PERIA_INTA210,
  IR_PERIA_INTA211,
  IR_PERIA_INTA212,
  IR_PERIA_INTA213,
  IR_PERIA_INTA214,
  IR_PERIA_INTA215,
  IR_PERIA_INTA216,
  IR_PERIA_INTA217,
  IR_PERIA_INTA218,
  IR_PERIA_INTA219,
  IR_PERIA_INTA220,
  IR_PERIA_INTA221,
  IR_PERIA_INTA222,
  IR_PERIA_INTA223,
  IR_PERIA_INTA224,
  IR_PERIA_INTA225,
  IR_PERIA_INTA226,
  IR_PERIA_INTA227,
  IR_PERIA_INTA228,
  IR_PERIA_INTA229,
  IR_PERIA_INTA230,
  IR_PERIA_INTA231,
  IR_PERIA_INTA232,
  IR_PERIA_INTA233,
  IR_PERIA_INTA234,
  IR_PERIA_INTA235,
  IR_PERIA_INTA236,
  IR_PERIA_INTA237,
  IR_PERIA_INTA238,
  IR_PERIA_INTA239,
  IR_PERIA_INTA240,
  IR_PERIA_INTA241,
  IR_PERIA_INTA242,
  IR_PERIA_INTA243,
  IR_PERIA_INTA244,
  IR_PERIA_INTA245,
  IR_PERIA_INTA246,
  IR_PERIA_INTA247,
  IR_PERIA_INTA248,
  IR_PERIA_INTA249,
  IR_PERIA_INTA250,
  IR_PERIA_INTA251,
  IR_PERIA_INTA252,
  IR_PERIA_INTA253,
  IR_PERIA_INTA254,
  IR_PERIA_INTA255
} enum_ir_t;

typedef enum enum_dtce
{
  DTCE_ICU_SWINT2 = 26,
  DTCE_ICU_SWINT,
  DTCE_CMT0_CMI0,
  DTCE_CMT1_CMI1,
  DTCE_CMTW0_CMWI0,
  DTCE_CMTW1_CMWI1,
  DTCE_USB0_D0FIFO0 = 34,
  DTCE_USB0_D1FIFO0,
  DTCE_RSPI0_SPRI0 = 38,
  DTCE_RSPI0_SPTI0,
  DTCE_RSPI1_SPRI1,
  DTCE_RSPI1_SPTI1,
  DTCE_QSPI_SPRI,
  DTCE_QSPI_SPTI,
  DTCE_SDHI_SBFAI,
  DTCE_MMCIF_MBFAI,
  DTCE_RIIC1_RXI1 = 50,
  DTCE_RIIC1_TXI1,
  DTCE_RIIC0_RXI0,
  DTCE_RIIC0_TXI0,
  DTCE_RIIC2_RXI2,
  DTCE_RIIC2_TXI2,
  DTCE_SCI0_RXI0 = 58,
  DTCE_SCI0_TXI0,
  DTCE_SCI1_RXI1,
  DTCE_SCI1_TXI1,
  DTCE_SCI2_RXI2,
  DTCE_SCI2_TXI2,
  DTCE_ICU_IRQ0,
  DTCE_ICU_IRQ1,
  DTCE_ICU_IRQ2,
  DTCE_ICU_IRQ3,
  DTCE_ICU_IRQ4,
  DTCE_ICU_IRQ5,
  DTCE_ICU_IRQ6,
  DTCE_ICU_IRQ7,
  DTCE_ICU_IRQ8,
  DTCE_ICU_IRQ9,
  DTCE_ICU_IRQ10,
  DTCE_ICU_IRQ11,
  DTCE_ICU_IRQ12,
  DTCE_ICU_IRQ13,
  DTCE_ICU_IRQ14,
  DTCE_ICU_IRQ15,
  DTCE_SCI3_RXI3,
  DTCE_SCI3_TXI3,
  DTCE_SCI4_RXI4,
  DTCE_SCI4_TXI4,
  DTCE_SCI5_RXI5,
  DTCE_SCI5_TXI5,
  DTCE_SCI6_RXI6,
  DTCE_SCI6_TXI6,
  DTCE_PDC_PCDFI = 97,
  DTCE_SCI7_RXI7,
  DTCE_SCI7_TXI7,
  DTCE_SCI8_RXI8,
  DTCE_SCI8_TXI8,
  DTCE_SCI9_RXI9,
  DTCE_SCI9_TXI9,
  DTCE_SCI10_RXI10,
  DTCE_SCI10_TXI10,
  DTCE_RSPI2_SPRI2 = 108,
  DTCE_RSPI2_SPTI2,
  DTCE_SCI11_RXI11 = 114,
  DTCE_SCI11_TXI11,
  DTCE_SCI12_RXI12,
  DTCE_SCI12_TXI12,
  DTCE_DMAC_DMAC0I = 120,
  DTCE_DMAC_DMAC1I,
  DTCE_DMAC_DMAC2I,
  DTCE_DMAC_DMAC3I,
  DTCE_EXDMAC_EXDMAC0I = 126,
  DTCE_EXDMAC_EXDMAC1I,
  DTCE_PERIB_INTB128,
  DTCE_PERIB_INTB129,
  DTCE_PERIB_INTB130,
  DTCE_PERIB_INTB131,
  DTCE_PERIB_INTB132,
  DTCE_PERIB_INTB133,
  DTCE_PERIB_INTB134,
  DTCE_PERIB_INTB135,
  DTCE_PERIB_INTB136,
  DTCE_PERIB_INTB137,
  DTCE_PERIB_INTB138,
  DTCE_PERIB_INTB139,
  DTCE_PERIB_INTB140,
  DTCE_PERIB_INTB141,
  DTCE_PERIB_INTB142,
  DTCE_PERIB_INTB143,
  DTCE_PERIB_INTB144,
  DTCE_PERIB_INTB145,
  DTCE_PERIB_INTB146,
  DTCE_PERIB_INTB147,
  DTCE_PERIB_INTB148,
  DTCE_PERIB_INTB149,
  DTCE_PERIB_INTB150,
  DTCE_PERIB_INTB151,
  DTCE_PERIB_INTB152,
  DTCE_PERIB_INTB153,
  DTCE_PERIB_INTB154,
  DTCE_PERIB_INTB155,
  DTCE_PERIB_INTB156,
  DTCE_PERIB_INTB157,
  DTCE_PERIB_INTB158,
  DTCE_PERIB_INTB159,
  DTCE_PERIB_INTB160,
  DTCE_PERIB_INTB161,
  DTCE_PERIB_INTB162,
  DTCE_PERIB_INTB163,
  DTCE_PERIB_INTB164,
  DTCE_PERIB_INTB165,
  DTCE_PERIB_INTB166,
  DTCE_PERIB_INTB167,
  DTCE_PERIB_INTB168,
  DTCE_PERIB_INTB169,
  DTCE_PERIB_INTB170,
  DTCE_PERIB_INTB171,
  DTCE_PERIB_INTB172,
  DTCE_PERIB_INTB173,
  DTCE_PERIB_INTB174,
  DTCE_PERIB_INTB175,
  DTCE_PERIB_INTB176,
  DTCE_PERIB_INTB177,
  DTCE_PERIB_INTB178,
  DTCE_PERIB_INTB179,
  DTCE_PERIB_INTB180,
  DTCE_PERIB_INTB181,
  DTCE_PERIB_INTB182,
  DTCE_PERIB_INTB183,
  DTCE_PERIB_INTB184,
  DTCE_PERIB_INTB185,
  DTCE_PERIB_INTB186,
  DTCE_PERIB_INTB187,
  DTCE_PERIB_INTB188,
  DTCE_PERIB_INTB189,
  DTCE_PERIB_INTB190,
  DTCE_PERIB_INTB191,
  DTCE_PERIB_INTB192,
  DTCE_PERIB_INTB193,
  DTCE_PERIB_INTB194,
  DTCE_PERIB_INTB195,
  DTCE_PERIB_INTB196,
  DTCE_PERIB_INTB197,
  DTCE_PERIB_INTB198,
  DTCE_PERIB_INTB199,
  DTCE_PERIB_INTB200,
  DTCE_PERIB_INTB201,
  DTCE_PERIB_INTB202,
  DTCE_PERIB_INTB203,
  DTCE_PERIB_INTB204,
  DTCE_PERIB_INTB205,
  DTCE_PERIB_INTB206,
  DTCE_PERIB_INTB207,
  DTCE_PERIA_INTA208,
  DTCE_PERIA_INTA209,
  DTCE_PERIA_INTA210,
  DTCE_PERIA_INTA211,
  DTCE_PERIA_INTA212,
  DTCE_PERIA_INTA213,
  DTCE_PERIA_INTA214,
  DTCE_PERIA_INTA215,
  DTCE_PERIA_INTA216,
  DTCE_PERIA_INTA217,
  DTCE_PERIA_INTA218,
  DTCE_PERIA_INTA219,
  DTCE_PERIA_INTA220,
  DTCE_PERIA_INTA221,
  DTCE_PERIA_INTA222,
  DTCE_PERIA_INTA223,
  DTCE_PERIA_INTA224,
  DTCE_PERIA_INTA225,
  DTCE_PERIA_INTA226,
  DTCE_PERIA_INTA227,
  DTCE_PERIA_INTA228,
  DTCE_PERIA_INTA229,
  DTCE_PERIA_INTA230,
  DTCE_PERIA_INTA231,
  DTCE_PERIA_INTA232,
  DTCE_PERIA_INTA233,
  DTCE_PERIA_INTA234,
  DTCE_PERIA_INTA235,
  DTCE_PERIA_INTA236,
  DTCE_PERIA_INTA237,
  DTCE_PERIA_INTA238,
  DTCE_PERIA_INTA239,
  DTCE_PERIA_INTA240,
  DTCE_PERIA_INTA241,
  DTCE_PERIA_INTA242,
  DTCE_PERIA_INTA243,
  DTCE_PERIA_INTA244,
  DTCE_PERIA_INTA245,
  DTCE_PERIA_INTA246,
  DTCE_PERIA_INTA247,
  DTCE_PERIA_INTA248,
  DTCE_PERIA_INTA249,
  DTCE_PERIA_INTA250,
  DTCE_PERIA_INTA251,
  DTCE_PERIA_INTA252,
  DTCE_PERIA_INTA253,
  DTCE_PERIA_INTA254,
  DTCE_PERIA_INTA255
} enum_dtce_t;

typedef enum enum_ier
{
  IER_BSC_BUSERR =  0x02,
  IER_RAM_RAMERR = 0x02,
  IER_FCU_FIFERR = 0x02,
  IER_FCU_FRDYI = 0x02,
  IER_ICU_SWINT2 = 0x03,
  IER_ICU_SWINT = 0x03,
  IER_CMT0_CMI0 = 0x03,
  IER_CMT1_CMI1 = 0x03,
  IER_CMTW0_CMWI0 = 0x03,
  IER_CMTW1_CMWI1 = 0x03,
  IER_USB0_D0FIFO0 = 0x04,
  IER_USB0_D1FIFO0 = 0x04,
  IER_RSPI0_SPRI0 = 0x04,
  IER_RSPI0_SPTI0 = 0x04,
  IER_RSPI1_SPRI1 = 0x05,
  IER_RSPI1_SPTI1 = 0x05,
  IER_QSPI_SPRI = 0x05,
  IER_QSPI_SPTI = 0x05,
  IER_SDHI_SBFAI = 0x05,
  IER_MMCIF_MBFAI = 0x05,
  IER_RIIC1_RXI1 = 0x06,
  IER_RIIC1_TXI1 = 0x06,
  IER_RIIC0_RXI0 = 0x06,
  IER_RIIC0_TXI0 = 0x06,
  IER_RIIC2_RXI2 = 0x06,
  IER_RIIC2_TXI2 = 0x06,
  IER_SCI0_RXI0 = 0x07,
  IER_SCI0_TXI0 = 0x07,
  IER_SCI1_RXI1 = 0x07,
  IER_SCI1_TXI1 = 0x07,
  IER_SCI2_RXI2 = 0x07,
  IER_SCI2_TXI2 = 0x07,
  IER_ICU_IRQ0 = 0x08,
  IER_ICU_IRQ1 = 0x08,
  IER_ICU_IRQ2 = 0x08,
  IER_ICU_IRQ3 = 0x08,
  IER_ICU_IRQ4 = 0x08,
  IER_ICU_IRQ5 = 0x08,
  IER_ICU_IRQ6 = 0x08,
  IER_ICU_IRQ7 = 0x08,
  IER_ICU_IRQ8 = 0x09,
  IER_ICU_IRQ9 = 0x09,
  IER_ICU_IRQ10 = 0x09,
  IER_ICU_IRQ11 = 0x09,
  IER_ICU_IRQ12 = 0x09,
  IER_ICU_IRQ13 = 0x09,
  IER_ICU_IRQ14 = 0x09,
  IER_ICU_IRQ15 = 0x09,
  IER_SCI3_RXI3 = 0x0a,
  IER_SCI3_TXI3 = 0x0a,
  IER_SCI4_RXI4 = 0x0a,
  IER_SCI4_TXI4 = 0x0a,
  IER_SCI5_RXI5 = 0x0a,
  IER_SCI5_TXI5 = 0x0a,
  IER_SCI6_RXI6 = 0x0a,
  IER_SCI6_TXI6 = 0x0a,
  IER_LVD1_LVD1 = 0x0b,
  IER_LVD2_LVD2 = 0x0b,
  IER_USB0_USBR0 = 0x0b,
  IER_RTC_ALM = 0x0b,
  IER_RTC_PRD = 0x0b,
  IER_IWDT_IWUNI = 0x0b,
  IER_WDT_WUNI = 0x0c,
  IER_PDC_PCDFI = 0x0c,
  IER_SCI7_RXI7 = 0x0c,
  IER_SCI7_TXI7 = 0x0c,
  IER_SCI8_RXI8 = 0x0c,
  IER_SCI8_TXI8 = 0x0c,
  IER_SCI9_RXI9 = 0x0c,
  IER_SCI9_TXI9 = 0x0c,
  IER_SCI10_RXI10 = 0x0d,
  IER_SCI10_TXI10 = 0x0d,
  IER_ICU_GROUPBE0 = 0x0d,
  IER_ICU_GROUPBL2 = 0x0d,
  IER_RSPI2_SPRI2 = 0x0d,
  IER_RSPI2_SPTI2 = 0x0d,
  IER_ICU_GROUPBL0 = 0x0d,
  IER_ICU_GROUPBL1 = 0x0d,
  IER_ICU_GROUPAL0 = 0x0e,
  IER_ICU_GROUPAL1 = 0x0e,
  IER_SCI11_RXI11 = 0x0e,
  IER_SCI11_TXI11 = 0x0e,
  IER_SCI12_RXI12 = 0x0e,
  IER_SCI12_TXI12 = 0x0e,
  IER_DMAC_DMAC0I = 0x0f,
  IER_DMAC_DMAC1I = 0x0f,
  IER_DMAC_DMAC2I = 0x0f,
  IER_DMAC_DMAC3I = 0x0f,
  IER_DMAC_DMAC74I = 0x0f,
  IER_OST_OSTDI = 0x0f,
  IER_EXDMAC_EXDMAC0I = 0x0f,
  IER_EXDMAC_EXDMAC1I = 0x0f,
  IER_PERIB_INTB128 = 0x10,
  IER_PERIB_INTB129 = 0x10,
  IER_PERIB_INTB130 = 0x10,
  IER_PERIB_INTB131 = 0x10,
  IER_PERIB_INTB132 = 0x10,
  IER_PERIB_INTB133 = 0x10,
  IER_PERIB_INTB134 = 0x10,
  IER_PERIB_INTB135 = 0x10,
  IER_PERIB_INTB136 = 0x11,
  IER_PERIB_INTB137 = 0x11,
  IER_PERIB_INTB138 = 0x11,
  IER_PERIB_INTB139 = 0x11,
  IER_PERIB_INTB140 = 0x11,
  IER_PERIB_INTB141 = 0x11,
  IER_PERIB_INTB142 = 0x11,
  IER_PERIB_INTB143 = 0x11,
  IER_PERIB_INTB144 = 0x12,
  IER_PERIB_INTB145 = 0x12,
  IER_PERIB_INTB146 = 0x12,
  IER_PERIB_INTB147 = 0x12,
  IER_PERIB_INTB148 = 0x12,
  IER_PERIB_INTB149 = 0x12,
  IER_PERIB_INTB150 = 0x12,
  IER_PERIB_INTB151 = 0x12,
  IER_PERIB_INTB152 = 0x13,
  IER_PERIB_INTB153 = 0x13,
  IER_PERIB_INTB154 = 0x13,
  IER_PERIB_INTB155 = 0x13,
  IER_PERIB_INTB156 = 0x13,
  IER_PERIB_INTB157 = 0x13,
  IER_PERIB_INTB158 = 0x13,
  IER_PERIB_INTB159 = 0x13,
  IER_PERIB_INTB160 = 0x14,
  IER_PERIB_INTB161 = 0x14,
  IER_PERIB_INTB162 = 0x14,
  IER_PERIB_INTB163 = 0x14,
  IER_PERIB_INTB164 = 0x14,
  IER_PERIB_INTB165 = 0x14,
  IER_PERIB_INTB166 = 0x14,
  IER_PERIB_INTB167 = 0x14,
  IER_PERIB_INTB168 = 0x15,
  IER_PERIB_INTB169 = 0x15,
  IER_PERIB_INTB170 = 0x15,
  IER_PERIB_INTB171 = 0x15,
  IER_PERIB_INTB172 = 0x15,
  IER_PERIB_INTB173 = 0x15,
  IER_PERIB_INTB174 = 0x15,
  IER_PERIB_INTB175 = 0x15,
  IER_PERIB_INTB176 = 0x16,
  IER_PERIB_INTB177 = 0x16,
  IER_PERIB_INTB178 = 0x16,
  IER_PERIB_INTB179 = 0x16,
  IER_PERIB_INTB180 = 0x16,
  IER_PERIB_INTB181 = 0x16,
  IER_PERIB_INTB182 = 0x16,
  IER_PERIB_INTB183 = 0x16,
  IER_PERIB_INTB184 = 0x17,
  IER_PERIB_INTB185 = 0x17,
  IER_PERIB_INTB186 = 0x17,
  IER_PERIB_INTB187 = 0x17,
  IER_PERIB_INTB188 = 0x17,
  IER_PERIB_INTB189 = 0x17,
  IER_PERIB_INTB190 = 0x17,
  IER_PERIB_INTB191 = 0x17,
  IER_PERIB_INTB192 = 0x18,
  IER_PERIB_INTB193 = 0x18,
  IER_PERIB_INTB194 = 0x18,
  IER_PERIB_INTB195 = 0x18,
  IER_PERIB_INTB196 = 0x18,
  IER_PERIB_INTB197 = 0x18,
  IER_PERIB_INTB198 = 0x18,
  IER_PERIB_INTB199 = 0x18,
  IER_PERIB_INTB200 = 0x19,
  IER_PERIB_INTB201 = 0x19,
  IER_PERIB_INTB202 = 0x19,
  IER_PERIB_INTB203 = 0x19,
  IER_PERIB_INTB204 = 0x19,
  IER_PERIB_INTB205 = 0x19,
  IER_PERIB_INTB206 = 0x19,
  IER_PERIB_INTB207 = 0x19,
  IER_PERIA_INTA208 = 0x1a,
  IER_PERIA_INTA209 = 0x1a,
  IER_PERIA_INTA210 = 0x1a,
  IER_PERIA_INTA211 = 0x1a,
  IER_PERIA_INTA212 = 0x1a,
  IER_PERIA_INTA213 = 0x1a,
  IER_PERIA_INTA214 = 0x1a,
  IER_PERIA_INTA215 = 0x1a,
  IER_PERIA_INTA216 = 0x1b,
  IER_PERIA_INTA217 = 0x1b,
  IER_PERIA_INTA218 = 0x1b,
  IER_PERIA_INTA219 = 0x1b,
  IER_PERIA_INTA220 = 0x1b,
  IER_PERIA_INTA221 = 0x1b,
  IER_PERIA_INTA222 = 0x1b,
  IER_PERIA_INTA223 = 0x1b,
  IER_PERIA_INTA224 = 0x1c,
  IER_PERIA_INTA225 = 0x1c,
  IER_PERIA_INTA226 = 0x1c,
  IER_PERIA_INTA227 = 0x1c,
  IER_PERIA_INTA228 = 0x1c,
  IER_PERIA_INTA229 = 0x1c,
  IER_PERIA_INTA230 = 0x1c,
  IER_PERIA_INTA231 = 0x1c,
  IER_PERIA_INTA232 = 0x1d,
  IER_PERIA_INTA233 = 0x1d,
  IER_PERIA_INTA234 = 0x1d,
  IER_PERIA_INTA235 = 0x1d,
  IER_PERIA_INTA236 = 0x1d,
  IER_PERIA_INTA237 = 0x1d,
  IER_PERIA_INTA238 = 0x1d,
  IER_PERIA_INTA239 = 0x1d,
  IER_PERIA_INTA240 = 0x1e,
  IER_PERIA_INTA241 = 0x1e,
  IER_PERIA_INTA242 = 0x1e,
  IER_PERIA_INTA243 = 0x1e,
  IER_PERIA_INTA244 = 0x1e,
  IER_PERIA_INTA245 = 0x1e,
  IER_PERIA_INTA246 = 0x1e,
  IER_PERIA_INTA247 = 0x1e,
  IER_PERIA_INTA248 = 0x1f,
  IER_PERIA_INTA249 = 0x1f,
  IER_PERIA_INTA250 = 0x1f,
  IER_PERIA_INTA251 = 0x1f,
  IER_PERIA_INTA252 = 0x1f,
  IER_PERIA_INTA253 = 0x1f,
  IER_PERIA_INTA254 = 0x1f,
  IER_PERIA_INTA255 = 0x1f
} enum_ier_t;

typedef enum enum_ipr
{
  IPR_BSC_BUSERR = 0,
  IPR_RAM_RAMERR = 0,
  IPR_FCU_FIFERR = 1,
  IPR_FCU_FRDYI = 2,
  IPR_ICU_SWINT2 = 3,
  IPR_ICU_SWINT = 3,
  IPR_CMT0_CMI0 = 4,
  IPR_CMT1_CMI1 = 5,
  IPR_CMTW0_CMWI0 = 6,
  IPR_CMTW1_CMWI1 = 7,
  IPR_USB0_D0FIFO0 = 34,
  IPR_USB0_D1FIFO0 = 35,
  IPR_RSPI0_SPRI0 = 38,
  IPR_RSPI0_SPTI0 = 39,
  IPR_RSPI1_SPRI1 = 40,
  IPR_RSPI1_SPTI1 = 41,
  IPR_QSPI_SPRI = 42,
  IPR_QSPI_SPTI = 43,
  IPR_SDHI_SBFAI = 44,
  IPR_MMCIF_MBFAI = 45,
  IPR_RIIC1_RXI1 = 50,
  IPR_RIIC1_TXI1 = 51,
  IPR_RIIC0_RXI0 = 52,
  IPR_RIIC0_TXI0 = 53,
  IPR_RIIC2_RXI2 = 54,
  IPR_RIIC2_TXI2 = 55,
  IPR_SCI0_RXI0 = 58,
  IPR_SCI0_TXI0 = 59,
  IPR_SCI1_RXI1 = 60,
  IPR_SCI1_TXI1 = 61,
  IPR_SCI2_RXI2 = 62,
  IPR_SCI2_TXI2 = 63,
  IPR_ICU_IRQ0 = 64,
  IPR_ICU_IRQ1 = 65,
  IPR_ICU_IRQ2 = 66,
  IPR_ICU_IRQ3 = 67,
  IPR_ICU_IRQ4 = 68,
  IPR_ICU_IRQ5 = 69,
  IPR_ICU_IRQ6 = 70,
  IPR_ICU_IRQ7 = 71,
  IPR_ICU_IRQ8 = 72,
  IPR_ICU_IRQ9 = 73,
  IPR_ICU_IRQ10 = 74,
  IPR_ICU_IRQ11 = 75,
  IPR_ICU_IRQ12 = 76,
  IPR_ICU_IRQ13 = 77,
  IPR_ICU_IRQ14 = 78,
  IPR_ICU_IRQ15 = 79,
  IPR_SCI3_RXI3 = 80,
  IPR_SCI3_TXI3 = 81,
  IPR_SCI4_RXI4 = 82,
  IPR_SCI4_TXI4 = 83,
  IPR_SCI5_RXI5 = 84,
  IPR_SCI5_TXI5 = 85,
  IPR_SCI6_RXI6 = 86,
  IPR_SCI6_TXI6 = 87,
  IPR_LVD1_LVD1 = 88,
  IPR_LVD2_LVD2 = 89,
  IPR_USB0_USBR0 = 90,
  IPR_RTC_ALM = 92,
  IPR_RTC_PRD = 93,
  IPR_IWDT_IWUNI = 95,
  IPR_WDT_WUNI = 96,
  IPR_PDC_PCDFI = 97,
  IPR_SCI7_RXI7 = 98,
  IPR_SCI7_TXI7 = 99,
  IPR_SCI8_RXI8 = 100,
  IPR_SCI8_TXI8 = 101,
  IPR_SCI9_RXI9 = 102,
  IPR_SCI9_TXI9 = 103,
  IPR_SCI10_RXI10 = 104,
  IPR_SCI10_TXI10 = 105,
  IPR_ICU_GROUPBE0 = 106,
  IPR_ICU_GROUPBL2 = 107,
  IPR_RSPI2_SPRI2 = 108,
  IPR_RSPI2_SPTI2 = 109,
  IPR_ICU_GROUPBL0 = 110,
  IPR_ICU_GROUPBL1 = 111,
  IPR_ICU_GROUPAL0 = 112,
  IPR_ICU_GROUPAL1 = 113,
  IPR_SCI11_RXI11 = 114,
  IPR_SCI11_TXI11 = 115,
  IPR_SCI12_RXI12 = 116,
  IPR_SCI12_TXI12 = 117,
  IPR_DMAC_DMAC0I = 120,
  IPR_DMAC_DMAC1I = 121,
  IPR_DMAC_DMAC2I = 122,
  IPR_DMAC_DMAC3I = 123,
  IPR_DMAC_DMAC74I = 124,
  IPR_OST_OSTDI = 125,
  IPR_EXDMAC_EXDMAC0I = 126,
  IPR_EXDMAC_EXDMAC1I = 127,
  IPR_PERIB_INTB128 = 128,
  IPR_PERIB_INTB129 = 129,
  IPR_PERIB_INTB130 = 130,
  IPR_PERIB_INTB131 = 131,
  IPR_PERIB_INTB132 = 132,
  IPR_PERIB_INTB133 = 133,
  IPR_PERIB_INTB134 = 134,
  IPR_PERIB_INTB135 = 135,
  IPR_PERIB_INTB136 = 136,
  IPR_PERIB_INTB137 = 137,
  IPR_PERIB_INTB138 = 138,
  IPR_PERIB_INTB139 = 139,
  IPR_PERIB_INTB140 = 140,
  IPR_PERIB_INTB141 = 141,
  IPR_PERIB_INTB142 = 142,
  IPR_PERIB_INTB143 = 143,
  IPR_PERIB_INTB144 = 144,
  IPR_PERIB_INTB145 = 145,
  IPR_PERIB_INTB146 = 146,
  IPR_PERIB_INTB147 = 147,
  IPR_PERIB_INTB148 = 148,
  IPR_PERIB_INTB149 = 149,
  IPR_PERIB_INTB150 = 150,
  IPR_PERIB_INTB151 = 151,
  IPR_PERIB_INTB152 = 152,
  IPR_PERIB_INTB153 = 153,
  IPR_PERIB_INTB154 = 154,
  IPR_PERIB_INTB155 = 155,
  IPR_PERIB_INTB156 = 156,
  IPR_PERIB_INTB157 = 157,
  IPR_PERIB_INTB158 = 158,
  IPR_PERIB_INTB159 = 159,
  IPR_PERIB_INTB160 = 160,
  IPR_PERIB_INTB161 = 161,
  IPR_PERIB_INTB162 = 162,
  IPR_PERIB_INTB163 = 163,
  IPR_PERIB_INTB164 = 164,
  IPR_PERIB_INTB165 = 165,
  IPR_PERIB_INTB166 = 166,
  IPR_PERIB_INTB167 = 167,
  IPR_PERIB_INTB168 = 168,
  IPR_PERIB_INTB169 = 169,
  IPR_PERIB_INTB170 = 170,
  IPR_PERIB_INTB171 = 171,
  IPR_PERIB_INTB172 = 172,
  IPR_PERIB_INTB173 = 173,
  IPR_PERIB_INTB174 = 174,
  IPR_PERIB_INTB175 = 175,
  IPR_PERIB_INTB176 = 176,
  IPR_PERIB_INTB177 = 177,
  IPR_PERIB_INTB178 = 178,
  IPR_PERIB_INTB179 = 179,
  IPR_PERIB_INTB180 = 180,
  IPR_PERIB_INTB181 = 181,
  IPR_PERIB_INTB182 = 182,
  IPR_PERIB_INTB183 = 183,
  IPR_PERIB_INTB184 = 184,
  IPR_PERIB_INTB185 = 185,
  IPR_PERIB_INTB186 = 186,
  IPR_PERIB_INTB187 = 187,
  IPR_PERIB_INTB188 = 188,
  IPR_PERIB_INTB189 = 189,
  IPR_PERIB_INTB190 = 190,
  IPR_PERIB_INTB191 = 191,
  IPR_PERIB_INTB192 = 192,
  IPR_PERIB_INTB193 = 193,
  IPR_PERIB_INTB194 = 194,
  IPR_PERIB_INTB195 = 195,
  IPR_PERIB_INTB196 = 196,
  IPR_PERIB_INTB197 = 197,
  IPR_PERIB_INTB198 = 198,
  IPR_PERIB_INTB199 = 199,
  IPR_PERIB_INTB200 = 200,
  IPR_PERIB_INTB201 = 201,
  IPR_PERIB_INTB202 = 202,
  IPR_PERIB_INTB203 = 203,
  IPR_PERIB_INTB204 = 204,
  IPR_PERIB_INTB205 = 205,
  IPR_PERIB_INTB206 = 206,
  IPR_PERIB_INTB207 = 207,
  IPR_PERIA_INTA208 = 208,
  IPR_PERIA_INTA209 = 209,
  IPR_PERIA_INTA210 = 210,
  IPR_PERIA_INTA211 = 211,
  IPR_PERIA_INTA212 = 212,
  IPR_PERIA_INTA213 = 213,
  IPR_PERIA_INTA214 = 214,
  IPR_PERIA_INTA215 = 215,
  IPR_PERIA_INTA216 = 216,
  IPR_PERIA_INTA217 = 217,
  IPR_PERIA_INTA218 = 218,
  IPR_PERIA_INTA219 = 219,
  IPR_PERIA_INTA220 = 220,
  IPR_PERIA_INTA221 = 221,
  IPR_PERIA_INTA222 = 222,
  IPR_PERIA_INTA223 = 223,
  IPR_PERIA_INTA224 = 224,
  IPR_PERIA_INTA225 = 225,
  IPR_PERIA_INTA226 = 226,
  IPR_PERIA_INTA227 = 227,
  IPR_PERIA_INTA228 = 228,
  IPR_PERIA_INTA229 = 229,
  IPR_PERIA_INTA230 = 230,
  IPR_PERIA_INTA231 = 231,
  IPR_PERIA_INTA232 = 232,
  IPR_PERIA_INTA233 = 233,
  IPR_PERIA_INTA234 = 234,
  IPR_PERIA_INTA235 = 235,
  IPR_PERIA_INTA236 = 236,
  IPR_PERIA_INTA237 = 237,
  IPR_PERIA_INTA238 = 238,
  IPR_PERIA_INTA239 = 239,
  IPR_PERIA_INTA240 = 240,
  IPR_PERIA_INTA241 = 241,
  IPR_PERIA_INTA242 = 242,
  IPR_PERIA_INTA243 = 243,
  IPR_PERIA_INTA244 = 244,
  IPR_PERIA_INTA245 = 245,
  IPR_PERIA_INTA246 = 246,
  IPR_PERIA_INTA247 = 247,
  IPR_PERIA_INTA248 = 248,
  IPR_PERIA_INTA249 = 249,
  IPR_PERIA_INTA250 = 250,
  IPR_PERIA_INTA251 = 251,
  IPR_PERIA_INTA252 = 252,
  IPR_PERIA_INTA253 = 253,
  IPR_PERIA_INTA254 = 254,
  IPR_PERIA_INTA255 = 255,
  IPR_ICU_SWI = 3,
  IPR_CMT0_ = 4,
  IPR_CMT1_ = 5,
  IPR_CMTW0_ = 6,
  IPR_CMTW1_ = 7,
  IPR_SDHI_ = 44,
  IPR_MMCIF_ = 45,
  IPR_LVD1_ = 88,
  IPR_LVD2_ = 89,
  IPR_IWDT_ = 95,
  IPR_WDT_ = 96,
  IPR_PDC_ = 97,
  IPR_OST_ = 125
} enum_ipr_t;

#pragma pack(4)

struct st_bsc_berclr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char STSCLR : 1;
  unsigned char : 7;
#else
  unsigned char : 7;
  unsigned char STSCLR : 1;
#endif
};

union un_berclr
{
  unsigned char BYTE;
  struct st_bsc_berclr_bit BIT;
};

struct st_bsc_beren_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IGAEN : 1;
  unsigned char TOEN : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TOEN : 1;
  unsigned char IGAEN : 1;
#endif
};

union un_beren
{
  unsigned char BYTE;
  struct st_bsc_beren_bit BIT;
};

struct st_bsc_bersr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IA : 1;
  unsigned char TO : 1;
  unsigned char  : 2;
  unsigned char MST : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MST : 3;
  unsigned char  : 2;
  unsigned char TO : 1;
  unsigned char IA : 1;
#endif
};

union un_bersr1
{
  unsigned char BYTE;
  struct st_bsc_bersr1_bit BIT;
};

struct st_bsc_bersr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 3;
  unsigned short ADDR : 13;
#else
  unsigned short ADDR : 13;
  unsigned short  : 3;
#endif
};

union un_bersr2
{
  unsigned short WORD;
  struct st_bsc_bersr2_bit BIT;
};

struct st_bsc_buspri_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short BPRA : 2;
  unsigned short BPRO : 2;
  unsigned short BPIB : 2;
  unsigned short BPGB : 2;
  unsigned short BPHB : 2;
  unsigned short BPFB : 2;
  unsigned short BPEB : 2;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short BPEB : 2;
  unsigned short BPFB : 2;
  unsigned short BPHB : 2;
  unsigned short BPGB : 2;
  unsigned short BPIB : 2;
  unsigned short BPRO : 2;
  unsigned short BPRA : 2;
#endif
};

union un_buspri
{
  unsigned short WORD;
  struct st_bsc_buspri_bit BIT;
};

struct st_bsc_cs0mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs0mod
{
  unsigned short WORD;
  struct st_bsc_cs0mod_bit BIT;
};

struct st_bsc_cs0wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs0wcr1
{
  unsigned long LONG;
  struct st_bsc_cs0wcr1_bit BIT;
};

struct st_bsc_cs1mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs1mod
{
  unsigned short WORD;
  struct st_bsc_cs1mod_bit BIT;
};

struct st_bsc_cs1wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs1wcr1
{
  unsigned long LONG;
  struct st_bsc_cs1wcr1_bit BIT;
};

struct st_bsc_cs1wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs1wcr2
{
  unsigned long LONG;
  struct st_bsc_cs1wcr2_bit BIT;
};

struct st_bsc_cs2mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs2mod
{
  unsigned short WORD;
  struct st_bsc_cs2mod_bit BIT;
};

struct st_bsc_cs2wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs2wcr1
{
  unsigned long LONG;
  struct st_bsc_cs2wcr1_bit BIT;
};

struct st_bsc_cs2wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs2wcr2
{
  unsigned long LONG;
  struct st_bsc_cs2wcr2_bit BIT;
};

struct st_bsc_cs3mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs3mod
{
  unsigned short WORD;
  struct st_bsc_cs3mod_bit BIT;
};

struct st_bsc_cs3wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs3wcr1
{
  unsigned long LONG;
  struct st_bsc_cs3wcr1_bit BIT;
};

struct st_bsc_cs3wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs3wcr2
{
  unsigned long LONG;
  struct st_bsc_cs3wcr2_bit BIT;
};

struct st_bsc_cs4mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs4mod
{
  unsigned short WORD;
  struct st_bsc_cs4mod_bit BIT;
};

struct st_bsc_cs4wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs4wcr1
{
  unsigned long LONG;
  struct st_bsc_cs4wcr1_bit BIT;
};

struct st_bsc_cs4wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs4wcr2
{
  unsigned long LONG;
  struct st_bsc_cs4wcr2_bit BIT;
};

struct st_bsc_cs5mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs5mod
{
  unsigned short WORD;
  struct st_bsc_cs5mod_bit BIT;
};

struct st_bsc_cs5wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs5wcr1
{
  unsigned long LONG;
  struct st_bsc_cs5wcr1_bit BIT;
};

struct st_bsc_cs5wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs5wcr2
{
  unsigned long LONG;
  struct st_bsc_cs5wcr2_bit BIT;
};

struct st_bsc_cs6mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs6mod
{
  unsigned short WORD;
  struct st_bsc_cs6mod_bit BIT;
};

struct st_bsc_cs6wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs6wcr1
{
  unsigned long LONG;
  struct st_bsc_cs6wcr1_bit BIT;
};

struct st_bsc_cs6wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs6wcr2
{
  unsigned long LONG;
  struct st_bsc_cs6wcr2_bit BIT;
};

struct st_bsc_cs7mod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short WRMOD : 1;
  unsigned short  : 2;
  unsigned short EWENB : 1;
  unsigned short  : 4;
  unsigned short PRENB : 1;
  unsigned short PWENB : 1;
  unsigned short  : 5;
  unsigned short PRMOD : 1;
#else
  unsigned short PRMOD : 1;
  unsigned short  : 5;
  unsigned short PWENB : 1;
  unsigned short PRENB : 1;
  unsigned short  : 4;
  unsigned short EWENB : 1;
  unsigned short  : 2;
  unsigned short WRMOD : 1;
#endif
};

union un_cs7mod
{
  unsigned short WORD;
  struct st_bsc_cs7mod_bit BIT;
};

struct st_bsc_cs7wcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSPWWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSWWAIT : 5;
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long CSRWAIT : 5;
  unsigned long  : 3;
  unsigned long CSWWAIT : 5;
  unsigned long  : 5;
  unsigned long CSPRWAIT : 3;
  unsigned long  : 5;
  unsigned long CSPWWAIT : 3;
#endif
};

union un_cs7wcr1
{
  unsigned long LONG;
  struct st_bsc_cs7wcr1_bit BIT;
};

struct st_bsc_cs7wcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CSROFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long AWAIT : 2;
  unsigned long  : 2;
  unsigned long RDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long CSON : 3;
  unsigned long  : 1;
  unsigned long WDON : 3;
  unsigned long  : 1;
  unsigned long WRON : 3;
  unsigned long  : 1;
  unsigned long RDON : 3;
  unsigned long  : 2;
  unsigned long AWAIT : 2;
  unsigned long  : 1;
  unsigned long WDOFF : 3;
  unsigned long  : 1;
  unsigned long CSWOFF : 3;
  unsigned long  : 1;
  unsigned long CSROFF : 3;
#endif
};

union un_cs7wcr2
{
  unsigned long LONG;
  struct st_bsc_cs7wcr2_bit BIT;
};

struct st_bsc_cs0cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs0cr
{
  unsigned short WORD;
  struct st_bsc_cs0cr_bit BIT;
};

struct st_bsc_cs0rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs0rec
{
  unsigned short WORD;
  struct st_bsc_cs0rec_bit BIT;
};

struct st_bsc_cs1cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs1cr
{
  unsigned short WORD;
  struct st_bsc_cs1cr_bit BIT;
};

struct st_bsc_cs1rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs1rec
{
  unsigned short WORD;
  struct st_bsc_cs1rec_bit BIT;
};

struct st_bsc_cs2cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs2cr
{
  unsigned short WORD;
  struct st_bsc_cs2cr_bit BIT;
};

struct st_bsc_cs2rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs2rec
{
  unsigned short WORD;
  struct st_bsc_cs2rec_bit BIT;
};

struct st_bsc_cs3cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs3cr
{
  unsigned short WORD;
  struct st_bsc_cs3cr_bit BIT;
};

struct st_bsc_cs3rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs3rec
{
  unsigned short WORD;
  struct st_bsc_cs3rec_bit BIT;
};

struct st_bsc_cs4cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs4cr
{
  unsigned short WORD;
  struct st_bsc_cs4cr_bit BIT;
};

struct st_bsc_cs4rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs4rec
{
  unsigned short WORD;
  struct st_bsc_cs4rec_bit BIT;
};

struct st_bsc_cs5cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs5cr
{
  unsigned short WORD;
  struct st_bsc_cs5cr_bit BIT;
};

struct st_bsc_cs5rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs5rec
{
  unsigned short WORD;
  struct st_bsc_cs5rec_bit BIT;
};

struct st_bsc_cs6cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs6cr
{
  unsigned short WORD;
  struct st_bsc_cs6cr_bit BIT;
};

struct st_bsc_cs6rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs6rec
{
  unsigned short WORD;
  struct st_bsc_cs6rec_bit BIT;
};

struct st_bsc_cs7cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short EXENB : 1;
  unsigned short  : 3;
  unsigned short BSIZE : 2;
  unsigned short  : 2;
  unsigned short EMODE : 1;
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short MPXEN : 1;
  unsigned short  : 3;
  unsigned short EMODE : 1;
  unsigned short  : 2;
  unsigned short BSIZE : 2;
  unsigned short  : 3;
  unsigned short EXENB : 1;
#endif
};

union un_cs7cr
{
  unsigned short WORD;
  struct st_bsc_cs7cr_bit BIT;
};

struct st_bsc_cs7rec_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RRCV : 4;
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short WRCV : 4;
  unsigned short  : 4;
  unsigned short RRCV : 4;
#endif
};

union un_cs7rec
{
  unsigned short WORD;
  struct st_bsc_cs7rec_bit BIT;
};

struct st_bsc_csrecen_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RCVEN0 : 1;
  unsigned short RCVEN1 : 1;
  unsigned short RCVEN2 : 1;
  unsigned short RCVEN3 : 1;
  unsigned short RCVEN4 : 1;
  unsigned short RCVEN5 : 1;
  unsigned short RCVEN6 : 1;
  unsigned short RCVEN7 : 1;
  unsigned short RCVENM0 : 1;
  unsigned short RCVENM1 : 1;
  unsigned short RCVENM2 : 1;
  unsigned short RCVENM3 : 1;
  unsigned short RCVENM4 : 1;
  unsigned short RCVENM5 : 1;
  unsigned short RCVENM6 : 1;
  unsigned short RCVENM7 : 1;
#else
  unsigned short RCVENM7 : 1;
  unsigned short RCVENM6 : 1;
  unsigned short RCVENM5 : 1;
  unsigned short RCVENM4 : 1;
  unsigned short RCVENM3 : 1;
  unsigned short RCVENM2 : 1;
  unsigned short RCVENM1 : 1;
  unsigned short RCVENM0 : 1;
  unsigned short RCVEN7 : 1;
  unsigned short RCVEN6 : 1;
  unsigned short RCVEN5 : 1;
  unsigned short RCVEN4 : 1;
  unsigned short RCVEN3 : 1;
  unsigned short RCVEN2 : 1;
  unsigned short RCVEN1 : 1;
  unsigned short RCVEN0 : 1;
#endif
};

union un_csrecen
{
  unsigned short WORD;
  struct st_bsc_csrecen_bit BIT;
};

struct st_bsc_sdccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EXENB : 1;
  unsigned char  : 3;
  unsigned char BSIZE : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BSIZE : 2;
  unsigned char  : 3;
  unsigned char EXENB : 1;
#endif
};

union un_sdccr
{
  unsigned char BYTE;
  struct st_bsc_sdccr_bit BIT;
};

struct st_bsc_sdcmod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EMODE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char EMODE : 1;
#endif
};

union un_sdcmod
{
  unsigned char BYTE;
  struct st_bsc_sdcmod_bit BIT;
};

struct st_bsc_sdamod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char BE : 1;
#endif
};

union un_sdamod
{
  unsigned char BYTE;
  struct st_bsc_sdamod_bit BIT;
};

struct st_bsc_sdself_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SFEN : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SFEN : 1;
#endif
};

union un_sdself
{
  unsigned char BYTE;
  struct st_bsc_sdself_bit BIT;
};

struct st_bsc_sdrfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RFC : 12;
  unsigned short REFW : 4;
#else
  unsigned short REFW : 4;
  unsigned short RFC : 12;
#endif
};

union un_sdrfcr
{
  unsigned short WORD;
  struct st_bsc_sdrfcr_bit BIT;
};

struct st_bsc_sdrfen_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RFEN : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char RFEN : 1;
#endif
};

union un_sdrfen
{
  unsigned char BYTE;
  struct st_bsc_sdrfen_bit BIT;
};

struct st_bsc_sdicr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char INIRQ : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char INIRQ : 1;
#endif
};

union un_sdicr
{
  unsigned char BYTE;
  struct st_bsc_sdicr_bit BIT;
};

struct st_bsc_sdir_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ARFI : 4;
  unsigned short ARFC : 4;
  unsigned short PRC : 3;
  unsigned short  : 5;
#else
  unsigned short  : 5;
  unsigned short PRC : 3;
  unsigned short ARFC : 4;
  unsigned short ARFI : 4;
#endif
};

union un_sdir
{
  unsigned short WORD;
  struct st_bsc_sdir_bit BIT;
};

struct st_bsc_sdar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MXC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char MXC : 2;
#endif
};

union un_sdar
{
  unsigned char BYTE;
  struct st_bsc_sdar_bit BIT;
};

struct st_bsc_sdtr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CL : 3;
  unsigned long  : 5;
  unsigned long WR : 1;
  unsigned long RP : 3;
  unsigned long RCD : 2;
  unsigned long  : 2;
  unsigned long RAS : 3;
  unsigned long  : 13;
#else
  unsigned long  : 13;
  unsigned long RAS : 3;
  unsigned long  : 2;
  unsigned long RCD : 2;
  unsigned long RP : 3;
  unsigned long WR : 1;
  unsigned long  : 5;
  unsigned long CL : 3;
#endif
};

union un_sdtr
{
  unsigned long LONG;
  struct st_bsc_sdtr_bit BIT;
};

struct st_bsc_sdmod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short MR : 15;
  unsigned short  : 1;
#else
  unsigned short  : 1;
  unsigned short MR : 15;
#endif
};

union un_sdmod
{
  unsigned short WORD;
  struct st_bsc_sdmod_bit BIT;
};

struct st_bsc_sdsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MRSST : 1;
  unsigned char  : 2;
  unsigned char INIST : 1;
  unsigned char SRFST : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char SRFST : 1;
  unsigned char INIST : 1;
  unsigned char  : 2;
  unsigned char MRSST : 1;
#endif
};

union un_sdsr
{
  unsigned char BYTE;
  struct st_bsc_sdsr_bit BIT;
};

struct st_bsc_ebmapcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PR1SEL : 3;
  unsigned long  : 1;
  unsigned long PR2SEL : 3;
  unsigned long  : 1;
  unsigned long PR3SEL : 3;
  unsigned long  : 1;
  unsigned long PR4SEL : 3;
  unsigned long  : 1;
  unsigned long PR5SEL : 3;
  unsigned long  : 10;
  unsigned long RPSTOP : 1;
  unsigned long  : 1;
  unsigned long PRERR : 1;
#else
  unsigned long PRERR : 1;
  unsigned long  : 1;
  unsigned long RPSTOP : 1;
  unsigned long  : 10;
  unsigned long PR5SEL : 3;
  unsigned long  : 1;
  unsigned long PR4SEL : 3;
  unsigned long  : 1;
  unsigned long PR3SEL : 3;
  unsigned long  : 1;
  unsigned long PR2SEL : 3;
  unsigned long  : 1;
  unsigned long PR1SEL : 3;
#endif
};

union un_ebmapcr
{
  unsigned long LONG;
  struct st_bsc_ebmapcr_bit BIT;
};

struct st_cac_cacr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CFME : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char CFME : 1;
#endif
};

union un_cacr0
{
  unsigned char BYTE;
  struct st_cac_cacr0_bit BIT;
};

struct st_cac_cacr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CACREFE : 1;
  unsigned char FMCS : 3;
  unsigned char TCSS : 2;
  unsigned char EDGES : 2;
#else
  unsigned char EDGES : 2;
  unsigned char TCSS : 2;
  unsigned char FMCS : 3;
  unsigned char CACREFE : 1;
#endif
};

union un_cacr1
{
  unsigned char BYTE;
  struct st_cac_cacr1_bit BIT;
};

struct st_cac_cacr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RPS : 1;
  unsigned char RSCS : 3;
  unsigned char RCDS : 2;
  unsigned char DFS : 2;
#else
  unsigned char DFS : 2;
  unsigned char RCDS : 2;
  unsigned char RSCS : 3;
  unsigned char RPS : 1;
#endif
};

union un_cacr2
{
  unsigned char BYTE;
  struct st_cac_cacr2_bit BIT;
};

struct st_cac_caicr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FERRIE : 1;
  unsigned char MENDIE : 1;
  unsigned char OVFIE : 1;
  unsigned char  : 1;
  unsigned char FERRFCL : 1;
  unsigned char MENDFCL : 1;
  unsigned char OVFFCL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char OVFFCL : 1;
  unsigned char MENDFCL : 1;
  unsigned char FERRFCL : 1;
  unsigned char  : 1;
  unsigned char OVFIE : 1;
  unsigned char MENDIE : 1;
  unsigned char FERRIE : 1;
#endif
};

union un_caicr
{
  unsigned char BYTE;
  struct st_cac_caicr_bit BIT;
};

struct st_cac_castr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FERRF : 1;
  unsigned char MENDF : 1;
  unsigned char OVFF : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char OVFF : 1;
  unsigned char MENDF : 1;
  unsigned char FERRF : 1;
#endif
};

union un_castr
{
  unsigned char BYTE;
  struct st_cac_castr_bit BIT;
};

struct st_cmt_cmstr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short STR0 : 1;
  unsigned short STR1 : 1;
  unsigned short  : 14;
#else
  unsigned short  : 14;
  unsigned short STR1 : 1;
  unsigned short STR0 : 1;
#endif
};

union un_cmstr0
{
  unsigned short WORD;
  struct st_cmt_cmstr0_bit BIT;
};

struct st_cmt_cmstr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short STR2 : 1;
  unsigned short STR3 : 1;
  unsigned short  : 14;
#else
  unsigned short  : 14;
  unsigned short STR3 : 1;
  unsigned short STR2 : 1;
#endif
};

union un_cmstr1
{
  unsigned short WORD;
  struct st_cmt_cmstr1_bit BIT;
};

struct st_cmt0_cmcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CKS : 2;
  unsigned short  : 4;
  unsigned short CMIE : 1;
  unsigned short  : 9;
#else
  unsigned short  : 9;
  unsigned short CMIE : 1;
  unsigned short  : 4;
  unsigned short CKS : 2;
#endif
};

union un_cmcr
{
  unsigned short WORD;
  struct st_cmt0_cmcr_bit BIT;
};

struct st_cmtw_cmwstr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short STR : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short STR : 1;
#endif
};

union un_cmtw_cmwstr
{
  unsigned short WORD;
  struct st_cmtw_cmwstr_bit BIT;
};

struct st_cmtw_cmwcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CKS : 2;
  unsigned short  : 1;
  unsigned short CMWIE : 1;
  unsigned short IC0IE : 1;
  unsigned short IC1IE : 1;
  unsigned short OC0IE : 1;
  unsigned short OC1IE : 1;
  unsigned short  : 1;
  unsigned short CMS : 1;
  unsigned short  : 3;
  unsigned short CCLR : 3;
#else
  unsigned short CCLR : 3;
  unsigned short  : 3;
  unsigned short CMS : 1;
  unsigned short  : 1;
  unsigned short OC1IE : 1;
  unsigned short OC0IE : 1;
  unsigned short IC1IE : 1;
  unsigned short IC0IE : 1;
  unsigned short CMWIE : 1;
  unsigned short  : 1;
  unsigned short CKS : 2;
#endif
};

union un_cmtw_cmwcr
{
  unsigned short WORD;
  struct st_cmtw_cmwcr_bit BIT;
};

struct st_cmtw_cmwior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short IC0 : 2;
  unsigned short IC1 : 2;
  unsigned short IC0E : 1;
  unsigned short IC1E : 1;
  unsigned short  : 2;
  unsigned short OC0 : 2;
  unsigned short OC1 : 2;
  unsigned short OC0E : 1;
  unsigned short OC1E : 1;
  unsigned short  : 1;
  unsigned short CMWE : 1;
#else
  unsigned short CMWE : 1;
  unsigned short  : 1;
  unsigned short OC1E : 1;
  unsigned short OC0E : 1;
  unsigned short OC1 : 2;
  unsigned short OC0 : 2;
  unsigned short  : 2;
  unsigned short IC1E : 1;
  unsigned short IC0E : 1;
  unsigned short IC1 : 2;
  unsigned short IC0 : 2;
#endif
};

union un_cmtw_cmwior
{
  unsigned short WORD;
  struct st_cmtw_cmwior_bit BIT;
};

struct st_icu_ier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IEN0 : 1;
  unsigned char IEN1 : 1;
  unsigned char IEN2 : 1;
  unsigned char IEN3 : 1;
  unsigned char IEN4 : 1;
  unsigned char IEN5 : 1;
  unsigned char IEN6 : 1;
  unsigned char IEN7 : 1;
#else
  unsigned char IEN7 : 1;
  unsigned char IEN6 : 1;
  unsigned char IEN5 : 1;
  unsigned char IEN4 : 1;
  unsigned char IEN3 : 1;
  unsigned char IEN2 : 1;
  unsigned char IEN1 : 1;
  unsigned char IEN0 : 1;
#endif
};

union un_ier32
{
  unsigned char BYTE;
  struct st_icu_ier_bit BIT;
};

struct st_icu_irqcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 2;
  unsigned char IRQMD : 2;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char IRQMD : 2;
  unsigned char  : 2;
#endif
};

union un_irqcr16
{
  unsigned char BYTE;
  struct st_icu_irqcr_bit BIT;
};

struct st_icu_swintr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWINT : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SWINT : 1;
#endif
};

union un_swintr
{
  unsigned char BYTE;
  struct st_icu_swintr_bit BIT;
};

struct st_icu_swint2r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWINT2 : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SWINT2 : 1;
#endif
};

union un_swint2r
{
  unsigned char BYTE;
  struct st_icu_swint2r_bit BIT;
};

struct st_icu_grpbl0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_grpbl0
{
  unsigned long LONG;
  struct st_icu_grpbl0_bit BIT;
};

struct st_icu_genbl0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_genbl0
{
  unsigned long LONG;
  struct st_icu_genbl0_bit BIT;
};

struct st_mpc_pwpr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char PFSWE : 1;
  unsigned char B0WI : 1;
#else
  unsigned char B0WI : 1;
  unsigned char PFSWE : 1;
  unsigned char  : 6;
#endif
};

struct st_icu_grpbl1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_grpbl1
{
  unsigned long LONG;
  struct st_icu_grpbl1_bit BIT;
};

struct st_icu_genbl1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_genbl1
{
  unsigned long LONG;
  struct st_icu_genbl1_bit BIT;
};

struct st_icu_grpal0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_grpal0
{
  unsigned long LONG;
  struct st_icu_grpal0_bit BIT;
};

struct st_icu_genal0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_genal0
{
  unsigned long LONG;
  struct st_icu_genal0_bit BIT;
};

struct st_icu_grpal1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_grpal1
{
  unsigned long LONG;
  struct st_icu_grpal1_bit BIT;
};

struct st_icu_genal1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_genal1
{
  unsigned long LONG;
  struct st_icu_genal1_bit BIT;
};

union un_pwpr
{
  unsigned char BYTE;
  struct st_mpc_pwpr_bit BIT;
};

struct st_icu_ir256_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char IR : 1;
#endif
};

union un_icu_ir256
{
  unsigned char BYTE;
  struct st_icu_ir256_bit BIT;
};

struct st_icu_dtcer256_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTCE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTCE : 1;
#endif
};

union un_icu_dtcer256
{
  unsigned char BYTE;
  struct st_icu_dtcer256_bit BIT;
};

struct st_icu_fir_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short FVCT : 8;
  unsigned short  : 7;
  unsigned short FIEN : 1;
#else
  unsigned short FIEN : 1;
  unsigned short  : 7;
  unsigned short FVCT : 8;
#endif
};

union un_icu_fir
{
  unsigned short WORD;
  struct st_icu_fir_bit BIT;
};

struct st_icu_ipr256_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IPR : 4;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char IPR : 4;
#endif
};

union un_icu_ipr256
{
  unsigned char BYTE;
  struct st_icu_ipr256_bit BIT;
};

struct st_icu_irqflte0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FLTEN0 : 1;
  unsigned char FLTEN1 : 1;
  unsigned char FLTEN2 : 1;
  unsigned char FLTEN3 : 1;
  unsigned char FLTEN4 : 1;
  unsigned char FLTEN5 : 1;
  unsigned char FLTEN6 : 1;
  unsigned char FLTEN7 : 1;
#else
  unsigned char FLTEN7 : 1;
  unsigned char FLTEN6 : 1;
  unsigned char FLTEN5 : 1;
  unsigned char FLTEN4 : 1;
  unsigned char FLTEN3 : 1;
  unsigned char FLTEN2 : 1;
  unsigned char FLTEN1 : 1;
  unsigned char FLTEN0 : 1;
#endif
};

union un_irqflte0
{
  unsigned char BYTE;
  struct st_icu_irqflte0_bit BIT;
};

struct st_icu_irqflte1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FLTEN8 : 1;
  unsigned char FLTEN9 : 1;
  unsigned char FLTEN10 : 1;
  unsigned char FLTEN11 : 1;
  unsigned char FLTEN12 : 1;
  unsigned char FLTEN13 : 1;
  unsigned char FLTEN14 : 1;
  unsigned char FLTEN15 : 1;
#else
  unsigned char FLTEN15 : 1;
  unsigned char FLTEN14 : 1;
  unsigned char FLTEN13 : 1;
  unsigned char FLTEN12 : 1;
  unsigned char FLTEN11 : 1;
  unsigned char FLTEN10 : 1;
  unsigned char FLTEN9 : 1;
  unsigned char FLTEN8 : 1;
#endif
};

union un_irqflte1
{
  unsigned char BYTE;
  struct st_icu_irqflte1_bit BIT;
};

struct st_icu_irqfltc0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short FCLKSEL0 : 2;
  unsigned short FCLKSEL1 : 2;
  unsigned short FCLKSEL2 : 2;
  unsigned short FCLKSEL3 : 2;
  unsigned short FCLKSEL4 : 2;
  unsigned short FCLKSEL5 : 2;
  unsigned short FCLKSEL6 : 2;
  unsigned short FCLKSEL7 : 2;
#else
  unsigned short FCLKSEL7 : 2;
  unsigned short FCLKSEL6 : 2;
  unsigned short FCLKSEL5 : 2;
  unsigned short FCLKSEL4 : 2;
  unsigned short FCLKSEL3 : 2;
  unsigned short FCLKSEL2 : 2;
  unsigned short FCLKSEL1 : 2;
  unsigned short FCLKSEL0 : 2;
#endif
};

union un_icu_irqfltc0
{
  unsigned short WORD;
  struct st_icu_irqfltc0_bit BIT;
};

struct st_icu_irqfltc1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short FCLKSEL8 : 2;
  unsigned short FCLKSEL9 : 2;
  unsigned short FCLKSEL10 : 2;
  unsigned short FCLKSEL11 : 2;
  unsigned short FCLKSEL12 : 2;
  unsigned short FCLKSEL13 : 2;
  unsigned short FCLKSEL14 : 2;
  unsigned short FCLKSEL15 : 2;
#else
  unsigned short FCLKSEL15 : 2;
  unsigned short FCLKSEL14 : 2;
  unsigned short FCLKSEL13 : 2;
  unsigned short FCLKSEL12 : 2;
  unsigned short FCLKSEL11 : 2;
  unsigned short FCLKSEL10 : 2;
  unsigned short FCLKSEL9 : 2;
  unsigned short FCLKSEL8 : 2;
#endif
};

union un_icu_irqfltc1
{
  unsigned short WORD;
  struct st_icu_irqfltc1_bit BIT;
};

struct st_icu_nmisr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NMIST : 1;
  unsigned char OSTST : 1;
  unsigned char WDTST : 1;
  unsigned char IWDTST : 1;
  unsigned char LVD1ST : 1;
  unsigned char LVD2ST : 1;
  unsigned char RAMST : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char RAMST : 1;
  unsigned char LVD2ST : 1;
  unsigned char LVD1ST : 1;
  unsigned char IWDTST : 1;
  unsigned char WDTST : 1;
  unsigned char OSTST : 1;
  unsigned char NMIST : 1;
#endif
};

union un_icu_nmisr
{
  unsigned char BYTE;
  struct st_icu_nmisr_bit BIT;
};

struct st_icu_nmier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NMIEN : 1;
  unsigned char OSTEN : 1;
  unsigned char WDTEN : 1;
  unsigned char IWDTEN : 1;
  unsigned char LVD1EN : 1;
  unsigned char LVD2EN : 1;
  unsigned char RAMEN : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char RAMEN : 1;
  unsigned char LVD2EN : 1;
  unsigned char LVD1EN : 1;
  unsigned char IWDTEN : 1;
  unsigned char WDTEN : 1;
  unsigned char OSTEN : 1;
  unsigned char NMIEN : 1;
#endif
};

union un_icu_nmier
{
  unsigned char BYTE;
  struct st_icu_nmier_bit BIT;
};

struct st_icu_nmiclr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NMICLR : 1;
  unsigned char OSTCLR : 1;
  unsigned char WDTCLR : 1;
  unsigned char IWDTCLR : 1;
  unsigned char LVD1CLR : 1;
  unsigned char LVD2CLR : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char LVD2CLR : 1;
  unsigned char LVD1CLR : 1;
  unsigned char IWDTCLR : 1;
  unsigned char WDTCLR : 1;
  unsigned char OSTCLR : 1;
  unsigned char NMICLR : 1;
#endif
};

union un_icu_nmiclr
{
  unsigned char BYTE;
  struct st_icu_nmiclr_bit BIT;
};

struct st_icu_nmicr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 3;
  unsigned char NMIMD : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char NMIMD : 1;
  unsigned char  : 3;
#endif
};

union un_icu_nmicr
{
  unsigned char BYTE;
  struct st_icu_nmicr_bit BIT;
};

struct st_icu_nmiflte_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFLTEN : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char NFLTEN : 1;
#endif
};

union un_icu_nmiflte
{
  unsigned char BYTE;
  struct st_icu_nmiflte_bit BIT;
};

struct st_icu_nmifltc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFCLKSEL : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char NFCLKSEL : 2;
#endif
};

union un_icu_nmifltc
{
  unsigned char BYTE;
  struct st_icu_nmifltc_bit BIT;
};

struct st_icu_grpbe0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_icu_grpbe0
{
  unsigned long LONG;
  struct st_icu_grpbe0_bit BIT;
};

struct st_icu_grpbl2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IS0 : 1;
  unsigned long IS1 : 1;
  unsigned long IS2 : 1;
  unsigned long IS3 : 1;
  unsigned long IS4 : 1;
  unsigned long IS5 : 1;
  unsigned long IS6 : 1;
  unsigned long IS7 : 1;
  unsigned long IS8 : 1;
  unsigned long IS9 : 1;
  unsigned long IS10 : 1;
  unsigned long IS11 : 1;
  unsigned long IS12 : 1;
  unsigned long IS13 : 1;
  unsigned long IS14 : 1;
  unsigned long IS15 : 1;
  unsigned long IS16 : 1;
  unsigned long IS17 : 1;
  unsigned long IS18 : 1;
  unsigned long IS19 : 1;
  unsigned long IS20 : 1;
  unsigned long IS21 : 1;
  unsigned long IS22 : 1;
  unsigned long IS23 : 1;
  unsigned long IS24 : 1;
  unsigned long IS25 : 1;
  unsigned long IS26 : 1;
  unsigned long IS27 : 1;
  unsigned long IS28 : 1;
  unsigned long IS29 : 1;
  unsigned long IS30 : 1;
  unsigned long IS31 : 1;
#else
  unsigned long IS31 : 1;
  unsigned long IS30 : 1;
  unsigned long IS29 : 1;
  unsigned long IS28 : 1;
  unsigned long IS27 : 1;
  unsigned long IS26 : 1;
  unsigned long IS25 : 1;
  unsigned long IS24 : 1;
  unsigned long IS23 : 1;
  unsigned long IS22 : 1;
  unsigned long IS21 : 1;
  unsigned long IS20 : 1;
  unsigned long IS19 : 1;
  unsigned long IS18 : 1;
  unsigned long IS17 : 1;
  unsigned long IS16 : 1;
  unsigned long IS15 : 1;
  unsigned long IS14 : 1;
  unsigned long IS13 : 1;
  unsigned long IS12 : 1;
  unsigned long IS11 : 1;
  unsigned long IS10 : 1;
  unsigned long IS9 : 1;
  unsigned long IS8 : 1;
  unsigned long IS7 : 1;
  unsigned long IS6 : 1;
  unsigned long IS5 : 1;
  unsigned long IS4 : 1;
  unsigned long IS3 : 1;
  unsigned long IS2 : 1;
  unsigned long IS1 : 1;
  unsigned long IS0 : 1;
#endif
};

union un_icu_grpbl2
{
  unsigned long LONG;
  struct st_icu_grpbl2_bit BIT;
};

struct st_icu_genbe0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_icu_genbe0
{
  unsigned long LONG;
  struct st_icu_genbe0_bit BIT;
};

struct st_icu_genbl2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN0 : 1;
  unsigned long EN1 : 1;
  unsigned long EN2 : 1;
  unsigned long EN3 : 1;
  unsigned long EN4 : 1;
  unsigned long EN5 : 1;
  unsigned long EN6 : 1;
  unsigned long EN7 : 1;
  unsigned long EN8 : 1;
  unsigned long EN9 : 1;
  unsigned long EN10 : 1;
  unsigned long EN11 : 1;
  unsigned long EN12 : 1;
  unsigned long EN13 : 1;
  unsigned long EN14 : 1;
  unsigned long EN15 : 1;
  unsigned long EN16 : 1;
  unsigned long EN17 : 1;
  unsigned long EN18 : 1;
  unsigned long EN19 : 1;
  unsigned long EN20 : 1;
  unsigned long EN21 : 1;
  unsigned long EN22 : 1;
  unsigned long EN23 : 1;
  unsigned long EN24 : 1;
  unsigned long EN25 : 1;
  unsigned long EN26 : 1;
  unsigned long EN27 : 1;
  unsigned long EN28 : 1;
  unsigned long EN29 : 1;
  unsigned long EN30 : 1;
  unsigned long EN31 : 1;
#else
  unsigned long EN31 : 1;
  unsigned long EN30 : 1;
  unsigned long EN29 : 1;
  unsigned long EN28 : 1;
  unsigned long EN27 : 1;
  unsigned long EN26 : 1;
  unsigned long EN25 : 1;
  unsigned long EN24 : 1;
  unsigned long EN23 : 1;
  unsigned long EN22 : 1;
  unsigned long EN21 : 1;
  unsigned long EN20 : 1;
  unsigned long EN19 : 1;
  unsigned long EN18 : 1;
  unsigned long EN17 : 1;
  unsigned long EN16 : 1;
  unsigned long EN15 : 1;
  unsigned long EN14 : 1;
  unsigned long EN13 : 1;
  unsigned long EN12 : 1;
  unsigned long EN11 : 1;
  unsigned long EN10 : 1;
  unsigned long EN9 : 1;
  unsigned long EN8 : 1;
  unsigned long EN7 : 1;
  unsigned long EN6 : 1;
  unsigned long EN5 : 1;
  unsigned long EN4 : 1;
  unsigned long EN3 : 1;
  unsigned long EN2 : 1;
  unsigned long EN1 : 1;
  unsigned long EN0 : 1;
#endif
};

union un_icu_genbl2
{
  unsigned long LONG;
  struct st_icu_genbl2_bit BIT;
};

struct st_icu_gcrbe0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CLR0 : 1;
  unsigned long CLR1 : 1;
  unsigned long CLR2 : 1;
  unsigned long CLR3 : 1;
  unsigned long CLR4 : 1;
  unsigned long CLR5 : 1;
  unsigned long CLR6 : 1;
  unsigned long CLR7 : 1;
  unsigned long CLR8 : 1;
  unsigned long CLR9 : 1;
  unsigned long CLR10 : 1;
  unsigned long CLR11 : 1;
  unsigned long CLR12 : 1;
  unsigned long CLR13 : 1;
  unsigned long CLR14 : 1;
  unsigned long CLR15 : 1;
  unsigned long CLR16 : 1;
  unsigned long CLR17 : 1;
  unsigned long CLR18 : 1;
  unsigned long CLR19 : 1;
  unsigned long CLR20 : 1;
  unsigned long CLR21 : 1;
  unsigned long CLR22 : 1;
  unsigned long CLR23 : 1;
  unsigned long CLR24 : 1;
  unsigned long CLR25 : 1;
  unsigned long CLR26 : 1;
  unsigned long CLR27 : 1;
  unsigned long CLR28 : 1;
  unsigned long CLR29 : 1;
  unsigned long CLR30 : 1;
  unsigned long CLR31 : 1;
#else
  unsigned long CLR31 : 1;
  unsigned long CLR30 : 1;
  unsigned long CLR29 : 1;
  unsigned long CLR28 : 1;
  unsigned long CLR27 : 1;
  unsigned long CLR26 : 1;
  unsigned long CLR25 : 1;
  unsigned long CLR24 : 1;
  unsigned long CLR23 : 1;
  unsigned long CLR22 : 1;
  unsigned long CLR21 : 1;
  unsigned long CLR20 : 1;
  unsigned long CLR19 : 1;
  unsigned long CLR18 : 1;
  unsigned long CLR17 : 1;
  unsigned long CLR16 : 1;
  unsigned long CLR15 : 1;
  unsigned long CLR14 : 1;
  unsigned long CLR13 : 1;
  unsigned long CLR12 : 1;
  unsigned long CLR11 : 1;
  unsigned long CLR10 : 1;
  unsigned long CLR9 : 1;
  unsigned long CLR8 : 1;
  unsigned long CLR7 : 1;
  unsigned long CLR6 : 1;
  unsigned long CLR5 : 1;
  unsigned long CLR4 : 1;
  unsigned long CLR3 : 1;
  unsigned long CLR2 : 1;
  unsigned long CLR1 : 1;
  unsigned long CLR0 : 1;
#endif
};

union un_icu_gcrbe0
{
  unsigned long LONG;
  struct st_icu_gcrbe0_bit BIT;
};

struct st_icu_pibr0_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr0
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr0_bit BIT;
#endif
};

struct st_icu_pibr1_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr1
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr1_bit BIT;
#endif
};

struct st_icu_pibr2_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr2
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr2_bit BIT;
#endif
};

struct st_icu_pibr3_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr3
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr3_bit BIT;
#endif
};

struct st_icu_pibr4_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr4
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr4_bit BIT;
#endif
};

struct st_icu_pibr5_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr5
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr5_bit BIT;
#endif
};

struct st_icu_pibr6_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr6
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_prib6_bit BIT;
#endif
};

struct st_icu_pibr7_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr7
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr7_bit BIT;
#endif
};

struct st_icu_pibr8_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr8
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr8_bit BIT;
#endif
};

struct st_icu_pibr9_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibr9
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibr9_bit BIT;
#endif
};

struct st_icu_pibra_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibra
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibra_bit BIT;
#endif
};

struct st_icu_pibrb_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_pibrb
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_pibrb_bit BIT;
#endif
};

struct st_icu_slibxr128_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr128
{
  unsigned char BYTE;
  struct st_icu_slibxr128_bit BIT;
};

struct st_icu_slibxr129_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr129
{
  unsigned char BYTE;
  struct st_icu_slibxr129_bit BIT;
};

struct st_icu_slibxr130_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr130
{
  unsigned char BYTE;
  struct st_icu_slibxr130_bit BIT;
};

struct st_icu_slibxr131_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr131
{
  unsigned char BYTE;
  struct st_icu_slibxr131_bit BIT;
};

struct st_icu_slibxr132_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr132
{
  unsigned char BYTE;
  struct st_icu_slibxr132_bit BIT;
};

struct st_icu_slibxr133_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr133
{
  unsigned char BYTE;
  struct st_icu_slibxr133_bit BIT;
};

struct st_icu_slibxr134_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr134
{
  unsigned char BYTE;
  struct st_icu_slibxr134_bit BIT;
};

struct st_icu_slibxr135_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr135
{
  unsigned char BYTE;
  struct st_icu_slibxr135_bit BIT;
};

struct st_icu_slibxr136_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr136
{
  unsigned char BYTE;
  struct st_icu_slibxr136_bit BIT;
};

struct st_icu_slibxr137_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr137
{
  unsigned char BYTE;
  struct st_icu_slibxr137_bit BIT;
};

struct st_icu_slibxr138_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr138
{
  unsigned char BYTE;
  struct st_icu_slibxr138_bit BIT;
};

struct st_icu_slibxr139_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr139
{
  unsigned char BYTE;
  struct st_icu_slibxr139_bit BIT;
};

struct st_icu_slibxr140_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr140
{
  unsigned char BYTE;
  struct st_icu_slibxr140_bit BIT;
};

struct st_icu_slibxr141_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr141
{
  unsigned char BYTE;
  struct st_icu_slibxr141_bit BIT;
};

struct st_icu_slibxr142_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr142
{
  unsigned char BYTE;
  struct st_icu_slibxr142_bit BIT;
};

struct st_icu_slibxr143_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibxr143
{
  unsigned char BYTE;
  struct st_icu_slibxr143_bit BIT;
};

struct st_icu_slibr144_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr144
{
  unsigned char BYTE;
  struct st_icu_slibr144_bit BIT;
};

struct st_icu_slibr145_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr145
{
  unsigned char BYTE;
  struct st_icu_slibr145_bit BIT;
};

struct st_icu_slibr146_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr146
{
  unsigned char BYTE;
  struct st_icu_slibr146_bit BIT;
};

struct st_icu_slibr147_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr147
{
  unsigned char BYTE;
  struct st_icu_slibr147_bit BIT;
};

struct st_icu_slibr148_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr148
{
  unsigned char BYTE;
  struct st_icu_slibr148_bit BIT;
};

struct st_icu_slibr149_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr149
{
  unsigned char BYTE;
  struct st_icu_slibr149_bit BIT;
};

struct st_icu_slibr150_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr150
{
  unsigned char BYTE;
  struct st_icu_slibr150_bit BIT;
};

struct st_icu_slibr151_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr151
{
  unsigned char BYTE;
  struct st_icu_slibr151_bit BIT;
};

struct st_icu_slibr152_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr152
{
  unsigned char BYTE;
  struct st_icu_slibr152_bit BIT;
};

struct st_icu_slibr153_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr153
{
  unsigned char BYTE;
  struct st_icu_slibr153_bit BIT;
};

struct st_icu_slibr154_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr154
{
  unsigned char BYTE;
  struct st_icu_slibr154_bit BIT;
};

struct st_icu_slibr155_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr155
{
  unsigned char BYTE;
  struct st_icu_slibr155_bit BIT;
};

struct st_icu_slibr156_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr156
{
  unsigned char BYTE;
  struct st_icu_slibr156_bit BIT;
};

struct st_icu_slibr157_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr157
{
  unsigned char BYTE;
  struct st_icu_slibr157_bit BIT;
};

struct st_icu_slibr158_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr158
{
  unsigned char BYTE;
  struct st_icu_slibr158_bit BIT;
};

struct st_icu_slibr159_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr159
{
  unsigned char BYTE;
  struct st_icu_slibr159_bit BIT;
};

struct st_icu_slibr160_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr160
{
  unsigned char BYTE;
  struct st_icu_slibr160_bit BIT;
};

struct st_icu_slibr161_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr161
{
  unsigned char BYTE;
  struct st_icu_slibr161_bit BIT;
};

struct st_icu_slibr162_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr162
{
  unsigned char BYTE;
  struct st_icu_slibr162_bit BIT;
};

struct st_icu_slibr163_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr163
{
  unsigned char BYTE;
  struct st_icu_slibr163_bit BIT;
};

struct st_icu_slibr164_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr164
{
  unsigned char BYTE;
  struct st_icu_slibr164_bit BIT;
};

struct st_icu_slibr165_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr165
{
  unsigned char BYTE;
  struct st_icu_slibr165_bit BIT;
};

struct st_icu_slibr166_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr166
{
  unsigned char BYTE;
  struct st_icu_slibr166_bit BIT;
};

struct st_icu_slibr167_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr167
{
  unsigned char BYTE;
  struct st_icu_slibr167_bit BIT;
};

struct st_icu_slibr168_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr168
{
  unsigned char BYTE;
  struct st_icu_slibr168_bit BIT;
};

struct st_icu_slibr169_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr169
{
  unsigned char BYTE;
  struct st_icu_slibr169_bit BIT;
};

struct st_icu_slibr170_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr170
{
  unsigned char BYTE;
  struct st_icu_slibr170_bit BIT;
};

struct st_icu_slibr171_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr171
{
  unsigned char BYTE;
  struct st_icu_slibr171_bit BIT;
};

struct st_icu_slibr172_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr172
{
  unsigned char BYTE;
  struct st_icu_slibr172_bit BIT;
};

struct st_icu_slibr173_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr173
{
  unsigned char BYTE;
  struct st_icu_slibr173_bit BIT;
};

struct st_icu_slibr174_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr174
{
  unsigned char BYTE;
  struct st_icu_slibr174_bit BIT;
};

struct st_icu_slibr175_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr175
{
  unsigned char BYTE;
  struct st_icu_slibr175_bit BIT;
};

struct st_icu_slibr176_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr176
{
  unsigned char BYTE;
  struct st_icu_slibr176_bit BIT;
};

struct st_icu_slibr177_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr177
{
  unsigned char BYTE;
  struct st_icu_slibr177_bit BIT;
};

struct st_icu_slibr178_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr178
{
  unsigned char BYTE;
  struct st_icu_slibr178_bit BIT;
};

struct st_icu_slibr179_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr179
{
  unsigned char BYTE;
  struct st_icu_slibr179_bit BIT;
};

struct st_icu_slibr180_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr180
{
  unsigned char BYTE;
  struct st_icu_slibr180_bit BIT;
};

struct st_icu_slibr181_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr181
{
  unsigned char BYTE;
  struct st_icu_slibr181_bit BIT;
};

struct st_icu_slibr182_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr182
{
  unsigned char BYTE;
  struct st_icu_slibr182_bit BIT;
};

struct st_icu_slibr183_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr183
{
  unsigned char BYTE;
  struct st_icu_slibr183_bit BIT;
};

struct st_icu_slibr184_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr184
{
  unsigned char BYTE;
  struct st_icu_slibr184_bit BIT;
};

struct st_icu_slibr185_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr185
{
  unsigned char BYTE;
  struct st_icu_slibr185_bit BIT;
};

struct st_icu_slibr186_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr186
{
  unsigned char BYTE;
  struct st_icu_slibr186_bit BIT;
};

struct st_icu_slibr187_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr187
{
  unsigned char BYTE;
  struct st_icu_slibr187_bit BIT;
};

struct st_icu_slibr188_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr188
{
  unsigned char BYTE;
  struct st_icu_slibr188_bit BIT;
};

struct st_icu_slibr189_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr189
{
  unsigned char BYTE;
  struct st_icu_slibr189_bit BIT;
};

struct st_icu_slibr190_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr190
{
  unsigned char BYTE;
  struct st_icu_slibr190_bit BIT;
};

struct st_icu_slibr191_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr191
{
  unsigned char BYTE;
  struct st_icu_slibr191_bit BIT;
};

struct st_icu_slibr192_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr192
{
  unsigned char BYTE;
  struct st_icu_slibr192_bit BIT;
};

struct st_icu_slibr193_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr193
{
  unsigned char BYTE;
  struct st_icu_slibr193_bit BIT;
};

struct st_icu_slibr194_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr194
{
  unsigned char BYTE;
  struct st_icu_slibr194_bit BIT;
};

struct st_icu_slibr195_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr195
{
  unsigned char BYTE;
  struct st_icu_slibr195_bit BIT;
};

struct st_icu_slibr196_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr196
{
  unsigned char BYTE;
  struct st_icu_slibr196_bit BIT;
};

struct st_icu_slibr197_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr197
{
  unsigned char BYTE;
  struct st_icu_slibr197_bit BIT;
};

struct st_icu_slibr198_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr198
{
  unsigned char BYTE;
  struct st_icu_slibr198_bit BIT;
};

struct st_icu_slibr199_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr199
{
  unsigned char BYTE;
  struct st_icu_slibr199_bit BIT;
};

struct st_icu_slibr200_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr200
{
  unsigned char BYTE;
  struct st_icu_slibr200_bit BIT;
};

struct st_icu_slibr201_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr201
{
  unsigned char BYTE;
  struct st_icu_slibr201_bit BIT;
};

struct st_icu_slibr202_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr202
{
  unsigned char BYTE;
  struct st_icu_slibr202_bit BIT;
};

struct st_icu_slibr203_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr203
{
  unsigned char BYTE;
  struct st_icu_slibr203_bit BIT;
};

struct st_icu_slibr204_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr204
{
  unsigned char BYTE;
  struct st_icu_slibr204_bit BIT;
};

struct st_icu_slibr205_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr205
{
  unsigned char BYTE;
  struct st_icu_slibr205_bit BIT;
};

struct st_icu_slibr206_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr206
{
  unsigned char BYTE;
  struct st_icu_slibr206_bit BIT;
};

struct st_icu_slibr207_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLI : 8;
#else
  unsigned char SLI : 8;
#endif
};

union un_icu_slibr207
{
  unsigned char BYTE;
  struct st_icu_slibr207_bit BIT;
};

struct st_icu_piar0_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar0
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar0_bit BIT;
#endif
};

struct st_icu_piar1_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar1
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar1_bit BIT;
#endif
};

struct st_icu_piar2_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar2
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar2_bit BIT;
#endif
};

struct st_icu_piar3_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar3
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar3_bit BIT;
#endif
};

struct st_icu_piar4_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar4
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar4_bit BIT;
#endif
};

struct st_icu_piar5_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piar5
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piar5_bit BIT;
#endif
};

struct st_icu_piarb_bit
{
  unsigned char PIR7:1;
  unsigned char PIR6:1;
  unsigned char PIR5:1;
  unsigned char PIR4:1;
  unsigned char PIR3:1;
  unsigned char PIR2:1;
  unsigned char PIR1:1;
  unsigned char PIR0:1;
};

union un_icu_piarb
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_icu_piarb_bit BIT;
#endif
};

union un_icu_sliar208
{
  unsigned char BYTE;
};

union un_icu_sliar209
{
  unsigned char BYTE;
};

union un_icu_sliar210
{
  unsigned char BYTE;
};

union un_icu_sliar211
{
  unsigned char BYTE;
};

union un_icu_sliar212
{
  unsigned char BYTE;
};

union un_icu_sliar213
{
  unsigned char BYTE;
};

union un_icu_sliar214
{
  unsigned char BYTE;
};

union un_icu_sliar215
{
  unsigned char BYTE;
};

union un_icu_sliar216
{
  unsigned char BYTE;
};

union un_icu_sliar217
{
  unsigned char BYTE;
};

union un_icu_sliar218
{
  unsigned char BYTE;
};

union un_icu_sliar219
{
  unsigned char BYTE;
};

union un_icu_sliar220
{
  unsigned char BYTE;
};

union un_icu_sliar221
{
  unsigned char BYTE;
};

union un_icu_sliar222
{
  unsigned char BYTE;
};

union un_icu_sliar223
{
  unsigned char BYTE;
};

union un_icu_sliar224
{
  unsigned char BYTE;
};

union un_icu_sliar225
{
  unsigned char BYTE;
};

union un_icu_sliar226
{
  unsigned char BYTE;
};

union un_icu_sliar227
{
  unsigned char BYTE;
};

union un_icu_sliar228
{
  unsigned char BYTE;
};

union un_icu_sliar229
{
  unsigned char BYTE;
};

union un_icu_sliar230
{
  unsigned char BYTE;
};

union un_icu_sliar231
{
  unsigned char BYTE;
};

union un_icu_sliar232
{
  unsigned char BYTE;
};

union un_icu_sliar233
{
  unsigned char BYTE;
};

union un_icu_sliar234
{
  unsigned char BYTE;
};

union un_icu_sliar235
{
  unsigned char BYTE;
};

union un_icu_sliar236
{
  unsigned char BYTE;
};

union un_icu_sliar237
{
  unsigned char BYTE;
};

union un_icu_sliar238
{
  unsigned char BYTE;
};

union un_icu_sliar239
{
  unsigned char BYTE;
};

union un_icu_sliar240
{
  unsigned char BYTE;
};

union un_icu_sliar241
{
  unsigned char BYTE;
};

union un_icu_sliar242
{
  unsigned char BYTE;
};

union un_icu_sliar243
{
  unsigned char BYTE;
};

union un_icu_sliar244
{
  unsigned char BYTE;
};

union un_icu_sliar245
{
  unsigned char BYTE;
};

union un_icu_sliar246
{
  unsigned char BYTE;
};

union un_icu_sliar247
{
  unsigned char BYTE;
};

union un_icu_sliar248
{
  unsigned char BYTE;
};

union un_icu_sliar249
{
  unsigned char BYTE;
};

union un_icu_sliar250
{
  unsigned char BYTE;
};

union un_icu_sliar251
{
  unsigned char BYTE;
};

union un_icu_sliar252
{
  unsigned char BYTE;
};

union un_icu_sliar253
{
  unsigned char BYTE;
};

union un_icu_sliar254
{
  unsigned char BYTE;
};

union un_icu_sliar255
{
  unsigned char BYTE;
};

struct st_icu_sliprcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char WPRC : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char WPRC : 1;
#endif
};

union un_icu_sliprcr
{
  unsigned char BYTE;
  struct st_icu_sliprcr_bit BIT;
};

struct st_icu_slexdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SELEXD0 : 1;
  unsigned char SELEXD1 : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char SELEXD1 : 1;
  unsigned char SELEXD0 : 1;
#endif
};

union un_icu_slexdr
{
  unsigned char BYTE;
  struct st_icu_slexdr_bit BIT;
};

struct st_rtc_r64cnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char F64HZ : 1;
  unsigned char F32HZ : 1;
  unsigned char F16HZ : 1;
  unsigned char F8HZ : 1;
  unsigned char F4HZ : 1;
  unsigned char F2HZ : 1;
  unsigned char F1HZ : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char F1HZ : 1;
  unsigned char F2HZ : 1;
  unsigned char F4HZ : 1;
  unsigned char F8HZ : 1;
  unsigned char F16HZ : 1;
  unsigned char F32HZ : 1;
  unsigned char F64HZ : 1;
#endif
};

union un_rtc_r64cnt
{
  unsigned char BYTE;
  struct st_rtc_r64cnt_bit BIT;
};

struct st_rtc_rseccnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEC1 : 4;
  unsigned char SEC10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SEC10 : 3;
  unsigned char SEC1 : 4;
#endif
};

union un_rtc_rseccnt
{
  unsigned char BYTE;
  struct st_rtc_rseccnt_bit BIT;
};

struct st_rtc_bcnt0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNT : 8;
#else
  unsigned char BCNT : 8;
#endif
};

union un_rtc_bcnt0
{
  unsigned char BYTE;
  struct st_rtc_bcnt0_bit BIT;
};

struct st_rtc_rmincnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MIN1 : 4;
  unsigned char MIN10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MIN10 : 3;
  unsigned char MIN1 : 4;
#endif
};

union un_rtc_rmincnt
{
  unsigned char BYTE;
  struct st_rtc_rmincnt_bit BIT;
} ;

struct st_rtc_bcnt1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNT : 8;
#else
  unsigned char BCNT : 8;
#endif
};

union un_rtc_bcnt1
{
  unsigned char BYTE;
  struct st_rtc_bcnt1_bit BIT;
};

struct st_rtc_rhrcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HR1 : 4;
  unsigned char HR10 : 2;
  unsigned char PM : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PM : 1;
  unsigned char HR10 : 2;
  unsigned char HR1 : 4;
#endif
};

union un_rtc_rhrcnt
{
  unsigned char BYTE;
  struct st_rtc_rhrcnt_bit BIT;
};

struct st_rtc_bcnt2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNT : 8;
#else
  unsigned char BCNT : 8;
#endif
};

union un_rtc_bcnt2
{
  unsigned char BYTE;
  struct st_rtc_bcnt2_bit BIT;
};

struct st_rtc_rwkcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DAYW : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char DAYW : 3;
#endif
};

union un_rtc_rwkcnt
{
  unsigned char BYTE;
  struct st_rtc_rwkcnt_bit BIT;
};

struct st_rtc_bcnt3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNT : 8;
#else
  unsigned char BCNT : 8;
#endif
};

union un_rtc_bcnt3
{
  unsigned char BYTE;
  struct st_rtc_bcnt3_bit BIT;
};

struct st_rtc_rdaycnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DATE1 : 4;
  unsigned char DATE10 : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char DATE10 : 2;
  unsigned char DATE1 : 4;
#endif
};

union un_rtc_rdaycnt
{
  unsigned char BYTE;
  struct st_rtc_rdaycnt_bit BIT;
};

struct st_rtc_rmoncnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MON1 : 4;
  unsigned char MON10 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char MON10 : 1;
  unsigned char MON1 : 4;
#endif
};

union un_rtc_rmoncnt
{
  unsigned char BYTE;
  struct st_rtc_rmoncnt_bit BIT;
};

struct st_rtc_ryrcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short YR1 : 4;
  unsigned short YR10 : 4;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short YR10 : 4;
  unsigned short YR1 : 4;
#endif
};

union un_rtc_ryrcnt
{
  unsigned short WORD;
  struct st_rtc_ryrcnt_bit BIT;
};

struct st_rtc_rsecar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEC1 : 4;
  unsigned char SEC10 : 3;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char SEC10 : 3;
  unsigned char SEC1 : 4;
#endif
};

union un_rtc_rsecar
{
  unsigned char BYTE;
  struct st_rtc_rsecar_bit BIT;
};

struct st_rtc_bcnt0ar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTAR : 8;
#else
  unsigned char BCNTAR : 8;
#endif
};

union un_rtc_bcnt0ar
{
  unsigned char BYTE;
  struct st_rtc_bcnt0ar_bit BIT;
};

struct st_rtc_rminar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MIN1 : 4;
  unsigned char MIN10 : 3;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char MIN10 : 3;
  unsigned char MIN1 : 4;
#endif
};

union un_rtc_rminar
{
  unsigned char BYTE;
  struct st_rtc_rminar_bit BIT;
};

struct st_rtc_bcnt1ar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTAR : 8;
#else
  unsigned char BCNTAR : 8;
#endif
};

union un_rtc_bcnt1ar
{
  unsigned char BYTE;
  struct st_rtc_bcnt1ar_bit BIT;
};

struct st_rtc_rhrar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HR1 : 4;
  unsigned char HR10 : 2;
  unsigned char PM : 1;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char PM : 1;
  unsigned char HR10 : 2;
  unsigned char HR1 : 4;
#endif
};

union un_rtc_rhrar
{
  unsigned char BYTE;
  struct st_rtc_rhrar_bit BIT;
};

struct st_rtc_bcnt2ar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTAR : 8;
#else
  unsigned char BCNTAR : 8;
#endif
};

union un_rtc_bcnt2ar
{
  unsigned char BYTE;
  struct st_rtc_bcnt2ar_bit BIT;
};

struct st_rtc_rwkar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DAYW : 3;
  unsigned char  : 4;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char  : 4;
  unsigned char DAYW : 3;
#endif
};

union un_rtc_rwkar
{
  unsigned char BYTE;
  struct st_rtc_rwkar_bit BIT;
};

struct st_rtc_bcnt3ar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTAR : 8;
#else
  unsigned char BCNTAR : 8;
#endif
};

union un_rtc_bcnt3ar
{
  unsigned char BYTE;
  struct st_rtc_bcnt3ar_bit BIT;
};

struct st_rtc_rdayar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DATE1 : 4;
  unsigned char DATE10 : 2;
  unsigned char  : 1;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char  : 1;
  unsigned char DATE10 : 2;
  unsigned char DATE1 : 4;
#endif
};

union un_rtc_rdayar
{
  unsigned char BYTE;
  struct st_rtc_rdayar_bit BIT;
};

struct st_rtc_bcnt0aer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ENB : 8;
#else
  unsigned char ENB : 8;
#endif
};

union un_rtc_bcnt0aer
{
  unsigned char BYTE;
  struct st_rtc_bcnt0aer_bit BIT;
};

struct st_rtc_rmonar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MON1 : 4;
  unsigned char MON10 : 1;
  unsigned char  : 2;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char  : 2;
  unsigned char MON10 : 1;
  unsigned char MON1 : 4;
#endif
};

union un_rtc_rmonar
{
  unsigned char BYTE;
  struct st_rtc_rmonar_bit BIT;
};

struct st_rtc_bcnt1aer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ENB : 8;
#else
  unsigned char ENB : 8;
#endif
};

union un_rtc_bcnt1aer
{
  unsigned char BYTE;
  struct st_rtc_bcnt1aer_bit BIT;
};

struct st_rtc_ryrar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short YR1 : 4;
  unsigned short YR10 : 4;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short YR10 : 4;
  unsigned short YR1 : 4;
#endif
};

union un_rtc_ryrar
{
  unsigned short WORD;
  struct st_rtc_ryrar_bit BIT;
};

struct st_rtc_bcnt2aer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ENB : 8;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short ENB : 8;
#endif
};

union un_rtc_bcnt2aer
{
  unsigned short WORD;
  struct st_rtc_bcnt2aer_bit BIT;
};

struct st_rtc_bcnt3aer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ENB : 8;
#else
  unsigned char ENB : 8;
#endif
};

union un_rtc_bcnt3aer
{
  unsigned char BYTE;
  struct st_rtc_bcnt3aer_bit BIT;
};

struct st_rtc_ryraren_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char ENB : 1;
#else
  unsigned char ENB : 1;
  unsigned char  : 7;
#endif
};

union un_rtc_ryraren
{
  unsigned char BYTE;
  struct st_rtc_ryraren_bit BIT;
};

struct st_rtc_rcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char AIE : 1;
  unsigned char CIE : 1;
  unsigned char PIE : 1;
  unsigned char RTCOS : 1;
  unsigned char PES : 4;
#else
  unsigned char PES : 4;
  unsigned char RTCOS : 1;
  unsigned char PIE : 1;
  unsigned char CIE : 1;
  unsigned char AIE : 1;
#endif
};

union un_rtc_rcr1
{
  unsigned char BYTE;
  struct st_rtc_rcr1_bit BIT;
};

struct st_rtc_rcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char START : 1;
  unsigned char RESET : 1;
  unsigned char ADJ30 : 1;
  unsigned char RTCOE : 1;
  unsigned char AADJE : 1;
  unsigned char AADJP : 1;
  unsigned char HR24 : 1;
  unsigned char CNTMD : 1;
#else
  unsigned char CNTMD : 1;
  unsigned char HR24 : 1;
  unsigned char AADJP : 1;
  unsigned char AADJE : 1;
  unsigned char RTCOE : 1;
  unsigned char ADJ30 : 1;
  unsigned char RESET : 1;
  unsigned char START : 1;
#endif
};

union un_rtc_rcr2
{
  unsigned char BYTE;
  struct st_rtc_rcr2_bit BIT;
};

struct st_rtc_rcr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RCKSEL : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char RCKSEL : 1;
#endif
};

union un_rtc_rcr4
{
  unsigned char BYTE;
  struct st_rtc_rcr4_bit BIT;
};

struct st_rtc_rfrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RFC : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short RFC : 1;
#endif
};

union un_rtc_rfrh
{
  unsigned short WORD;
  struct st_rtc_rfrh_bit BIT;
};

struct st_rtc_rfrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RFC : 16;
#else
  unsigned short RFC : 16;
#endif
};

union un_rtc_rfrl
{
  unsigned short WORD;
  struct st_rtc_rfrl_bit BIT;
};

struct st_rtc_radj_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADJ : 6;
  unsigned char PMADJ : 2;
#else
  unsigned char PMADJ : 2;
  unsigned char ADJ : 6;
#endif
};

union un_rtc_radj
{
  unsigned char BYTE;
  struct st_rtc_radj_bit BIT;
};

struct st_rtc_rtccr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCCT : 2;
  unsigned char TCST : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCEN : 1;
#else
  unsigned char TCEN : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCST : 1;
  unsigned char TCCT : 2;
#endif
};

union un_rtc_rtccr0
{
  unsigned char BYTE;
  struct st_rtc_rtccr0_bit BIT;
};

struct st_rtc_rtccr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCCT : 2;
  unsigned char TCST : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCEN : 1;
#else
  unsigned char TCEN : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCST : 1;
  unsigned char TCCT : 2;
#endif
};

union un_rtc_rtccr1
{
  unsigned char BYTE;
  struct st_rtc_rtccr1_bit BIT;
};

struct st_rtc_rtccr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCCT : 2;
  unsigned char TCST : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCEN : 1;
#else
  unsigned char TCEN : 1;
  unsigned char  : 1;
  unsigned char TCNF : 2;
  unsigned char  : 1;
  unsigned char TCST : 1;
  unsigned char TCCT : 2;
#endif
};

union un_rtc_rtccr2
{
  unsigned char BYTE;
  struct st_rtc_rtccr2_bit BIT;
};

struct st_rtc_rseccp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEC1 : 4;
  unsigned char SEC10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SEC10 : 3;
  unsigned char SEC1 : 4;
#endif
};

union un_rtc_rseccp0
{
  unsigned char BYTE;
  struct st_rtc_rseccp0_bit BIT;
};

struct st_rtc_bcnt0cp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP0 : 8;
#else
  unsigned char BCNTCP0 : 8;
#endif
};

union un_rtc_bcnt0cp0
{
  unsigned char BYTE;
  struct st_rtc_bcnt0cp0_bit BIT;
};

struct st_rtc_rmincp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MIN1 : 4;
  unsigned char MIN10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MIN10 : 3;
  unsigned char MIN1 : 4;
#endif
};

union un_rtc_rmincp0
{
  unsigned char BYTE;
  struct st_rtc_rmincp0_bit BIT;
};

struct st_rtc_bcnt1cp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP0 : 8;
#else
  unsigned char BCNTCP0 : 8;
#endif
};

union un_rtc_bcnt1cp0
{
  unsigned char BYTE;
  struct st_rtc_bcnt1cp0_bit BIT;
};

struct st_rtc_rhrcp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HR1 : 4;
  unsigned char HR10 : 2;
  unsigned char PM : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PM : 1;
  unsigned char HR10 : 2;
  unsigned char HR1 : 4;
#endif
};

union un_rtc_rhrcp0
{
  unsigned char BYTE;
  struct st_rtc_rhrcp0_bit BIT;
};

struct st_rtc_bcnt2cp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP0 : 8;
#else
  unsigned char BCNTCP0 : 8;
#endif
};

union un_rtc_bcnt2cp0
{
  unsigned char BYTE;
  struct st_rtc_bcnt2cp0_bit BIT;
};

struct st_rtc_rdaycp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DATE1 : 4;
  unsigned char DATE10 : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char DATE10 : 2;
  unsigned char DATE1 : 4;
#endif
};

union un_rtc_rdaycp0
{
  unsigned char BYTE;
  struct st_rtc_rdaycp0_bit BIT;
};

struct st_rtc_bcnt3cp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP0 : 8;
#else
  unsigned char BCNTCP0 : 8;
#endif
};

union un_rtc_bcnt3cp0
{
  unsigned char BYTE;
  struct st_rtc_bcnt3cp0_bit BIT;
};

struct st_rtc_rmoncp0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MON1 : 4;
  unsigned char MON10 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char MON10 : 1;
  unsigned char MON1 : 4;
#endif
};

union un_rtc_rmoncp0
{
  unsigned char BYTE;
  struct st_rtc_rmoncp0_bit BIT;
};

struct st_rtc_rseccp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEC1 : 4;
  unsigned char SEC10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SEC10 : 3;
  unsigned char SEC1 : 4;
#endif
};

union un_rtc_rseccp1
{
  unsigned char BYTE;
  struct st_rtc_rseccp1_bit BIT;
};

struct st_rtc_bcnt0cp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP1 : 8;
#else
  unsigned char BCNTCP1 : 8;
#endif
};

union un_rtc_bcnt0cp1
{
  unsigned char BYTE;
  struct st_rtc_bcnt0cp1_bit BIT;
};

struct st_rtc_rmincp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MIN1 : 4;
  unsigned char MIN10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MIN10 : 3;
  unsigned char MIN1 : 4;
#endif
};

union un_rtc_rmincp1
{
  unsigned char BYTE;
  struct st_rtc_rmincp1_bit BIT;
};

struct st_rtc_bcnt1cp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP1 : 8;
#else
  unsigned char BCNTCP1 : 8;
#endif
};

union un_rtc_bcnt1cp1
{
  unsigned char BYTE;
  struct st_rtc_bcnt1cp1_bit BIT;
};

struct st_rtc_rhrcp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HR1 : 4;
  unsigned char HR10 : 2;
  unsigned char PM : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PM : 1;
  unsigned char HR10 : 2;
  unsigned char HR1 : 4;
#endif
};

union un_rtc_rhrcp1
{
  unsigned char BYTE;
  struct st_rtc_rhrcp1_bit BIT;
};

struct st_rtc_bcnt2cp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP1 : 8;
#else
  unsigned char BCNTCP1 : 8;
#endif
};

union un_rtc_bcnt2cp1
{
  unsigned char BYTE;
  struct st_rtc_bcnt2cp1_bit BIT;
};

struct st_rtc_rdaycp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DATE1 : 4;
  unsigned char DATE10 : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char DATE10 : 2;
  unsigned char DATE1 : 4;
#endif
};

union un_rtc_rdaycp1
{
  unsigned char BYTE;
  struct st_rtc_rdaycp1_bit BIT;
};

struct st_rtc_bcnt3cp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP1 : 8;
#else
  unsigned char BCNTCP1 : 8;
#endif
};

union un_rtc_bcnt3cp1
{
  unsigned char BYTE;
  struct st_rtc_bcnt3cp1_bit BIT;
};

struct st_rtc_rmoncp1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MON1 : 4;
  unsigned char MON10 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char MON10 : 1;
  unsigned char MON1 : 4;
#endif
};

union un_rtc_rmoncp1
{
  unsigned char BYTE;
  struct st_rtc_rmoncp1_bit BIT;
};

struct st_rtc_rseccp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEC1 : 4;
  unsigned char SEC10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SEC10 : 3;
  unsigned char SEC1 : 4;
#endif
};

union un_rtc_rseccp2
{
  unsigned char BYTE;
  struct st_rtc_rseccp2_bit BIT;
};

struct st_rtc_bcnt0cp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP2 : 8;
#else
  unsigned char BCNTCP2 : 8;
#endif
};

union un_rtc_bcnt0cp2
{
  unsigned char BYTE;
  struct st_rtc_bcnt0cp2_bit BIT;
};

struct st_rtc_rmincp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MIN1 : 4;
  unsigned char MIN10 : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MIN10 : 3;
  unsigned char MIN1 : 4;
#endif
};

union un_rtc_rmincp2
{
  unsigned char BYTE;
  struct st_rtc_rmincp2_bit BIT;
};

struct st_rtc_bcnt1cp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP2 : 8;
#else
  unsigned char BCNTCP2 : 8;
#endif
};

union un_rtc_bcnt1cp2
{
  unsigned char BYTE;
  struct st_rtc_bcnt1cp2_bit BIT;
};

struct st_rtc_rhrcp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HR1 : 4;
  unsigned char HR10 : 2;
  unsigned char PM : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PM : 1;
  unsigned char HR10 : 2;
  unsigned char HR1 : 4;
#endif
};

union un_rtc_rhrcp2
{
  unsigned char BYTE;
  struct st_rtc_rhrcp2_bit BIT;
};

struct st_rtc_bcnt2cp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP2 : 8;
#else
  unsigned char BCNTCP2 : 8;
#endif
};

union un_rtc_bcnt2cp2
{
  unsigned char BYTE;
  struct st_rtc_bcnt2cp2_bit BIT;
};

struct st_rtc_rdaycp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DATE1 : 4;
  unsigned char DATE10 : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char DATE10 : 2;
  unsigned char DATE1 : 4;
#endif
};

union un_rtc_rdaycp2
{
  unsigned char BYTE;
  struct st_rtc_rdaycp2_bit BIT;
};

struct st_rtc_bcnt3cp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCNTCP2 : 8;
#else
  unsigned char BCNTCP2 : 8;
#endif
};

union un_rtc_bcnt3cp2
{
  unsigned char BYTE;
  struct st_rtc_bcnt3cp2_bit BIT;
};

struct st_rtc_rmoncp2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MON1 : 4;
  unsigned char MON10 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char MON10 : 1;
  unsigned char MON1 : 4;
#endif
};

union un_rtc_rmoncp2
{
  unsigned char BYTE;
  struct st_rtc_rmoncp2_bit BIT;
};

struct st_mpc_p00pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_p00pfs
{
  unsigned char BYTE;
  struct st_mpc_p00pfs_bit BIT;
};

struct st_mpc_p01pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_p01pfs
{
  unsigned char BYTE;
  struct st_mpc_p01pfs_bit BIT;
};

struct st_mpc_pfcse_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CS0E : 1;
  unsigned char CS1E : 1;
  unsigned char CS2E : 1;
  unsigned char CS3E : 1;
  unsigned char CS4E : 1;
  unsigned char CS5E : 1;
  unsigned char CS6E : 1;
  unsigned char CS7E : 1;
#else
  unsigned char CS7E : 1;
  unsigned char CS6E : 1;
  unsigned char CS5E : 1;
  unsigned char CS4E : 1;
  unsigned char CS3E : 1;
  unsigned char CS2E : 1;
  unsigned char CS1E : 1;
  unsigned char CS0E : 1;
#endif
};

union un_mpc_pfcse
{
  unsigned char BYTE;
  struct st_mpc_pfcse_bit BIT;
};

struct st_mpc_pfcss0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CS0S : 1;
  unsigned char  : 1;
  unsigned char CS1S : 2;
  unsigned char CS2S : 2;
  unsigned char CS3S : 2;
#else
  unsigned char CS3S : 2;
  unsigned char CS2S : 2;
  unsigned char CS1S : 2;
  unsigned char  : 1;
  unsigned char CS0S : 1;
#endif
};

union un_mpc_pfcss0
{
  unsigned char BYTE;
  struct st_mpc_pfcss0_bit BIT;
};

struct st_mpc_pfcss1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CS4S : 2;
  unsigned char CS5S : 2;
  unsigned char CS6S : 2;
  unsigned char CS7S : 2;
#else
  unsigned char CS7S : 2;
  unsigned char CS6S : 2;
  unsigned char CS5S : 2;
  unsigned char CS4S : 2;
#endif
};

union un_mpc_pfcss1
{
  unsigned char BYTE;
  struct st_mpc_pfcss1_bit BIT;
};

struct st_mpc_pfa0e0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char A8E : 1;
  unsigned char A9E : 1;
  unsigned char A10E : 1;
  unsigned char A11E : 1;
  unsigned char A12E : 1;
  unsigned char A13E : 1;
  unsigned char A14E : 1;
  unsigned char A15E : 1;
#else
  unsigned char A15E : 1;
  unsigned char A14E : 1;
  unsigned char A13E : 1;
  unsigned char A12E : 1;
  unsigned char A11E : 1;
  unsigned char A10E : 1;
  unsigned char A9E : 1;
  unsigned char A8E : 1;
#endif
};

union un_mpc_pfa0e0
{
  unsigned char BYTE;
  struct st_mpc_pfa0e0_bit BIT;
};

struct st_mpc_pfa0e1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char A16E : 1;
  unsigned char A17E : 1;
  unsigned char A18E : 1;
  unsigned char A19E : 1;
  unsigned char A20E : 1;
  unsigned char A21E : 1;
  unsigned char A22E : 1;
  unsigned char A23E : 1;
#else
  unsigned char A23E : 1;
  unsigned char A22E : 1;
  unsigned char A21E : 1;
  unsigned char A20E : 1;
  unsigned char A19E : 1;
  unsigned char A18E : 1;
  unsigned char A17E : 1;
  unsigned char A16E : 1;
#endif
};

union un_mpc_pfa0e1
{
  unsigned char BYTE;
  struct st_mpc_pfa0e1_bit BIT;
};

struct st_mpc_pfbcr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADRLE : 1;
  unsigned char ADRHMS : 1;
  unsigned char ADRHMS2 : 1;
  unsigned char BCLKO : 1;
  unsigned char DHE : 1;
  unsigned char DH32E : 1;
  unsigned char WR1BC1E : 1;
  unsigned char WR32BC32E : 1;
#else
  unsigned char WR32BC32E : 1;
  unsigned char WR1BC1E : 1;
  unsigned char DH32E : 1;
  unsigned char DHE : 1;
  unsigned char BCLKO : 1;
  unsigned char ADRHMS2 : 1;
  unsigned char ADRHMS : 1;
  unsigned char ADRLE : 1;
#endif
};

union un_mpc_pfbcr0
{
  unsigned char BYTE;
  struct st_mpc_pfbcr0_bit BIT;
};

struct st_mpc_pfbcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char WAITS : 2;
  unsigned char ALEOE : 1;
  unsigned char ALES : 1;
  unsigned char MDSDE : 1;
  unsigned char  : 1;
  unsigned char DQM1E : 1;
  unsigned char SDCLKE : 1;
#else
  unsigned char SDCLKE : 1;
  unsigned char DQM1E : 1;
  unsigned char  : 1;
  unsigned char MDSDE : 1;
  unsigned char ALES : 1;
  unsigned char ALEOE : 1;
  unsigned char WAITS : 2;
#endif
};

union un_mpc_pfbcr1
{
  unsigned char BYTE;
  struct st_mpc_pfbcr1_bit BIT;
};

struct st_mpc_pfbcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char D0S : 2;
  unsigned char D1S : 2;
  unsigned char D2S : 2;
  unsigned char D3S : 2;
#else
  unsigned char D3S : 2;
  unsigned char D2S : 2;
  unsigned char D1S : 2;
  unsigned char D0S : 2;
#endif
};

union un_mpc_pfbcr2
{
  unsigned char BYTE;
  struct st_mpc_pfbcr2_bit BIT;
};

struct st_mpc_pfbcr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DLHS : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DLHS : 1;
#endif
};

union un_mpc_pfbcr3
{
  unsigned char BYTE;
  struct st_mpc_pfbcr3_bit BIT;
};

struct st_mpc_pfenet_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 4;
  unsigned char PHYMODE0 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char PHYMODE0 : 1;
  unsigned char  : 4;
#endif
};

union un_mpc_pfenet
{
  unsigned char BYTE;
  struct st_mpc_pfenet_bit BIT;
};

struct st_mpc_p02pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p02pfs
{
  unsigned char BYTE;
  struct st_mpc_p02pfs_bit BIT;
};

struct st_mpc_p03pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p03pfs
{
  unsigned char BYTE;
  struct st_mpc_p03pfs_bit BIT;
};

struct st_mpc_p05pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p05pfs
{
  unsigned char BYTE;
  struct st_mpc_p05pfs_bit BIT;
};

struct st_mpc_p07pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p07pfs
{
  unsigned char BYTE;
  struct st_mpc_p07pfs_bit BIT;
};

struct st_mpc_p10pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p10pfs
{
  unsigned char BYTE;
  struct st_mpc_p10pfs_bit BIT;
};

struct st_mpc_p11pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p11pfs
{
  unsigned char BYTE;
  struct st_mpc_p11pfs_bit BIT;
};

struct st_system_mdmonr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short MD : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short MD : 1;
#endif
};

struct st_mpc_p12pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p12pfs
{
  unsigned char BYTE;
  struct st_mpc_p12pfs_bit BIT;
};

struct st_mpc_p13pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p13pfs
{
  unsigned char BYTE;
  struct st_mpc_p13pfs_bit BIT;
};

struct st_mpc_p14pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p14pfs
{
  unsigned char BYTE;
  struct st_mpc_p14pfs_bit BIT;
};

struct st_mpc_p15pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p15pfs
{
  unsigned char BYTE;
  struct st_mpc_p15pfs_bit BIT;
};

struct st_mpc_p16pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p16pfs
{
  unsigned char BYTE;
  struct st_mpc_p16pfs_bit BIT;
};

struct st_mpc_p17pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p17pfs
{
  unsigned char BYTE;
  struct st_mpc_p17pfs_bit BIT;
};

struct st_mpc_p20pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p20pfs
{
  unsigned char BYTE;
  struct st_mpc_p20pfs_bit BIT;
};

struct st_mpc_p21pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p21pfs
{
  unsigned char BYTE;
  struct st_mpc_p21pfs_bit BIT;
};

struct st_mpc_p22pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p22pfs
{
  unsigned char BYTE;
  struct st_mpc_p22pfs_bit BIT;
};

struct st_mpc_p23pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p23pfs
{
  unsigned char BYTE;
  struct st_mpc_p23pfs_bit BIT;
};

struct st_mpc_p24pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p24pfs
{
  unsigned char BYTE;
  struct st_mpc_p24pfs_bit BIT;
};

struct st_mpc_p25pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p25pfs
{
  unsigned char BYTE;
  struct st_mpc_p25pfs_bit BIT;
};

struct st_mpc_p26pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p26pfs
{
  unsigned char BYTE;
  struct st_mpc_p26pfs_bit BIT;
};

struct st_mpc_p27pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p27pfs
{
  unsigned char BYTE;
  struct st_mpc_p27pfs_bit BIT;
};

struct st_mpc_p30pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p30pfs
{
  unsigned char BYTE;
  struct st_mpc_p30pfs_bit BIT;
};

struct st_mpc_p31pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p31pfs
{
  unsigned char BYTE;
  struct st_mpc_p31pfs_bit BIT;
};

struct st_mpc_p32pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p32pfs
{
  unsigned char BYTE;
  struct st_mpc_p32pfs_bit BIT;
};

struct st_mpc_p33pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p33pfs
{
  unsigned char BYTE;
  struct st_mpc_p33pfs_bit BIT;
};

struct st_mpc_p34pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p34pfs
{
  unsigned char BYTE;
  struct st_mpc_p34pfs_bit BIT;
};

struct st_mpc_p40pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p40pfs
{
  unsigned char BYTE;
  struct st_mpc_p40pfs_bit BIT;
};

struct st_mpc_p41pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p41pfs
{
  unsigned char BYTE;
  struct st_mpc_p41pfs_bit BIT;
};

struct st_mpc_p42pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p42pfs
{
  unsigned char BYTE;
  struct st_mpc_p42pfs_bit BIT;
};

struct st_mpc_p43pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p43pfs
{
  unsigned char BYTE;
  struct st_mpc_p43pfs_bit BIT;
};

struct st_mpc_p44pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p44pfs
{
  unsigned char BYTE;
  struct st_mpc_p44pfs_bit BIT;
};

struct st_mpc_p45pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p45pfs
{
  unsigned char BYTE;
  struct st_mpc_p45pfs_bit BIT;
};

struct st_mpc_p46pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p46pfs
{
  unsigned char BYTE;
  struct st_mpc_p46pfs_bit BIT;
};

struct st_mpc_p47pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_p47pfs
{
  unsigned char BYTE;
  struct st_mpc_p47pfs_bit BIT;
};

struct st_mpc_p50pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p50pfs
{
  unsigned char BYTE;
  struct st_mpc_p50pfs_bit BIT;
};

struct st_mpc_p51pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p51pfs
{
  unsigned char BYTE;
  struct st_mpc_p51pfs_bit BIT;
};

struct st_mpc_p52pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p52pfs
{
  unsigned char BYTE;
  struct st_mpc_p52pfs_bit BIT;
};

struct st_mpc_p54pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p54pfs
{
  unsigned char BYTE;
  struct st_mpc_p54pfs_bit BIT;
};

struct st_mpc_p55pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p55pfs
{
  unsigned char BYTE;
  struct st_mpc_p55pfs_bit BIT;
};

struct st_mpc_p56pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p56pfs
{
  unsigned char BYTE;
  struct st_mpc_p56pfs_bit BIT;
};

struct st_mpc_p57pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p57pfs
{
  unsigned char BYTE;
  struct st_mpc_p57pfs_bit BIT;
};

struct st_mpc_p66pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p66pfs
{
  unsigned char BYTE;
  struct st_mpc_p66pfs_bit BIT;
};

struct st_mpc_p67pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p67pfs
{
  unsigned char BYTE;
  struct st_mpc_p67pfs_bit BIT;
};

struct st_mpc_p71pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p71pfs
{
  unsigned char BYTE;
  struct st_mpc_p71pfs_bit BIT;
};

struct st_mpc_p72pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p72pfs
{
  unsigned char BYTE;
  struct st_mpc_p72pfs_bit BIT;
};

struct st_mpc_p73pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p73pfs
{
  unsigned char BYTE;
  struct st_mpc_p73pfs_bit BIT;
};

struct st_mpc_p74pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p74pfs
{
  unsigned char BYTE;
  struct st_mpc_p74pfs_bit BIT;
};

struct st_mpc_p75pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p75pfs
{
  unsigned char BYTE;
  struct st_mpc_p75pfs_bit BIT;
};

struct st_mpc_p76pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p76pfs
{
  unsigned char BYTE;
  struct st_mpc_p76pfs_bit BIT;
};

struct st_mpc_p77pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p77pfs
{
  unsigned char BYTE;
  struct st_mpc_p77pfs_bit BIT;
};

struct st_mpc_p80pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p80pfs
{
  unsigned char BYTE;
  struct st_mpc_p80pfs_bit BIT;
};

struct st_mpc_p81pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p81pfs
{
  unsigned char BYTE;
  struct st_mpc_p81pfs_bit BIT;
};

struct st_mpc_p82pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p82pfs
{
  unsigned char BYTE;
  struct st_mpc_p82pfs_bit BIT;
};

struct st_mpc_p83pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p83pfs
{
  unsigned char BYTE;
  struct st_mpc_p83pfs_bit BIT;
};

struct st_mpc_p84pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p84pfs
{
  unsigned char BYTE;
  struct st_mpc_p84pfs_bit BIT;
};

struct st_mpc_p85pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p85pfs
{
  unsigned char BYTE;
  struct st_mpc_p85pfs_bit BIT;
};

struct st_mpc_p86pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p86pfs
{
  unsigned char BYTE;
  struct st_mpc_p86pfs_bit BIT;
};

struct st_mpc_p87pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p87pfs
{
  unsigned char BYTE;
  struct st_mpc_p87pfs_bit BIT;
};

struct st_mpc_p90pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p90pfs
{
  unsigned char BYTE;
  struct st_mpc_p90pfs_bit BIT;
};

struct st_mpc_p91pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p91pfs
{
  unsigned char BYTE;
  struct st_mpc_p91pfs_bit BIT;
};

struct st_mpc_p92pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p92pfs
{
  unsigned char BYTE;
  struct st_mpc_p92pfs_bit BIT;
};

struct st_mpc_p93pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_p93pfs
{
  unsigned char BYTE;
  struct st_mpc_p93pfs_bit BIT;
};

struct st_mpc_pa0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa0pfs
{
  unsigned char BYTE;
  struct st_mpc_pa0pfs_bit BIT;
};

struct st_mpc_pa1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa1pfs
{
  unsigned char BYTE;
  struct st_mpc_pa1pfs_bit BIT;
};

struct st_mpc_pa2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa2pfs
{
  unsigned char BYTE;
  struct st_mpc_pa2pfs_bit BIT;
};

struct st_mpc_pa3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa3pfs
{
  unsigned char BYTE;
  struct st_mpc_pa3pfs_bit BIT;
};

struct st_mpc_pa4pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa4pfs
{
  unsigned char BYTE;
  struct st_mpc_pa4pfs_bit BIT;
};

struct st_mpc_pa5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa5pfs
{
  unsigned char BYTE;
  struct st_mpc_pa5pfs_bit BIT;
};

struct st_mpc_pa6pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};
union un_mpc_pa6pfs
{
  unsigned char BYTE;
  struct st_mpc_pa6pfs_bit BIT;
};

struct st_mpc_pa7pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pa7pfs
{
  unsigned char BYTE;
  struct st_mpc_pa7pfs_bit BIT;
};

struct st_mpc_pb0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb0pfs
{
  unsigned char BYTE;
  struct st_mpc_pb0pfs_bit BIT;
};

struct st_mpc_pb1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb1pfs
{
  unsigned char BYTE;
  struct st_mpc_pb1pfs_bit BIT;
};

struct st_mpc_pb2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb2pfs
{
  unsigned char BYTE;
  struct st_mpc_pb2pfs_bit BIT;
};

struct st_mpc_pb3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb3pfs
{
  unsigned char BYTE;
  struct st_mpc_pb3pfs_bit BIT;
};

struct st_mpc_pb4pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb4pfs
{
  unsigned char BYTE;
  struct st_mpc_pb4pfs_bit BIT;
};

struct st_mpc_pb5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb5pfs
{
  unsigned char BYTE;
  struct st_mpc_pb5pfs_bit BIT;
};

struct st_mpc_pb6pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb6pfs
{
  unsigned char BYTE;
  struct st_mpc_pb6pfs_bit BIT;
};

struct st_mpc_pb7pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pb7pfs
{
  unsigned char BYTE;
  struct st_mpc_pb7pfs_bit BIT;
};

struct st_mpc_pc0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc0pfs
{
  unsigned char BYTE;
  struct st_mpc_pc0pfs_bit BIT;
};

struct st_mpc_pc1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc1pfs
{
  unsigned char BYTE;
  struct st_mpc_pc1pfs_bit BIT;
};

struct st_mpc_pc2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc2pfs
{
  unsigned char BYTE;
  struct st_mpc_pc2pfs_bit BIT;
};

struct st_mpc_pc3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc3pfs
{
  unsigned char BYTE;
  struct st_mpc_pc3pfs_bit BIT;
};

struct st_mpc_pc4pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc4pfs
{
  unsigned char BYTE;
  struct st_mpc_pc4pfs_bit BIT;
};

struct st_mpc_pc5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc5pfs
{
  unsigned char BYTE;
  struct st_mpc_pc5pfs_bit BIT;
};

struct st_mpc_pc6pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc6pfs
{
  unsigned char BYTE;
  struct st_mpc_pc6pfs_bit BIT;
};

struct st_mpc_pc7pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pc7pfs
{
  unsigned char BYTE;
  struct st_mpc_pc7pfs_bit BIT;
};

struct st_mpc_pd0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd0pfs
{
  unsigned char BYTE;
  struct st_mpc_pd0pfs_bit BIT;
};

struct st_mpc_pd1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd1pfs
{
  unsigned char BYTE;
  struct st_mpc_pd1pfs_bit BIT;
};

struct st_mpc_pd2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd2pfs
{
  unsigned char BYTE;
  struct st_mpc_pd2pfs_bit BIT;
};

struct st_mpc_pd3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd3pfs
{
  unsigned char BYTE;
  struct st_mpc_pd3pfs_bit BIT;
};

struct st_mpc_pd4pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd4pfs
{
  unsigned char BYTE;
  struct st_mpc_pd4pfs_bit BIT;
};

struct st_mpc_pd5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd5pfs
{
  unsigned char BYTE;
  struct st_mpc_pd5pfs_bit BIT;
};

struct st_mpc_pd6pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd6pfs
{
  unsigned char BYTE;
  struct st_mpc_pd6pfs_bit BIT;
};

struct st_mpc_pd7pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pd7pfs
{
  unsigned char BYTE;
  struct st_mpc_pd7pfs_bit BIT;
};

struct st_mpc_pe0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe0pfs
{
  unsigned char BYTE;
  struct st_mpc_pe0pfs_bit BIT;
};

struct st_mpc_pe1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe1pfs
{
  unsigned char BYTE;
  struct st_mpc_pe1pfs_bit BIT;
};

struct st_mpc_pe2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe2pfs
{
  unsigned char BYTE;
  struct st_mpc_pe2pfs_bit BIT;
};

struct st_mpc_pe3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe3pfs
{
  unsigned char BYTE;
  struct st_mpc_pe3pfs_bit BIT;
};

struct st_mpc_pe4pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char  : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe4pfs
{
  unsigned char BYTE;
  struct st_mpc_pe4pfs_bit BIT;
};

struct st_mpc_pe5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe5pfs
{
  unsigned char BYTE;
  struct st_mpc_pe5pfs_bit BIT;
};

struct st_mpc_pe6pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe6pfs
{
  unsigned char BYTE;
  struct st_mpc_pe6pfs_bit BIT;
};

struct st_mpc_pe7pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char ISEL : 1;
  unsigned char ASEL : 1;
#else
  unsigned char ASEL : 1;
  unsigned char ISEL : 1;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pe7pfs
{
  unsigned char BYTE;
  struct st_mpc_pe7pfs_bit BIT;
};

struct st_mpc_pf0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pf0pfs
{
  unsigned char BYTE;
  struct st_mpc_pf0pfs_bit BIT;
};

struct st_mpc_pf1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pf1pfs
{
  unsigned char BYTE;
  struct st_mpc_pf1pfs_bit BIT;
};

struct st_mpc_pf2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pf2pfs
{
  unsigned char BYTE;
  struct st_mpc_pf2pfs_bit BIT;
};

struct st_mpc_pf5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char ISEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ISEL : 1;
  unsigned char  : 6;
#endif
};

union un_mpc_pf5pfs
{
  unsigned char BYTE;
  struct st_mpc_pf5pfs_bit BIT;
};

struct st_mpc_pj0pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pj0pfs
{
  unsigned char BYTE;
  struct st_mpc_pj0pfs_bit BIT;
};

struct st_mpc_pj1pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pj1pfs
{
  unsigned char BYTE;
  struct st_mpc_pj1pfs_bit BIT;
};

struct st_mpc_pj2pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pj2pfs
{
  unsigned char BYTE;
  struct st_mpc_pj2pfs_bit BIT;
};

struct st_mpc_pj3pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pj3pfs
{
  unsigned char BYTE;
  struct st_mpc_pj3pfs_bit BIT;
};

struct st_mpc_pj5pfs_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSEL : 6;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char PSEL : 6;
#endif
};

union un_mpc_pj5pfs
{
  unsigned char BYTE;
  struct st_mpc_pj5pfs_bit BIT;
};

union un_mdmonr
{
  unsigned short WORD;
  struct st_system_mdmonr_bit BIT;
};

struct st_system_syscr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ROME : 1;
  unsigned short EXBE : 1;
  unsigned short  : 6;
  unsigned short KEY : 8;
#else
  unsigned short KEY : 8;
  unsigned short  : 6;
  unsigned short EXBE : 1;
  unsigned short ROME : 1;
#endif
};

union un_syscr0
{
  unsigned short WORD;
  struct st_system_syscr0_bit BIT;
};

struct st_system_syscr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RAME : 1;
  unsigned short  : 6;
  unsigned short SBYRAME : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short SBYRAME : 1;
  unsigned short  : 6;
  unsigned short RAME : 1;
#endif
};

union un_syscr1
{
  unsigned short WORD;
  struct st_system_syscr1_bit BIT;
};

struct st_system_sbycr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 14;
  unsigned short OPE : 1;
  unsigned short SSBY : 1;
#else
  unsigned short SSBY : 1;
  unsigned short OPE : 1;
  unsigned short  : 14;
#endif
};

union un_sbycr
{
  unsigned short WORD;
  struct st_system_sbycr_bit BIT;
};

struct st_system_mstpcra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MSTPA0 : 1;
  unsigned long MSTPA1 : 1;
  unsigned long  : 2;
  unsigned long MSTPA4 : 1;
  unsigned long MSTPA5 : 1;
  unsigned long  : 3;
  unsigned long MSTPA9 : 1;
  unsigned long MSTPA10 : 1;
  unsigned long MSTPA11 : 1;
  unsigned long  : 1;
  unsigned long MSTPA13 : 1;
  unsigned long MSTPA14 : 1;
  unsigned long MSTPA15 : 1;
  unsigned long MSTPA16 : 1;
  unsigned long MSTPA17 : 1;
  unsigned long  : 1;
  unsigned long MSTPA19 : 1;
  unsigned long  : 4;
  unsigned long MSTPA24 : 1;
  unsigned long  : 2;
  unsigned long MSTPA27 : 1;
  unsigned long MSTPA28 : 1;
  unsigned long MSTPA29 : 1;
  unsigned long  : 1;
  unsigned long ACSE : 1;
#else
  unsigned long ACSE : 1;
  unsigned long  : 1;
  unsigned long MSTPA29 : 1;
  unsigned long MSTPA28 : 1;
  unsigned long MSTPA27 : 1;
  unsigned long  : 2;
  unsigned long MSTPA24 : 1;
  unsigned long  : 4;
  unsigned long MSTPA19 : 1;
  unsigned long  : 1;
  unsigned long MSTPA17 : 1;
  unsigned long MSTPA16 : 1;
  unsigned long MSTPA15 : 1;
  unsigned long MSTPA14 : 1;
  unsigned long MSTPA13 : 1;
  unsigned long  : 1;
  unsigned long MSTPA11 : 1;
  unsigned long MSTPA10 : 1;
  unsigned long MSTPA9 : 1;
  unsigned long  : 3;
  unsigned long MSTPA5 : 1;
  unsigned long MSTPA4 : 1;
  unsigned long  : 2;
  unsigned long MSTPA1 : 1;
  unsigned long MSTPA0 : 1;
#endif
};

union un_mstpcra
{
  unsigned long LONG;
  struct st_system_mstpcra_bit BIT;
};

struct st_system_mstpcrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MSTPB0 : 1;
  unsigned long MSTPB1 : 1;
  unsigned long  : 2;
  unsigned long MSTPB4 : 1;
  unsigned long  : 1;
  unsigned long MSTPB6 : 1;
  unsigned long  : 1;
  unsigned long MSTPB8 : 1;
  unsigned long MSTPB9 : 1;
  unsigned long  : 5;
  unsigned long MSTPB15 : 1;
  unsigned long MSTPB16 : 1;
  unsigned long MSTPB17 : 1;
  unsigned long  : 1;
  unsigned long MSTPB19 : 1;
  unsigned long MSTPB20 : 1;
  unsigned long MSTPB21 : 1;
  unsigned long MSTPB22 : 1;
  unsigned long MSTPB23 : 1;
  unsigned long MSTPB24 : 1;
  unsigned long MSTPB25 : 1;
  unsigned long MSTPB26 : 1;
  unsigned long MSTPB27 : 1;
  unsigned long MSTPB28 : 1;
  unsigned long MSTPB29 : 1;
  unsigned long MSTPB30 : 1;
  unsigned long MSTPB31 : 1;
#else
  unsigned long MSTPB31 : 1;
  unsigned long MSTPB30 : 1;
  unsigned long MSTPB29 : 1;
  unsigned long MSTPB28 : 1;
  unsigned long MSTPB27 : 1;
  unsigned long MSTPB26 : 1;
  unsigned long MSTPB25 : 1;
  unsigned long MSTPB24 : 1;
  unsigned long MSTPB23 : 1;
  unsigned long MSTPB22 : 1;
  unsigned long MSTPB21 : 1;
  unsigned long MSTPB20 : 1;
  unsigned long MSTPB19 : 1;
  unsigned long  : 1;
  unsigned long MSTPB17 : 1;
  unsigned long MSTPB16 : 1;
  unsigned long MSTPB15 : 1;
  unsigned long  : 5;
  unsigned long MSTPB9 : 1;
  unsigned long MSTPB8 : 1;
  unsigned long  : 1;
  unsigned long MSTPB6 : 1;
  unsigned long  : 1;
  unsigned long MSTPB4 : 1;
  unsigned long  : 2;
  unsigned long MSTPB1 : 1;
  unsigned long MSTPB0 : 1;
#endif
};

union un_mstpcrb
{
  unsigned long LONG;
  struct st_system_mstpcrb_bit BIT;
};

struct st_system_mstpcrc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MSTPC0 : 1;
  unsigned long  : 1;
  unsigned long MSTPC2 : 1;
  unsigned long  : 4;
  unsigned long MSTPC7 : 1;
  unsigned long  : 9;
  unsigned long MSTPC17 : 1;
  unsigned long  : 1;
  unsigned long MSTPC19 : 1;
  unsigned long  : 2;
  unsigned long MSTPC22 : 1;
  unsigned long MSTPC23 : 1;
  unsigned long MSTPC24 : 1;
  unsigned long MSTPC25 : 1;
  unsigned long MSTPC26 : 1;
  unsigned long MSTPC27 : 1;
  unsigned long MSTPC28 : 1;
  unsigned long MSTPC29 : 1;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long MSTPC29 : 1;
  unsigned long MSTPC28 : 1;
  unsigned long MSTPC27 : 1;
  unsigned long MSTPC26 : 1;
  unsigned long MSTPC25 : 1;
  unsigned long MSTPC24 : 1;
  unsigned long MSTPC23 : 1;
  unsigned long MSTPC22 : 1;
  unsigned long  : 2;
  unsigned long MSTPC19 : 1;
  unsigned long  : 1;
  unsigned long MSTPC17 : 1;
  unsigned long  : 9;
  unsigned long MSTPC7 : 1;
  unsigned long  : 4;
  unsigned long MSTPC2 : 1;
  unsigned long  : 1;
  unsigned long MSTPC0 : 1;
#endif
};

union un_mstpcrc
{
  unsigned long LONG;
  struct st_system_mstpcrc_bit BIT;
};

struct st_system_mstpcrd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MSTPD0 : 1;
  unsigned long MSTPD1 : 1;
  unsigned long MSTPD2 : 1;
  unsigned long MSTPD3 : 1;
  unsigned long MSTPD4 : 1;
  unsigned long MSTPD5 : 1;
  unsigned long MSTPD6 : 1;
  unsigned long MSTPD7 : 1;
  unsigned long  : 5;
  unsigned long MSTPD13 : 1;
  unsigned long  : 5;
  unsigned long MSTPD19 : 1;
  unsigned long  : 1;
  unsigned long MSTPD21 : 1;
  unsigned long  : 5;
  unsigned long MSTPD27 : 1;
  unsigned long  : 4;
#else
  unsigned long  : 4;
  unsigned long MSTPD27 : 1;
  unsigned long  : 5;
  unsigned long MSTPD21 : 1;
  unsigned long  : 1;
  unsigned long MSTPD19 : 1;
  unsigned long  : 5;
  unsigned long MSTPD13 : 1;
  unsigned long  : 5;
  unsigned long MSTPD7 : 1;
  unsigned long MSTPD6 : 1;
  unsigned long MSTPD5 : 1;
  unsigned long MSTPD4 : 1;
  unsigned long MSTPD3 : 1;
  unsigned long MSTPD2 : 1;
  unsigned long MSTPD1 : 1;
  unsigned long MSTPD0 : 1;
#endif
};

union un_mstpcrd
{
  unsigned long LONG;
  struct st_system_mstpcrd_bit BIT;
};

struct st_system_sckcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PCKD : 4;
  unsigned long PCKC : 4;
  unsigned long PCKB : 4;
  unsigned long PCKA : 4;
  unsigned long BCK : 4;
  unsigned long  : 2;
  unsigned long PSTOP0 : 1;
  unsigned long PSTOP1 : 1;
  unsigned long ICK : 4;
  unsigned long FCK : 4;
#else
  unsigned long FCK : 4;
  unsigned long ICK : 4;
  unsigned long PSTOP1 : 1;
  unsigned long PSTOP0 : 1;
  unsigned long  : 2;
  unsigned long BCK : 4;
  unsigned long PCKA : 4;
  unsigned long PCKB : 4;
  unsigned long PCKC : 4;
  unsigned long PCKD : 4;
#endif
};

union un_sckcr
{
  unsigned long LONG;
  struct st_system_sckcr_bit BIT;
};

struct st_system_sckcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 4;
  unsigned short UCK : 4;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short UCK : 4;
  unsigned short  : 4;
#endif
};

union un_sckcr2
{
  unsigned short WORD;
  struct st_system_sckcr2_bit BIT;
};

struct st_system_sckcr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 8;
  unsigned short CKSEL : 3;
  unsigned short  : 5;
#else
  unsigned short  : 5;
  unsigned short CKSEL : 3;
  unsigned short  : 8;
#endif
};

union un_sckcr3
{
  unsigned short WORD;
  struct st_system_sckcr3_bit BIT;
};

struct st_system_pllcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PLIDIV : 2;
  unsigned short  : 2;
  unsigned short PLLSRCSEL : 1;
  unsigned short  : 3;
  unsigned short STC : 6;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short STC : 6;
  unsigned short  : 3;
  unsigned short PLLSRCSEL : 1;
  unsigned short  : 2;
  unsigned short PLIDIV : 2;
#endif
};

union un_pllcr
{
  unsigned short WORD;
  struct st_system_pllcr_bit BIT;
};

struct st_system_pllcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PLLEN : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char PLLEN : 1;
#endif
};

union un_pllcr2
{
  unsigned char BYTE;
  struct st_system_pllcr2_bit BIT;
};

struct st_system_bckcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCLKDIV : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char BCLKDIV : 1;
#endif
};

union un_bckcr
{
  unsigned char BYTE;
  struct st_system_bckcr_bit BIT;
};

struct st_system_mosccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MOSTP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char MOSTP : 1;
#endif
};

union un_mosccr
{
  unsigned char BYTE;
  struct st_system_mosccr_bit BIT;
};

struct st_system_sosccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SOSTP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SOSTP : 1;
#endif
};

union un_sosccr
{
  unsigned char BYTE;
  struct st_system_sosccr_bit BIT;
};

struct st_system_lococcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LCSTP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char LCSTP : 1;
#endif
};

union un_lococr
{
  unsigned char BYTE;
  struct st_system_lococcr_bit BIT;
};

struct st_system_ilococr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ILCSTP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char ILCSTP : 1;
#endif
};

union un_ilococr
{
  unsigned char BYTE;
  struct st_system_ilococr_bit BIT;
};

struct st_system_hococr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HCSTP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char HCSTP : 1;
#endif
};

union un_hococr
{
  unsigned char BYTE;
  struct st_system_hococr_bit BIT;
};

struct st_system_hococr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HCFRQ : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char HCFRQ : 2;
#endif
};

union un_hococr2
{
  unsigned char BYTE;
  struct st_system_hococr2_bit BIT;
};

struct st_system_oscovfsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MOOVF : 1;
  unsigned char SOOVF : 1;
  unsigned char PLOVF : 1;
  unsigned char HCOVF : 1;
  unsigned char ILCOVF : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char ILCOVF : 1;
  unsigned char HCOVF : 1;
  unsigned char PLOVF : 1;
  unsigned char SOOVF : 1;
  unsigned char MOOVF : 1;
#endif
};

union un_oscovfsr
{
  unsigned char BYTE;
  struct st_system_oscovfsr_bit BIT;
};

struct st_system_ostdcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OSTDIE : 1;
  unsigned char  : 6;
  unsigned char OSTDE : 1;
#else
  unsigned char OSTDE : 1;
  unsigned char  : 6;
  unsigned char OSTDIE : 1;
#endif
};

union un_ostdcr
{
  unsigned char BYTE;
  struct st_system_ostdcr_bit BIT;
};

struct st_system_ostdsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OSTDF : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char OSTDF : 1;
#endif
};

union un_ostdsr
{
  unsigned char BYTE;
  struct st_system_ostdsr_bit BIT;
};

struct st_system_opccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OPCM : 3;
  unsigned char  : 1;
  unsigned char OPCMTSF : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char OPCMTSF : 1;
  unsigned char  : 1;
  unsigned char OPCM : 3;
#endif
};

struct st_system_rstckcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RSTCKSEL : 3;
  unsigned char  : 4;
  unsigned char RSTCKEN : 1;
#else
  unsigned char RSTCKEN : 1;
  unsigned char  : 4;
  unsigned char RSTCKSEL : 3;
#endif
};

union un_rstckcr
{
  unsigned char BYTE;
  struct st_system_rstckcr_bit BIT;
};

union un_opccr
{
  unsigned char BYTE;
  struct st_system_opccr_bit BIT;
};

struct st_system_moscwtcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MSTS : 8;
#else
  unsigned char MSTS : 8;
#endif
};

union un_moscwtcr
{
  unsigned char BYTE;
  struct st_system_moscwtcr_bit BIT;
};

struct st_system_soscwtcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSTS : 8;
#else
  unsigned char SSTS : 8;
#endif
};

union un_soscwtcr
{
  unsigned char BYTE;
  struct st_system_soscwtcr_bit BIT;
};

struct st_system_rstsr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IWDTRF : 1;
  unsigned char WDTRF : 1;
  unsigned char SWRF : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SWRF : 1;
  unsigned char WDTRF : 1;
  unsigned char IWDTRF : 1;
#endif
};

union un_rstsr2
{
  unsigned char BYTE;
  struct st_system_rstsr2_bit BIT;
};

struct st_system_lvd1cr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD1IDTSEL : 2;
  unsigned char LVD1IRQSEL : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char LVD1IRQSEL : 1;
  unsigned char LVD1IDTSEL : 2;
#endif
};

union un_lvd1cr1
{
  unsigned char BYTE;
  struct st_system_lvd1cr1_bit BIT;
};

struct st_system_lvd1sr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD1DET : 1;
  unsigned char LVD1MON : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char LVD1MON : 1;
  unsigned char LVD1DET : 1;
#endif
};

union un_lvd1sr
{
  unsigned char BYTE;
  struct st_system_lvd1sr_bit BIT;
};

struct st_system_lvd2cr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD2IDTSEL : 2;
  unsigned char LVD2IRQSEL : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char LVD2IRQSEL : 1;
  unsigned char LVD2IDTSEL : 2;
#endif
};

union un_lvd2cr1
{
  unsigned char BYTE;
  struct st_system_lvd2cr1_bit BIT;
};

struct st_system_lvd2sr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD2DET : 1;
  unsigned char LVD2MON : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char LVD2MON : 1;
  unsigned char LVD2DET : 1;
#endif
};

union un_lvd2sr
{
  unsigned char BYTE;
  struct st_system_lvd2sr_bit BIT;
};

struct st_system_prcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PRC0 : 1;
  unsigned short PRC1 : 1;
  unsigned short  : 1;
  unsigned short PRC3 : 1;
  unsigned short  : 4;
  unsigned short PRKEY : 8;
#else
  unsigned short PRKEY : 8;
  unsigned short  : 4;
  unsigned short PRC3 : 1;
  unsigned short  : 1;
  unsigned short PRC1 : 1;
  unsigned short PRC0 : 1;
#endif
};

union un_prcr
{
  unsigned short WORD;
  struct st_system_prcr_bit BIT;
};

struct st_system_romwt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ROMWT : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char ROMWT : 2;
#endif
};

union un_romwt
{
  unsigned char BYTE;
  struct st_system_romwt_bit BIT;
};

struct st_system_dpsbycr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DEEPCUT : 2;
  unsigned char  : 4;
  unsigned char IOKEEP : 1;
  unsigned char DPSBY : 1;
#else
  unsigned char DPSBY : 1;
  unsigned char IOKEEP : 1;
  unsigned char  : 4;
  unsigned char DEEPCUT : 2;
#endif
};

union un_dpsbycr
{
  unsigned char BYTE;
  struct st_system_dpsbycr_bit BIT;
};

struct st_system_dpsier0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ0E : 1;
  unsigned char DIRQ1E : 1;
  unsigned char DIRQ2E : 1;
  unsigned char DIRQ3E : 1;
  unsigned char DIRQ4E : 1;
  unsigned char DIRQ5E : 1;
  unsigned char DIRQ6E : 1;
  unsigned char DIRQ7E : 1;
#else
  unsigned char DIRQ7E : 1;
  unsigned char DIRQ6E : 1;
  unsigned char DIRQ5E : 1;
  unsigned char DIRQ4E : 1;
  unsigned char DIRQ3E : 1;
  unsigned char DIRQ2E : 1;
  unsigned char DIRQ1E : 1;
  unsigned char DIRQ0E : 1;
#endif
};

union un_dpsier0
{
  unsigned char BYTE;
  struct st_system_dpsier0_bit BIT;
};

struct st_system_dpsier1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ8E : 1;
  unsigned char DIRQ9E : 1;
  unsigned char DIRQ10E : 1;
  unsigned char DIRQ11E : 1;
  unsigned char DIRQ12E : 1;
  unsigned char DIRQ13E : 1;
  unsigned char DIRQ14E : 1;
  unsigned char DIRQ15E : 1;
#else
  unsigned char DIRQ15E : 1;
  unsigned char DIRQ14E : 1;
  unsigned char DIRQ13E : 1;
  unsigned char DIRQ12E : 1;
  unsigned char DIRQ11E : 1;
  unsigned char DIRQ10E : 1;
  unsigned char DIRQ9E : 1;
  unsigned char DIRQ8E : 1;
#endif
};

union un_dpsier1
{
  unsigned char BYTE;
  struct st_system_dpsier1_bit BIT;
};

struct st_system_dpsier2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DLVD1IE : 1;
  unsigned char DLVD2IE : 1;
  unsigned char DRTCIIE : 1;
  unsigned char DRTCAIE : 1;
  unsigned char DNMIE : 1;
  unsigned char DRIICDIE : 1;
  unsigned char DRIICCIE : 1;
  unsigned char DUSBIE : 1;
#else
  unsigned char DUSBIE : 1;
  unsigned char DRIICCIE : 1;
  unsigned char DRIICDIE : 1;
  unsigned char DNMIE : 1;
  unsigned char DRTCAIE : 1;
  unsigned char DRTCIIE : 1;
  unsigned char DLVD2IE : 1;
  unsigned char DLVD1IE : 1;
#endif
};

union un_dpsier2
{
  unsigned char BYTE;
  struct st_system_dpsier2_bit BIT;
};

struct st_system_dpsier3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DCANIE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DCANIE : 1;
#endif
};

union un_dpsier3
{
  unsigned char BYTE;
  struct st_system_dpsier3_bit BIT;
};

struct st_system_dpsifr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ0F : 1;
  unsigned char DIRQ1F : 1;
  unsigned char DIRQ2F : 1;
  unsigned char DIRQ3F : 1;
  unsigned char DIRQ4F : 1;
  unsigned char DIRQ5F : 1;
  unsigned char DIRQ6F : 1;
  unsigned char DIRQ7F : 1;
#else
  unsigned char DIRQ7F : 1;
  unsigned char DIRQ6F : 1;
  unsigned char DIRQ5F : 1;
  unsigned char DIRQ4F : 1;
  unsigned char DIRQ3F : 1;
  unsigned char DIRQ2F : 1;
  unsigned char DIRQ1F : 1;
  unsigned char DIRQ0F : 1;
#endif
};

union un_dpsifr0
{
  unsigned char BYTE;
  struct st_system_dpsifr0_bit BIT;
};

struct st_system_dpsifr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ8F : 1;
  unsigned char DIRQ9F : 1;
  unsigned char DIRQ10F : 1;
  unsigned char DIRQ11F : 1;
  unsigned char DIRQ12F : 1;
  unsigned char DIRQ13F : 1;
  unsigned char DIRQ14F : 1;
  unsigned char DIRQ15F : 1;
#else
  unsigned char DIRQ15F : 1;
  unsigned char DIRQ14F : 1;
  unsigned char DIRQ13F : 1;
  unsigned char DIRQ12F : 1;
  unsigned char DIRQ11F : 1;
  unsigned char DIRQ10F : 1;
  unsigned char DIRQ9F : 1;
  unsigned char DIRQ8F : 1;
#endif
};

union un_dpsifr1
{
  unsigned char BYTE;
  struct st_system_dpsifr1_bit BIT;
};

struct st_system_dpsifr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DLVD1IF : 1;
  unsigned char DLVD2IF : 1;
  unsigned char DRTCIIF : 1;
  unsigned char DRTCAIF : 1;
  unsigned char DNMIF : 1;
  unsigned char DRIICDIF : 1;
  unsigned char DRIICCIF : 1;
  unsigned char DUSBIF : 1;
#else
  unsigned char DUSBIF : 1;
  unsigned char DRIICCIF : 1;
  unsigned char DRIICDIF : 1;
  unsigned char DNMIF : 1;
  unsigned char DRTCAIF : 1;
  unsigned char DRTCIIF : 1;
  unsigned char DLVD2IF : 1;
  unsigned char DLVD1IF : 1;
#endif
};

union un_dpsifr2
{
  unsigned char BYTE;
  struct st_system_dpsifr2_bit BIT;
};

struct st_system_dpsifr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DCANIF : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DCANIF : 1;
#endif
};

union un_dpsifr3
{
  unsigned char BYTE;
  struct st_system_dpsifr3_bit BIT;
};

struct st_system_dpsiegr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ0EG : 1;
  unsigned char DIRQ1EG : 1;
  unsigned char DIRQ2EG : 1;
  unsigned char DIRQ3EG : 1;
  unsigned char DIRQ4EG : 1;
  unsigned char DIRQ5EG : 1;
  unsigned char DIRQ6EG : 1;
  unsigned char DIRQ7EG : 1;
#else
  unsigned char DIRQ7EG : 1;
  unsigned char DIRQ6EG : 1;
  unsigned char DIRQ5EG : 1;
  unsigned char DIRQ4EG : 1;
  unsigned char DIRQ3EG : 1;
  unsigned char DIRQ2EG : 1;
  unsigned char DIRQ1EG : 1;
  unsigned char DIRQ0EG : 1;
#endif
};

union un_dpsiegr0
{
  unsigned char BYTE;
  struct st_system_dpsiegr0_bit BIT;
};

struct st_system_dpsiegr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DIRQ8EG : 1;
  unsigned char DIRQ9EG : 1;
  unsigned char DIRQ10EG : 1;
  unsigned char DIRQ11EG : 1;
  unsigned char DIRQ12EG : 1;
  unsigned char DIRQ13EG : 1;
  unsigned char DIRQ14EG : 1;
  unsigned char DIRQ15EG : 1;
#else
  unsigned char DIRQ15EG : 1;
  unsigned char DIRQ14EG : 1;
  unsigned char DIRQ13EG : 1;
  unsigned char DIRQ12EG : 1;
  unsigned char DIRQ11EG : 1;
  unsigned char DIRQ10EG : 1;
  unsigned char DIRQ9EG : 1;
  unsigned char DIRQ8EG : 1;
#endif
};

union un_dpsiegr1
{
  unsigned char BYTE;
  struct st_system_dpsiegr1_bit BIT;
};

struct st_system_dpsiegr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DLVD1EG : 1;
  unsigned char DLVD2EG : 1;
  unsigned char  : 2;
  unsigned char DNMIEG : 1;
  unsigned char DRIICDEG : 1;
  unsigned char DRIICCEG : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char DRIICCEG : 1;
  unsigned char DRIICDEG : 1;
  unsigned char DNMIEG : 1;
  unsigned char  : 2;
  unsigned char DLVD2EG : 1;
  unsigned char DLVD1EG : 1;
#endif
};

union un_dpsiegr2
{
  unsigned char BYTE;
  struct st_system_dpsiegr2_bit BIT;
};

struct st_system_dpsiegr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DCANIEG : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DCANIEG : 1;
#endif
};

union un_dpsiegr3
{
  unsigned char BYTE;
  struct st_system_dpsiegr3_bit BIT;
};

struct st_system_rstsr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PORF : 1;
  unsigned char LVD0RF : 1;
  unsigned char LVD1RF : 1;
  unsigned char LVD2RF : 1;
  unsigned char  : 3;
  unsigned char DPSRSTF : 1;
#else
  unsigned char DPSRSTF : 1;
  unsigned char  : 3;
  unsigned char LVD2RF : 1;
  unsigned char LVD1RF : 1;
  unsigned char LVD0RF : 1;
  unsigned char PORF : 1;
#endif
};

union un_rstsr0
{
  unsigned char BYTE;
  struct st_system_rstsr0_bit BIT;
};

struct st_system_rstr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CWSF : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char CWSF : 1;
#endif
};

union un_rstsr1
{
  unsigned char BYTE;
  struct st_system_rstr1_bit BIT;
};

struct st_system_mofcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MOFXIN : 1;
  unsigned char  : 3;
  unsigned char MODRV2 : 2;
  unsigned char MOSEL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char MOSEL : 1;
  unsigned char MODRV2 : 2;
  unsigned char  : 3;
  unsigned char MOFXIN : 1;
#endif
};

union un_mofcr
{
  unsigned char BYTE;
  struct st_system_mofcr_bit BIT;
};

struct st_system_hocopcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char HOCOPCNT : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char HOCOPCNT : 1;
#endif
};

union un_hocopcr
{
  unsigned char BYTE;
  struct st_system_hocopcr_bit BIT;
};

struct st_system_lvcmpcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 5;
  unsigned char LVD1E : 1;
  unsigned char LVD2E : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char LVD2E : 1;
  unsigned char LVD1E : 1;
  unsigned char  : 5;
#endif
};

union un_lvcmpcr
{
  unsigned char BYTE;
  struct st_system_lvcmpcr_bit BIT;
};

struct st_system_lvdlvlr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD1LVL : 4;
  unsigned char LVD2LVL : 4;
#else
  unsigned char LVD2LVL : 4;
  unsigned char LVD1LVL : 4;
#endif
};

union un_lvdlvlr
{
  unsigned char BYTE;
  struct st_system_lvdlvlr_bit BIT;
};

struct st_system_lvd1cr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD1RIE : 1;
  unsigned char LVD1DFDIS : 1;
  unsigned char LVD1CMPE : 1;
  unsigned char  : 1;
  unsigned char LVD1FSAMP : 2;
  unsigned char LVD1RI : 1;
  unsigned char LVD1RN : 1;
#else
  unsigned char LVD1RN : 1;
  unsigned char LVD1RI : 1;
  unsigned char LVD1FSAMP : 2;
  unsigned char  : 1;
  unsigned char LVD1CMPE : 1;
  unsigned char LVD1DFDIS : 1;
  unsigned char LVD1RIE : 1;
#endif
};

union un_lvd1cr0
{
  unsigned char BYTE;
  struct st_system_lvd1cr0_bit BIT;
};

struct st_system_lvd2cr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LVD2RIE : 1;
  unsigned char LVD2DFDIS : 1;
  unsigned char LVD2CMPE : 1;
  unsigned char  : 1;
  unsigned char LVD2FSAMP : 2;
  unsigned char LVD2RI : 1;
  unsigned char LVD2RN : 1;
#else
  unsigned char LVD2RN : 1;
  unsigned char LVD2RI : 1;
  unsigned char LVD2FSAMP : 2;
  unsigned char  : 1;
  unsigned char LVD2CMPE : 1;
  unsigned char LVD2DFDIS : 1;
  unsigned char LVD2RIE : 1;
#endif
};

union un_lvd2cr0
{
  unsigned char BYTE;
  struct st_system_lvd2cr0_bit BIT;
};

struct st_rtc_rcr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RTCEN : 1;
  unsigned char RTCDV : 3;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char RTCDV : 3;
  unsigned char RTCEN : 1;
#endif
};

union un_rcr3
{
  unsigned char BYTE;
  struct st_rtc_rcr3_bit BIT;
};

struct st_port0_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_pdr
{
  unsigned char BYTE;
  struct st_port0_pdr_bit BIT;
};

struct st_port0_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_podr
{
  unsigned char BYTE;
  struct st_port0_podr_bit BIT;
};

struct st_port0_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_pidr
{
  unsigned char BYTE;
  struct st_port0_pidr_bit BIT;
};

struct st_port0_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_pmr
{
  unsigned char BYTE;
  struct st_port0_pmr_bit BIT;
};

struct st_port0_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_odr0
{
  unsigned char BYTE;
  struct st_port0_odr0_bit BIT;
};

struct st_port0_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 2;
  unsigned char B2 : 1;
  unsigned char  : 3;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 3;
  unsigned char B2 : 1;
  unsigned char  : 2;
#endif
};

union un_odr1
{
  unsigned char BYTE;
  struct st_port0_odr1_bit BIT;
};

struct st_port0_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_pcr
{
  unsigned char BYTE;
  struct st_port0_pcr_bit BIT;
};

struct st_port0_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_dscr
{
  unsigned char BYTE;
  struct st_port0_dscr_bit BIT;
};

struct st_port0_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_dscr2
{
  unsigned char BYTE;
  struct st_port0_dscr2_bit BIT;
};

struct st_port1_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_pdr
{
  unsigned char BYTE;
  struct st_port1_pdr_bit BIT;
};

struct st_port1_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_podr
{
  unsigned char BYTE;
  struct st_port1_podr_bit BIT;
};

struct st_port1_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_pidr
{
  unsigned char BYTE;
  struct st_port1_pidr_bit BIT;
};

struct st_port1_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_pmr
{
  unsigned char BYTE;
  struct st_port1_pmr_bit BIT;
};

struct st_port1_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_odr0
{
  unsigned char BYTE;
  struct st_port1_odr0_bit BIT;
};

struct st_port1_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_odr1
{
  unsigned char BYTE;
  struct st_port1_odr1_bit BIT;
};

struct st_port1_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port1_pcr
{
  unsigned char BYTE;
  struct st_port1_pcr_bit BIT;
};

struct st_port1_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char  : 1;
#endif
};

union un_port1_dscr
{
  unsigned char BYTE;
  struct st_port1_dscr_bit BIT;
};

struct st_port1_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 2;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 2;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char  : 1;
#endif
};

union un_port1_dscr2
{
  unsigned char BYTE;
  struct st_port1_dscr2_bit BIT;
};

struct st_port2_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_pdr
{
  unsigned char BYTE;
  struct st_port2_pdr_bit BIT;
};

struct st_port2_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_podr
{
  unsigned char BYTE;
  struct st_port2_podr_bit BIT;
};

struct st_port2_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_pidr
{
  unsigned char BYTE;
  struct st_port2_pidr_bit BIT;
};

struct st_port2_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_pmr
{
  unsigned char BYTE;
  struct st_port2_pmr_bit BIT;
};

struct st_port2_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_odr0
{
  unsigned char BYTE;
  struct st_port2_odr0_bit BIT;
};

struct st_port2_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_odr1
{
  unsigned char BYTE;
  struct st_port2_odr1_bit BIT;
};

struct st_port2_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_pcr
{
  unsigned char BYTE;
  struct st_port2_pcr_bit BIT;
};

struct st_port2_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 7;
#endif
};

union un_port2_dscr
{
  unsigned char BYTE;
  struct st_port2_dscr_bit BIT;
};

struct st_port2_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 3;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 3;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port2_dscr2
{
  unsigned char BYTE;
  struct st_port2_dscr2_bit BIT;
};

struct st_port3_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_pdr
{
  unsigned char BYTE;
  struct st_port3_pdr_bit BIT;
};

struct st_port3_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_podr
{
  unsigned char BYTE;
  struct st_port3_podr_bit BIT;
};

struct st_port3_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_pidr
{
  unsigned char BYTE;
  struct st_port3_pidr_bit BIT;
};

struct st_port3_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_pmr
{
  unsigned char BYTE;
  struct st_port3_pmr_bit BIT;
};

struct st_port3_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_odr0
{
  unsigned char BYTE;
  struct st_port3_odr0_bit BIT;
};

struct st_port3_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 3;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 3;
  unsigned char BT0 : 1;
#endif
};

union un_port3_odr1
{
  unsigned char BYTE;
  struct st_port3_odr1_bit BIT;
};

struct st_port3_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_pcr
{
  unsigned char BYTE;
  struct st_port3_pcr_bit BIT;
};

struct st_port3_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port3_dscr2
{
  unsigned char BYTE;
  struct st_port3_dscr2_bit BIT;
};

struct st_port5_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_pdr
{
  unsigned char BYTE;
  struct st_port5_pdr_bit BIT;
};

struct st_port5_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_podr
{
  unsigned char BYTE;
  struct st_port5_podr_bit BIT;
};

struct st_port5_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_pidr
{
  unsigned char BYTE;
  struct st_port5_pidr_bit BIT;
};

struct st_port5_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_pmr
{
  unsigned char BYTE;
  struct st_port5_pmr_bit BIT;
};

struct st_port5_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_odr0
{
  unsigned char BYTE;
  struct st_port5_odr0_bit BIT;
};

struct st_port5_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_odr1
{
  unsigned char BYTE;
  struct st_port5_odr1_bit BIT;
};

struct st_port5_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_pcr
{
  unsigned char BYTE;
  struct st_port5_pcr_bit BIT;
};

struct st_port5_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_dscr
{
  unsigned char BYTE;
  struct st_port5_dscr_bit BIT;
};

struct st_port5_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port5_dscr2
{
  unsigned char BYTE;
  struct st_port5_dscr2_bit BIT;
};

struct st_port7_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_pdr
{
  unsigned char BYTE;
  struct st_port7_pdr_bit BIT;
};

struct st_port7_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_podr
{
  unsigned char BYTE;
  struct st_port7_podr_bit BIT;
};

struct st_port7_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_pidr
{
  unsigned char BYTE;
  struct st_port7_pidr_bit BIT;
};

struct st_port7_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_pmr
{
  unsigned char BYTE;
  struct st_port7_pmr_bit BIT;
};

struct st_port7_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_odr0
{
  unsigned char BYTE;
  struct st_port7_odr0_bit BIT;
};

struct st_port7_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_odr1
{
  unsigned char BYTE;
  struct st_port7_odr1_bit BIT;
};

struct st_port7_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_pcr
{
  unsigned char BYTE;
  struct st_port7_pcr_bit BIT;
};

struct st_port7_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 2;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 2;
#endif
};

union un_port7_dscr
{
  unsigned char BYTE;
  struct st_port7_dscr_bit BIT;
};

struct st_port7_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port7_dscr2
{
  unsigned char BYTE;
  struct st_port7_dscr2_bit BIT;
};

struct st_port8_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_pdr
{
  unsigned char BYTE;
  struct st_port8_pdr_bit BIT;
};

struct st_port8_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_podr
{
  unsigned char BYTE;
  struct st_port8_podr_bit BIT;
};

struct st_port8_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_pidr
{
  unsigned char BYTE;
  struct st_port8_pidr_bit BIT;
};

struct st_port8_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_pmr
{
  unsigned char BYTE;
  struct st_port8_pmr_bit BIT;
};

struct st_port8_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_odr0
{
  unsigned char BYTE;
  struct st_port8_odr0_bit BIT;
};

struct st_port8_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_odr1
{
  unsigned char BYTE;
  struct st_port8_odr1_bit BIT;
};

struct st_port8_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_pcr
{
  unsigned char BYTE;
  struct st_port8_pcr_bit BIT;
};

struct st_port8_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_dscr
{
  unsigned char BYTE;
  struct st_port8_dscr_bit BIT;
};

struct st_port8_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port8_dscr2
{
  unsigned char BYTE;
  struct st_port8_dscr2_bit BIT;
};

struct st_port9_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_pdr
{
  unsigned char BYTE;
  struct st_port9_pdr_bit BIT;
};

struct st_port9_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_podr
{
  unsigned char BYTE;
  struct st_port9_podr_bit BIT;
};

struct st_port9_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_pidr
{
  unsigned char BYTE;
  struct st_port9_pidr_bit BIT;
};

struct st_port9_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_pmr
{
  unsigned char BYTE;
  struct st_port9_pmr_bit BIT;
};

struct st_port9_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_odr0
{
  unsigned char BYTE;
  struct st_port9_odr0_bit BIT;
};

struct st_port9_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_odr1
{
  unsigned char BYTE;
  struct st_port9_odr1_bit BIT;
};

struct st_port9_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_pcr
{
  unsigned char BYTE;
  struct st_port9_pcr_bit BIT;
};

struct st_port9_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_dscr
{
  unsigned char BYTE;
  struct st_port9_dscr_bit BIT;
};

struct st_port9_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_port9_dscr2
{
  unsigned char BYTE;
  struct st_port9_dscr2_bit BIT;
};

struct st_porta_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_pdr
{
  unsigned char BYTE;
  struct st_porta_pdr_bit BIT;
};

struct st_porta_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_podr
{
  unsigned char BYTE;
  struct st_porta_podr_bit BIT;
};

struct st_porta_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_pidr
{
  unsigned char BYTE;
  struct st_porta_pidr_bit BIT;
};

struct st_porta_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_pmr
{
  unsigned char BYTE;
  struct st_porta_pmr_bit BIT;
};

struct st_porta_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_odr0
{
  unsigned char BYTE;
  struct st_porta_odr0_bit BIT;
};

struct st_porta_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_odr1
{
  unsigned char BYTE;
  struct st_porta_odr1_bit BIT;
};

struct st_porta_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_pcr
{
  unsigned char BYTE;
  struct st_porta_pcr_bit BIT;
};

struct st_porta_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_dscr
{
  unsigned char BYTE;
  struct st_porta_dscr_bit BIT;
};

struct st_porta_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porta_dscr2
{
  unsigned char BYTE;
  struct st_porta_dscr2_bit BIT;
};

struct st_portb_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_pdr
{
  unsigned char BYTE;
  struct st_portb_pdr_bit BIT;
};

struct st_portb_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_podr
{
  unsigned char BYTE;
  struct st_portb_podr_bit BIT;
};

struct st_portb_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_pidr
{
  unsigned char BYTE;
  struct st_portb_pidr_bit BIT;
};

struct st_portb_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_pmr
{
  unsigned char BYTE;
  struct st_portb_pmr_bit BIT;
};

struct st_portb_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_odr0
{
  unsigned char BYTE;
  struct st_portb_odr0_bit BIT;
};

struct st_portb_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_odr1
{
  unsigned char BYTE;
  struct st_portb_odr1_bit BIT;
};

struct st_portb_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_pcr
{
  unsigned char BYTE;
  struct st_portb_pcr_bit BIT;
};

struct st_portb_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_dscr
{
  unsigned char BYTE;
  struct st_portb_dscr_bit BIT;
};

struct st_portb_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portb_dscr2
{
  unsigned char BYTE;
  struct st_portb_dscr2_bit BIT;
};

struct st_portc_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_pdr
{
  unsigned char BYTE;
  struct st_portc_pdr_bit BIT;
};

struct st_portc_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_podr
{
  unsigned char BYTE;
  struct st_portc_podr_bit BIT;
};

struct st_portc_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_pidr
{
  unsigned char BYTE;
  struct st_portc_pidr_bit BIT;
};

struct st_portc_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_pmr
{
  unsigned char BYTE;
  struct st_portc_pmr_bit BIT;
};

struct st_portc_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_odr0
{
  unsigned char BYTE;
  struct st_portc_odr0_bit BIT;
};

struct st_portc_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_odr1
{
  unsigned char BYTE;
  struct st_portc_odr1_bit BIT;
};

struct st_portc_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_pcr
{
  unsigned char BYTE;
  struct st_portc_pcr_bit BIT;
};

struct st_portc_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_dscr
{
  unsigned char BYTE;
  struct st_portc_dscr_bit BIT;
};

struct st_portc_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portc_dscr2
{
  unsigned char BYTE;
  struct st_portc_dscr2_bit BIT;
};

struct st_porte_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_pdr
{
  unsigned char BYTE;
  struct st_porte_pdr_bit BIT;
};

struct st_porte_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_podr
{
  unsigned char BYTE;
  struct st_porte_podr_bit BIT;
};

struct st_porte_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_pidr
{
  unsigned char BYTE;
  struct st_porte_pidr_bit BIT;
};

struct st_porte_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_pmr
{
  unsigned char BYTE;
  struct st_porte_pmr_bit BIT;
};

struct st_porte_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_odr0
{
  unsigned char BYTE;
  struct st_porte_odr0_bit BIT;
};

struct st_porte_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_odr1
{
  unsigned char BYTE;
  struct st_porte_odr1_bit BIT;
};

struct st_porte_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_pcr
{
  unsigned char BYTE;
  struct st_porte_pcr_bit BIT;
};

struct st_porte_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_dscr
{
  unsigned char BYTE;
  struct st_porte_dscr_bit BIT;
};

struct st_porte_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

struct st_portf_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_porte_dscr2
{
  unsigned char BYTE;
  struct st_porte_dscr2_bit BIT;
};

union un_portf_pdr
{
  unsigned char BYTE;
  struct st_portf_pdr_bit BIT;
};

struct st_portf_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_podr
{
  unsigned char BYTE;
  struct st_portf_podr_bit BIT;
};

struct st_portf_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_pidr
{
  unsigned char BYTE;
  struct st_portf_pidr_bit BIT;
};

struct st_portf_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_pmr
{
  unsigned char BYTE;
  struct st_portf_pmr_bit BIT;
};

struct st_portf_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_odr0
{
  unsigned char BYTE;
  struct st_portf_odr0_bit BIT;
};

struct st_portf_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_odr1
{
  unsigned char BYTE;
  struct st_portf_odr1_bit BIT;
};

struct st_portf_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portf_pcr
{
  unsigned char BYTE;
  struct st_portf_pcr_bit BIT;
};

struct st_portg_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_pdr
{
  unsigned char BYTE;
  struct st_portg_pdr_bit BIT;
};

struct st_portg_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_podr
{
  unsigned char BYTE;
  struct st_portg_podr_bit BIT;
};

struct st_portg_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_pidr
{
  unsigned char BYTE;
  struct st_portg_pidr_bit BIT;
};

struct st_portg_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_pmr
{
  unsigned char BYTE;
  struct st_portg_pmr_bit BIT;
};

struct st_portg_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_odr0
{
  unsigned char BYTE;
  struct st_portg_odr0_bit BIT;
};

struct st_portg_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_odr1
{
  unsigned char BYTE;
  struct st_portg_odr1_bit BIT;
};

struct st_portg_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_pcr
{
  unsigned char BYTE;
  struct st_portg_pcr_bit BIT;
};

struct st_portg_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_dscr
{
  unsigned char BYTE;
  struct st_portg_dscr_bit BIT;
};

struct st_portg_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char B4 : 1;
  unsigned char B5 : 1;
  unsigned char B6 : 1;
  unsigned char B7 : 1;
#else
  unsigned char B7 : 1;
  unsigned char B6 : 1;
  unsigned char B5 : 1;
  unsigned char B4 : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portg_dscr2
{
  unsigned char BYTE;
  struct st_portg_dscr2_bit BIT;
};

struct st_portj_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_pdr
{
  unsigned char BYTE;
  struct st_portj_pdr_bit BIT;
};

struct st_portj_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_podr
{
  unsigned char BYTE;
  struct st_portj_podr_bit BIT;
};

struct st_portj_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_pidr
{
  unsigned char BYTE;
  struct st_portj_pidr_bit BIT;
};

struct st_portj_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_pmr
{
  unsigned char BYTE;
  struct st_portj_pmr_bit BIT;
};

struct st_portj_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char B6 : 1;
  unsigned char  : 1;
  unsigned char B4 : 1;
  unsigned char  : 1;
  unsigned char B2 : 1;
  unsigned char  : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_odr0
{
  unsigned char BYTE;
  struct st_portj_odr0_bit BIT;
};

struct st_portj_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 2;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char  : 2;
#endif
};

union un_portj_odr1
{
  unsigned char BYTE;
  struct st_portj_odr1_bit BIT;
};

struct st_portj_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char B3 : 1;
  unsigned char  : 1;
  unsigned char B5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char B5 : 1;
  unsigned char  : 1;
  unsigned char B3 : 1;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_pcr
{
  unsigned char BYTE;
  struct st_portj_pcr_bit BIT;
};

struct st_portj_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_dscr
{
  unsigned char BYTE;
  struct st_portj_dscr_bit BIT;
};

struct st_portj_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BT0 : 1;
  unsigned char B1 : 1;
  unsigned char B2 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char B2 : 1;
  unsigned char B1 : 1;
  unsigned char BT0 : 1;
#endif
};

union un_portj_dscr2
{
  unsigned char BYTE;
  struct st_portj_dscr2_bit BIT;
};

struct st_sci0_smr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 2;
  unsigned char MP : 1;
  unsigned char STOP : 1;
  unsigned char PM : 1;
  unsigned char PE : 1;
  unsigned char CHR : 1;
  unsigned char CM : 1;
#else
  unsigned char CM : 1;
  unsigned char CHR : 1;
  unsigned char PE : 1;
  unsigned char PM : 1;
  unsigned char STOP : 1;
  unsigned char MP : 1;
  unsigned char CKS : 2;
#endif
};

union un_sci0_smr
{
  unsigned char BYTE;
  struct st_sci0_smr_bit BIT;
};

struct st_sci0_scr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKE : 2;
  unsigned char TEIE : 1;
  unsigned char MPIE : 1;
  unsigned char RE : 1;
  unsigned char TE : 1;
  unsigned char RIE : 1;
  unsigned char TIE : 1;
#else
  unsigned char TIE : 1;
  unsigned char RIE : 1;
  unsigned char TE : 1;
  unsigned char RE : 1;
  unsigned char MPIE : 1;
  unsigned char TEIE : 1;
  unsigned char CKE : 2;
#endif
};

union un_sci0_scr
{
  unsigned char BYTE;
  struct st_sci0_scr_bit BIT;
};

struct st_sci0_ssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MPBT : 1;
  unsigned char MPB : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char FER : 1;
  unsigned char ORER : 1;
  unsigned char RDRF : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char RDRF : 1;
  unsigned char ORER : 1;
  unsigned char FER : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char MPB : 1;
  unsigned char MPBT : 1;
#endif
};

union un_sci0_ssr
{
  unsigned char BYTE;
  struct st_sci0_ssr_bit BIT;
};

struct st_sci0_scmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SMIF : 1;
  unsigned char  : 1;
  unsigned char SINV : 1;
  unsigned char SDIR : 1;
  unsigned char CHR1 : 1;
  unsigned char  : 2;
  unsigned char BCP2 : 1;
#else
  unsigned char BCP2 : 1;
  unsigned char  : 2;
  unsigned char CHR1 : 1;
  unsigned char SDIR : 1;
  unsigned char SINV : 1;
  unsigned char  : 1;
  unsigned char SMIF : 1;
#endif
};

union un_sci0_scmr
{
  unsigned char BYTE;
  struct st_sci0_scmr_bit BIT;
};

struct st_sci0_semr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ACS0 : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ABCS : 1;
  unsigned char NFEN : 1;
  unsigned char BGDM : 1;
  unsigned char RXDESEL : 1;
#else
  unsigned char RXDESEL : 1;
  unsigned char BGDM : 1;
  unsigned char NFEN : 1;
  unsigned char ABCS : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ACS0 : 1;
#endif
};

union un_sci0_semr
{
  unsigned char BYTE;
  struct st_sci0_semr_bit BIT;
};

struct st_sci0_snfr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFCS : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char NFCS : 3;
#endif
};

union un_sci0_snfr
{
  unsigned char BYTE;
  struct st_sci0_snfr_bit BIT;
};

struct st_sci0_simr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICM : 1;
  unsigned char  : 2;
  unsigned char IICDL : 5;
#else
  unsigned char IICDL : 5;
  unsigned char  : 2;
  unsigned char IICM : 1;
#endif
};

union un_sci0_simr1
{
  unsigned char BYTE;
  struct st_sci0_simr1_bit BIT;
};

struct st_sci0_simr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICINTM : 1;
  unsigned char IICCSC : 1;
  unsigned char  : 3;
  unsigned char IICACKT : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char IICACKT : 1;
  unsigned char  : 3;
  unsigned char IICCSC : 1;
  unsigned char IICINTM : 1;
#endif
};

union un_sci0_simr2
{
  unsigned char BYTE;
  struct st_sci0_simr2_bit BIT;
};

struct st_sci0_simr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICSTAREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICSTIF : 1;
  unsigned char IICSDAS : 2;
  unsigned char IICSCLS : 2;
#else
  unsigned char IICSCLS : 2;
  unsigned char IICSDAS : 2;
  unsigned char IICSTIF : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTAREQ : 1;
#endif
};

union un_sci0_simr3
{
  unsigned char BYTE;
  struct st_sci0_simr3_bit BIT;
};

struct st_sci0_sisr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICACKR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char IICACKR : 1;
#endif
};

union un_sci0_sisr
{
  unsigned char BYTE;
  struct st_sci0_sisr_bit BIT;
};

struct st_sci0_spmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSE : 1;
  unsigned char CTSE : 1;
  unsigned char MSS : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char CKPOL : 1;
  unsigned char CKPH : 1;
#else
  unsigned char CKPH : 1;
  unsigned char CKPOL : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char MSS : 1;
  unsigned char CTSE : 1;
  unsigned char SSE : 1;
#endif
};

union un_sci0_spmr
{
  unsigned char BYTE;
  struct st_sci0_spmr_bit BIT;
};

struct st_sci0_tdrhl_byte
{
  unsigned char TDRH;
  unsigned char TDRL;
};

union un_sci0_tdrhl
{
  unsigned short WORD;
  struct st_sci0_tdrhl_byte BYTE;
};

struct st_sci0_rdrhl_byte
{
  unsigned char RDRH;
  unsigned char RDRL;
};

union un_sci0_rdrhl
{
  unsigned short WORD;
  struct st_sci0_rdrhl_byte BYTE;
};

struct st_sci10_smr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 2;
  unsigned char MP : 1;
  unsigned char STOP : 1;
  unsigned char PM : 1;
  unsigned char PE : 1;
  unsigned char CHR : 1;
  unsigned char CM : 1;
#else
  unsigned char CM : 1;
  unsigned char CHR : 1;
  unsigned char PE : 1;
  unsigned char PM : 1;
  unsigned char STOP : 1;
  unsigned char MP : 1;
  unsigned char CKS : 2;
#endif
};

union un_sci10_smr
{
  unsigned char BYTE;
  struct st_sci10_smr_bit BIT;
};

struct st_sci10_scr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKE : 2;
  unsigned char TEIE : 1;
  unsigned char MPIE : 1;
  unsigned char RE : 1;
  unsigned char TE : 1;
  unsigned char RIE : 1;
  unsigned char TIE : 1;
#else
  unsigned char TIE : 1;
  unsigned char RIE : 1;
  unsigned char TE : 1;
  unsigned char RE : 1;
  unsigned char MPIE : 1;
  unsigned char TEIE : 1;
  unsigned char CKE : 2;
#endif
};

union un_sci10_scr
{
  unsigned char BYTE;
  struct st_sci10_scr_bit BIT;
};

struct st_sci10_ssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MPBT : 1;
  unsigned char MPB : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char FER : 1;
  unsigned char ORER : 1;
  unsigned char RDRF : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char RDRF : 1;
  unsigned char ORER : 1;
  unsigned char FER : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char MPB : 1;
  unsigned char MPBT : 1;
#endif
};

union un_sci10_ssr
{
  unsigned char BYTE;
  struct st_sci10_ssr_bit BIT;
};

struct st_sci10_ssrfifo_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DR : 1;
  unsigned char  : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char FER : 1;
  unsigned char ORER : 1;
  unsigned char RDF : 1;
  unsigned char TDFE : 1;
#else
  unsigned char TDFE : 1;
  unsigned char RDF : 1;
  unsigned char ORER : 1;
  unsigned char FER : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char  : 1;
  unsigned char DR : 1;
#endif
};

union un_sci10_ssrfifo
{
  unsigned char BYTE;
  struct st_sci10_ssrfifo_bit BIT;
};

struct st_sci10_scmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SMIF : 1;
  unsigned char  : 1;
  unsigned char SINV : 1;
  unsigned char SDIR : 1;
  unsigned char CHR1 : 1;
  unsigned char  : 2;
  unsigned char BCP2 : 1;
#else
  unsigned char BCP2 : 1;
  unsigned char  : 2;
  unsigned char CHR1 : 1;
  unsigned char SDIR : 1;
  unsigned char SINV : 1;
  unsigned char  : 1;
  unsigned char SMIF : 1;
#endif
};

union un_sci10_scmr
{
  unsigned char BYTE;
  struct st_sci10_scmr_bit BIT;
};

struct st_sci10_semr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ACS0 : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ABCS : 1;
  unsigned char NFEN : 1;
  unsigned char BGDM : 1;
  unsigned char RXDESEL : 1;
#else
  unsigned char RXDESEL : 1;
  unsigned char BGDM : 1;
  unsigned char NFEN : 1;
  unsigned char ABCS : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ACS0 : 1;
#endif
};

union un_sci10_semr
{
  unsigned char BYTE;
  struct st_sci10_semr_bit BIT;
};

struct st_sci10_snfr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFCS : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char NFCS : 3;
#endif
};

union un_sci10_snfr
{
  unsigned char BYTE;
  struct st_sci10_snfr_bit BIT;
};

struct st_sci10_simr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICM : 1;
  unsigned char  : 2;
  unsigned char IICDL : 5;
#else
  unsigned char IICDL : 5;
  unsigned char  : 2;
  unsigned char IICM : 1;
#endif
};

union un_sci10_simr1
{
  unsigned char BYTE;
  struct st_sci10_simr1_bit BIT;
};

struct st_sci10_simr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICINTM : 1;
  unsigned char IICCSC : 1;
  unsigned char  : 3;
  unsigned char IICACKT : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char IICACKT : 1;
  unsigned char  : 3;
  unsigned char IICCSC : 1;
  unsigned char IICINTM : 1;
#endif
};

union un_sci10_simr2
{
  unsigned char BYTE;
  struct st_sci10_simr2_bit BIT;
};

struct st_sci10_simr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICSTAREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICSTIF : 1;
  unsigned char IICSDAS : 2;
  unsigned char IICSCLS : 2;
#else
  unsigned char IICSCLS : 2;
  unsigned char IICSDAS : 2;
  unsigned char IICSTIF : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTAREQ : 1;
#endif
};

union un_sci10_simr3
{
  unsigned char BYTE;
  struct st_sci10_simr3_bit BIT;
};

struct st_sci10_sisr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICACKR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char IICACKR : 1;
#endif
};

union un_sci10_sisr
{
  unsigned char BYTE;
  struct st_sci10_sisr_bit BIT;
};

struct st_sci10_spmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSE : 1;
  unsigned char CTSE : 1;
  unsigned char MSS : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char CKPOL : 1;
  unsigned char CKPH : 1;
#else
  unsigned char CKPH : 1;
  unsigned char CKPOL : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char MSS : 1;
  unsigned char CTSE : 1;
  unsigned char SSE : 1;
#endif
};

union un_sci10_spmr
{
  unsigned char BYTE;
  struct st_sci10_spmr_bit BIT;
};

struct st_sci10_tdrhl_byte
{
  unsigned char TDRH;
  unsigned char TDRL;
};

union un_sci10_tdrhl
{
  unsigned short WORD;
  struct st_sci10_tdrhl_byte BYTE;
};

struct st_sci10_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_ftdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TDAT : 9;
  unsigned short MPBT : 1;
  unsigned short  : 6;
#else
  unsigned short  : 6;
  unsigned short MPBT : 1;
  unsigned short TDAT : 9;
#endif
};

union un_sci10_ftdr_bit_byte
{
  unsigned short WORD;
  struct st_sci10_byte BYTE;
  struct st_sci10_ftdr_bit BIT;
};

struct st_sci10_rdrhl_byte
{
  unsigned char RDRH;
  unsigned char RDRL;
};

union un_sci10_rdrhl_byte
{
  unsigned short WORD;
  struct st_sci10_rdrhl_byte BYTE;
};

struct st_sci10_frdr_bit_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_frdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RDAT : 9;
  unsigned short MPB : 1;
  unsigned short DR : 1;
  unsigned short PER : 1;
  unsigned short FER : 1;
  unsigned short ORER : 1;
  unsigned short RDF : 1;
  unsigned short  : 1;
#else
  unsigned short  : 1;
  unsigned short RDF : 1;
  unsigned short ORER : 1;
  unsigned short FER : 1;
  unsigned short PER : 1;
  unsigned short DR : 1;
  unsigned short MPB : 1;
  unsigned short RDAT : 9;
#endif
};

union un_sci10_frdr_bit_byte
{
  unsigned short WORD;
  struct st_sci10_frdr_bit_byte BYTE;
  struct st_sci10_frdr_bit BIT;
};

struct st_sci10_dccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DCMF : 1;
  unsigned char  : 2;
  unsigned char DPER : 1;
  unsigned char DFER : 1;
  unsigned char  : 1;
  unsigned char IDSEL : 1;
  unsigned char DCME : 1;
#else
  unsigned char DCME : 1;
  unsigned char IDSEL : 1;
  unsigned char  : 1;
  unsigned char DFER : 1;
  unsigned char DPER : 1;
  unsigned char  : 2;
  unsigned char DCMF : 1;
#endif
};

union un_sci10_dccr
{
  unsigned char BYTE;
  struct st_sci10_dccr_bit BIT;
};

struct st_sci10_fcr_bit_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_fcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short FM : 1;
  unsigned short RFRST : 1;
  unsigned short TFRST : 1;
  unsigned short DRES : 1;
  unsigned short TTRG : 4;
  unsigned short RTRG : 4;
  unsigned short RSTRG : 4;
#else
  unsigned short RSTRG : 4;
  unsigned short RTRG : 4;
  unsigned short TTRG : 4;
  unsigned short DRES : 1;
  unsigned short TFRST : 1;
  unsigned short RFRST : 1;
  unsigned short FM : 1;
#endif
};

union un_sci10_fcr
{
  unsigned short WORD;
  struct st_sci10_fcr_bit_byte BYTE;
  struct st_sci10_fcr_bit BIT;
};

struct st_sci10_fdr_bit_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_fdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short R : 5;
  unsigned short  : 3;
  unsigned short T : 5;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short T : 5;
  unsigned short  : 3;
  unsigned short R : 5;
#endif
};

union un_sci10_fdr
{
  unsigned short WORD;
  struct st_sci10_fdr_bit_byte BYTE;
  struct st_sci10_fdr_bit BIT;
};

struct st_sci10_lsr_bit_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_lsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ORER : 1;
  unsigned short  : 1;
  unsigned short FNUM : 5;
  unsigned short  : 1;
  unsigned short PNUM : 5;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short PNUM : 5;
  unsigned short  : 1;
  unsigned short FNUM : 5;
  unsigned short  : 1;
  unsigned short ORER : 1;
#endif
};

union un_sci10_lsr
{
  unsigned short WORD;
  struct st_sci10_lsr_bit_byte BYTE;
  struct st_sci10_lsr_bit BIT;
};

struct st_sci10_cdr_bit_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_sci10_cdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPD : 9;
  unsigned short  : 7;
#else
  unsigned short  : 7;
  unsigned short CMPD : 9;
#endif
};

union un_sci10_cdr
{
  unsigned short WORD;
  struct st_sci10_cdr_bit_byte BYTE;
  struct st_sci10_cdr_bit BIT;
};

struct st_sci10_sptr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RXDMON : 1;
  unsigned char SPB2DT : 1;
  unsigned char SPB2IO : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SPB2IO : 1;
  unsigned char SPB2DT : 1;
  unsigned char RXDMON : 1;
#endif
};

union st_sci10_sptr
{
  unsigned char BYTE;
  struct st_sci10_sptr_bit BIT;
};

struct st_sci12_smr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 2;
  unsigned char MP : 1;
  unsigned char STOP : 1;
  unsigned char PM : 1;
  unsigned char PE : 1;
  unsigned char CHR : 1;
  unsigned char CM : 1;
#else
  unsigned char CM : 1;
  unsigned char CHR : 1;
  unsigned char PE : 1;
  unsigned char PM : 1;
  unsigned char STOP : 1;
  unsigned char MP : 1;
  unsigned char CKS : 2;
#endif
};

union un_sci12_smr
{
  unsigned char BYTE;
  struct st_sci12_smr_bit BIT;
};

struct st_sci12_scr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKE : 2;
  unsigned char TEIE : 1;
  unsigned char MPIE : 1;
  unsigned char RE : 1;
  unsigned char TE : 1;
  unsigned char RIE : 1;
  unsigned char TIE : 1;
#else
  unsigned char TIE : 1;
  unsigned char RIE : 1;
  unsigned char TE : 1;
  unsigned char RE : 1;
  unsigned char MPIE : 1;
  unsigned char TEIE : 1;
  unsigned char CKE : 2;
#endif
};

union un_sci12_scr
{
  unsigned char BYTE;
  struct st_sci12_scr_bit BIT;
};

struct st_sci12_ssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MPBT : 1;
  unsigned char MPB : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char FER : 1;
  unsigned char ORER : 1;
  unsigned char RDRF : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char RDRF : 1;
  unsigned char ORER : 1;
  unsigned char FER : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char MPB : 1;
  unsigned char MPBT : 1;
#endif
};

union un_sci12_ssr
{
  unsigned char BYTE;
  struct st_sci12_ssr_bit BIT;
};

struct st_sci12_scmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SMIF : 1;
  unsigned char  : 1;
  unsigned char SINV : 1;
  unsigned char SDIR : 1;
  unsigned char CHR1 : 1;
  unsigned char  : 2;
  unsigned char BCP2 : 1;
#else
  unsigned char BCP2 : 1;
  unsigned char  : 2;
  unsigned char CHR1 : 1;
  unsigned char SDIR : 1;
  unsigned char SINV : 1;
  unsigned char  : 1;
  unsigned char SMIF : 1;
#endif
};

union un_sci12_scmr
{
  unsigned char BYTE;
  struct st_sci12_scmr_bit BIT;
};

struct st_sci12_semr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ACS0 : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ABCS : 1;
  unsigned char NFEN : 1;
  unsigned char BGDM : 1;
  unsigned char RXDESEL : 1;
#else
  unsigned char RXDESEL : 1;
  unsigned char BGDM : 1;
  unsigned char NFEN : 1;
  unsigned char ABCS : 1;
  unsigned char  : 1;
  unsigned char BRME : 1;
  unsigned char  : 1;
  unsigned char ACS0 : 1;
#endif
};

union un_sci12_semr
{
  unsigned char BYTE;
  struct st_sci12_semr_bit BIT;
};

struct st_sci12_snfr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFCS : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char NFCS : 3;
#endif
};

union un_sci12_snfr
{
  unsigned char BYTE;
  struct st_sci12_snfr_bit BIT;
};

struct st_sci12_simr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICM : 1;
  unsigned char  : 2;
  unsigned char IICDL : 5;
#else
  unsigned char IICDL : 5;
  unsigned char  : 2;
  unsigned char IICM : 1;
#endif
};

union un_sci12_simr1
{
  unsigned char BYTE;
  struct st_sci12_simr1_bit BIT;
};

struct st_sci12_simr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICINTM : 1;
  unsigned char IICCSC : 1;
  unsigned char  : 3;
  unsigned char IICACKT : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char IICACKT : 1;
  unsigned char  : 3;
  unsigned char IICCSC : 1;
  unsigned char IICINTM : 1;
#endif
};

union un_sci12_simr2
{
  unsigned char BYTE;
  struct st_sci12_simr2_bit BIT;
};

struct st_sci12_simr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICSTAREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICSTIF : 1;
  unsigned char IICSDAS : 2;
  unsigned char IICSCLS : 2;
#else
  unsigned char IICSCLS : 2;
  unsigned char IICSDAS : 2;
  unsigned char IICSTIF : 1;
  unsigned char IICSTPREQ : 1;
  unsigned char IICRSTAREQ : 1;
  unsigned char IICSTAREQ : 1;
#endif
};

union un_sci12_simr3
{
  unsigned char BYTE;
  struct st_sci12_simr3_bit BIT;
};

struct st_sci12_sisr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IICACKR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char IICACKR : 1;
#endif
};

union un_sci12_sisr
{
  unsigned char BYTE;
  struct st_sci12_sisr_bit BIT;
};

struct st_sci12_spmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSE : 1;
  unsigned char CTSE : 1;
  unsigned char MSS : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char CKPOL : 1;
  unsigned char CKPH : 1;
#else
  unsigned char CKPH : 1;
  unsigned char CKPOL : 1;
  unsigned char  : 1;
  unsigned char MFF : 1;
  unsigned char  : 1;
  unsigned char MSS : 1;
  unsigned char CTSE : 1;
  unsigned char SSE : 1;
#endif
};

union un_sci12_spmr
{
  unsigned char BYTE;
  struct st_sci12_spmr_bit BIT;
};

struct st_sci12_tdrhl_byte
{
  unsigned char TDRH;
  unsigned char TDRL;
};

union un_sci12_tdrhl
{
  unsigned short WORD;
  struct st_sci12_tdrhl_byte BYTE;
};

struct st_sci12_rdrhl_byte
{
  unsigned char RDRH;
  unsigned char RDRL;
};

union un_sci12_rdrhl
{
  unsigned short WORD;
  struct st_sci12_rdrhl_byte BYTE;
};

struct st_sci12_esmer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ESME : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char ESME : 1;
#endif
};

union un_sci12_esmer
{
  unsigned char BYTE;
  struct st_sci12_esmer_bit BIT;
};

struct st_sci12_ctr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char SFSF : 1;
  unsigned char RXDSF : 1;
  unsigned char BRME : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char BRME : 1;
  unsigned char RXDSF : 1;
  unsigned char SFSF : 1;
  unsigned char  : 1;
#endif
};

union un_sci12_ctr0
{
  unsigned char BYTE;
  struct st_sci12_ctr0_bit BIT;
};

struct st_sci12_ctr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BFE : 1;
  unsigned char CF0RE : 1;
  unsigned char CF1DS : 2;
  unsigned char PIBE : 1;
  unsigned char PIBS : 3;
#else
  unsigned char PIBS : 3;
  unsigned char PIBE : 1;
  unsigned char CF1DS : 2;
  unsigned char CF0RE : 1;
  unsigned char BFE : 1;
#endif
};

union un_sci12_ctr1
{
  unsigned char BYTE;
  struct st_sci12_ctr1_bit BIT;
};

struct st_sci12_ctr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DFCS : 3;
  unsigned char  : 1;
  unsigned char BCCS : 2;
  unsigned char RTS : 2;
#else
  unsigned char RTS : 2;
  unsigned char BCCS : 2;
  unsigned char  : 1;
  unsigned char DFCS : 3;
#endif
};

union un_sci12_ctr2
{
  unsigned char BYTE;
  struct st_sci12_ctr2_bit BIT;
};

struct st_sci12_ctr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SDST : 1;
#endif
};

union un_sci12_ctr3
{
  unsigned char BYTE;
  struct st_sci12_ctr3_bit BIT;
};

struct st_sci12_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TXDXPS : 1;
  unsigned char RXDXPS : 1;
  unsigned char  : 2;
  unsigned char SHARPS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char SHARPS : 1;
  unsigned char  : 2;
  unsigned char RXDXPS : 1;
  unsigned char TXDXPS : 1;
#endif
};

union un_sci12_pcr
{
  unsigned char BYTE;
  struct st_sci12_pcr_bit BIT;
};

struct st_sci12_icr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BFDIE : 1;
  unsigned char CF0MIE : 1;
  unsigned char CF1MIE : 1;
  unsigned char PIBDIE : 1;
  unsigned char BCDIE : 1;
  unsigned char AEDIE : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char AEDIE : 1;
  unsigned char BCDIE : 1;
  unsigned char PIBDIE : 1;
  unsigned char CF1MIE : 1;
  unsigned char CF0MIE : 1;
  unsigned char BFDIE : 1;
#endif
};

union un_sci12_icr
{
  unsigned char BYTE;
  struct st_sci12_icr_bit BIT;
};

struct st_sci12_str_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BFDF : 1;
  unsigned char CF0MF : 1;
  unsigned char CF1MF : 1;
  unsigned char PIBDF : 1;
  unsigned char BCDF : 1;
  unsigned char AEDF : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char AEDF : 1;
  unsigned char BCDF : 1;
  unsigned char PIBDF : 1;
  unsigned char CF1MF : 1;
  unsigned char CF0MF : 1;
  unsigned char BFDF : 1;
#endif
};

union un_sci12_str
{
  unsigned char BYTE;
  struct st_sci12_str_bit BIT;
};

struct un_sci12_stcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BFDCL : 1;
  unsigned char CF0MCL : 1;
  unsigned char CF1MCL : 1;
  unsigned char PIBDCL : 1;
  unsigned char BCDCL : 1;
  unsigned char AEDCL : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char AEDCL : 1;
  unsigned char BCDCL : 1;
  unsigned char PIBDCL : 1;
  unsigned char CF1MCL : 1;
  unsigned char CF0MCL : 1;
  unsigned char BFDCL : 1;
#endif
};

union un_sci12_stcr
{
  unsigned char BYTE;
  struct un_sci12_stcr_bit BIT;
};

struct st_sci12_cf0cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CF0CE0 : 1;
  unsigned char CF0CE1 : 1;
  unsigned char CF0CE2 : 1;
  unsigned char CF0CE3 : 1;
  unsigned char CF0CE4 : 1;
  unsigned char CF0CE5 : 1;
  unsigned char CF0CE6 : 1;
  unsigned char CF0CE7 : 1;
#else
  unsigned char CF0CE7 : 1;
  unsigned char CF0CE6 : 1;
  unsigned char CF0CE5 : 1;
  unsigned char CF0CE4 : 1;
  unsigned char CF0CE3 : 1;
  unsigned char CF0CE2 : 1;
  unsigned char CF0CE1 : 1;
  unsigned char CF0CE0 : 1;
#endif
};

union un_sci12_cf0cr
{
  unsigned char BYTE;
  struct st_sci12_cf0cr_bit BIT;
};

struct st_sci12_cf1cr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CF1CE0 : 1;
  unsigned char CF1CE1 : 1;
  unsigned char CF1CE2 : 1;
  unsigned char CF1CE3 : 1;
  unsigned char CF1CE4 : 1;
  unsigned char CF1CE5 : 1;
  unsigned char CF1CE6 : 1;
  unsigned char CF1CE7 : 1;
#else
  unsigned char CF1CE7 : 1;
  unsigned char CF1CE6 : 1;
  unsigned char CF1CE5 : 1;
  unsigned char CF1CE4 : 1;
  unsigned char CF1CE3 : 1;
  unsigned char CF1CE2 : 1;
  unsigned char CF1CE1 : 1;
  unsigned char CF1CE0 : 1;
#endif
};

union un_sci12_cf1cr
{
  unsigned char BYTE;
  struct st_sci12_cf1cr_bit BIT;
};

struct st_sci12_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TCST : 1;
#endif
};

union un_sci12_tcr
{
  unsigned char BYTE;
  struct st_sci12_tcr_bit BIT;
};

struct st_sci12_tmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TOMS : 2;
  unsigned char  : 1;
  unsigned char TWRC : 1;
  unsigned char TCSS : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char TCSS : 3;
  unsigned char TWRC : 1;
  unsigned char  : 1;
  unsigned char TOMS : 2;
#endif
};

union un_sci12_tmr
{
  unsigned char BYTE;
  struct st_sci12_tmr_bit BIT;
};

typedef struct st_bsc
{
  union un_berclr BERCLR;
  char  wk0[3];
  char  wk1[3];
  union un_beren BEREN;
  char  wk2[1];
  union un_bersr1 BERSR1;
  char  wk3[4];
  union un_bersr2 BERSR2;
  char  wk4[7408];
  union un_buspri BUSPRI;
  union un_cs0mod CS0MOD;
  union un_cs0wcr1 CS0WCR1;
  char  wk5[6];
  union un_cs1mod CS1MOD;
  union un_cs1wcr1 CS1WCR1;
  union un_cs1wcr2 CS1WCR2;
  char  wk6[6];
  union un_cs2mod CS2MOD;
  union un_cs2wcr1 CS2WCR1;
  union un_cs2wcr2 CS2WCR2;
  char  wk7[6];
  union un_cs3mod CS3MOD;
  union un_cs3wcr1 CS3WCR1;
  union un_cs3wcr2 CS3WCR2;
  char  wk8[6];
  union un_cs4mod CS4MOD;
  union un_cs4wcr1 CS4WCR1;
  union un_cs4wcr2 CS4WCR2;
  char  wk9[6];
  union un_cs5mod CS5MOD;
  union un_cs5wcr1 CS5WCR1;
  union un_cs5wcr2 CS5WCR2;
  char  wk10[6];
  union un_cs6mod CS6MOD;
  union un_cs6wcr1 CS6WCR1;
  union un_cs6wcr2 CS6WCR2;
  char  wk11[6];
  union un_cs7mod CS7MOD;
  union un_cs7wcr1 CS7WCR1;
  union un_cs7wcr2 CS7WCR2;
  char  wk12[1926];
  union un_cs0cr CS0CR;
  char  wk13[6];
  union un_cs0rec CS0REC;
  char  wk14[6];
  union un_cs1cr CS1CR;
  char  wk15[6];
  union un_cs1rec CS1REC;
  char  wk16[6];
  union un_cs2cr CS2CR;
  char  wk17[6];
  union un_cs2rec CS2REC;
  char  wk18[6];
  union un_cs3cr CS3CR;
  char  wk19[6];
  union un_cs3rec CS3REC;
  char  wk20[6];
  union un_cs4cr CS4CR;
  char  wk21[6];
  union un_cs4rec CS4REC;
  char  wk22[6];
  union un_cs5cr CS5CR;
  char  wk23[6];
  union un_cs5rec CS5REC;
  char  wk24[6];
  union un_cs6cr CS6CR;
  char  wk25[6];
  union un_cs6rec CS6REC;
  char  wk26[6];
  union un_cs7cr CS7CR;
  char  wk27[6];
  union un_cs7rec CS7REC;
  char  wk28[4];
  union un_csrecen CSRECEN;
  char  wk29[894];
  union un_sdccr SDCCR;
  union un_sdcmod SDCMOD;
  union un_sdamod SDAMOD;
  char  wk30[13];
  union un_sdself SDSELF;
  char  wk31[3];
  union un_sdrfcr SDRFCR;
  union un_sdrfen SDRFEN;
  char  wk32[9];
  union un_sdicr SDICR;
  char  wk33[3];
  union un_sdir SDIR;
  char  wk34[26];
  union un_sdar SDAR;
  char  wk35[3];
  union un_sdtr SDTR;
  union un_sdmod SDMOD;
  char  wk36[6];
  union un_sdsr SDSR;
  char  wk37[269231];
  union un_ebmapcr EBMAPCR;
} st_bsc_t;

typedef struct st_cac
{
        union un_cacr0 CACR0;
        union un_cacr1 CACR1;
        union un_cacr2 CACR2;
        union un_caicr CAICR;
        union un_castr CASTR;
        char  wk0[1];
        unsigned short CAULVR;
        unsigned short CALLVR;
        unsigned short CACNTBR;
} st_cac_t;

typedef struct st_cmt
{
  union un_cmstr0 CMSTR0;
  char  wk0[14];
  union un_cmstr1 CMSTR1;
} st_cmt_t;

typedef struct st_cmt0
{
  union un_cmcr CMCR;
  unsigned short CMCNT;
  unsigned short CMCOR;
} st_cmt0_t;

typedef struct st_icu
{
  union un_icu_ir256 IR[256];
  union un_icu_dtcer256 DTCER[256];
  union un_ier32 IER[32];
  char  wk0[192];
  union un_swintr SWINTR;
  union un_swint2r SWINT2R;
  char  wk1[14];
  union un_icu_fir FIR;
  char    wk2[14];
  union un_icu_ipr256 IPR[256];
  unsigned char  DMRSR0;
  char    wk3[3];
  unsigned char  DMRSR1;
  char    wk4[3];
  unsigned char  DMRSR2;
  char   wk5[3];
  unsigned char  DMRSR3;
  char  wk6[3];
  unsigned char  DMRSR4;
  char  wk7[3];
  unsigned char  DMRSR5;
  char  wk8[3];
  unsigned char  DMRSR6;
  char  wk9[3];
  unsigned char  DMRSR7;
  char  wk10[227];
  union un_irqcr16 IRQCR[16];
  char  wk11[16];
  union un_irqflte0 IRQFLTE0;
  union un_irqflte1 IRQFLTE1;
  char  wk12[6];
  union un_icu_irqfltc0 IRQFLTC0;
  union un_icu_irqfltc1 IRQFLCT1;
  char  wk13[84];
  union un_icu_nmisr NMISR;
  union un_icu_nmier NMIER;
  union un_icu_nmiclr NMICLR;
  union un_icu_nmicr NMICR;
  char  wk14[12];
  union un_icu_nmiflte NMIFLTE;
  char  wk15[3];
  union un_icu_nmifltc NMIFLTC;
  char  wk16[107];
  union un_icu_grpbe0 GRPBE0;
  char  wk17[44];
  union un_grpbl0 GRPBL0;
  union un_grpbl1 GRPBL1;
  union un_icu_grpbl2 GRPBL2;
  char  wk18[4];
  union un_icu_genbe0 GENBE0;
  char  wk19[44];
  union un_genbl0 GENBL0;
  union un_genbl1 GENBL1;
  union un_icu_genbl2 GENBL2;
  char  wk20[4];
  union un_icu_gcrbe0 GCRBE0;
  char  wk21[124];
  union un_icu_pibr0 PIBR0;
  union un_icu_pibr1 PIBR1;
  union un_icu_pibr2 PIBR2;
  union un_icu_pibr3 PIBR3;
  union un_icu_pibr4 PIBR4;
  union un_icu_pibr5 PIBR5;
  union un_icu_pibr6 PIBR6;
  union un_icu_pibr7 PIBR7;
  union un_icu_pibr8 PIBR8;
  union un_icu_pibr9 PIBR9;
  union un_icu_pibra PIBRA;
  union un_icu_pibrb PIBRB;
  char  wk22[116];
  union un_icu_slibxr128 SLIBXR128;
  union un_icu_slibxr129 SLIBXR129;
  union un_icu_slibxr130 SLIBXR130;
  union un_icu_slibxr131 SLIBXR131;
  union un_icu_slibxr132 SLIBXR132;
  union un_icu_slibxr133 SLIBXR133;
  union un_icu_slibxr134 SLIBXR134;
  union un_icu_slibxr135 SLIBXR135;
  union un_icu_slibxr136 SLIBXR136;
  union un_icu_slibxr137 SLIBXR137;
  union un_icu_slibxr138 SLIBXR138;
  union un_icu_slibxr139 SLIBXR139;
  union un_icu_slibxr140 SLIBXR140;
  union un_icu_slibxr141 SLIBXR141;
  union un_icu_slibxr142 SLIBXR142;
  union un_icu_slibxr143 SLIBXR143;
  union un_icu_slibr144 SLIBR144;
  union un_icu_slibr145 SLIBR145;
  union un_icu_slibr146 SLIBR146;
  union un_icu_slibr147 SLIBR147;
  union un_icu_slibr148 SLIBR148;
  union un_icu_slibr149 SLIBR149;
  union un_icu_slibr150 SLIBR150;
  union un_icu_slibr151 SLIBR151;
  union un_icu_slibr152 SLIBR152;
  union un_icu_slibr153 SLIBR153;
  union un_icu_slibr154 SLIBR154;
  union un_icu_slibr155 SLIBR155;
  union un_icu_slibr156 SLIBR156;
  union un_icu_slibr157 SLIBR157;
  union un_icu_slibr158 SLIBR158;
  union un_icu_slibr159 SLIBR159;
  union un_icu_slibr160 SLIBR160;
  union un_icu_slibr161 SLIBR161;
  union un_icu_slibr162 SLIBR162;
  union un_icu_slibr163 SLIBR163;
  union un_icu_slibr164 SLIBR164;
  union un_icu_slibr165 SLIBR165;
  union un_icu_slibr166 SLIBR166;
  union un_icu_slibr167 SLIBR167;
  union un_icu_slibr168 SLIBR168;
  union un_icu_slibr169 SLIBR169;
  union un_icu_slibr170 SLIBR170;
  union un_icu_slibr171 SLIBR171;
  union un_icu_slibr172 SLIBR172;
  union un_icu_slibr173 SLIBR173;
  union un_icu_slibr174 SLIBR174;
  union un_icu_slibr175 SLIBR175;
  union un_icu_slibr176 SLIBR176;
  union un_icu_slibr177 SLIBR177;
  union un_icu_slibr178 SLIBR178;
  union un_icu_slibr179 SLIBR179;
  union un_icu_slibr180 SLIBR180;
  union un_icu_slibr181 SLIBR181;
  union un_icu_slibr182 SLIBR182;
  union un_icu_slibr183 SLIBR183;
  union un_icu_slibr184 SLIBR184;
  union un_icu_slibr185 SLIBR185;
  union un_icu_slibr186 SLIBR186;
  union un_icu_slibr187 SLIBR187;
  union un_icu_slibr188 SLIBR188;
  union un_icu_slibr189 SLIBR189;
  union un_icu_slibr190 SLIBR190;
  union un_icu_slibr191 SLIBR191;
  union un_icu_slibr192 SLIBR192;
  union un_icu_slibr193 SLIBR193;
  union un_icu_slibr194 SLIBR194;
  union un_icu_slibr195 SLIBR195;
  union un_icu_slibr196 SLIBR196;
  union un_icu_slibr197 SLIBR197;
  union un_icu_slibr198 SLIBR198;
  union un_icu_slibr199 SLIBR199;
  union un_icu_slibr200 SLIBR200;
  union un_icu_slibr201 SLIBR201;
  union un_icu_slibr202 SLIBR202;
  union un_icu_slibr203 SLIBR203;
  union un_icu_slibr204 SLIBR204;
  union un_icu_slibr205 SLIBR205;
  union un_icu_slibr206 SLIBR206;
  union un_icu_slibr207 SLIBR207;
  char  wk23[96];
  union un_grpal0 GRPAL0;
  union un_grpal1 GRPAL1;
  char  wk24[56];
  union un_genal0 GENAL0;
  union un_genal1 GENAL1;
  char  wk25[136];
  union un_icu_piar0 PIAR0;
  union un_icu_piar1 PIAR1;
  union un_icu_piar2 PIAR2;
  union un_icu_piar3 PIAR3;
  union un_icu_piar4 PIAR4;
  union un_icu_piar5 PIAR5;
  char  wk26[5];
  union un_icu_piarb PIARB;
  char  wk27[196];
  union un_icu_sliar208 SLIAR208;
  union un_icu_sliar209 SLIAR209;
  union un_icu_sliar210 SLIAR210;
  union un_icu_sliar211 SLIAR211;
  union un_icu_sliar212 SLIAR212;
  union un_icu_sliar213 SLIAR213;
  union un_icu_sliar214 SLIAR214;
  union un_icu_sliar215 SLIAR215;
  union un_icu_sliar216 SLIAR216;
  union un_icu_sliar217 SLIAR217;
  union un_icu_sliar218 SLIAR218;
  union un_icu_sliar219 SLIAR219;
  union un_icu_sliar220 SLIAR220;
  union un_icu_sliar221 SLIAR221;
  union un_icu_sliar222 SLIAR222;
  union un_icu_sliar223 SLIAR223;
  union un_icu_sliar224 SLIAR224;
  union un_icu_sliar225 SLIAR225;
  union un_icu_sliar226 SLIAR226;
  union un_icu_sliar227 SLIAR227;
  union un_icu_sliar228 SLIAR228;
  union un_icu_sliar229 SLIAR229;
  union un_icu_sliar230 SLIAR230;
  union un_icu_sliar231 SLIAR231;
  union un_icu_sliar232 SLIAR232;
  union un_icu_sliar233 SLIAR233;
  union un_icu_sliar234 SLIAR234;
  union un_icu_sliar235 SLIAR235;
  union un_icu_sliar236 SLIAR236;
  union un_icu_sliar237 SLIAR238;
  union un_icu_sliar239 SLIAR239;
  union un_icu_sliar240 SLIAR240;
  union un_icu_sliar241 SLIAR241;
  union un_icu_sliar242 SLIAR242;
  union un_icu_sliar243 SLIAR243;
  union un_icu_sliar244 SLIAR244;
  union un_icu_sliar245 SLIAR245;
  union un_icu_sliar246 SLIAR246;
  union un_icu_sliar247 SLIAR247;
  union un_icu_sliar248 SLIAR248;
  union un_icu_sliar249 SLIAR249;
  union un_icu_sliar250 SLIAR250;
  union un_icu_sliar251 SLIAR251;
  union un_icu_sliar252 SLIAR252;
  union un_icu_sliar253 SLIAR253;
  union un_icu_sliar254 SLIAR254;
  union un_icu_sliar255 SLIAR255;
  union un_icu_sliprcr  SLIPRCR;
  union un_icu_slexdr   SLEXDR;
} st_icu_t;

typedef struct st_mpc
{
  union un_mpc_pfcse PFCSE;
  char wk0[1];
  union un_mpc_pfcss0 PFCSS0;
  union un_mpc_pfcss1 PFCSS1;
  union un_mpc_pfa0e0 PFA0E0;
  union un_mpc_pfa0e1 PFA0E1;
  union un_mpc_pfbcr0 PFBCR0;
  union un_mpc_pfbcr1 PFBCR1;
  union un_mpc_pfbcr2 PFBCR2;
  union un_mpc_pfbcr3 PFBCR3;
  char  wk1[4];
  union un_mpc_pfenet PFENET;
  char  wk2[16];
  union un_pwpr PWPR;
  char  wk3[32];
  union un_p00pfs P00PFS;
  union un_p01pfs P01PFS;
  union un_mpc_p02pfs P02PFS;
  union un_mpc_p03pfs P03PFS;
  char  wk4[1];
  union un_mpc_p05pfs P05PFS;
  char  wk5[1];
  union un_mpc_p07pfs P07PFS;
  union un_mpc_p10pfs P10PFS;
  union un_mpc_p11pfs P11PFS;
  union un_mpc_p12pfs P12PFS;
  union un_mpc_p13pfs P13PFS;
  union un_mpc_p14pfs P14PFS;
  union un_mpc_p15pfs P15PFS;
  union un_mpc_p16pfs P16PFS;
  union un_mpc_p17pfs P17PFS;
  union un_mpc_p20pfs P20PFS;
  union un_mpc_p21pfs P21PFS;
  union un_mpc_p22pfs P22PFS;
  union un_mpc_p23pfs P23PFS;
  union un_mpc_p24pfs P24PFS;
  union un_mpc_p25pfs P25PFS;
  union un_mpc_p26pfs P26PFS;
  union un_mpc_p27pfs P27PFS;
  union un_mpc_p30pfs P30PFS;
  union un_mpc_p31pfs P31PFS;
  union un_mpc_p32pfs P32PFS;
  union un_mpc_p33pfs P33PFS;
  union un_mpc_p34pfs P34PFS;
  char  wk6[3];
  union un_mpc_p40pfs P40PFS;
  union un_mpc_p41pfs P41PFS;
  union un_mpc_p42pfs P42PFS;
  union un_mpc_p43pfs P43PFS;
  union un_mpc_p44pfs P44PFS;
  union un_mpc_p45pfs P45PFS;
  union un_mpc_p46pfs P46PFS;
  union un_mpc_p47pfs P47PFS;
  union un_mpc_p50pfs P50PFS;
  union un_mpc_p51pfs P51PFS;
  union un_mpc_p52pfs P52PFS;
  char  wk7[1];
  union un_mpc_p54pfs P54PFS;
  union un_mpc_p55pfs P55PFS;
  union un_mpc_p56pfs P56PFS;
  union un_mpc_p57pfs P57PFS;
  char  wk8[6];
  union un_mpc_p66pfs P66PFS;
  union un_mpc_p67pfs P67PFS;
  char  wk9[1];
  union un_mpc_p71pfs P71PFS;
  union un_mpc_p72pfs P72PFS;
  union un_mpc_p73pfs P73PFS;
  union un_mpc_p74pfs P74PFS;
  union un_mpc_p75pfs P75PFS;
  union un_mpc_p76pfs P76PFS;
  union un_mpc_p77pfs P77PFS;
  union un_mpc_p80pfs P80PFS;
  union un_mpc_p81pfs P81PFS;
  union un_mpc_p82pfs P82PFS;
  union un_mpc_p83pfs P83PFS;
  union un_mpc_p84pfs P84PFS;
  union un_mpc_p85pfs P85PFS;
  union un_mpc_p86pfs P86PFS;
  union un_mpc_p87pfs P87PFS;
  union un_mpc_p90pfs P90PFS;
  union un_mpc_p91pfs P91PFS;
  union un_mpc_p92pfs P92PFS;
  union un_mpc_p93pfs P93PFS;
  char  wk10[4];
  union un_mpc_pa0pfs PA0PFS;
  union un_mpc_pa1pfs PA1PFS;
  union un_mpc_pa2pfs PA2PFS;
  union un_mpc_pa3pfs PA3PFS;
  union un_mpc_pa4pfs PA4PFS;
  union un_mpc_pa5pfs PA5PFS;
  union un_mpc_pa6pfs PA6PFS;
  union un_mpc_pa7pfs PA7PFS;
  union un_mpc_pb0pfs PB0PFS;
  union un_mpc_pb1pfs PB1PFS;
  union un_mpc_pb2pfs PB2PFS;
  union un_mpc_pb3pfs PB3PFS;
  union un_mpc_pb4pfs PB4PFS;
  union un_mpc_pb5pfs PB5PFS;
  union un_mpc_pb6pfs PB6PFS;
  union un_mpc_pb7pfs PB7PFS;
  union un_mpc_pc0pfs PC0PFS;
  union un_mpc_pc1pfs PC1PFS;
  union un_mpc_pc2pfs PC2PFS;
  union un_mpc_pc3pfs PC3PFS;
  union un_mpc_pc4pfs PC4PFS;
  union un_mpc_pc5pfs PC5PFS;
  union un_mpc_pc6pfs PC6PFS;
  union un_mpc_pc7pfs PC7PFS;
  union un_mpc_pd0pfs PD0PFS;
  union un_mpc_pd1pfs PD1PFS;
  union un_mpc_pd2pfs PD2PFS;
  union un_mpc_pd3pfs PD3PFS;
  union un_mpc_pd4pfs PD4PFS;
  union un_mpc_pd5pfs PD5PFS;
  union un_mpc_pd6pfs PD6PFS;
  union un_mpc_pd7pfs PD7PFS;
  union un_mpc_pe0pfs PE0PFS;
  union un_mpc_pe1pfs PE1PFS;
  union un_mpc_pe2pfs PE2PFS;
  union un_mpc_pe3pfs PE3PFS;
  union un_mpc_pe4pfs PE4PFS;
  union un_mpc_pe5pfs PE5PFS;
  union un_mpc_pe6pfs PE6PFS;
  union un_mpc_pe7pfs PE7PFS;
  union un_mpc_pf0pfs PF0PFS;
  union un_mpc_pf1pfs PF1PFS;
  union un_mpc_pf2pfs PF2PFS;
  char  wk11[2];
  union un_mpc_pf5pfs PF5PFS;
  char  wk12[18];
  union un_mpc_pj0pfs PJ0PFS;
  union un_mpc_pj1pfs PJ1PFS;
  union un_mpc_pj2pfs PJ2PFS;
  union un_mpc_pj3pfs PJ3PFS;
  char  wk13[1];
  union un_mpc_pj5pfs PJ5PFS;
} st_mpc_t;

typedef struct st_port0
{
  union un_pdr PDR;
  char  wk0[31];
  union un_podr PODR;
  char  wk1[31];
  union un_pidr PIDR;
  char  wk2[31];
  union un_pmr PMR;
  char  wk3[31];
  union un_odr0 ODR0;
  union un_odr1 ODR1;
  char  wk4[62];
  union un_pcr PCR;
  char  wk5[31];
  union un_dscr DSCR;
  char  wk6[71];
  union un_dscr2 DSCR2;
} st_port0_t;

typedef struct st_port1
{
  union un_port1_pdr PDR;
  char  wk0[31];
  union un_port1_podr PODR;
  char  wk1[31];
  union un_port1_pidr PIDR;
  char  wk2[31];
  union un_port1_pmr PMR;
  char  wk3[32];
  union un_port1_odr0 ODR0;
  union un_port1_odr1 ODR1;
  char  wk4[61];
  union un_port1_pcr PCR;
  char  wk5[31];
  union un_port1_dscr DSCR;
  char  wk6[71];
  union un_port1_dscr2 DSCR2;
} st_port1_t;

typedef struct st_port2
{
  union un_port2_pdr PDR;
  char  wk0[31];
  union un_port2_podr PODR;
  char  wk1[31];
  union un_port2_pidr PIDR;
  char  wk2[31];
  union un_port2_pmr PMR;
  char  wk3[33];
  union un_port2_odr0 ODR0;
  union un_port2_odr1 ODR1;
  char  wk4[60];
  union un_port2_pcr PCR;
  char  wk5[31];
  union un_port2_dscr DSCR;
  char  wk6[71];
  union un_port2_dscr2 DSCR2;
} st_port2_t;

typedef struct st_port3
{
  union un_port3_pdr PDR;
  char  wk0[31];
  union un_port3_podr PODR;
  char  wk1[31];
  union un_port3_pidr PIDR;
  char  wk2[31];
  union un_port3_pmr PMR;
  char  wk3[34];
  union un_port3_odr0 ODR0;
  union un_port3_odr1 ODR1;
  char  wk4[59];
  union un_port3_pcr PCR;
  char  wk5[103];
  union un_port3_dscr2 DSCR2;
} st_port3_t;

typedef struct st_port5
{
  union un_port5_pdr PDR;
  char  wk0[31];
  union un_port5_podr PODR;
  char  wk1[31];
  union un_port5_pidr PIDR;
  char  wk2[31];
  union un_port5_pmr PMR;
  char  wk3[36];
  union un_port5_odr0 ODR0;
  union un_port5_odr1 ODR1;
  char  wk4[57];
  union un_port5_pcr PCR;
  char  wk5[31];
  union un_port5_dscr DSCR;
  char  wk6[71];
  union un_port5_dscr2 DSCR2;
} st_port5_t;

typedef struct st_port7
{
  union un_port7_pdr PDR;
  char  wk0[31];
  union un_port7_podr PODR;
  char  wk1[31];
  union un_port7_pidr PIDR;
  char  wk2[31];
  union un_port7_pmr PMR;
  char  wk3[38];
  union un_port7_odr0 ODR0;
  union un_port7_odr1 ODR1;
  char  wk4[55];
  union un_port7_pcr PCR;
  char  wk5[31];
  union un_port7_dscr DSCR;
  char  wk6[71];
  union un_port7_dscr2 DSCR2;
} st_port7_t;

typedef struct st_port8
{
  union un_port8_pdr PDR;
  char  wk0[31];
  union un_port8_podr PODR;
  char  wk1[31];
  union un_port8_pidr PIDR;
  char  wk2[31];
  union un_port8_pmr PMR;
  char  wk3[39];
  union un_port8_odr0 ODR0;
  union un_port8_odr1 ODR1;
  char  wk4[54];
  union un_port8_pcr PCR;
  char  wk5[31];
  union un_port8_dscr DSCR;
  char  wk6[71];
  union un_port8_dscr2 DSCR2;
} st_port8_t;

typedef struct st_port9
{
  union un_port9_pdr PDR;
  char  wk0[31];
  union un_port9_podr PODR;
  char  wk1[31];
  union un_port9_pidr PIDR;
  char  wk2[31];
  union un_port9_pmr PMR;
  char  wk3[40];
  union un_port9_odr0 ODR0;
  union un_port9_odr1 ODR1;
  char  wk4[53];
  union un_port9_pcr PCR;
  char  wk5[31];
  union un_port9_dscr DSCR;
  char  wk6[71];
  union un_port9_dscr2 DSCR2;
} st_port9_t;

typedef struct st_porta
{
  union un_porta_pdr PDR;
  char  wk0[31];
  union un_porta_podr PODR;
  char  wk1[31];
  union un_porta_pidr PIDR;
  char  wk2[31];
  union un_porta_pmr PMR;
  char  wk3[41];
  union un_porta_odr0 ODR0;
  union un_porta_odr1 ODR1;
  char  wk4[52];
  union un_porta_pcr PCR;
  char  wk5[31];
  union un_porta_dscr DSCR;
  char  wk6[71];
  union un_porta_dscr2 DSCR2;
} st_porta_t;

typedef struct st_portb
{
  union un_portb_pdr PDR;
  char  wk0[31];
  union un_portb_podr PODR;
  char  wk1[31];
  union un_portb_pidr PIDR;
  char  wk2[31];
  union un_portb_pmr PMR;
  char  wk3[42];
  union un_portb_odr0 ODR0;
  union un_portb_odr1 ODR1;
  char  wk4[51];
  union un_portb_pcr PCR;
  char  wk5[31];
  union un_portb_dscr DSCR;
  char  wk6[71];
  union un_portb_dscr2 DSCR2;
} st_portb_t;

typedef struct st_portc
{
  union un_portc_pdr PDR;
  char  wk0[31];
  union un_portc_podr PODR;
  char  wk1[31];
  union un_portc_pidr PIDR;
  char  wk2[31];
  union un_portc_pmr PMR;
  char  wk3[43];
  union un_portc_odr0 ODR0;
  union un_portc_odr1 ODR1;
  char  wk4[50];
  union un_portc_pcr PCR;
  char  wk5[31];
  union un_portc_dscr DSCR;
  char  wk6[71];
  union un_portc_dscr2 DSCR2;
} st_portc_t;

typedef struct st_porte
{
  union un_porte_pdr PDR;
  char wk0[31];
  union un_porte_podr PODR;
  char wk1[31];
  union un_porte_pidr PIDR;
  char wk2[31];
  union un_porte_pmr PMR;
  char wk3[45];
  union un_porte_odr0 ODR0;
  union un_porte_odr1 ODR1;
  char wk4[48];
  union un_porte_pcr PCR;
  char wk5[31];
  union un_porte_dscr DSCR;
  char wk6[71];
  union un_porte_dscr2 DSCR2;
} st_porte_t;

typedef struct st_portf
{
  union un_portf_pdr PDR;
  char  wk0[31];
  union un_portf_podr PODR;
  char  wk1[31];
  union un_portf_pidr PIDR;
  char  wk2[31];
  union un_portf_pmr PMR;
  char  wk3[46];
  union un_portf_odr0 ODR0;
  union un_portf_odr1 ODR1;
  char  wk4[47];
  union un_portf_pcr PCR;
} st_portf_t;

typedef struct st_portg
{
  union un_portg_pdr PDR;
  char  wk0[31];
  union un_portg_podr PODR;
  char  wk1[31];
  union un_portg_pidr PIDR;
  char  wk2[31];
  union un_portg_pmr PMR;
  char  wk3[47];
  union un_portg_odr0 ODR0;
  union un_portg_odr1 ODR1;
  char  wk4[46];
  union un_portg_pcr PCR;
  char  wk5[31];
  union un_portg_dscr DSCR;
  char  wk6[71];
  union un_portg_dscr2 DSCR2;
} st_portg_t;

typedef struct st_portj
{
  union un_portj_pdr PDR;
  char  wk0[31];
  union un_portj_podr PODR;
  char  wk1[31];
  union un_portj_pidr PIDR;
  char  wk2[31];
  union un_portj_pmr PMR;
  char  wk3[49];
  union un_portj_odr0 ODR0;
  union un_portj_odr1 ODR1;
  char  wk4[44];
  union un_portj_pcr PCR;
  char  wk5[31];
  union un_portj_dscr DSCR;
  char  wk6[71];
  union un_portj_dscr2 DSCR2;
} st_portj_t;

typedef struct st_rtc
{
  union un_rtc_r64cnt R64CNT;
  char  wk0[1];
  union
  {
    union un_rtc_rseccnt RSECCNT;
    union un_rtc_bcnt0 BCNT0;
  };
  char  wk1[1];
  union
  {
    union un_rtc_rmincnt RMINCNT;
    union un_rtc_bcnt1   BCNT1;
  };
  char  wk2[1];
  union
  {
     union un_rtc_rhrcnt RHRCNT;
     union un_rtc_bcnt2  BCNT2;
  };
  char  wk3[1];
  union
  {
     union un_rtc_rwkcnt RWKCNT;
     union un_rtc_bcnt3 BCNT3;
  };
  char  wk4[1];
  union un_rtc_rdaycnt RDAYCNT;
  char  wk5[1];
  union un_rtc_rmoncnt RMONCNT;
  char  wk6[1];
  union un_rtc_ryrcnt RYRCNT;
  union
  {
     union un_rtc_rsecar RSECAR;
     union un_rtc_bcnt0ar BCNT0AR;
  };
  char  wk7[1];
  union
  {
     union un_rtc_rminar RMINAR;
     union un_rtc_bcnt1ar BCNT1AR;
  };
  char wk8[1];
  union
  {
     union un_rtc_rhrar RHRAR;
     union un_rtc_bcnt2ar BCNT2AR;
  };
  char wk9[1];
  union
  {
     union un_rtc_rwkar RWKAR;
     union un_rtc_bcnt3ar BCNT3AR;
  };
  char  wk10[1];
  union
  {
     union un_rtc_rdayar RDAYAR;
     union un_rtc_bcnt0aer BCNT0AER;
  };
  char  wk11[1];
  union
  {
     union un_rtc_rmonar RMONAR;
     union un_rtc_bcnt1aer BCNT1AER;
  };
  char wk12[1];
  union
  {
     union un_rtc_ryrar RYRAR;
     union un_rtc_bcnt2aer BCNT2AER;
  };
  union
  {
     union un_rtc_ryraren RYRAREN;
     union un_rtc_bcnt3aer BCNT3AER;
  };
  char wk13[3];
  union un_rtc_rcr1 RCR1;
  char  wk14[1];
  union un_rtc_rcr2 RCR2;
  char  wk15[1];
  union un_rcr3 RCR3;
  char  wk16[1];
  union un_rtc_rcr4 RCR4;
  char  wk17[1];
  union un_rtc_rfrh RFRH;
  union un_rtc_rfrl RFRL;
  union un_rtc_radj RADJ;
  char  wk18[17];
  union un_rtc_rfrh RTCCR0;
  char  wk19[1];
  union un_rtc_rtccr1 RTCCR1;
  char  wk20[1];
  union un_rtc_rtccr2 RTCCR2;
  char  wk21[13];
  union
  {
    union un_rtc_rseccp0 RSECCP0;
    union un_rtc_bcnt0cp0 BCNT0PC0;
  };
  char wk22[1];
  union
  {
    union un_rtc_rmincp0 RMINCP0;
    union un_rtc_bcnt1cp0 BCNT1CP0;
  };
  char  wk23[1];
  union
  {
    union un_rtc_rhrcp0 RHRCP0;
    union un_rtc_bcnt2cp0 BCNT2CP0;
  };
  char  wk24[3];
  union
  {
    union un_rtc_rdaycp0 RDAYCP0;
    union un_rtc_bcnt3cp0 BCNT3CP0;
  };
  char wk25[1];
  union un_rtc_rmoncp0 RMONCP0;
  char  wk26[5];
  union
  {
    union un_rtc_rseccp1 RSECCP1;
    union un_rtc_bcnt0cp1 BCNT0CP1;
  };
  char wk27[1];
  union
  {
    union un_rtc_rmincp1 RMINCP1;
    union un_rtc_bcnt1cp1 BCNT1CP1;
  };
  char wk28[1];
  union
  {
    union un_rtc_rhrcp1 RHRCP1;
    union un_rtc_bcnt2cp1 BCNT2CP1;
  };
  char  wk29[3];
  union
  {
    union un_rtc_rdaycp1 RDAYCP1;
    union un_rtc_bcnt3cp1 BCNT3CP1;
  };
  char  wk30[1];
  union un_rtc_rmoncp1 RMONCP1;
  char  wk31[5];
  union
  {
    union un_rtc_rseccp2 RSECCP2;
    union un_rtc_bcnt0cp2 BCNT0CP2;
  };
  char  wk32[1];
  union
  {
     union un_rtc_rmincp2 RMINCP2;
     union un_rtc_bcnt1cp2 BCNT1CP2;
  };
  char  wk33[1];
  union
  {
     union un_rtc_rhrcp2 RHRCP2;
     union un_rtc_bcnt2cp2 BCNT2CP2;
  };
  char  wk34[3];
  union
  {
     union un_rtc_rdaycp2 RDAYCP2;
     union un_rtc_bcnt3cp2 BCNT3CP2;
  };
  char wk35[1];
  union un_rtc_rmoncp2 RMONCP2;
} st_rtc_t;

typedef struct st_sci0
{
  union un_sci0_smr SMR;
  unsigned char  BRR;
  union un_sci0_scr SCR;
  unsigned char  TDR;
  union un_sci0_ssr SSR;
  unsigned char  RDR;
  union un_sci0_scmr SCMR;
  union un_sci0_semr SEMR;
  union un_sci0_snfr SNFR;
  union un_sci0_simr1 SIMR1;
  union un_sci0_simr2 SIMR2;
  union un_sci0_simr3 SIMR3;
  union un_sci0_sisr SISR;
  union un_sci0_spmr SPMR;
  union un_sci0_tdrhl TDRHL;
  union un_sci0_rdrhl RDRHL;
  unsigned char  MDDR;
} st_sci0_t;

typedef struct st_system
{
  union un_mdmonr MDMONR;
  char  wk0[4];
  union un_syscr0 SYSCR0;
  union un_syscr1 SYSCR1;
  char  wk1[2];
  union un_sbycr SBYCR;
  char  wk2[2];
  union un_mstpcra MSTPCRA;
  union un_mstpcrb MSTPCRB;
  union un_mstpcrc MSTPCRC;
  union un_mstpcrd MSTPCRD;
  union un_sckcr SCKCR;
  union un_sckcr2 SCKCR2;
  union un_sckcr3 SCKCR3;
  union un_pllcr PLLCR;
  union un_pllcr2 PLLCR2;
  char  wk3[5];
  union un_bckcr BCKCR;
  char  wk4[1];
  union un_mosccr MOSCCR;
  union un_sosccr SOSCCR;
  union un_lococr LOCOCR;
  union un_ilococr ILOCOCR;
  union un_hococr HOCOCR;
  union un_hococr2 HOCOCR2;
  char  wk5[4];
  union un_oscovfsr OSCOVFSR;
  char  wk6[3];
  union un_ostdcr OSTDCR;
  union un_ostdsr OSTDSR;
  char  wk7[94];
  union un_opccr OPCCR;
  union un_rstckcr RSTCKCR;
  union un_moscwtcr MOSCWTCR;
  union un_soscwtcr SOSCWTCR;
  char  wk8[28];
  union un_rstsr2 RSTSR2;
  char  wk9[1];
  unsigned short SWRR;
  char  wk10[28];
  union un_lvd1cr1 LVD1CR1;
  union un_lvd1sr LVD1SR;
  union un_lvd2cr1 LVD2CR1;
  union un_lvd2sr LVD2SR;
  char  wk11[794];
  union un_prcr PRCR;
  char  wk12[3100];
  union un_romwt ROMWT;
  char  wk13[45667];
  union un_dpsbycr DPSBYCR;
  char  wk14[1];
  union un_dpsier0 DPSIER0;
  union un_dpsier1 DPSIER1;
  union un_dpsier2 DPSIER2;
  union un_dpsier3 DPSIER3;
  union un_dpsifr0 DPSIFR0;
  union un_dpsifr1 DPSIFR1;
  union un_dpsifr2 DPSIFR2;
  union un_dpsifr3 DPSIFR3;
  union un_dpsiegr0 DPSIEGR0;
  union un_dpsiegr1 DPSIEGR1;
  union un_dpsiegr2 DPSIEGR2;
  union un_dpsiegr3 DPSIEGR3;
  char  wk15[2];
  union un_rstsr0 RSTSR0;
  union un_rstsr1 RSTSR1;
  char  wk16[1];
  union un_mofcr MOFCR;
  union un_hocopcr HOCOPCR;
  char  wk17[2];
  union un_lvcmpcr LVCMPCR;
  union un_lvdlvlr LVDLVLR;
  char  wk18[1];
  union un_lvd1cr0 LVD1CR0;
  union un_lvd2cr0 LVD2CR0;
  char  wk19[4];
  unsigned char  DPSBKR[32];
} st_system_t;

typedef struct st_sci10
{
  union un_sci10_smr SMR;
  unsigned char  BRR;
  union un_sci10_scr SCR;
  unsigned char  TDR;
  union
    {
      union un_sci10_ssr SSR;
      union un_sci10_ssrfifo SSRFIFO;
    };
  unsigned char  RDR;
  union un_sci10_scmr SCMR;
  union un_sci10_semr SEMR;
  union un_sci10_snfr SNFR;
  union un_sci10_simr1 SIMR1;
  union un_sci10_simr2 SIMR2;
  union un_sci10_simr3 SIMR3;
  union un_sci10_sisr SISR;
  union un_sci10_spmr SPMR;
  union
    {
      union un_sci10_tdrhl TDRHL;
      union un_sci10_ftdr_bit_byte FTDR;
    };
  union
    {
      union un_sci10_rdrhl_byte RDRHL;
      union un_sci10_frdr_bit_byte FRDR;
    };
  unsigned char  MDDR;
  union un_sci10_dccr DCCR;
  union un_sci10_fcr FCR;
  union un_sci10_fdr FDR;
  union un_sci10_lsr LSR;
  union un_sci10_cdr CDR;
  union st_sci10_sptr SPTR;
} st_sci10_t;

typedef struct st_sci12
{
  union un_sci12_smr SMR;
  unsigned char  BRR;
  union un_sci12_scr SCR;
  unsigned char  TDR;
  union un_sci12_ssr SSR;
  unsigned char  RDR;
  union un_sci12_scmr SCMR;
  union un_sci12_semr SEMR;
  union un_sci12_snfr SNFR;
  union un_sci12_simr1 SIMR1;
  union un_sci12_simr2 SIMR2;
  union un_sci12_simr3 SIMR3;
  union un_sci12_sisr SISR;
  union un_sci12_spmr SPMR;
  union un_sci12_tdrhl TDRHL;
  union un_sci12_rdrhl RDRHL;
  unsigned char  MDDR;
  char   wk0[13];
  union un_sci12_esmer ESMER;
  union un_sci12_ctr0 CTR0;
  union un_sci12_ctr1 CTR1;
  union un_sci12_ctr2 CTR2;
  union un_sci12_ctr3 CTR3;
  union un_sci12_pcr PCR;
  union un_sci12_icr ICR;
  union un_sci12_str STR;
  union un_sci12_stcr STCR;
  unsigned char  CF0DR;
  union un_sci12_cf0cr CF0CR;
  unsigned char  CF0RR;
  unsigned char  PCF1DR;
  unsigned char  SCF1DR;
  union un_sci12_cf1cr CF1CR;
  unsigned char  CF1RR;
  union un_sci12_tcr TCR;
  union un_sci12_tmr TMR;
  unsigned char  TPRE;
  unsigned char  TCNT;
} st_sci12_t;

typedef struct st_cmtw
{
  union un_cmtw_cmwstr CMWSTR;
  char   wk0[2];
  union un_cmtw_cmwcr CMWCR;
  char   wk1[2];
  union un_cmtw_cmwior CMWIOR;
  char   wk2[6];
  unsigned long  CMWCNT;
  unsigned long  CMWCOR;
  unsigned long  CMWICR0;
  unsigned long  CMWICR1;
  unsigned long  CMWOCR0;
  unsigned long  CMWOCR1;
} st_cmtw_t;

#pragma pack()

#endif  /* __ASSEMBLER__ */
#endif


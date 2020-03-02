/****************************************************************************
 * arch/renesas/include/rx65n/iodefine.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __RX65NIODEFINE_HEADER__
#define __RX65NIODEFINE_HEADER__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#define BSC     (*(volatile struct st_bsc      *)0x81300)
#define CAC     (*(volatile struct st_cac      *)0x8b000)
#define CAN0    (*(volatile struct st_can      *)0x90200)
#define CAN1    (*(volatile struct st_can      *)0x91200)
#define CMT     (*(volatile struct st_cmt      *)0x88000)
#define CMT0    (*(volatile struct st_cmt0     *)0x88002)
#define CMT1    (*(volatile struct st_cmt0     *)0x88008)
#define CMT2    (*(volatile struct st_cmt0     *)0x88012)
#define CMT3    (*(volatile struct st_cmt0     *)0x88018)
#define CMTW0   (*(volatile struct st_cmtw     *)0x94200)
#define CMTW1   (*(volatile struct st_cmtw     *)0x94280)
#define CRC     (*(volatile struct st_crc      *)0x88280)
#define DA      (*(volatile struct st_da       *)0x88040)
#define DMAC    (*(volatile struct st_dmac     *)0x82200)
#define DMAC0   (*(volatile struct st_dmac0    *)0x82000)
#define DMAC1   (*(volatile struct st_dmac1    *)0x82040)
#define DMAC2   (*(volatile struct st_dmac1    *)0x82080)
#define DMAC3   (*(volatile struct st_dmac1    *)0x820c0)
#define DMAC4   (*(volatile struct st_dmac1    *)0x82100)
#define DMAC5   (*(volatile struct st_dmac1    *)0x82140)
#define DMAC6   (*(volatile struct st_dmac1    *)0x82180)
#define DMAC7   (*(volatile struct st_dmac1    *)0x821c0)
#define DOC     (*(volatile struct st_doc      *)0x8b080)
#define DRW2D   (*(volatile struct st_drw2d    *)0xe3000)
#define DTC     (*(volatile struct st_dtc      *)0x82400)
#define EDMAC0  (*(volatile struct st_edmac    *)0xc0000)
#define ELC     (*(volatile struct st_elc      *)0x8b100)
#define ETHERC0 (*(volatile struct st_etherc   *)0xc0100)
#define EXDMAC  (*(volatile struct st_exdmac   *)0x82a00)
#define EXDMAC0 (*(volatile struct st_exdmac0  *)0x82800)
#define EXDMAC1 (*(volatile struct st_exdmac1  *)0x82840)
#define FLASH   (*(volatile struct st_flash    *)0x81000)
#define GLCDC   (*(volatile struct st_glcdc    *)0xe0000)
#define ICU     (*(volatile struct st_icu      *)0x87000)
#define IWDT    (*(volatile struct st_iwdt     *)0x88030)
#define MMCIF   (*(volatile struct st_mmcif    *)0x88500)
#define MPC     (*(volatile struct st_mpc      *)0x8c100)
#define MPU     (*(volatile struct st_mpu      *)0x86400)
#define MTU     (*(volatile struct st_mtu      *)0xc120a)
#define MTU0    (*(volatile struct st_mtu0     *)0xc1290)
#define MTU1    (*(volatile struct st_mtu1     *)0xc1290)
#define MTU2    (*(volatile struct st_mtu2     *)0xc1292)
#define MTU3    (*(volatile struct st_mtu3     *)0xc1200)
#define MTU4    (*(volatile struct st_mtu4     *)0xc1200)
#define MTU5    (*(volatile struct st_mtu5     *)0xc1a94)
#define MTU6    (*(volatile struct st_mtu6     *)0xc1a00)
#define MTU7    (*(volatile struct st_mtu7     *)0xc1a00)
#define MTU8    (*(volatile struct st_mtu8     *)0xc1298)
#define PDC     (*(volatile struct st_pdc      *)0xa0500)
#define POE3    (*(volatile struct st_poe      *)0x8c4c0)
#define PORT0   (*(volatile struct st_port0    *)0x8c000)
#define PORT1   (*(volatile struct st_port1    *)0x8c001)
#define PORT2   (*(volatile struct st_port2    *)0x8c002)
#define PORT3   (*(volatile struct st_port3    *)0x8c003)
#define PORT4   (*(volatile struct st_port4    *)0x8c004)
#define PORT5   (*(volatile struct st_port5    *)0x8c005)
#define PORT6   (*(volatile struct st_port6    *)0x8c006)
#define PORT7   (*(volatile struct st_port7    *)0x8c007)
#define PORT8   (*(volatile struct st_port8    *)0x8c008)
#define PORT9   (*(volatile struct st_port9    *)0x8c009)
#define PORTA   (*(volatile struct st_porta    *)0x8c00a)
#define PORTB   (*(volatile struct st_portb    *)0x8c00b)
#define PORTC   (*(volatile struct st_portc    *)0x8c00c)
#define PORTD   (*(volatile struct st_portd    *)0x8c00d)
#define PORTE   (*(volatile struct st_porte    *)0x8c00e)
#define PORTF   (*(volatile struct st_portf    *)0x8c00f)
#define PORTG   (*(volatile struct st_portg    *)0x8c010)
#define PORTJ   (*(volatile struct st_portj    *)0x8c012)
#define PPG0    (*(volatile struct st_ppg0     *)0x881e6)
#define PPG1    (*(volatile struct st_ppg1     *)0x881f0)
#define QSPI    (*(volatile struct st_qspi     *)0x89e00)
#define RAM     (*(volatile struct st_ram      *)0x81200)
#define RIIC0   (*(volatile struct st_riic     *)0x88300)
#define RIIC1   (*(volatile struct st_riic     *)0x88320)
#define RIIC2   (*(volatile struct st_riic     *)0x88340)
#define RSPI0   (*(volatile struct st_rspi     *)0xd0100)
#define RSPI1   (*(volatile struct st_rspi     *)0xd0140)
#define RSPI2   (*(volatile struct st_rspi     *)0xd0300)
#define RTC     (*(volatile struct st_rtc      *)0x8c400)
#define S12AD   (*(volatile struct st_s12ad    *)0x89000)
#define S12AD1  (*(volatile struct st_s12ad1   *)0x89100)
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
#define SDHI    (*(volatile struct st_sdhi     *)0x8ac00)
#define SDSI    (*(volatile struct st_sdsi     *)0x95000)
#define SMCI0   (*(volatile struct st_smci0    *)0x8a000)
#define SMCI1   (*(volatile struct st_smci0    *)0x8a020)
#define SMCI2   (*(volatile struct st_smci0    *)0x8a040)
#define SMCI3   (*(volatile struct st_smci0    *)0x8a060)
#define SMCI4   (*(volatile struct st_smci0    *)0x8a080)
#define SMCI5   (*(volatile struct st_smci0    *)0x8a0a0)
#define SMCI6   (*(volatile struct st_smci0    *)0x8a0c0)
#define SMCI7   (*(volatile struct st_smci0    *)0x8a0e0)
#define SMCI8   (*(volatile struct st_smci0    *)0x8a100)
#define SMCI9   (*(volatile struct st_smci0    *)0x8a120)
#define SMCI10  (*(volatile struct st_smci10   *)0xd0040)
#define SMCI11  (*(volatile struct st_smci10   *)0xd0060)
#define SMCI12  (*(volatile struct st_smci0    *)0x8b300)
#define SYSTEM  (*(volatile struct st_system   *)0x80000)
#define TEMPS   (*(volatile struct st_temps    *)0x8c500)
#define TMR0    (*(volatile struct st_tmr0     *)0x88200)
#define TMR1    (*(volatile struct st_tmr1     *)0x88201)
#define TMR2    (*(volatile struct st_tmr0     *)0x88210)
#define TMR3    (*(volatile struct st_tmr1     *)0x88211)
#define TMR01   (*(volatile struct st_tmr01    *)0x88204)
#define TMR23   (*(volatile struct st_tmr01    *)0x88214)
#define TPU0    (*(volatile struct st_tpu0     *)0x88108)
#define TPU1    (*(volatile struct st_tpu1     *)0x88108)
#define TPU2    (*(volatile struct st_tpu2     *)0x8810a)
#define TPU3    (*(volatile struct st_tpu3     *)0x8810a)
#define TPU4    (*(volatile struct st_tpu4     *)0x8810c)
#define TPU5    (*(volatile struct st_tpu5     *)0x8810c)
#define TPUA    (*(volatile struct st_tpua     *)0x88100)
#define USB     (*(volatile struct st_usb      *)0xa0400)
#define USB0    (*(volatile struct st_usb0     *)0xa0000)
#define WDT     (*(volatile struct st_wdt      *)0x88020)
#define FLASHCONST      (*(volatile struct st_flashconst      *)0xfe7f7d90)
#define TEMPSCONST      (*(volatile struct st_tempsconst      *)0xfe7f7d7c)

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

union un_bsc_berclr
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

union un_bsc_beren
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

union un_bsc_bersr1
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

union un_bsc_bersr2
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

union un_bsc_buspri
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

union un_bsc_cs0mod
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

union un_bsc_cs0wcr1
{
  unsigned long LONG;
  struct st_bsc_cs0wcr1_bit BIT;
};

struct st_bsc_cs0wcr2_bit
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

union un_bsc_cs0wcr2
{
  unsigned long LONG;
  struct st_bsc_cs0wcr2_bit BIT;
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

union un_bsc_cs1mod
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

union un_bsc_cs1wcr1
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

union un_bsc_cs1wcr2
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

union un_bsc_cs2mod
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

union un_bsc_cs2wcr1
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

union un_bsc_cs2wcr2
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

union un_bsc_cs3mod
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

union un_bsc_cs3wcr1
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

union un_bsc_cs3wcr2
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

union un_bsc_cs4mod
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

union un_bsc_cs4wcr1
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

union un_bsc_cs4wcr2
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

union un_bsc_cs5mod
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

union un_bsc_cs5wcr1
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

union un_bsc_cs5wcr2
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

union un_bsc_cs6mod
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

union un_bsc_cs6wcr1
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

union un_bsc_cs6wcr2
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

union un_bsc_cs7mod
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

union un_bsc_cs7wcr1
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

union un_bsc_cs7wcr2
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

union un_bsc_cs0cr
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

union un_bsc_cs0rec
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

union un_bsc_cs1cr
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

union un_bsc_cs1rec
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

union un_bsc_cs2cr
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

union un_bsc_cs2rec
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

union un_bsc_cs3cr
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

union un_bsc_cs3rec
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

union un_bsc_cs4cr
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

union un_bsc_cs4rec
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

union un_bsc_cs5cr
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

union un_bsc_cs5rec
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

union un_bsc_cs6cr
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

union un_bsc_cs6rec
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

union un_bsc_cs7cr
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

union un_bsc_cs7rec
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

union un_bsc_csrecen
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

union un_bsc_sdccr
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

union un_bsc_sdcmod
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

union un_bsc_sdamod
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

union un_bsc_sdself
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

union un_bsc_sdrfcr
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

union un_bsc_sdrfen
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

union un_bsc_sdicr
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

union un_bsc_sdir
{
  unsigned short WORD;
  struct st_bsc_sdir_bit BIT;
};

struct st_bsc_sdadr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MXC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char MXC : 2;
#endif
};

union un_bsc_sdadr
{
  unsigned char BYTE;
  struct st_bsc_sdadr_bit BIT;
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

union un_bsc_sdtr
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

union un_bsc_sdmod
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

union un_bsc_sdsr
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

union un_bsc_ebmapcr
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

union un_cac_cacr0
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

union un_cac_cacr1
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

union un_cac_cacr2
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

union un_cac_caicr
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

union un_cac_castr
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

union un_cmt_cmstr0
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

union un_cmt_cmstr1
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

union un_cmt0_cmcr
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

union un_icu_ier32
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

union un_icu_irqcr16
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

union un_icu_swintr
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

union un_icu_swint2r
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

union un_icu_grpbl0
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

union un_icu_genbl0
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

union un_icu_grpbl1
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

union un_icu_genbl1
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

union un_icu_grpal0
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

union un_icu_genal0
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

union un_icu_grpal1
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

union un_icu_genal1
{
  unsigned long LONG;
  struct st_icu_genal1_bit BIT;
};

union un_mpc_pwpr
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

union un_icu_irqflte0
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

union un_icu_irqflte1
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
};

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

union un_mpc_p00pfs
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

union un_mpc_p01pfs
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

union un_system_mdmonr
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

union un_system_syscr0
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

union un_system_syscr1
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

union un_system_sbycr
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

union un_system_mstpcra
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

union un_system_mstpcrb
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

union un_system_mstpcrc
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

union un_system_mstpcrd
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

union un_system_sckcr
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

union un_system_sckcr2
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

union un_system_sckcr3
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

union un_system_pllcr
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

union un_system_pllcr2
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

union un_system_bckcr
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

union un_system_mosccr
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

union un_system_sosccr
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

union un_system_lococr
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

union un_system_ilococr
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

union un_system_hococr
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

union un_system_hococr2
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

union un_system_oscovfsr
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

union un_system_ostdcr
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

union un_system_ostdsr
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

union un_system_rstckcr
{
  unsigned char BYTE;
  struct st_system_rstckcr_bit BIT;
};

union un_system_opccr
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

union un_system_moscwtcr
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

union un_system_soscwtcr
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

union un_system_rstsr2
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

union un_system_lvd1cr1
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

union un_system_lvd1sr
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

union un_system_lvd2cr1
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

union un_system_lvd2sr
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

union un_system_prcr
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

union un_system_romwt
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

union un_system_dpsbycr
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

union un_system_dpsier0
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

union un_system_dpsier1
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

union un_system_dpsier2
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

union un_system_dpsier3
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

union un_system_dpsifr0
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

union un_system_dpsifr1
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

union un_system_dpsifr2
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

union un_system_dpsifr3
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

union un_system_dpsiegr0
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

union un_system_dpsiegr1
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

union un_system_dpsiegr2
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

union un_system_dpsiegr3
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

union un_system_rstsr0
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

union un_system_rstsr1
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

union un_system_mofcr
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

union un_system_hocopcr
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

union un_system_lvcmpcr
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

union un_system_lvdlvlr
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

union un_system_lvd1cr0
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

union un_system_lvd2cr0
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

union un_rtc_rcr3
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

union un_port0_pdr
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

union un_port0_podr
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

union un_port0_pidr
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

union un_port0_pmr
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

union un_port0_odr0
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

union un_port0_odr1
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

union un_port0_pcr
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

union un_port0_dscr
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

union un_port0_dscr2
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

struct st_port4_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_pdr
{
  unsigned char BYTE;
  struct st_port4_pdr_bit BIT;
};

struct st_port4_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_podr
{
  unsigned char BYTE;
  struct st_port4_podr_bit BIT;
};

struct st_port4_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_pidr
{
  unsigned char BYTE;
  struct st_port4_pidr_bit BIT;
};

struct st_port4_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_pmr
{
  unsigned char BYTE;
  struct st_port4_pmr_bit BIT;
};

struct st_port4_odr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_odr0
{
  unsigned char BYTE;
  struct st_port4_odr0_bit BIT;
};

struct st_port4_odr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_odr1
{
  unsigned char BYTE;
  struct st_port4_odr1_bit BIT;
};

struct st_port4_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port4_pcr
{
  unsigned char BYTE;
  struct st_port4_pcr_bit BIT;
};

typedef struct st_port4
{
        union un_port4_pdr PDR;
        char           wk0[31];
        union un_port4_podr PODR;
        char           wk1[31];
        union un_port4_pidr PIDR;
        char           wk2[31];
        union un_port4_pmr PMR;
        char           wk3[35];
        union un_port4_odr0 ORD0;
        union un_port4_odr1 ORD1;
        char           wk4[58];
        union un_port4_pcr PCR;
} st_port4_t;

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

struct st_temps_tscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 4;
  unsigned char TSOE : 1;
  unsigned char  : 2;
  unsigned char TSEN : 1;
#else
  unsigned char TSEN : 1;
  unsigned char  : 2;
  unsigned char TSOE : 1;
  unsigned char  : 4;
#endif
};

union un_temps_tscr
{
  unsigned char BYTE;
  struct st_temps_tscr_bit BIT;
};

struct st_tmr0_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 3;
  unsigned char CCLR : 2;
  unsigned char OVIE : 1;
  unsigned char CMIEA : 1;
  unsigned char CMIEB : 1;
#else
  unsigned char CMIEB : 1;
  unsigned char CMIEA : 1;
  unsigned char OVIE : 1;
  unsigned char CCLR : 2;
  unsigned char  : 3;
#endif
};

union un_tmr0_tcr
{
  unsigned char BYTE;
  struct st_tmr0_tcr_bit BIT;
};

struct st_tmr0_tcsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OSA : 2;
  unsigned char OSB : 2;
  unsigned char ADTE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char ADTE : 1;
  unsigned char OSB : 2;
  unsigned char OSA : 2;
#endif
};

union un_tmr0_tcsr
{
  unsigned char BYTE;
  struct st_tmr0_tcsr_bit BIT;
};

struct st_tmr0_tccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 3;
  unsigned char CSS : 2;
  unsigned char  : 2;
  unsigned char TMRIS : 1;
#else
  unsigned char TMRIS : 1;
  unsigned char  : 2;
  unsigned char CSS : 2;
  unsigned char CKS : 3;
#endif
};

union un_tmr0_tccr
{
  unsigned char BYTE;
  struct st_tmr0_tccr_bit BIT;
};

struct st_tmr0_tcstr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCS : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TCS : 1;
#endif
};

union un_tmr0_tcstr
{
  unsigned char BYTE;
  struct st_tmr0_tcstr_bit BIT;
};

struct st_tmr1_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 3;
  unsigned char CCLR : 2;
  unsigned char OVIE : 1;
  unsigned char CMIEA : 1;
  unsigned char CMIEB : 1;
#else
  unsigned char CMIEB : 1;
  unsigned char CMIEA : 1;
  unsigned char OVIE : 1;
  unsigned char CCLR : 2;
  unsigned char  : 3;
#endif
};

union un_tmr1_tcr
{
  unsigned char BYTE;
  struct st_tmr1_tcr_bit BIT;
};

struct st_tmr1_tcsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OSA : 2;
  unsigned char OSB : 2;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char OSB : 2;
  unsigned char OSA : 2;
#endif
};

union un_tmr1_tcsr
{
  unsigned char BYTE;
  struct st_tmr1_tcsr_bit BIT;
};

struct st_tmr1_tccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 3;
  unsigned char CSS : 2;
  unsigned char  : 2;
  unsigned char TMRIS : 1;
#else
  unsigned char TMRIS : 1;
  unsigned char  : 2;
  unsigned char CSS : 2;
  unsigned char CKS : 3;
#endif
};

union un_tmr1_tccr
{
  unsigned char BYTE;
  struct st_tmr1_tccr_bit BIT;
};

struct st_tmr1_tcstr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TCS : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TCS : 1;
#endif
};

union un_tmr1_tcstr
{
  unsigned char BYTE;
  struct st_tmr1_tcstr_bit BIT;
};

struct st_tpu0_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
                        unsigned char NFAEN : 1;
                        unsigned char NFBEN : 1;
                        unsigned char NFCEN : 1;
                        unsigned char NFDEN : 1;
                        unsigned char NFCS : 2;
                        unsigned char  : 2;
#else
                        unsigned char  : 2;
                        unsigned char NFCS : 2;
                        unsigned char NFDEN : 1;
                        unsigned char NFCEN : 1;
                        unsigned char NFBEN : 1;
                        unsigned char NFAEN : 1;
#endif
};

union un_tpu0_nfcr
{
  unsigned char BYTE;
  struct st_tpu0_nfcr_bit BIT;
};

struct st_tpu0_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu0_tcr
{
  unsigned char BYTE;
  struct st_tpu0_tcr_bit BIT;
};

struct st_tpu0_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char ICSELB : 1;
  unsigned char ICSELD : 1;
#else
  unsigned char ICSELD : 1;
  unsigned char ICSELB : 1;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_tpu0_tmdr
{
  unsigned char BYTE;
  struct st_tpu0_tmdr_bit BIT;
};

struct st_tpu0_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu0_tiorh
{
  unsigned char BYTE;
  struct st_tpu0_tiorh_bit BIT;
};

struct st_tpu0_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_tpu0_tiorl
{
  unsigned char BYTE;
  struct st_tpu0_tiorl_bit BIT;
};

struct st_tpu0_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu0_tier
{
  unsigned char BYTE;
  struct st_tpu0_tier_bit BIT;
};

struct un_tpu0_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char TGFC : 1;
  unsigned char TGFD : 1;
  unsigned char TCFV : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char TCFV : 1;
  unsigned char TGFD : 1;
  unsigned char TGFC : 1;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu0_tsr
{
  unsigned char BYTE;
  struct un_tpu0_tsr_bit BIT;
};

struct st_tpu1_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_tpu1_nfcr
{
  unsigned char BYTE;
  struct st_tpu1_nfcr_bit BIT;
};

struct st_tpu1_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu1_tcr
{
  unsigned char BYTE;
  struct st_tpu1_tcr_bit BIT;
};

struct st_tpu1_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 2;
  unsigned char ICSELB : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ICSELB : 1;
  unsigned char  : 2;
  unsigned char MD : 4;
#endif
};

union un_tpu1_tmdr
{
  unsigned char BYTE;
  struct st_tpu1_tmdr_bit BIT;
};

struct st_tpu1_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu1_tior
{
  unsigned char BYTE;
  struct st_tpu1_tior_bit BIT;
};

struct st_tpu1_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 1;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu1_tier
{
  unsigned char BYTE;
  struct st_tpu1_tier_bit BIT;
};

struct st_tpu1_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char  : 2;
  unsigned char TCFV : 1;
  unsigned char TCFU : 1;
  unsigned char  : 1;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 1;
  unsigned char TCFU : 1;
  unsigned char TCFV : 1;
  unsigned char  : 2;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu1_tsr
{
  unsigned char BYTE;
  struct st_tpu1_tsr_bit BIT;
};

struct st_tpu2_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_tpu2_nfcr
{
  unsigned char BYTE;
  struct st_tpu2_nfcr_bit BIT;
};

struct st_tpu2_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu2_tcr
{
  unsigned char BYTE;
  struct st_tpu2_tcr_bit BIT;
};

struct st_tpu2_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 2;
  unsigned char ICSELB : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ICSELB : 1;
  unsigned char  : 2;
  unsigned char MD : 4;
#endif
};

union un_tpu2_tmdr
{
  unsigned char BYTE;
  struct st_tpu2_tmdr_bit BIT;
};

struct st_tpu2_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu2_tior
{
  unsigned char BYTE;
  struct st_tpu2_tior_bit BIT;
};

struct st_tpu2_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 1;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu2_tier
{
  unsigned char BYTE;
  struct st_tpu2_tier_bit BIT;
};

struct st_tpu2_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char  : 2;
  unsigned char TCFV : 1;
  unsigned char TCFU : 1;
  unsigned char  : 1;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 1;
  unsigned char TCFU : 1;
  unsigned char TCFV : 1;
  unsigned char  : 2;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu2_tsr
{
  unsigned char BYTE;
  struct st_tpu2_tsr_bit BIT;
};

struct st_tpu3_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_tpu3_nfcr
{
  unsigned char BYTE;
  struct st_tpu3_nfcr_bit BIT;
};

struct st_tpu3_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu3_tcr
{
  unsigned char BYTE;
  struct st_tpu3_tcr_bit BIT;
};

struct st_tpu3_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char ICSELB : 1;
  unsigned char ICSELD : 1;
#else
  unsigned char ICSELD : 1;
  unsigned char ICSELB : 1;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_tpu3_tmdr
{
  unsigned char BYTE;
  struct st_tpu3_tmdr_bit BIT;
};

struct st_tpu3_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu3_tiorh
{
  unsigned char BYTE;
  struct st_tpu3_tiorh_bit BIT;
};

struct st_tpu3_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_tpu3_tiorl
{
  unsigned char BYTE;
  struct st_tpu3_tiorl_bit BIT;
};

struct st_tpu3_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu3_tier
{
  unsigned char BYTE;
  struct st_tpu3_tier_bit BIT;
};

struct st_tpu3_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char TGFC : 1;
  unsigned char TGFD : 1;
  unsigned char TCFV : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char TCFV : 1;
  unsigned char TGFD : 1;
  unsigned char TGFC : 1;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu3_tsr
{
  unsigned char BYTE;
  struct st_tpu3_tsr_bit BIT;
};

struct st_tpu4_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_tpu4_nfcr
{
  unsigned char BYTE;
  struct st_tpu4_nfcr_bit BIT;
};

struct st_tpu4_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu4_tcr
{
  unsigned char BYTE;
  struct st_tpu4_tcr_bit BIT;
};

struct st_tpu4_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 2;
  unsigned char ICSELB : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ICSELB : 1;
  unsigned char  : 2;
  unsigned char MD : 4;
#endif
};

union un_tpu4_tmdr
{
  unsigned char BYTE;
  struct st_tpu4_tmdr_bit BIT;
};

struct un_tpu4_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu4_tior
{
  unsigned char BYTE;
  struct un_tpu4_tior_bit BIT;
};

struct st_tpu4_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 1;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu4_tier
{
  unsigned char BYTE;
  struct st_tpu4_tier_bit BIT;
};

struct st_tpu4_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char  : 2;
  unsigned char TCFV : 1;
  unsigned char TCFU : 1;
  unsigned char  : 1;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 1;
  unsigned char TCFU : 1;
  unsigned char TCFV : 1;
  unsigned char  : 2;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu4_tsr
{
  unsigned char BYTE;
  struct st_tpu4_tsr_bit BIT;
};

struct st_tpu5_nfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 2;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_tpu5_nfcr
{
  unsigned char BYTE;
  struct st_tpu5_nfcr_bit BIT;
};

struct st_tpu5_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_tpu5_tcr
{
  unsigned char BYTE;
  struct st_tpu5_tcr_bit BIT;
};

struct st_tpu5_tmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 2;
  unsigned char ICSELB : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char ICSELB : 1;
  unsigned char  : 2;
  unsigned char MD : 4;
#endif
};

union un_tpu5_tmdr
{
  unsigned char BYTE;
  struct st_tpu5_tmdr_bit BIT;
};

struct st_tpu5_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_tpu5_tior
{
  unsigned char BYTE;
  struct st_tpu5_tior_bit BIT;
};

struct st_tpu5_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_tpu5_tier
{
  unsigned char BYTE;
  struct st_tpu5_tier_bit BIT;
};

struct st_tpu5_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGFA : 1;
  unsigned char TGFB : 1;
  unsigned char  : 2;
  unsigned char TCFV : 1;
  unsigned char TCFU : 1;
  unsigned char  : 1;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 1;
  unsigned char TCFU : 1;
  unsigned char TCFV : 1;
  unsigned char  : 2;
  unsigned char TGFB : 1;
  unsigned char TGFA : 1;
#endif
};

union un_tpu5_tsr
{
  unsigned char BYTE;
  struct st_tpu5_tsr_bit BIT;
};

struct st_tpua_tstr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CST0 : 1;
  unsigned char CST1 : 1;
  unsigned char CST2 : 1;
  unsigned char CST3 : 1;
  unsigned char CST4 : 1;
  unsigned char CST5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char CST5 : 1;
  unsigned char CST4 : 1;
  unsigned char CST3 : 1;
  unsigned char CST2 : 1;
  unsigned char CST1 : 1;
  unsigned char CST0 : 1;
#endif
};

union un_tpua_tstr
{
  unsigned char BYTE;
  struct st_tpua_tstr_bit BIT;
};

struct st_tpua_tsyr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SYNC0 : 1;
  unsigned char SYNC1 : 1;
  unsigned char SYNC2 : 1;
  unsigned char SYNC3 : 1;
  unsigned char SYNC4 : 1;
  unsigned char SYNC5 : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char SYNC5 : 1;
  unsigned char SYNC4 : 1;
  unsigned char SYNC3 : 1;
  unsigned char SYNC2 : 1;
  unsigned char SYNC1 : 1;
  unsigned char SYNC0 : 1;
#endif
};

union un_tpua_tsyr
{
  unsigned char BYTE;
  struct st_tpua_tsyr_bit BIT;
};

struct st_usb_dpusr0r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SRPC0 : 1;
  unsigned long RPUE0 : 1;
  unsigned long  : 1;
  unsigned long DRPD0 : 1;
  unsigned long FIXPHY0 : 1;
  unsigned long  : 11;
  unsigned long DP0 : 1;
  unsigned long DM0 : 1;
  unsigned long  : 2;
  unsigned long DOVCA0 : 1;
  unsigned long DOVCB0 : 1;
  unsigned long  : 1;
  unsigned long DVBSTS0 : 1;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long DVBSTS0 : 1;
  unsigned long  : 1;
  unsigned long DOVCB0 : 1;
  unsigned long DOVCA0 : 1;
  unsigned long  : 2;
  unsigned long DM0 : 1;
  unsigned long DP0 : 1;
  unsigned long  : 11;
  unsigned long FIXPHY0 : 1;
  unsigned long DRPD0 : 1;
  unsigned long  : 1;
  unsigned long RPUE0 : 1;
  unsigned long SRPC0 : 1;
#endif
};

union un_usb_dpusr0r
{
  unsigned long LONG;
  struct st_usb_dpusr0r_bit BIT;
};

struct st_usb_dpusr1r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DPINTE0 : 1;
  unsigned long DMINTE0 : 1;
  unsigned long  : 2;
  unsigned long DOVRCRAE0 : 1;
  unsigned long DOVRCRBE0 : 1;
  unsigned long  : 1;
  unsigned long DVBSE0 : 1;
  unsigned long  : 8;
  unsigned long DPINT0 : 1;
  unsigned long DMINT0 : 1;
  unsigned long  : 2;
  unsigned long DOVRCRA0 : 1;
  unsigned long DOVRCRB0 : 1;
  unsigned long  : 1;
  unsigned long DVBINT0 : 1;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long DVBINT0 : 1;
  unsigned long  : 1;
  unsigned long DOVRCRB0 : 1;
  unsigned long DOVRCRA0 : 1;
  unsigned long  : 2;
  unsigned long DMINT0 : 1;
  unsigned long DPINT0 : 1;
  unsigned long  : 8;
  unsigned long DVBSE0 : 1;
  unsigned long  : 1;
  unsigned long DOVRCRBE0 : 1;
  unsigned long DOVRCRAE0 : 1;
  unsigned long  : 2;
  unsigned long DMINTE0 : 1;
  unsigned long DPINTE0 : 1;
#endif
};

union un_usb_dpusr1r
{
  unsigned long LONG;
  struct st_usb_dpusr1r_bit BIT;
};

struct st_usb0_syscfg_bit
{
  unsigned short :5;
  unsigned short SCKE:1;
  unsigned short :3;
  unsigned short DCFM:1;
  unsigned short DRPD:1;
  unsigned short DPRPU:1;
  unsigned short :3;
  unsigned short USBE:1;
};

union un_usb0_syscfg
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_syscfg_bit BIT;
#endif
};

struct st_usb0_syssts0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short LNST : 2;
  unsigned short IDMON : 1;
  unsigned short  : 2;
  unsigned short SOFEA : 1;
  unsigned short HTACT : 1;
  unsigned short  : 7;
  unsigned short OVCMON : 2;
#else
  unsigned short OVCMON : 2;
  unsigned short  : 7;
  unsigned short HTACT : 1;
  unsigned short SOFEA : 1;
  unsigned short  : 2;
  unsigned short IDMON : 1;
  unsigned short LNST : 2;
#endif
};

union un_usb0_syssts0
{
  unsigned short WORD;
  struct st_usb0_syssts0_bit BIT;
};

struct st_usb0_dvstctr0_bit
{
  unsigned short :4;
  unsigned short HNPBTOA:1;
  unsigned short EXICEN:1;
  unsigned short VBUSEN:1;
  unsigned short WKUP:1;
  unsigned short RWUPE:1;
  unsigned short USBRST:1;
  unsigned short RESUME:1;
  unsigned short UACT:1;
  unsigned short :1;
  unsigned short RHST:3;
};

union un_usb0_dvstctr0
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_dvstctr0_bit BIT;
#endif
};

struct st_usb0_cfifo_byte
{
  unsigned char L;
  unsigned char H;
};

union un_usb0_cfifo
{
  unsigned short WORD;
  struct st_usb0_cfifo_byte BYTE;
};

struct st_usb0_d0fifo_byte
{
  unsigned char L;
  unsigned char H;
};

union un_usb0_d0fifo
{
  unsigned short WORD;
  struct st_usb0_d0fifo_byte BYTE;
};

struct st_usb0_d1fifo_byte
{
  unsigned char L;
  unsigned char H;
};

union un_usb0_d1fifo
{
  unsigned short WORD;
  struct st_usb0_d1fifo_byte BYTE;
};

struct st_usb0_cfifosel_bit
{
  unsigned short RCNT:1;
  unsigned short REW:1;
  unsigned short :3;
  unsigned short MBW:1;
  unsigned short :1;
  unsigned short BIGEND:1;
  unsigned short :2;
  unsigned short ISEL:1;
  unsigned short :1;
  unsigned short CURPIPE:4;
};

union un_usb0_cfifosel
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_cfifosel_bit BIT;
#endif
};

struct st_usb0_cfifoctr_bit
{
  unsigned short BVAL:1;
  unsigned short BCLR:1;
  unsigned short FRDY:1;
  unsigned short :4;
  unsigned short DTLN:9;
};

union un_usb0_cfifoctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_cfifoctr_bit BIT;
#endif
};

struct st_usb0_d0fifosel_bit
{
  unsigned short RCNT:1;
  unsigned short REW:1;
  unsigned short DCLRM:1;
  unsigned short DREQE:1;
  unsigned short :1;
  unsigned short MBW:1;
  unsigned short :1;
  unsigned short BIGEND:1;
  unsigned short :4;
  unsigned short CURPIPE:4;
};

union un_usb0_d0fifosel
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_d0fifosel_bit BIT;
#endif
};

struct st_usb0_d0fifoctr_bit
{
  unsigned short BVAL:1;
  unsigned short BCLR:1;
  unsigned short FRDY:1;
  unsigned short :4;
  unsigned short DTLN:9;
};

union un_usb0_d0fifoctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_d0fifoctr_bit BIT;
#endif
};

struct st_usb0_d1fifosel_bit
{
  unsigned short RCNT:1;
  unsigned short REW:1;
  unsigned short DCLRM:1;
  unsigned short DREQE:1;
  unsigned short :1;
  unsigned short MBW:1;
  unsigned short :1;
  unsigned short BIGEND:1;
  unsigned short :4;
  unsigned short CURPIPE:4;
};

union un_usb0_d1fifosel
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_d1fifosel_bit BIT;
#endif
};

struct st_usb0_d1fifoctr_bit
{
  unsigned short BVAL:1;
  unsigned short BCLR:1;
  unsigned short FRDY:1;
  unsigned short :4;
  unsigned short DTLN:9;
};

union un_usb0_d1fifoctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_d1fifoctr_bit BIT;
#endif
};

struct st_usb0_intenb0_bit
{
  unsigned short VBSE:1;
  unsigned short RSME:1;
  unsigned short SOFE:1;
  unsigned short DVSE:1;
  unsigned short CTRE:1;
  unsigned short BEMPE:1;
  unsigned short NRDYE:1;
  unsigned short BRDYE:1;
  unsigned short :8;
};

union un_usb0_intenb0
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_intenb0_bit BIT;
#endif
};

struct st_usb0_intenb1_bit
{
  unsigned short OVRCRE:1;
  unsigned short BCHGE:1;
  unsigned short :1;
  unsigned short DTCHE:1;
  unsigned short ATTCHE:1;
  unsigned short :4;
  unsigned short EOFERRE:1;
  unsigned short SIGNE:1;
  unsigned short SACKE:1;
  unsigned short :4;
};

union un_usb0_intenb1
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_intenb1_bit BIT;
#endif
};

struct st_usb0_brdyenb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PIPE0BRDYE : 1;
  unsigned short PIPE1BRDYE : 1;
  unsigned short PIPE2BRDYE : 1;
  unsigned short PIPE3BRDYE : 1;
  unsigned short PIPE4BRDYE : 1;
  unsigned short PIPE5BRDYE : 1;
  unsigned short PIPE6BRDYE : 1;
  unsigned short PIPE7BRDYE : 1;
  unsigned short PIPE8BRDYE : 1;
  unsigned short PIPE9BRDYE : 1;
  unsigned short  : 6;
#else
  unsigned short  : 6;
  unsigned short PIPE9BRDYE : 1;
  unsigned short PIPE8BRDYE : 1;
  unsigned short PIPE7BRDYE : 1;
  unsigned short PIPE6BRDYE : 1;
  unsigned short PIPE5BRDYE : 1;
  unsigned short PIPE4BRDYE : 1;
  unsigned short PIPE3BRDYE : 1;
  unsigned short PIPE2BRDYE : 1;
  unsigned short PIPE1BRDYE : 1;
  unsigned short PIPE0BRDYE : 1;
#endif
};

union un_usb0_brdyenb
{
  unsigned short WORD;
  struct st_usb0_brdyenb_bit BIT;
};

struct st_usb0_nrdyenb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PIPE0NRDYE : 1;
  unsigned short PIPE1NRDYE : 1;
  unsigned short PIPE2NRDYE : 1;
  unsigned short PIPE3NRDYE : 1;
  unsigned short PIPE4NRDYE : 1;
  unsigned short PIPE5NRDYE : 1;
  unsigned short PIPE6NRDYE : 1;
  unsigned short PIPE7NRDYE : 1;
  unsigned short PIPE8NRDYE : 1;
  unsigned short PIPE9NRDYE : 1;
  unsigned short  : 6;
#else
  unsigned short  : 6;
  unsigned short PIPE9NRDYE : 1;
  unsigned short PIPE8NRDYE : 1;
  unsigned short PIPE7NRDYE : 1;
  unsigned short PIPE6NRDYE : 1;
  unsigned short PIPE5NRDYE : 1;
  unsigned short PIPE4NRDYE : 1;
  unsigned short PIPE3NRDYE : 1;
  unsigned short PIPE2NRDYE : 1;
  unsigned short PIPE1NRDYE : 1;
  unsigned short PIPE0NRDYE : 1;
#endif
};

union un_usb0_nrdyenb
{
  unsigned short WORD;
  struct st_usb0_nrdyenb_bit BIT;
};

struct st_usb0_bempenb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PIPE0BEMPE : 1;
  unsigned short PIPE1BEMPE : 1;
  unsigned short PIPE2BEMPE : 1;
  unsigned short PIPE3BEMPE : 1;
  unsigned short PIPE4BEMPE : 1;
  unsigned short PIPE5BEMPE : 1;
  unsigned short PIPE6BEMPE : 1;
  unsigned short PIPE7BEMPE : 1;
  unsigned short PIPE8BEMPE : 1;
  unsigned short PIPE9BEMPE : 1;
  unsigned short  : 6;
#else
  unsigned short  : 6;
  unsigned short PIPE9BEMPE : 1;
  unsigned short PIPE8BEMPE : 1;
  unsigned short PIPE7BEMPE : 1;
  unsigned short PIPE6BEMPE : 1;
  unsigned short PIPE5BEMPE : 1;
  unsigned short PIPE4BEMPE : 1;
  unsigned short PIPE3BEMPE : 1;
  unsigned short PIPE2BEMPE : 1;
  unsigned short PIPE1BEMPE : 1;
  unsigned short PIPE0BEMPE : 1;
#endif
};

union un_usb0_bempenb
{
  unsigned short WORD;
  struct st_usb0_bempenb_bit BIT;
};

struct st_usb0_sofcfg_bit
{
  unsigned short :7;
  unsigned short TRNENSEL:1;
  unsigned short :1;
  unsigned short BRDYM:1;
  unsigned short :1;
  unsigned short EDGESTS:1;
  unsigned short :4;
};

union un_usb0_sofcfg
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_sofcfg_bit BIT;
#endif
};

struct st_usb0_intsts0_bit
{
  unsigned short VBINT:1;
  unsigned short RESM:1;
  unsigned short SOFR:1;
  unsigned short DVST:1;
  unsigned short CTRT:1;
  unsigned short BEMP:1;
  unsigned short NRDY:1;
  unsigned short BRDY:1;
  unsigned short VBSTS:1;
  unsigned short DVSQ:3;
  unsigned short VALID:1;
  unsigned short CTSQ:3;
};

union un_usb0_intsts0
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_intsts0_bit BIT;
#endif
};

struct st_usb0_intsts1_bit
{
  unsigned short OVRCR:1;
  unsigned short BCHG:1;
  unsigned short :1;
  unsigned short DTCH:1;
  unsigned short ATTCH:1;
  unsigned short :4;
  unsigned short EOFERR:1;
  unsigned short SIGN:1;
  unsigned short SACK:1;
  unsigned short :4;
};

union un_usb0_intsts1
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_intsts1_bit BIT;
#endif
};

struct st_usb0_brdysts_bit
{
  unsigned short :6;
  unsigned short PIPE9BRDY:1;
  unsigned short PIPE8BRDY:1;
  unsigned short PIPE7BRDY:1;
  unsigned short PIPE6BRDY:1;
  unsigned short PIPE5BRDY:1;
  unsigned short PIPE4BRDY:1;
  unsigned short PIPE3BRDY:1;
  unsigned short PIPE2BRDY:1;
  unsigned short PIPE1BRDY:1;
  unsigned short PIPE0BRDY:1;
};

union un_usb0_brdysts
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_brdysts_bit BIT;
#endif
};

struct st_usb0_nrdysts_bit
{
  unsigned short :6;
  unsigned short PIPE9NRDY:1;
  unsigned short PIPE8NRDY:1;
  unsigned short PIPE7NRDY:1;
  unsigned short PIPE6NRDY:1;
  unsigned short PIPE5NRDY:1;
  unsigned short PIPE4NRDY:1;
  unsigned short PIPE3NRDY:1;
  unsigned short PIPE2NRDY:1;
  unsigned short PIPE1NRDY:1;
  unsigned short PIPE0NRDY:1;
};

union un_usb0_nrdysts
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_nrdysts_bit BIT;
#endif
};

struct st_usb0_bempsts_bit
{
  unsigned short :6;
  unsigned short PIPE9BEMP:1;
  unsigned short PIPE8BEMP:1;
  unsigned short PIPE7BEMP:1;
  unsigned short PIPE6BEMP:1;
  unsigned short PIPE5BEMP:1;
  unsigned short PIPE4BEMP:1;
  unsigned short PIPE3BEMP:1;
  unsigned short PIPE2BEMP:1;
  unsigned short PIPE1BEMP:1;
  unsigned short PIPE0BEMP:1;
};

union un_usb0_bempsts
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_bempsts_bit BIT;
#endif
};

struct st_usb0_frmnum_bit
{
  unsigned short OVRN:1;
  unsigned short CRCE:1;
  unsigned short :3;
  unsigned short FRNM:11;
};

union un_usb0_frmnum
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_frmnum_bit BIT;
#endif
};

struct st_usb0_dvchgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 15;
  unsigned short DVCHG : 1;
#else
  unsigned short DVCHG : 1;
  unsigned short  : 15;
#endif
};

union un_usb0_dvchgr
{
  unsigned short WORD;
  struct st_usb0_dvchgr_bit BIT;
};

struct st_usb0_usbaddr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short USBADDR : 7;
  unsigned short  : 1;
  unsigned short STSRECOV : 4;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short STSRECOV : 4;
  unsigned short  : 1;
  unsigned short USBADDR : 7;
#endif
};

union un_usb0_usbaddr
{
  unsigned short WORD;
  struct st_usb0_usbaddr_bit BIT;
};

struct st_usb0_usbreq_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short BMREQUESTTYPE : 8;
  unsigned short BREQUEST : 8;
#else
  unsigned short BREQUEST : 8;
  unsigned short BMREQUESTTYPE : 8;
#endif
};

union un_usb0_usbreq
{
  unsigned short WORD;
  struct st_usb0_usbreq_bit BIT;
};

struct st_usb0_dcpcfg_bit
{
  unsigned short :8;
  unsigned short SHTNAK:1;
  unsigned short :2;
  unsigned short DIR:1;
  unsigned short :4;
};

union un_usb0_dcpcfg
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_dcpcfg_bit BIT;
#endif
};

struct st_usb0_dcpmaxp_bit
{
  unsigned short DEVSEL:4;
  unsigned short :5;
  unsigned short MXPS:7;
};

union un_usb0_dcpmaxp
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_dcpmaxp_bit BIT;
#endif
};

struct st_usb0_dcpctr_bit
{
  unsigned short BSTS:1;
  unsigned short SUREQ:1;
  unsigned short :2;
  unsigned short SUREQCLR:1;
  unsigned short :2;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :2;
  unsigned short CCPL:1;
  unsigned short PID:2;
};

union un_usb0_dcpctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_dcpctr_bit BIT;
#endif
};

struct st_usb0_pipesel_bit
{
  unsigned short :12;
  unsigned short PIPESEL:4;
};

union un_usb0_pipesel
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipesel_bit BIT;
#endif
};

struct st_usb0_pipecfg_bit
{
  unsigned short TYPE:2;
  unsigned short :3;
  unsigned short BFRE:1;
  unsigned short DBLB:1;
  unsigned short :1;
  unsigned short SHTNAK:1;
  unsigned short :2;
  unsigned short DIR:1;
  unsigned short EPNUM:4;
};

union un_usb0_pipecfg
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipecfg_bit BIT;
#endif
};

struct st_usb0_pipemaxp_bit
{
  unsigned short DEVSEL:4;
  unsigned short :3;
  unsigned short MXPS:9;
};

union un_usb0_pipemaxp
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipemaxp_bit BIT;
#endif
};

struct st_usb0_pipeperi_bit
{
  unsigned short :3;
  unsigned short IFIS:1;
  unsigned short :9;
  unsigned short IITV:3;
};

union un_usb0_pipeperi
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipeperi_bit BIT;
#endif
};

struct st_usb0_pipe1ctr_bit
{
  unsigned short BSTS:1;
  unsigned short INBUFM:1;
  unsigned short :3;
  unsigned short ATREPM:1;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe1ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe1ctr_bit BIT;
#endif
};

struct st_usb0_pipe2ctr_bit
{
  unsigned short BSTS:1;
  unsigned short INBUFM:1;
  unsigned short :3;
  unsigned short ATREPM:1;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe2ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe2ctr_bit BIT;
#endif
};

struct st_usb0_pipe3ctr_bit
{
  unsigned short BSTS:1;
  unsigned short INBUFM:1;
  unsigned short :3;
  unsigned short ATREPM:1;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe3ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe3ctr_bit BIT;
#endif
};

struct st_usb0_pipe4ctr_bit
{
  unsigned short BSTS:1;
  unsigned short INBUFM:1;
  unsigned short :3;
  unsigned short ATREPM:1;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe4ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe4ctr_bit BIT;
#endif
};

struct st_usb0_pipe5ctr_bit
{
  unsigned short BSTS:1;
  unsigned short INBUFM:1;
  unsigned short :3;
  unsigned short ATREPM:1;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe5ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe5ctr_bit BIT;
#endif
};

struct st_usb0_pipe6ctr_bit
{
  unsigned short BSTS:1;
  unsigned short :5;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe6ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe6ctr_bit BIT;
#endif
};

struct st_usb0_pipe7ctr_bit
{
  unsigned short BSTS:1;
  unsigned short :5;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe7ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe7ctr_bit BIT;
#endif
};

struct st_usb0_pipe8ctr_bit
{
  unsigned short BSTS:1;
  unsigned short :5;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe8ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe8ctr_bit BIT;
#endif
};

struct st_usb0_pipe9ctr_bit
{
  unsigned short BSTS:1;
  unsigned short :5;
  unsigned short ACLRM:1;
  unsigned short SQCLR:1;
  unsigned short SQSET:1;
  unsigned short SQMON:1;
  unsigned short PBUSY:1;
  unsigned short :3;
  unsigned short PID:2;
};

union un_usb0_pipe9ctr
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe9ctr_bit BIT;
#endif
};

struct st_usb0_pipe1tre_bit
{
  unsigned short :6;
  unsigned short TRENB:1;
  unsigned short TRCLR:1;
  unsigned short :8;
};

union un_usb0_pipe1tre
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe1tre_bit BIT;
#endif
};

struct st_usb0_pipe2tre_bit
{
  unsigned short :6;
  unsigned short TRENB:1;
  unsigned short TRCLR:1;
  unsigned short :8;
};

union un_usb0_pipe2tre
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe2tre_bit BIT;
#endif
};

struct st_usb0_pipe3tre_bit
{
  unsigned short :6;
  unsigned short TRENB:1;
  unsigned short TRCLR:1;
  unsigned short :8;
};

union un_usb0_pipe3tre
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe3tre_bit BIT;
#endif
};

struct st_usb0_pipe4tre_bit
{
  unsigned short :6;
  unsigned short TRENB:1;
  unsigned short TRCLR:1;
  unsigned short :8;
};

union un_usb0_pipe4tre
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe4tre_bit BIT;
#endif
};

struct st_usb0_pipe5tre_bit
{
  unsigned short :6;
  unsigned short TRENB:1;
  unsigned short TRCLR:1;
  unsigned short :8;
};

union un_usb0_pipe5tre
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_pipe5tre_bit BIT;
#endif
};

struct st_usb0_devadd0_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd0
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd0_bit BIT;
#endif
};

struct st_usb0_devadd1_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd1
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd1_bit BIT;
#endif
};

struct st_usb0_devadd2_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd2
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd2_bit BIT;
#endif
};

struct st_usb0_devadd3_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd3
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd3_bit BIT;
#endif
};

struct st_usb0_devadd4_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd4
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd4_bit BIT;
#endif
};

struct st_usb0_devadd5_bit
{
  unsigned short :8;
  unsigned short USBSPD:2;
  unsigned short :6;
};

union un_usb0_devadd5
{
  unsigned short WORD;
#ifdef IODEFINE_H_HISTORY
  struct st_usb0_devadd5_bit BIT;
#endif
};

struct st_usb0_physlew_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SLEWR00 : 1;
  unsigned long SLEWR01 : 1;
  unsigned long SLEWF00 : 1;
  unsigned long SLEWF01 : 1;
  unsigned long  : 28;
#else
  unsigned long  : 28;
  unsigned long SLEWF01 : 1;
  unsigned long SLEWF00 : 1;
  unsigned long SLEWR01 : 1;
  unsigned long SLEWR00 : 1;
#endif
};

union un_usb0_physlew
{
  unsigned long LONG;
  struct st_usb0_physlew_bit BIT;
};

struct st_wdt_wdtcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TOPS : 2;
  unsigned short  : 2;
  unsigned short CKS : 4;
  unsigned short RPES : 2;
  unsigned short  : 2;
  unsigned short RPSS : 2;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short RPSS : 2;
  unsigned short  : 2;
  unsigned short RPES : 2;
  unsigned short CKS : 4;
  unsigned short  : 2;
  unsigned short TOPS : 2;
#endif
};

union un_wdt_wdtcr
{
  unsigned short WORD;
  struct st_wdt_wdtcr_bit BIT;
};

struct st_wdt_wdtsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CNTVAL : 14;
  unsigned short UNDFF : 1;
  unsigned short REFEF : 1;
#else
  unsigned short REFEF : 1;
  unsigned short UNDFF : 1;
  unsigned short CNTVAL : 14;
#endif
};

union un_wdt_wdtsr
{
  unsigned short WORD;
  struct st_wdt_wdtsr_bit BIT;
};

struct st_wdt_wdtrcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char RSTIRQS : 1;
#else
  unsigned char RSTIRQS : 1;
  unsigned char  : 7;
#endif
};

union un_wdt_wdtrcr
{
  unsigned char BYTE;
  struct st_wdt_wdtrcr_bit BIT;
};

struct st_crc_crccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char GPS : 3;
  unsigned char  : 3;
  unsigned char LMS : 1;
  unsigned char DORCLR : 1;
#else
  unsigned char DORCLR : 1;
  unsigned char LMS : 1;
  unsigned char  : 3;
  unsigned char GPS : 3;
#endif
};

union un_crc_crccr
{
  unsigned char BYTE;
  struct st_crc_crccr_bit BIT;
};

union un_crc_crcdir
{
  unsigned long LONG;
  unsigned char BYTE;
};

union un_crc_crcdor
{
  unsigned long LONG;
  unsigned short WORD;
  unsigned char BYTE;
};

struct st_da_dacr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 5;
  unsigned char DAE : 1;
  unsigned char DAOE0 : 1;
  unsigned char DAOE1 : 1;
#else
  unsigned char DAOE1 : 1;
  unsigned char DAOE0 : 1;
  unsigned char DAE : 1;
  unsigned char  : 5;
#endif
};

union un_da_dacr
{
  unsigned char BYTE;
  struct st_da_dacr_bit BIT;
};

struct st_da_dadpr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char DPSEL : 1;
#else
  unsigned char DPSEL : 1;
  unsigned char  : 7;
#endif
};

union un_da_dadpr
{
  unsigned char BYTE;
  struct st_da_dadpr_bit BIT;
};

struct st_da_daadscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char DAADST : 1;
#else
  unsigned char DAADST : 1;
  unsigned char  : 7;
#endif
};

union un_da_daadscr
{
  unsigned char BYTE;
  struct st_da_daadscr_bit BIT;
};

struct st_da_daampcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char DAAMP0 : 1;
  unsigned char DAAMP1 : 1;
#else
  unsigned char DAAMP1 : 1;
  unsigned char DAAMP0 : 1;
  unsigned char  : 6;
#endif
};

union un_da_daampcr
{
  unsigned char BYTE;
  struct st_da_daampcr_bit BIT;
};

struct st_da_daaswcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char DAASW0 : 1;
  unsigned char DAASW1 : 1;
#else
  unsigned char DAASW1 : 1;
  unsigned char DAASW0 : 1;
  unsigned char  : 6;
#endif
};

union un_da_daaswcr
{
  unsigned char BYTE;
  struct st_da_daaswcr_bit BIT;
};

struct st_da_daadusr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char AMADSEL1 : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char AMADSEL1 : 1;
  unsigned char  : 1;
#endif
};

union un_da_daadusr
{
  unsigned char BYTE;
  struct st_da_daadusr_bit BIT;
};

struct st_doc_docr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OMS : 2;
  unsigned char DCSEL : 1;
  unsigned char  : 1;
  unsigned char DOPCIE : 1;
  unsigned char DOPCF : 1;
  unsigned char DOPCFCL : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char DOPCFCL : 1;
  unsigned char DOPCF : 1;
  unsigned char DOPCIE : 1;
  unsigned char  : 1;
  unsigned char DCSEL : 1;
  unsigned char OMS : 2;
#endif
};

union un_doc_docr
{
  unsigned char BYTE;
  struct st_doc_docr_bit BIT;
};

struct st_mtu_toera_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OE3B : 1;
  unsigned char OE4A : 1;
  unsigned char OE4B : 1;
  unsigned char OE3D : 1;
  unsigned char OE4C : 1;
  unsigned char OE4D : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char OE4D : 1;
  unsigned char OE4C : 1;
  unsigned char OE3D : 1;
  unsigned char OE4B : 1;
  unsigned char OE4A : 1;
  unsigned char OE3B : 1;
#endif
};

union un_mtu_toera
{
  unsigned char BYTE;
  struct st_mtu_toera_bit BIT;
};

struct st_mtu_tgcra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char UF : 1;
  unsigned char VF : 1;
  unsigned char WF : 1;
  unsigned char FB : 1;
  unsigned char P : 1;
  unsigned char N : 1;
  unsigned char BDC : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char BDC : 1;
  unsigned char N : 1;
  unsigned char P : 1;
  unsigned char FB : 1;
  unsigned char WF : 1;
  unsigned char VF : 1;
  unsigned char UF : 1;
#endif
};

union un_mtu_tgcra
{
  unsigned char BYTE;
  struct st_mtu_tgcra_bit BIT;
};

struct st_mtu_tocr1a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLSP : 1;
  unsigned char OLSN : 1;
  unsigned char TOCS : 1;
  unsigned char TOCL : 1;
  unsigned char  : 2;
  unsigned char PSYE : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSYE : 1;
  unsigned char  : 2;
  unsigned char TOCL : 1;
  unsigned char TOCS : 1;
  unsigned char OLSN : 1;
  unsigned char OLSP : 1;
#endif
};

union un_mtu_tocr1a
{
  unsigned char BYTE;
  struct st_mtu_tocr1a_bit BIT;
};

struct st_mtu_tocr2a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLS1P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS3N : 1;
  unsigned char BF : 2;
#else
  unsigned char BF : 2;
  unsigned char OLS3N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS1P : 1;
#endif
};

union un_mtu_tocr2a
{
  unsigned char BYTE;
  struct st_mtu_tocr2a_bit BIT;
};

struct st_mtu_titcr1a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char T4VCOR : 3;
  unsigned char T4VEN : 1;
  unsigned char T3ACOR : 3;
  unsigned char T3AEN : 1;
#else
  unsigned char T3AEN : 1;
  unsigned char T3ACOR : 3;
  unsigned char T4VEN : 1;
  unsigned char T4VCOR : 3;
#endif
};

union un_mtu_titcr1a
{
  unsigned char BYTE;
  struct st_mtu_titcr1a_bit BIT;
};

struct st_mtu_titcnt1a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char T4VCNT : 3;
  unsigned char  : 1;
  unsigned char T3ACNT : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char T3ACNT : 3;
  unsigned char  : 1;
  unsigned char T4VCNT : 3;
#endif
};

union un_mtu_titcnt1a
{
  unsigned char BYTE;
  struct st_mtu_titcnt1a_bit BIT;
};

struct st_mtu_tbtera_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BTE : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char BTE : 2;
#endif
};

union un_mtu_tbtera
{
  unsigned char BYTE;
  struct st_mtu_tbtera_bit BIT;
};

struct st_mtu_tdera_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TDER : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TDER : 1;
#endif
};

union un_mtu_tdera
{
  unsigned char BYTE;
  struct st_mtu_tdera_bit BIT;
};

struct st_mtu_tolbra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLS1P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS3N : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char OLS3N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS1P : 1;
#endif
};

union un_mtu_tolbra
{
  unsigned char BYTE;
  struct st_mtu_tolbra_bit BIT;
};

struct st_mtu_titmra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TITM : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TITM : 1;
#endif
};

union un_mtu_titmra
{
  unsigned char BYTE;
  struct st_mtu_titmra_bit BIT;
};

struct st_mtu_titcr2a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRG4COR : 3;
  unsigned char  : 5;
  #else
  unsigned char  : 5;
  unsigned char TRG4COR : 3;
#endif
};

union un_mtu_titcr2a
{
  unsigned char BYTE;
  struct st_mtu_titcr2a_bit BIT;
};

struct st_mtu_titcnt2a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRG4CNT : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TRG4CNT : 3;
#endif
};

union un_mtu_titcnt2a
{
  unsigned char BYTE;
  struct st_mtu_titcnt2a_bit BIT;
};

struct st_mtu_twcra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char WRE : 1;
  unsigned char SCC : 1;
  unsigned char  : 5;
  unsigned char CCE : 1;
#else
  unsigned char CCE : 1;
  unsigned char  : 5;
  unsigned char SCC : 1;
  unsigned char WRE : 1;
#endif
};

union un_mtu_twcra
{
  unsigned char BYTE;
  struct st_mtu_twcra_bit BIT;
};

struct st_mtu_tmdr2a_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DRS : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DRS : 1;
#endif
};

union un_mtu_tmdr2a
{
  unsigned char BYTE;
  struct st_mtu_tmdr2a_bit BIT;
};

struct st_mtu_tstra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CST0 : 1;
  unsigned char CST1 : 1;
  unsigned char CST2 : 1;
  unsigned char CST8 : 1;
  unsigned char  : 2;
  unsigned char CST3 : 1;
  unsigned char CST4 : 1;
#else
  unsigned char CST4 : 1;
  unsigned char CST3 : 1;
  unsigned char  : 2;
  unsigned char CST8 : 1;
  unsigned char CST2 : 1;
  unsigned char CST1 : 1;
  unsigned char CST0 : 1;
#endif
};

union un_mtu_tstra
{
  unsigned char BYTE;
  struct st_mtu_tstra_bit BIT;
};

struct st_mtu_tsyra_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SYNC0 : 1;
  unsigned char SYNC1 : 1;
  unsigned char SYNC2 : 1;
  unsigned char  : 3;
  unsigned char SYNC3 : 1;
  unsigned char SYNC4 : 1;
#else
  unsigned char SYNC4 : 1;
  unsigned char SYNC3 : 1;
  unsigned char  : 3;
  unsigned char SYNC2 : 1;
  unsigned char SYNC1 : 1;
  unsigned char SYNC0 : 1;
#endif
};

union un_mtu_tsyra
{
  unsigned char BYTE;
  struct st_mtu_tsyra_bit BIT;
};

struct st_mtu_tcsystr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SCH7 : 1;
  unsigned char SCH6 : 1;
  unsigned char  : 1;
  unsigned char SCH4 : 1;
  unsigned char SCH3 : 1;
  unsigned char SCH2 : 1;
  unsigned char SCH1 : 1;
  unsigned char SCH0 : 1;
#else
  unsigned char SCH0 : 1;
  unsigned char SCH1 : 1;
  unsigned char SCH2 : 1;
  unsigned char SCH3 : 1;
  unsigned char SCH4 : 1;
  unsigned char  : 1;
  unsigned char SCH6 : 1;
  unsigned char SCH7 : 1;
#endif
};

union un_mtu_tcsystr
{
  unsigned char BYTE;
  struct st_mtu_tcsystr_bit BIT;
};

struct st_mtu_trwera_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RWE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char RWE : 1;
#endif
};

union un_mtu_trwera
{
  unsigned char BYTE;
  struct st_mtu_trwera_bit BIT;
};

struct st_mtu_toerb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OE6B : 1;
  unsigned char OE7A : 1;
  unsigned char OE7B : 1;
  unsigned char OE6D : 1;
  unsigned char OE7C : 1;
  unsigned char OE7D : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char OE7D : 1;
  unsigned char OE7C : 1;
  unsigned char OE6D : 1;
  unsigned char OE7B : 1;
  unsigned char OE7A : 1;
  unsigned char OE6B : 1;
#endif
};

union un_mtu_toerb
{
  unsigned char BYTE;
  struct st_mtu_toerb_bit BIT;
};

struct st_mtu_tocr1b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLSP : 1;
  unsigned char OLSN : 1;
  unsigned char TOCS : 1;
  unsigned char TOCL : 1;
  unsigned char  : 2;
  unsigned char PSYE : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSYE : 1;
  unsigned char  : 2;
  unsigned char TOCL : 1;
  unsigned char TOCS : 1;
  unsigned char OLSN : 1;
  unsigned char OLSP : 1;
#endif
};

union un_mtu_tocr1b
{
  unsigned char BYTE;
  struct st_mtu_tocr1b_bit BIT;
};

struct st_mtu_tocr2b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLS1P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS3N : 1;
  unsigned char BF : 2;
#else
  unsigned char BF : 2;
  unsigned char OLS3N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS1P : 1;
#endif
};

union un_mtu_tocr2b
{
  unsigned char BYTE;
  struct st_mtu_tocr2b_bit BIT;
};

struct st_mtu_titcr1b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char T7VCOR : 3;
  unsigned char T7VEN : 1;
  unsigned char T6ACOR : 3;
  unsigned char T6AEN : 1;
#else
  unsigned char T6AEN : 1;
  unsigned char T6ACOR : 3;
  unsigned char T7VEN : 1;
  unsigned char T7VCOR : 3;
#endif
};

union un_mtu_titcr1b
{
  unsigned char BYTE;
  struct st_mtu_titcr1b_bit BIT;
};

struct st_mtu_titcnt1b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char T7VCNT : 3;
  unsigned char  : 1;
  unsigned char T6ACNT : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char T6ACNT : 3;
  unsigned char  : 1;
  unsigned char T7VCNT : 3;
#endif
};

union un_mtu_titcnt1b
{
  unsigned char BYTE;
  struct st_mtu_titcnt1b_bit BIT;
};

struct st_mtu_tbterb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BTE : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char BTE : 2;
#endif
};

union un_mtu_tbterb
{
  unsigned char BYTE;
  struct st_mtu_tbterb_bit BIT;
};

struct st_mtu_tderb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TDER : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TDER : 1;
#endif
};

union un_mtu_tderb
{
  unsigned char BYTE;
  struct st_mtu_tderb_bit BIT;
};

struct st_mtu_tolbrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OLS1P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS3N : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char OLS3N : 1;
  unsigned char OLS3P : 1;
  unsigned char OLS2N : 1;
  unsigned char OLS2P : 1;
  unsigned char OLS1N : 1;
  unsigned char OLS1P : 1;
#endif
};

union un_mtu_tolbrb
{
  unsigned char BYTE;
  struct st_mtu_tolbrb_bit BIT;
};

struct st_mtu_titmrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TITM : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char TITM : 1;
#endif
};

union un_mtu_titmrb
{
  unsigned char BYTE;
  struct st_mtu_titmrb_bit BIT;
};

struct st_mtu_titcr2b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRG7COR : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TRG7COR : 3;
#endif
};

union un_mtu_titcr2b
{
  unsigned char BYTE;
  struct st_mtu_titcr2b_bit BIT;
};

struct st_mtu_titcnt2b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRG7CNT : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TRG7CNT : 3;
#endif
};

union un_mtu_titcnt2b
{
  unsigned char BYTE;
  struct st_mtu_titcnt2b_bit BIT;
};

struct st_mtu_twcrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char WRE : 1;
  unsigned char SCC : 1;
  unsigned char  : 5;
  unsigned char CCE : 1;
#else
  unsigned char CCE : 1;
  unsigned char  : 5;
  unsigned char SCC : 1;
  unsigned char WRE : 1;
#endif
};

union un_mtu_twcrb
{
  unsigned char BYTE;
  struct st_mtu_twcrb_bit BIT;
};

struct st_mtu_tmdr2b_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DRS : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DRS : 1;
#endif
};

union un_mtu_twdr2b
{
  unsigned char BYTE;
  struct st_mtu_tmdr2b_bit BIT;
};

struct st_mtu_tstrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char CST6 : 1;
  unsigned char CST7 : 1;
#else
  unsigned char CST7 : 1;
  unsigned char CST6 : 1;
  unsigned char  : 6;
#endif
};

union un_mtu_tstrb
{
  unsigned char BYTE;
  struct st_mtu_tstrb_bit BIT;
};

struct st_mtu_tsyrb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 6;
  unsigned char SYNC6 : 1;
  unsigned char SYNC7 : 1;
#else
  unsigned char SYNC7 : 1;
  unsigned char SYNC6 : 1;
  unsigned char  : 6;
#endif
};

union un_mtu_tsyrb
{
  unsigned char BYTE;
  struct st_mtu_tsyrb_bit BIT;
};

struct st_mtu_trwerb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RWE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char RWE : 1;
#endif
};

union un_mtu_trwerb
{
  unsigned char BYTE;
  struct st_mtu_trwerb_bit BIT;
};

struct st_mtu0_nfcro_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu0_nfcro
{
  unsigned char BYTE;
  struct st_mtu0_nfcro_bit BIT;
};

struct st_mtu0_nfcrc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu0_nfcrc
{
  unsigned char BYTE;
  struct st_mtu0_nfcrc_bit BIT;
};

struct st_mtu0_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu0_tcr
{
  unsigned char BYTE;
  struct st_mtu0_tcr_bit BIT;
};

struct st_mtu0_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char BFE : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char BFE : 1;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu0_tmdr1
{
  unsigned char BYTE;
  struct st_mtu0_tmdr1_bit BIT;
};

struct st_mtu0_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu0_tiorh
{
  unsigned char BYTE;
  struct st_mtu0_tiorh_bit BIT;
};

struct st_mtu0_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu0_tiorl
{
  unsigned char BYTE;
  struct st_mtu0_tiorl_bit BIT;
};

struct st_mtu0_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu0_tier
{
  unsigned char BYTE;
  struct st_mtu0_tier_bit BIT;
};

struct st_mtu0_tier2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEE : 1;
  unsigned char TGIEF : 1;
  unsigned char  : 5;
  unsigned char TTGE2 : 1;
#else
  unsigned char TTGE2 : 1;
  unsigned char  : 5;
  unsigned char TGIEF : 1;
  unsigned char TGIEE : 1;
#endif
};

union un_mtu0_tier2
{
  unsigned char BYTE;
  struct st_mtu0_tier2_bit BIT;
};

struct st_mtu0_tbtm_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TTSA : 1;
  unsigned char TTSB : 1;
  unsigned char TTSE : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TTSE : 1;
  unsigned char TTSB : 1;
  unsigned char TTSA : 1;
#endif
};

union un_mtu0_tbtm
{
  unsigned char BYTE;
  struct st_mtu0_tbtm_bit BIT;
};

struct st_mtu0_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu0_tcr2
{
  unsigned char BYTE;
  struct st_mtu0_tcr2_bit BIT;
};

struct st_mtu1_nfcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu1_nfcr1
{
  unsigned char BYTE;
  struct st_mtu1_nfcr1_bit BIT;
};

struct st_mtu1_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu1_tcr
{
  unsigned char BYTE;
  struct st_mtu1_tcr_bit BIT;
};

struct st_mtu1_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char MD : 4;
#endif
};

union un_mtu1_tmdr1
{
  unsigned char BYTE;
  struct st_mtu1_tmdr1_bit BIT;
};

struct st_mtu1_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
  #else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu1_tior
{
  unsigned char BYTE;
  struct st_mtu1_tior_bit BIT;
};

struct st_mtu1_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 1;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu1_tier
{
  unsigned char BYTE;
  struct st_mtu1_tier_bit BIT;
};

struct st_mtu1_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};
union un_mtu1_tsr
{
  unsigned char BYTE;
  struct st_mtu1_tsr_bit BIT;
};

struct st_mtu1_ticcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char I1AE : 1;
  unsigned char I1BE : 1;
  unsigned char I2AE : 1;
  unsigned char I2BE : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char I2BE : 1;
  unsigned char I2AE : 1;
  unsigned char I1BE : 1;
  unsigned char I1AE : 1;
#endif
};

union un_mtu1_ticcr
{
  unsigned char BYTE;
  struct st_mtu1_ticcr_bit BIT;
};

struct st_mtu1_tmdr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char LWA : 1;
  unsigned char PHCKSEL : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char PHCKSEL : 1;
  unsigned char LWA : 1;
#endif
};

union un_mtu1_tmdr3
{
  unsigned char BYTE;
  struct st_mtu1_tmdr3_bit BIT;
};

struct st_mtu1_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char PCB : 2;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char PCB : 2;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu1_tcr2
{
  unsigned char BYTE;
  struct st_mtu1_tcr2_bit BIT;
};
struct st_mtu2_nfcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu2_nfcr2
{
  unsigned char BYTE;
  struct st_mtu2_nfcr2_bit BIT;
};

struct st_mtu2_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char CCLR : 2;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu2_tcr
{
  unsigned char BYTE;
  struct st_mtu2_tcr_bit BIT;
};

struct st_mtu2_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char MD : 4;
#endif
};

union un_mtu2_tmdr1
{
  unsigned char BYTE;
  struct st_mtu2_tmdr1_bit BIT;
};

struct st_mtu2_tior_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu2_tior
{
  unsigned char BYTE;
  struct st_mtu2_tior_bit BIT;
};

struct st_mtu2_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TCIEU : 1;
  unsigned char  : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 1;
  unsigned char TCIEU : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu2_tier
{
  unsigned char BYTE;
  struct st_mtu2_tier_bit BIT;
};

struct st_mtu2_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};

union un_mtu2_tsr
{
  unsigned char BYTE;
  struct st_mtu2_tsr_bit BIT;
};

struct st_mtu2_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char PCB : 2;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char PCB : 2;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu2_tcr2
{
  unsigned char BYTE;
  struct st_mtu2_tcr2_bit BIT;
};

struct st_mtu3_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu3_tcr
{
  unsigned char BYTE;
  struct st_mtu3_tcr_bit BIT;
};

struct st_mtu3_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu3_tmdr1
{
  unsigned char BYTE;
  struct st_mtu3_tmdr1_bit BIT;
};

struct st_mtu3_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu3_tiorh
{
  unsigned char BYTE;
  struct st_mtu3_tiorh_bit BIT;
};

struct st_mtu3_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu3_tiorl
{
  unsigned char BYTE;
  struct st_mtu3_tiorl_bit BIT;
};

struct st_mtu3_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu3_tier
{
  unsigned char BYTE;
  struct st_mtu3_tier_bit BIT;
};

struct st_mtu3_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};

union un_mtu3_tsr
{
  unsigned char BYTE;
  struct st_mtu3_tsr_bit BIT;
};

struct st_mtu3_tbtm_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TTSA : 1;
  unsigned char TTSB : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TTSB : 1;
  unsigned char TTSA : 1;
#endif
};

union un_mtu3_tbtm
{
  unsigned char BYTE;
  struct st_mtu3_tbtm_bit BIT;
};

struct st_mtu3_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu3_tcr2
{
  unsigned char BYTE;
  struct st_mtu3_tcr2_bit BIT;
};

struct st_mtu3_nfcr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu3_nfcr3
{
  unsigned char BYTE;
  struct st_mtu3_nfcr3_bit BIT;
};

struct st_iwdt_iwdtcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TOPS : 2;
  unsigned short  : 2;
  unsigned short CKS : 4;
  unsigned short RPES : 2;
  unsigned short  : 2;
  unsigned short RPSS : 2;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short RPSS : 2;
  unsigned short  : 2;
  unsigned short RPES : 2;
  unsigned short CKS : 4;
  unsigned short  : 2;
  unsigned short TOPS : 2;
#endif
};

union un_iwdt_iwdtcr
{
  unsigned short WORD;
  struct st_iwdt_iwdtcr_bit BIT;
};

struct st_iwdt_iwdtsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CNTVAL : 14;
  unsigned short UNDFF : 1;
  unsigned short REFEF : 1;
#else
  unsigned short REFEF : 1;
  unsigned short UNDFF : 1;
  unsigned short CNTVAL : 14;
#endif
};

union un_iwdt_iwdtsr
{
  unsigned short WORD;
  struct st_iwdt_iwdtsr_bit BIT;
};

struct st_iwdt_iwdtrcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char RSTIRQS : 1;
#else
  unsigned char RSTIRQS : 1;
  unsigned char  : 7;
#endif
};

union un_iwdt_iwdtrcr
{
  unsigned char BYTE;
  struct st_iwdt_iwdtrcr_bit BIT;
};

struct st_iwdt_iwdtcstpr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char SLCSTP : 1;
#else
  unsigned char SLCSTP : 1;
  unsigned char  : 7;
#endif
};

union un_iwdt_iwdtcstpr
{
  unsigned char BYTE;
  struct st_iwdt_iwdtcstpr_bit BIT;
};

struct st_mpu_rspage0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage0
{
  unsigned long LONG;
  struct st_mpu_rspage0_bit BIT;
};

struct st_mpu_repage0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage0
{
  unsigned long LONG;
  struct st_mpu_repage0_bit BIT;
};

struct st_mpu_rspage1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage1
{
  unsigned long LONG;
  struct st_mpu_rspage1_bit BIT;
};

struct st_mpu_repage1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};
union un_mpu_repage1
{
  unsigned long LONG;
  struct st_mpu_repage1_bit BIT;
};

struct st_mpu_rspage2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage2
{
  unsigned long LONG;
  struct st_mpu_rspage2_bit BIT;
};

struct st_mpu_repage2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage2
{
  unsigned long LONG;
  struct st_mpu_repage2_bit BIT;
};

struct st_mpu_rspage3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage3
{
  unsigned long LONG;
  struct st_mpu_rspage3_bit BIT;
};

struct st_mpu_repage3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage3
{
  unsigned long LONG;
  struct st_mpu_repage3_bit BIT;
};

struct st_mpu_rspage4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage4
{
  unsigned long LONG;
  struct st_mpu_rspage4_bit BIT;
};

struct st_mpu_repage4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage4
{
  unsigned long LONG;
  struct st_mpu_repage4_bit BIT;
};

struct st_mpu_rspage5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage5
{
  unsigned long LONG;
  struct st_mpu_rspage5_bit BIT;
};

struct st_mpu_repage5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage5
{
  unsigned long LONG;
  struct st_mpu_repage5_bit BIT;
};

struct st_mpu_rspage6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage6
{
  unsigned long LONG;
  struct st_mpu_rspage6_bit BIT;
};

struct st_mpu_repage6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage6
{
  unsigned long LONG;
  struct st_mpu_repage6_bit BIT;
};

struct st_mpu_rspage7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RSPN : 28;
#else
  unsigned long RSPN : 28;
  unsigned long  : 4;
#endif
};

union un_mpu_rspage7
{
  unsigned long LONG;
  struct st_mpu_rspage7_bit BIT;
};

struct st_mpu_repage7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long V : 1;
  unsigned long UAC : 3;
  unsigned long REPN : 28;
#else
  unsigned long REPN : 28;
  unsigned long UAC : 3;
  unsigned long V : 1;
#endif
};

union un_mpu_repage7
{
  unsigned long LONG;
  struct st_mpu_repage7_bit BIT;
};

struct st_mpu_mpen_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MPEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long MPEN : 1;
#endif
};

union un_mpu_mpen
{
  unsigned long LONG;
  struct st_mpu_mpen_bit BIT;
};

struct st_mpu_mpbac_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 1;
  unsigned long UBAC : 3;
  unsigned long  : 28;
#else
  unsigned long  : 28;
  unsigned long UBAC : 3;
  unsigned long  : 1;
#endif
};

union un_mpu_mpbac
{
  unsigned long LONG;
  struct st_mpu_mpbac_bit BIT;
};

struct st_mpu_mpeclr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CLR : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long CLR : 1;
#endif
};

union un_mpu_mpeclr
{
  unsigned long LONG;
  struct st_mpu_mpeclr_bit BIT;
};

struct st_mpu_mpests_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IMPER : 1;
  unsigned long DMPER : 1;
  unsigned long DRW : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long DRW : 1;
  unsigned long DMPER : 1;
  unsigned long IMPER : 1;
#endif
};

union un_mpu_mpests
{
  unsigned long LONG;
  struct st_mpu_mpests_bit BIT;
};

struct st_mpu_mpdea_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DEA : 32;
#else
  unsigned long DEA : 32;
#endif
};

union un_mpu_mpdea
{
  unsigned long LONG;
  struct st_mpu_mpdea_bit BIT;
};

struct st_mpu_mpsa_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SA : 32;
#else
  unsigned long SA : 32;
#endif
};

union un_mpu_mpsa
{
  unsigned long LONG;
  struct st_mpu_mpsa_bit BIT;
};

struct st_mpu_mpops_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short S : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short S : 1;
#endif
};

union un_mpu_mpops
{
  unsigned short WORD;
  struct st_mpu_mpops_bit BIT;
};

struct st_mpu_mpopi_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short INV : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short INV : 1;
#endif
};

union un_mpu_mpopi
{
  unsigned short WORD;
  struct st_mpu_mpopi_bit BIT;
};

struct st_mpu_mhiti_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 1;
  unsigned long UHACI : 3;
  unsigned long  : 12;
  unsigned long HITI : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long HITI : 8;
  unsigned long  : 12;
  unsigned long UHACI : 3;
  unsigned long  : 1;
#endif
};

union un_mpu_mhiti
{
  unsigned long LONG;
  struct st_mpu_mhiti_bit BIT;
};

struct st_mpu_mhitd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 1;
  unsigned long UHACD : 3;
  unsigned long  : 12;
  unsigned long HITD : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long HITD : 8;
  unsigned long  : 12;
  unsigned long UHACD : 3;
  unsigned long  : 1;
#endif
};

union un_mpu_mhitd
{
  unsigned long LONG;
  struct st_mpu_mhitd_bit BIT;
};

struct st_mmcif_cecmdset_bit
{
  unsigned long :1;
  unsigned long BOOT:1;
  unsigned long CMD:6;
  unsigned long RTYP:2;
  unsigned long RBSY:1;
  unsigned long :1;
  unsigned long WDAT:1;
  unsigned long DWEN:1;
  unsigned long CMLTE:1;
  unsigned long CMD12EN:1;
  unsigned long RIDXC:2;
  unsigned long RCRC7C:2;
  unsigned long :1;
  unsigned long CRC16C:1;
  unsigned long BOOTACK:1;
  unsigned long CRCSTE:1;
  unsigned long TBIT:1;
  unsigned long OPDM:1;
  unsigned long :2;
  unsigned long SBIT:1;
  unsigned long :1;
  unsigned long DATW:2;
};

union un_mmcif_cecmdset
{
  unsigned long LONG;
  struct st_mmcif_cecmdset_bit BIT;
};

union un_mmcif_cearg
{
  unsigned long LONG;
};

struct st_mmcif_ceargcmd12_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long C12ARG : 32;
#else
  unsigned long C12ARG : 32;
#endif
};

union un_mmcif_ceargcmd12
{
  unsigned long LONG;
  struct st_mmcif_ceargcmd12_bit BIT;
};

struct st_mmcif_cecmdctrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BREAK : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long BREAK : 1;
#endif
};

union un_mmcif_cecmdctrl
{
  unsigned long LONG;
  struct st_mmcif_cecmdctrl_bit BIT;
};

struct st_mmcif_ceblockset_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BLKSIZ : 16;
  unsigned long BLKCNT : 16;
#else
  unsigned long BLKCNT : 16;
  unsigned long BLKSIZ : 16;
#endif
};

union un_mmcif_ceblockset
{
  unsigned long LONG;
  struct st_mmcif_ceblockset_bit BIT;
};

struct st_mmcif_ceclkctrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long SRWDTO : 4;
  unsigned long SRBSYTO : 4;
  unsigned long SRSPTO : 2;
  unsigned long  : 2;
  unsigned long CLKDIV : 4;
  unsigned long  : 4;
  unsigned long CLKEN : 1;
  unsigned long  : 6;
  unsigned long MMCBUSBSY : 1;
#else
  unsigned long MMCBUSBSY : 1;
  unsigned long  : 6;
  unsigned long CLKEN : 1;
  unsigned long  : 4;
  unsigned long CLKDIV : 4;
  unsigned long  : 2;
  unsigned long SRSPTO : 2;
  unsigned long SRBSYTO : 4;
  unsigned long SRWDTO : 4;
  unsigned long  : 4;
#endif
};

union un_mmcif_ceclkctrl
{
  unsigned long LONG;
  struct st_mmcif_ceclkctrl_bit BIT;
};

struct st_mmcif_cebufacc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 16;
  unsigned long ATYP : 1;
  unsigned long  : 7;
  unsigned long DMAREN : 1;
  unsigned long DMAWEN : 1;
  unsigned long DMATYP : 1;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long DMATYP : 1;
  unsigned long DMAWEN : 1;
  unsigned long DMAREN : 1;
  unsigned long  : 7;
  unsigned long ATYP : 1;
  unsigned long  : 16;
#endif
};

union un_mmcif_cebufacc
{
  unsigned long LONG;
  struct st_mmcif_cebufacc_bit BIT;
};

struct st_mmcif_cerespcmd12_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RSP12 : 32;
#else
  unsigned long RSP12 : 32;
#endif
};

union un_mmcif_cerespcmd12
{
  unsigned long LONG;
  struct st_mmcif_cerespcmd12_bit BIT;
};

struct st_mmcif_cedata_bit
{
  unsigned long DATA:32;
};

union un_mmcif_cedata
{
  unsigned long LONG;
  struct st_mmcif_cedata_bit BIT;
};

struct st_mmcif_ceboot_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 16;
  unsigned long SBTDATTO : 4;
  unsigned long SFSTBTDATTO : 4;
  unsigned long SBTACKTO : 4;
  unsigned long SBTCLKDIV : 4;
#else
  unsigned long SBTCLKDIV : 4;
  unsigned long SBTACKTO : 4;
  unsigned long SFSTBTDATTO : 4;
  unsigned long SBTDATTO : 4;
  unsigned long  : 16;
#endif
};

union un_mmcif_ceboot
{
  unsigned long LONG;
  struct st_mmcif_ceboot_bit BIT;
};

struct st_mmcif_ceint_bit
{
  unsigned long :5;
  unsigned long CMD12DRE:1;
  unsigned long CMD12RBE:1;
  unsigned long CMD12CRE:1;
  unsigned long DTRANE:1;
  unsigned long BUFRE:1;
  unsigned long BUFWEN:1;
  unsigned long BUFREN:1;
  unsigned long :2;
  unsigned long RBSYE:1;
  unsigned long CRSPE:1;
  unsigned long CMDVIO:1;
  unsigned long BUFVIO:1;
  unsigned long :2;
  unsigned long WDATERR:1;
  unsigned long RDATERR:1;
  unsigned long RIDXERR:1;
  unsigned long RSPERR:1;
  unsigned long :3;
  unsigned long CRCSTO:1;
  unsigned long WDATTO:1;
  unsigned long RDATTO:1;
  unsigned long RBSYTO:1;
  unsigned long RSPTO:1;
};

union un_mmcif_ceint
{
  unsigned long LONG;
  struct st_mmcif_ceint_bit BIT;
};

struct st_mmcif_ceinten_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MRSPTO : 1;
  unsigned long MRBSYTO : 1;
  unsigned long MRDATTO : 1;
  unsigned long MWDATTO : 1;
  unsigned long MCRCSTO : 1;
  unsigned long  : 3;
  unsigned long MRSPERR : 1;
  unsigned long MRIDXERR : 1;
  unsigned long MRDATERR : 1;
  unsigned long MWDATERR : 1;
  unsigned long  : 2;
  unsigned long MBUFVIO : 1;
  unsigned long MCMDVIO : 1;
  unsigned long MCRSPE : 1;
  unsigned long MRBSYE : 1;
  unsigned long  : 2;
  unsigned long MBUFREN : 1;
  unsigned long MBUFWEN : 1;
  unsigned long MBUFRE : 1;
  unsigned long MDTRANE : 1;
  unsigned long MCMD12CRE : 1;
  unsigned long MCMD12RBE : 1;
  unsigned long MCMD12DRE : 1;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long MCMD12DRE : 1;
  unsigned long MCMD12RBE : 1;
  unsigned long MCMD12CRE : 1;
  unsigned long MDTRANE : 1;
  unsigned long MBUFRE : 1;
  unsigned long MBUFWEN : 1;
  unsigned long MBUFREN : 1;
  unsigned long  : 2;
  unsigned long MRBSYE : 1;
  unsigned long MCRSPE : 1;
  unsigned long MCMDVIO : 1;
  unsigned long MBUFVIO : 1;
  unsigned long  : 2;
  unsigned long MWDATERR : 1;
  unsigned long MRDATERR : 1;
  unsigned long MRIDXERR : 1;
  unsigned long MRSPERR : 1;
  unsigned long  : 3;
  unsigned long MCRCSTO : 1;
  unsigned long MWDATTO : 1;
  unsigned long MRDATTO : 1;
  unsigned long MRBSYTO : 1;
  unsigned long MRSPTO : 1;
#endif
};

union un_mmcif_ceinten
{
  unsigned long LONG;
  struct st_mmcif_ceinten_bit BIT;
};

struct st_mmcif_cehoststs1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RCVBLK : 16;
  unsigned long DATSIG : 8;
  unsigned long RSPIDX : 6;
  unsigned long CMDSIG : 1;
  unsigned long CMDSEQ : 1;
#else
  unsigned long CMDSEQ : 1;
  unsigned long CMDSIG : 1;
  unsigned long RSPIDX : 6;
  unsigned long DATSIG : 8;
  unsigned long RCVBLK : 16;
#endif
};

union un_mmcif_cehoststs1
{
  unsigned long LONG;
  struct st_mmcif_cehoststs1_bit BIT;
};

struct st_mmcif_cehoststs2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 5;
  unsigned long BTDATTO : 1;
  unsigned long FSTBTDATTO : 1;
  unsigned long BTACKTO : 1;
  unsigned long STRSPTO : 1;
  unsigned long AC12RSPTO : 1;
  unsigned long RSPBSYTO : 1;
  unsigned long AC12BSYTO : 1;
  unsigned long CRCSTTO : 1;
  unsigned long DATBSYTO : 1;
  unsigned long STRDATTO : 1;
  unsigned long  : 1;
  unsigned long CRCST : 3;
  unsigned long  : 1;
  unsigned long BTACKEBE : 1;
  unsigned long BTACKPATE : 1;
  unsigned long RSPIDXE : 1;
  unsigned long AC12IDXE : 1;
  unsigned long RSPEBE : 1;
  unsigned long AC12REBE : 1;
  unsigned long RDATEBE : 1;
  unsigned long CRCSTEBE : 1;
  unsigned long RSPCRC7E : 1;
  unsigned long AC12CRCE : 1;
  unsigned long CRC16E : 1;
  unsigned long CRCSTE : 1;
#else
  unsigned long CRCSTE : 1;
  unsigned long CRC16E : 1;
  unsigned long AC12CRCE : 1;
  unsigned long RSPCRC7E : 1;
  unsigned long CRCSTEBE : 1;
  unsigned long RDATEBE : 1;
  unsigned long AC12REBE : 1;
  unsigned long RSPEBE : 1;
  unsigned long AC12IDXE : 1;
  unsigned long RSPIDXE : 1;
  unsigned long BTACKPATE : 1;
  unsigned long BTACKEBE : 1;
  unsigned long  : 1;
  unsigned long CRCST : 3;
  unsigned long  : 1;
  unsigned long STRDATTO : 1;
  unsigned long DATBSYTO : 1;
  unsigned long CRCSTTO : 1;
  unsigned long AC12BSYTO : 1;
  unsigned long RSPBSYTO : 1;
  unsigned long AC12RSPTO : 1;
  unsigned long STRSPTO : 1;
  unsigned long BTACKTO : 1;
  unsigned long FSTBTDATTO : 1;
  unsigned long BTDATTO : 1;
  unsigned long  : 5;
#endif
};

union un_mmcif_cehoststs2
{
  unsigned long LONG;
  struct st_mmcif_cehoststs2_bit BIT;
};

struct st_mmcif_cedetect_bit
{
  unsigned long :17;
  unsigned long CDSIG:1;
  unsigned long CDRISE:1;
  unsigned long CDFALL:1;
  unsigned long :6;
  unsigned long MCDRISE:1;
  unsigned long MCDFALL:1;
  unsigned long :4;
};

union un_mmcif_cedetect
{
  unsigned long LONG;
  struct st_mmcif_cedetect_bit BIT;
};

struct st_mmcif_ceaddmode_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 19;
  unsigned long CLKMAIN : 1;
  unsigned long  : 1;
  unsigned long RESNOUT : 1;
  unsigned long  : 10;
#else
  unsigned long  : 10;
  unsigned long RESNOUT : 1;
  unsigned long  : 1;
  unsigned long CLKMAIN : 1;
  unsigned long  : 19;
#endif
};

union un_mmcif_ceaddmode
{
  unsigned long LONG;
  struct st_mmcif_ceaddmode_bit BIT;
};

struct st_mmcif_ceversion_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VERSION : 16;
  unsigned long  : 15;
  unsigned long SWRST : 1;
#else
  unsigned long SWRST : 1;
  unsigned long  : 15;
  unsigned long VERSION : 16;
#endif
};

union un_mmcif_ceversion
{
  unsigned long LONG;
  struct st_mmcif_ceversion_bit BIT;
};

struct st_glcdc_gr1clut0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long A : 8;
#else
  unsigned long A : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_glcdc_gr1clut0
{
  unsigned long LONG;
  struct st_glcdc_gr1clut0_bit BIT;
};

struct st_glcdc_gr1clut1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long A : 8;
#else
  unsigned long A : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_glcdc_gr1clut1
{
  unsigned long LONG;
  struct st_glcdc_gr1clut1_bit BIT;
};

struct st_glcdc_gr2clut0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long A : 8;
#else
  unsigned long A : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_glcdc_gr2clut0
{
  unsigned long LONG;
  struct st_glcdc_gr2clut0_bit BIT;
};

struct st_glcdc_gr2clut1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long A : 8;
#else
  unsigned long A : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_glcdc_gr2clut1
{
  unsigned long LONG;
  struct st_glcdc_gr2clut1_bit BIT;
};

struct st_glcdc_bgen_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN : 1;
  unsigned long  : 7;
  unsigned long VEN : 1;
  unsigned long  : 7;
  unsigned long SWRST : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long SWRST : 1;
  unsigned long  : 7;
  unsigned long VEN : 1;
  unsigned long  : 7;
  unsigned long EN : 1;
#endif
};

union un_glcdc_bgen
{
  unsigned long LONG;
  struct st_glcdc_bgen_bit BIT;
};

struct st_glcdc_bgperi_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FH : 11;
  unsigned long  : 5;
  unsigned long FV : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long FV : 11;
  unsigned long  : 5;
  unsigned long FH : 11;
#endif
};

union un_glcdc_bgperi
{
  unsigned long LONG;
  struct st_glcdc_bgperi_bit BIT;
};

struct st_glcdc_bgsync_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long HP : 4;
  unsigned long  : 12;
  unsigned long VP : 4;
  unsigned long  : 12;
#else
  unsigned long  : 12;
  unsigned long VP : 4;
  unsigned long  : 12;
  unsigned long HP : 4;
#endif
};

union un_glcdc_bgsync
{
  unsigned long LONG;
  struct st_glcdc_bgsync_bit BIT;
};

struct st_glcdc_bgvsize_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VW : 11;
  unsigned long  : 5;
  unsigned long VP : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long VP : 11;
  unsigned long  : 5;
  unsigned long VW : 11;
#endif
};

union un_glcdc_bgvsize
{
  unsigned long LONG;
  struct st_glcdc_bgvsize_bit BIT;
};

struct st_glcdc_bghsize_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long HW : 11;
  unsigned long  : 5;
  unsigned long HP : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long HP : 11;
  unsigned long  : 5;
  unsigned long HW : 11;
#endif
};

union un_glcdc_bghsize
{
  unsigned long LONG;
  struct st_glcdc_bghsize_bit BIT;
};

struct st_glcdc_bgcolor_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_glcdc_bgcolor
{
  unsigned long LONG;
  struct st_glcdc_bgcolor_bit BIT;
};

struct st_glcdc_bgmon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EN : 1;
  unsigned long  : 7;
  unsigned long VEN : 1;
  unsigned long  : 7;
  unsigned long SWRST : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long SWRST : 1;
  unsigned long  : 7;
  unsigned long VEN : 1;
  unsigned long  : 7;
  unsigned long EN : 1;
#endif
};

union un_glcdc_bgmon
{
  unsigned long LONG;
  struct st_glcdc_bgmon_bit BIT;
};

struct st_glcdc_gr1ven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_gr1ven
{
  unsigned long LONG;
  struct st_glcdc_gr1ven_bit BIT;
};

struct st_glcdc_grlflmrd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RENB : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long RENB : 1;
#endif
};

union un_glcdc_grlflmrd
{
  unsigned long LONG;
  struct st_glcdc_grlflmrd_bit BIT;
};

struct st_glcdc_grlflm3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 16;
  unsigned long LNOFF : 16;
#else
  unsigned long LNOFF : 16;
  unsigned long  : 16;
#endif
};

union un_glcdc_gr1flm3
{
  unsigned long LONG;
  struct st_glcdc_grlflm3_bit BIT;
};

struct st_glcdc_grlflm5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DATANUM : 16;
  unsigned long LNNUM : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long LNNUM : 11;
  unsigned long DATANUM : 16;
#endif
};

union un_glcdc_gr1flm5
{
  unsigned long LONG;
  struct st_glcdc_grlflm5_bit BIT;
};

struct st_glcdc_grlflm6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 28;
  unsigned long FORMAT : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long FORMAT : 3;
  unsigned long  : 28;
#endif
};

union un_glcdc_gr1flm6
{
  unsigned long LONG;
  struct st_glcdc_grlflm6_bit BIT;
};

struct st_glcdc_gr1ab1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DISPSEL : 2;
  unsigned long  : 2;
  unsigned long GRCDISPON : 1;
  unsigned long  : 3;
  unsigned long ARCDISPON : 1;
  unsigned long  : 3;
  unsigned long ARCON : 1;
  unsigned long  : 19;
#else
  unsigned long  : 19;
  unsigned long ARCON : 1;
  unsigned long  : 3;
  unsigned long ARCDISPON : 1;
  unsigned long  : 3;
  unsigned long GRCDISPON : 1;
  unsigned long  : 2;
  unsigned long DISPSEL : 2;
#endif
};

union un_glcdc_gr1ab1
{
  unsigned long LONG;
  struct st_glcdc_gr1ab1_bit BIT;
};

struct st_glcdc_gr1ab2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GRCVW : 11;
  unsigned long  : 5;
  unsigned long GRCVS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GRCVS : 11;
  unsigned long  : 5;
  unsigned long GRCVW : 11;
#endif
};
union un_glcdc_gr1ab2
{
  unsigned long LONG;
  struct st_glcdc_gr1ab2_bit BIT;
};

struct st_glcdc_gr1ab3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GRCHW : 11;
  unsigned long  : 5;
  unsigned long GRCHS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GRCHS : 11;
  unsigned long  : 5;
  unsigned long GRCHW : 11;
#endif
};

union un_glcdc_gr1ab3
{
  unsigned long LONG;
  struct st_glcdc_gr1ab3_bit BIT;
};

struct st_glcdc_gr1ab4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCVW : 11;
  unsigned long  : 5;
  unsigned long ARCVS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long ARCVS : 11;
  unsigned long  : 5;
  unsigned long ARCVW : 11;
#endif
};

union un_glcdc_gr1ab4
{
  unsigned long LONG;
  struct st_glcdc_gr1ab4_bit BIT;
};

struct st_glcdc_gr1ab5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCHW : 11;
  unsigned long  : 5;
  unsigned long ARCHS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long ARCHS : 11;
  unsigned long  : 5;
  unsigned long ARCHW : 11;
#endif
};

union un_glcdc_gr1ab5
{
  unsigned long LONG;
  struct st_glcdc_gr1ab5_bit BIT;
};

struct st_glcdc_gr1ab6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCRATE : 8;
  unsigned long  : 8;
  unsigned long ARCCOEF : 9;
  unsigned long  : 7;
#else
  unsigned long  : 7;
  unsigned long ARCCOEF : 9;
  unsigned long  : 8;
  unsigned long ARCRATE : 8;
#endif
};

union un_glcdc_gr1ab6
{
  unsigned long LONG;
  struct st_glcdc_gr1ab6_bit BIT;
};

struct st_glcdc_gr1ab7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKON : 1;
  unsigned long  : 15;
  unsigned long ARCDEF : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long ARCDEF : 8;
  unsigned long  : 15;
  unsigned long CKON : 1;
#endif
};

union un_glcdc_gr1ab7
{
  unsigned long LONG;
  struct st_glcdc_gr1ab7_bit BIT;
};

struct st_glcdc_gr1ab8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKKR : 8;
  unsigned long CKKB : 8;
  unsigned long CKKG : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long CKKG : 8;
  unsigned long CKKB : 8;
  unsigned long CKKR : 8;
#endif
};

union un_glcdc_gr1ab8
{
  unsigned long LONG;
  struct st_glcdc_gr1ab8_bit BIT;
};

struct st_glcdc_gr1ab9_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKR : 8;
  unsigned long CKB : 8;
  unsigned long CKG : 8;
  unsigned long CKA : 8;
#else
  unsigned long CKA : 8;
  unsigned long CKG : 8;
  unsigned long CKB : 8;
  unsigned long CKR : 8;
#endif
};

union un_glcdc_gr1ab9
{
  unsigned long LONG;
  struct st_glcdc_gr1ab9_bit BIT;
};

struct st_glcdc_gr1base_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long R : 8;
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long G : 8;
  unsigned long B : 8;
  unsigned long R : 8;
#endif
};

union un_glcdc_gr1base
{
  unsigned long LONG;
  struct st_glcdc_gr1base_bit BIT;
};

struct st_glcdc_gr1clutint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LINE : 11;
  unsigned long  : 5;
  unsigned long SEL : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long SEL : 1;
  unsigned long  : 5;
  unsigned long LINE : 11;
#endif
};

union un_glcdc_gr1clutint
{
  unsigned long LONG;
  struct st_glcdc_gr1clutint_bit BIT;
};

struct st_glcdc_gr1mon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCST : 1;
  unsigned long  : 15;
  unsigned long UFST : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long UFST : 1;
  unsigned long  : 15;
  unsigned long ARCST : 1;
#endif
};

union un_glcdc_gr1mon
{
  unsigned long LONG;
  struct st_glcdc_gr1mon_bit BIT;
};

struct st_glcdc_gr2ven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_gr2ven
{
  unsigned long LONG;
  struct st_glcdc_gr2ven_bit BIT;
};

struct st_glcdc_gr2flmrd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RENB : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long RENB : 1;
#endif
};

union un_glcdc_gr2flmrd
{
  unsigned long LONG;
  struct st_glcdc_gr2flmrd_bit BIT;
};

struct st_glcdc_gr2flm3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 16;
  unsigned long LNOFF : 16;
#else
  unsigned long LNOFF : 16;
  unsigned long  : 16;
#endif
};

union un_glcdc_gr2flm3
{
  unsigned long LONG;
  struct st_glcdc_gr2flm3_bit BIT;
};

struct st_glcdc_gr2flm5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DATANUM : 16;
  unsigned long LNNUM : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long LNNUM : 11;
  unsigned long DATANUM : 16;
#endif
};

union un_glcdc_gr2flm5
{
  unsigned long LONG;
  struct st_glcdc_gr2flm5_bit BIT;
};

struct st_glcdc_gr2flm6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 28;
  unsigned long FORMAT : 3;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long FORMAT : 3;
  unsigned long  : 28;
#endif
};

union un_glcdc_gr2flm6
{
  unsigned long LONG;
  struct st_glcdc_gr2flm6_bit BIT;
};

struct st_glcdc_gr2ab1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DISPSEL : 2;
  unsigned long  : 2;
  unsigned long GRCDISPON : 1;
  unsigned long  : 3;
  unsigned long ARCDISPON : 1;
  unsigned long  : 3;
  unsigned long ARCON : 1;
  unsigned long  : 19;
#else
  unsigned long  : 19;
  unsigned long ARCON : 1;
  unsigned long  : 3;
  unsigned long ARCDISPON : 1;
  unsigned long  : 3;
  unsigned long GRCDISPON : 1;
  unsigned long  : 2;
  unsigned long DISPSEL : 2;
#endif
};

union un_glcdc_gr2ab1
{
  unsigned long LONG;
  struct st_glcdc_gr2ab1_bit BIT;
};

struct st_glcdc_gr2ab2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GRCVW : 11;
  unsigned long  : 5;
  unsigned long GRCVS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GRCVS : 11;
  unsigned long  : 5;
  unsigned long GRCVW : 11;
#endif
};

union un_glcdc_gr2ab2
{
  unsigned long LONG;
  struct st_glcdc_gr2ab2_bit BIT;
};

struct st_glcdc_gr2ab3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GRCHW : 11;
  unsigned long  : 5;
  unsigned long GRCHS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GRCHS : 11;
  unsigned long  : 5;
  unsigned long GRCHW : 11;
#endif
};

union un_glcdc_gr2ab3
{
  unsigned long LONG;
  struct st_glcdc_gr2ab3_bit BIT;
};

struct st_glcdc_gr2ab4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCVW : 11;
  unsigned long  : 5;
  unsigned long ARCVS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long ARCVS : 11;
  unsigned long  : 5;
  unsigned long ARCVW : 11;
#endif
};

union un_glcdc_gr2ab4
{
  unsigned long LONG;
  struct st_glcdc_gr2ab4_bit BIT;
};

struct st_glcdc_gr2ab5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCHW : 11;
  unsigned long  : 5;
  unsigned long ARCHS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long ARCHS : 11;
  unsigned long  : 5;
  unsigned long ARCHW : 11;
#endif
};

union un_glcdc_gr2ab5
{
  unsigned long LONG;
  struct st_glcdc_gr2ab5_bit BIT;
};

struct st_glcdc_gr2ab6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCRATE : 8;
  unsigned long  : 8;
  unsigned long ARCCOEF : 9;
  unsigned long  : 7;
#else
  unsigned long  : 7;
  unsigned long ARCCOEF : 9;
  unsigned long  : 8;
  unsigned long ARCRATE : 8;
#endif
};

union un_glcdc_gr2ab6
{
  unsigned long LONG;
  struct st_glcdc_gr2ab6_bit BIT;
};

struct st_glcdc_gr2ab7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKON : 1;
  unsigned long  : 15;
  unsigned long ARCDEF : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long ARCDEF : 8;
  unsigned long  : 15;
  unsigned long CKON : 1;
#endif
};

union un_glcdc_gr2ab7
{
  unsigned long LONG;
  struct st_glcdc_gr2ab7_bit BIT;
};

struct st_glcdc_gr2ab8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKKR : 8;
  unsigned long CKKB : 8;
  unsigned long CKKG : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long CKKG : 8;
  unsigned long CKKB : 8;
  unsigned long CKKR : 8;
#endif
};

union un_glcdc_gr2ab8
{
  unsigned long LONG;
  struct st_glcdc_gr2ab8_bit BIT;
};

struct st_glcdc_gr2ab9_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CKR : 8;
  unsigned long CKB : 8;
  unsigned long CKG : 8;
  unsigned long CKA : 8;
#else
  unsigned long CKA : 8;
  unsigned long CKG : 8;
  unsigned long CKB : 8;
  unsigned long CKR : 8;
#endif
};

union un_glcdc_gr2ab9
{
  unsigned long LONG;
  struct st_glcdc_gr2ab9_bit BIT;
};

struct st_glcdc_gr2base_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long R : 8;
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long G : 8;
  unsigned long B : 8;
  unsigned long R : 8;
#endif
};

union un_glcdc_gr2base
{
  unsigned long LONG;
  struct st_glcdc_gr2base_bit BIT;
};

struct st_glcdc_gr2clutint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LINE : 11;
  unsigned long  : 5;
  unsigned long SEL : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long SEL : 1;
  unsigned long  : 5;
  unsigned long LINE : 11;
#endif
};

union un_glcdc_gr2clutint
{
  unsigned long LONG;
  struct st_glcdc_gr2clutint_bit BIT;
};

struct st_glcdc_gr2mon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ARCST : 1;
  unsigned long  : 15;
  unsigned long UFST : 1;
  unsigned long  : 15;
#else
  unsigned long  : 15;
  unsigned long UFST : 1;
  unsigned long  : 15;
  unsigned long ARCST : 1;
#endif
};

union un_glcdc_gr2mon
{
  unsigned long LONG;
  struct st_glcdc_gr2mon_bit BIT;
};

struct st_glcdc_gamgven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_gamgven
{
  unsigned long LONG;
  struct st_glcdc_gamgven_bit BIT;
};

struct st_glcdc_gamsw_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAMON : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long GAMON : 1;
#endif
};

union un_glcdc_gamsw
{
  unsigned long LONG;
  struct st_glcdc_gamsw_bit BIT;
};

struct st_glcdc_gamglut1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN01 : 11;
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
  unsigned long GAIN01 : 11;
#endif
};

union un_glcdc_gamglut1
{
  unsigned long LONG;
  struct st_glcdc_gamglut1_bit BIT;
};

struct st_glcdc_gamglut2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN03 : 11;
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
  unsigned long GAIN03 : 11;
#endif
};

union un_glcdc_gamglut2
{
  unsigned long LONG;
  struct st_glcdc_gamglut2_bit BIT;
};

struct st_glcdc_gamglut3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN05 : 11;
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
  unsigned long GAIN05 : 11;
#endif
};

union un_glcdc_gamglut3
{
  unsigned long LONG;
  struct st_glcdc_gamglut3_bit BIT;
};

struct st_glcdc_gamglut4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN07 : 11;
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
  unsigned long GAIN07 : 11;
#endif
};

union un_glcdc_gamglut4
{
  unsigned long LONG;
  struct st_glcdc_gamglut4_bit BIT;
};

struct st_glcdc_gamglut5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN09 : 11;
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
  unsigned long GAIN09 : 11;
#endif
};

union un_glcdc_gamglut5
{
  unsigned long LONG;
  struct st_glcdc_gamglut5_bit BIT;
};

struct st_glcdc_gamglut6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN11 : 11;
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
  unsigned long GAIN11 : 11;
#endif
};

union un_glcdc_gamglut6
{
  unsigned long LONG;
  struct st_glcdc_gamglut6_bit BIT;
};

struct st_glcdc_gamglut7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN13 : 11;
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
  unsigned long GAIN13 : 11;
#endif
};

union un_glcdc_gamglut7
{
  unsigned long LONG;
  struct st_glcdc_gamglut7_bit BIT;
};

struct st_glcdc_gamglut8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN15 : 11;
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
  unsigned long GAIN15 : 11;
#endif
};

union un_glcdc_gamglut8
{
  unsigned long LONG;
  struct st_glcdc_gamglut8_bit BIT;
};

struct st_glcdc_gamgarea1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH03 : 10;
  unsigned long TH02 : 10;
  unsigned long TH01 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH01 : 10;
  unsigned long TH02 : 10;
  unsigned long TH03 : 10;
#endif
};

union un_glcdc_gamgarea1
{
  unsigned long LONG;
  struct st_glcdc_gamgarea1_bit BIT;
};

struct st_glcdc_gamgarea2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH06 : 10;
  unsigned long TH05 : 10;
  unsigned long TH04 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH04 : 10;
  unsigned long TH05 : 10;
  unsigned long TH06 : 10;
#endif
};

union un_glcdc_gamgarea2
{
  unsigned long LONG;
  struct st_glcdc_gamgarea2_bit BIT;
};

struct st_glcdc_gamgarea3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH09 : 10;
  unsigned long TH08 : 10;
  unsigned long TH07 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH07 : 10;
  unsigned long TH08 : 10;
  unsigned long TH09 : 10;
#endif
};

union un_glcdc_gamgarea3
{
  unsigned long LONG;
  struct st_glcdc_gamgarea3_bit BIT;
};

struct st_glcdc_gamgarea4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH12 : 10;
  unsigned long TH11 : 10;
  unsigned long TH10 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH10 : 10;
  unsigned long TH11 : 10;
  unsigned long TH12 : 10;
#endif
};

union un_glcdc_gamgarea4
{
  unsigned long LONG;
  struct st_glcdc_gamgarea4_bit BIT;
};

struct st_glcdc_gamgarea5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH15 : 10;
  unsigned long TH14 : 10;
  unsigned long TH13 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH13 : 10;
  unsigned long TH14 : 10;
  unsigned long TH15 : 10;
#endif
};

union un_glcdc_gamgarea5
{
  unsigned long LONG;
  struct st_glcdc_gamgarea5_bit BIT;
};

struct st_glcdc_gambven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_gambven
{
  unsigned long LONG;
  struct st_glcdc_gambven_bit BIT;
};

struct st_glcdc_gamblut1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN01 : 11;
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
  unsigned long GAIN01 : 11;
#endif
};

union un_glcdc_gamblut1
{
  unsigned long LONG;
  struct st_glcdc_gamblut1_bit BIT;
};

struct st_glcdc_gamblut2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN03 : 11;
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
  unsigned long GAIN03 : 11;
#endif
};

union un_glcdc_gamblut2
{
  unsigned long LONG;
  struct st_glcdc_gamblut2_bit BIT;
};

struct st_glcdc_gamblut3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN05 : 11;
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
  unsigned long GAIN05 : 11;
#endif
};

union un_glcdc_gamblut3
{
  unsigned long LONG;
  struct st_glcdc_gamblut3_bit BIT;
};

struct st_glcdc_gamblut4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN07 : 11;
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
  unsigned long GAIN07 : 11;
#endif
};

union un_glcdc_gamblut4
{
  unsigned long LONG;
  struct st_glcdc_gamblut4_bit BIT;
};

struct st_glcdc_gamblut5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN09 : 11;
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
  unsigned long GAIN09 : 11;
#endif
};

union un_glcdc_gamblut5
{
  unsigned long LONG;
  struct st_glcdc_gamblut5_bit BIT;
};

struct st_glcdc_gamblut6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN11 : 11;
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
  unsigned long GAIN11 : 11;
#endif
};

union un_glcdc_gamblut6
{
  unsigned long LONG;
  struct st_glcdc_gamblut6_bit BIT;
};

struct st_glcdc_gamblut7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN13 : 11;
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
  unsigned long GAIN13 : 11;
#endif
};

union un_glcdc_gamblut7
{
  unsigned long LONG;
  struct st_glcdc_gamblut7_bit BIT;
};

struct st_glcdc_gamblut8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN15 : 11;
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
  unsigned long GAIN15 : 11;
#endif
};

union un_glcdc_gamblut8
{
  unsigned long LONG;
  struct st_glcdc_gamblut8_bit BIT;
};

struct st_glcdc_gambarea1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH03 : 10;
  unsigned long TH02 : 10;
  unsigned long TH01 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH01 : 10;
  unsigned long TH02 : 10;
  unsigned long TH03 : 10;
#endif
};

union un_glcdc_gambarea1
{
  unsigned long LONG;
  struct st_glcdc_gambarea1_bit BIT;
};

struct st_glcdc_gambarea2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH06 : 10;
  unsigned long TH05 : 10;
  unsigned long TH04 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH04 : 10;
  unsigned long TH05 : 10;
  unsigned long TH06 : 10;
#endif
};

union un_glcdc_gambarea2
{
  unsigned long LONG;
  struct st_glcdc_gambarea2_bit BIT;
};

struct st_glcdc_gambarea3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH09 : 10;
  unsigned long TH08 : 10;
  unsigned long TH07 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH07 : 10;
  unsigned long TH08 : 10;
  unsigned long TH09 : 10;
#endif
};

union un_glcdc_gambarea3
{
  unsigned long LONG;
  struct st_glcdc_gambarea3_bit BIT;
};

struct st_glcdc_gambarea4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH12 : 10;
  unsigned long TH11 : 10;
  unsigned long TH10 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH10 : 10;
  unsigned long TH11 : 10;
  unsigned long TH12 : 10;
#endif
};

union un_glcdc_gambarea4
{
  unsigned long LONG;
  struct st_glcdc_gambarea4_bit BIT;
};

struct st_glcdc_gambarea5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH15 : 10;
  unsigned long TH14 : 10;
  unsigned long TH13 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH13 : 10;
  unsigned long TH14 : 10;
  unsigned long TH15 : 10;
#endif
};

union un_glcdc_gambarea5
{
  unsigned long LONG;
  struct st_glcdc_gambarea5_bit BIT;
};

struct st_glcdc_gamrven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_gamrven
{
  unsigned long LONG;
  struct st_glcdc_gamrven_bit BIT;
};

struct st_glcdc_gamrlut1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN01 : 11;
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN00 : 11;
  unsigned long  : 5;
  unsigned long GAIN01 : 11;
#endif
};

union un_glcdc_gamrlut1
{
  unsigned long LONG;
  struct st_glcdc_gamrlut1_bit BIT;
};

struct st_glcdc_gamrlut2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN03 : 11;
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN02 : 11;
  unsigned long  : 5;
  unsigned long GAIN03 : 11;
#endif
};

union un_glcdc_gamrlut2
{
  unsigned long LONG;
  struct st_glcdc_gamrlut2_bit BIT;
};
struct st_glcdc_gamrlut3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN05 : 11;
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN04 : 11;
  unsigned long  : 5;
  unsigned long GAIN05 : 11;
#endif
};

union un_glcdc_gamrlut3
{
  unsigned long LONG;
  struct st_glcdc_gamrlut3_bit BIT;
};

struct st_glcdc_gamrlut4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN07 : 11;
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN06 : 11;
  unsigned long  : 5;
  unsigned long GAIN07 : 11;
#endif
};

union un_glcdc_gamrlut4
{
  unsigned long LONG;
  struct st_glcdc_gamrlut4_bit BIT;
};
struct st_glcdc_gamrlut5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN09 : 11;
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN08 : 11;
  unsigned long  : 5;
  unsigned long GAIN09 : 11;
#endif
};

union un_glcdc_gamrlut5
{
  unsigned long LONG;
  struct st_glcdc_gamrlut5_bit BIT;
};

struct st_glcdc_gamrlut6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN11 : 11;
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN10 : 11;
  unsigned long  : 5;
  unsigned long GAIN11 : 11;
#endif
};

union un_glcdc_gamrlut6
{
  unsigned long LONG;
  struct st_glcdc_gamrlut6_bit BIT;
};

struct st_glcdc_gamrlut7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN13 : 11;
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN12 : 11;
  unsigned long  : 5;
  unsigned long GAIN13 : 11;
#endif
};

union un_glcdc_gamrlut7
{
  unsigned long LONG;
  struct st_glcdc_gamrlut7_bit BIT;
};

struct st_glcdc_gamrlut8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long GAIN15 : 11;
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long GAIN14 : 11;
  unsigned long  : 5;
  unsigned long GAIN15 : 11;
#endif
};

union un_glcdc_gamrlut8
{
  unsigned long LONG;
  struct st_glcdc_gamrlut8_bit BIT;
};

struct st_glcdc_gamrarea1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH03 : 10;
  unsigned long TH02 : 10;
  unsigned long TH01 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH01 : 10;
  unsigned long TH02 : 10;
  unsigned long TH03 : 10;
#endif
};

union un_glcdc_gamrarea1
{
  unsigned long LONG;
  struct st_glcdc_gamrarea1_bit BIT;
};

struct st_glcdc_gamrarea2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH06 : 10;
  unsigned long TH05 : 10;
  unsigned long TH04 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH04 : 10;
  unsigned long TH05 : 10;
  unsigned long TH06 : 10;
#endif
};

union un_glcdc_gamrarea2
{
  unsigned long LONG;
  struct st_glcdc_gamrarea2_bit BIT;
};

struct st_glcdc_gamrarea3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH09 : 10;
  unsigned long TH08 : 10;
  unsigned long TH07 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH07 : 10;
  unsigned long TH08 : 10;
  unsigned long TH09 : 10;
#endif
};

union un_glcdc_gamrarea3
{
  unsigned long LONG;
  struct st_glcdc_gamrarea3_bit BIT;
};

struct st_glcdc_gamrarea4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH12 : 10;
  unsigned long TH11 : 10;
  unsigned long TH10 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH10 : 10;
  unsigned long TH11 : 10;
  unsigned long TH12 : 10;
#endif
};

union un_glcdc_gamrarea4
{
  unsigned long LONG;
  struct st_glcdc_gamrarea4_bit BIT;
};

struct st_glcdc_gamrarea5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TH15 : 10;
  unsigned long TH14 : 10;
  unsigned long TH13 : 10;
  unsigned long  : 2;
#else
  unsigned long  : 2;
  unsigned long TH13 : 10;
  unsigned long TH14 : 10;
  unsigned long TH15 : 10;
#endif
};

union un_glcdc_gamrarea5
{
  unsigned long LONG;
  struct st_glcdc_gamrarea5_bit BIT;
};

struct st_glcdc_outven_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VEN : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long VEN : 1;
#endif
};

union un_glcdc_outven
{
  unsigned long LONG;
  struct st_glcdc_outven_bit BIT;
};

struct st_glcdc_outset_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PHASE : 2;
  unsigned long  : 2;
  unsigned long DIRSEL : 1;
  unsigned long  : 4;
  unsigned long FRQSEL : 1;
  unsigned long  : 2;
  unsigned long FORMAT : 2;
  unsigned long  : 10;
  unsigned long SWAPON : 1;
  unsigned long  : 3;
  unsigned long ENDIANON : 1;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long ENDIANON : 1;
  unsigned long  : 3;
  unsigned long SWAPON : 1;
  unsigned long  : 10;
  unsigned long FORMAT : 2;
  unsigned long  : 2;
  unsigned long FRQSEL : 1;
  unsigned long  : 4;
  unsigned long DIRSEL : 1;
  unsigned long  : 2;
  unsigned long PHASE : 2;
#endif
};

union un_glcdc_outset
{
  unsigned long LONG;
  struct st_glcdc_outset_bit BIT;
};

struct st_glcdc_bright1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BRTG : 10;
  unsigned long  : 22;
#else
  unsigned long  : 22;
  unsigned long BRTG : 10;
#endif
};

union un_glcdc_bright1
{
  unsigned long LONG;
  struct st_glcdc_bright1_bit BIT;
};

struct st_glcdc_bright2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BRTR : 10;
  unsigned long  : 6;
  unsigned long BRTB : 10;
  unsigned long  : 6;
#else
  unsigned long  : 6;
  unsigned long BRTB : 10;
  unsigned long  : 6;
  unsigned long BRTR : 10;
#endif
};

union un_glcdc_bright2
{
  unsigned long LONG;
  struct st_glcdc_bright2_bit BIT;
};

struct st_glcdc_contrast_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CONTR : 8;
  unsigned long CONTB : 8;
  unsigned long CONTG : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long CONTG : 8;
  unsigned long CONTB : 8;
  unsigned long CONTR : 8;
#endif
};

union un_glcdc_contrast
{
  unsigned long LONG;
  struct st_glcdc_contrast_bit BIT;
};

struct st_glcdc_paneldtha_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PD : 2;
  unsigned long  : 2;
  unsigned long PC : 2;
  unsigned long  : 2;
  unsigned long PB : 2;
  unsigned long  : 2;
  unsigned long PA : 2;
  unsigned long  : 2;
  unsigned long FORM : 2;
  unsigned long  : 2;
  unsigned long SEL : 2;
  unsigned long  : 10;
#else
  unsigned long  : 10;
  unsigned long SEL : 2;
  unsigned long  : 2;
  unsigned long FORM : 2;
  unsigned long  : 2;
  unsigned long PA : 2;
  unsigned long  : 2;
  unsigned long PB : 2;
  unsigned long  : 2;
  unsigned long PC : 2;
  unsigned long  : 2;
  unsigned long PD : 2;
#endif
};

union un_glcdc_paneldtha
{
  unsigned long LONG;
  struct st_glcdc_paneldtha_bit BIT;
};

struct st_glcdc_clkphase_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 3;
  unsigned long TCON3EDG : 1;
  unsigned long TCON2EDG : 1;
  unsigned long TCON1EDG : 1;
  unsigned long TCON0EDG : 1;
  unsigned long  : 1;
  unsigned long LCDEDG : 1;
  unsigned long  : 3;
  unsigned long FRONTGAM : 1;
  unsigned long  : 19;
#else
  unsigned long  : 19;
  unsigned long FRONTGAM : 1;
  unsigned long  : 3;
  unsigned long LCDEDG : 1;
  unsigned long  : 1;
  unsigned long TCON0EDG : 1;
  unsigned long TCON1EDG : 1;
  unsigned long TCON2EDG : 1;
  unsigned long TCON3EDG : 1;
  unsigned long  : 3;
#endif
};

union un_glcdc_clkphase
{
  unsigned long LONG;
  struct st_glcdc_clkphase_bit BIT;
};

struct st_glcdc_tcontim_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long OFFSET : 11;
  unsigned long  : 5;
  unsigned long HALF : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long HALF : 11;
  unsigned long  : 5;
  unsigned long OFFSET : 11;
#endif
};

union un_glcdc_tcontim
{
  unsigned long LONG;
  struct st_glcdc_tcontim_bit BIT;
};

struct st_glcdc_tconstva1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VW : 11;
  unsigned long  : 5;
  unsigned long VS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long VS : 11;
  unsigned long  : 5;
  unsigned long VW : 11;
#endif
};

union un_glcdc_tconstva1
{
  unsigned long LONG;
  struct st_glcdc_tconstva1_bit BIT;
};

struct st_glcdc_tconstvat2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SEL : 3;
  unsigned long  : 1;
  unsigned long INV : 1;
  unsigned long  : 27;
#else
  unsigned long  : 27;
  unsigned long INV : 1;
  unsigned long  : 1;
  unsigned long SEL : 3;
#endif
};

union un_glcdc_tconstvat2
{
  unsigned long LONG;
  struct st_glcdc_tconstvat2_bit BIT;
};

struct st_glcdc_tconstvb1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VW : 11;
  unsigned long  : 5;
  unsigned long VS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long VS : 11;
  unsigned long  : 5;
  unsigned long VW : 11;
#endif
};

union un_glcdc_tconstvb1
{
  unsigned long LONG;
  struct st_glcdc_tconstvb1_bit BIT;
};

struct st_glcdc_tconstvb2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SEL : 3;
  unsigned long  : 1;
  unsigned long INV : 1;
  unsigned long  : 27;
#else
  unsigned long  : 27;
  unsigned long INV : 1;
  unsigned long  : 1;
  unsigned long SEL : 3;
#endif
};

union un_glcdc_tconstvb2
{
  unsigned long LONG;
  struct st_glcdc_tconstvb2_bit BIT;
};

struct st_glcdc_tconstha1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long HW : 11;
  unsigned long  : 5;
  unsigned long HS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long HS : 11;
  unsigned long  : 5;
  unsigned long HW : 11;
#endif
};

union un_glcdc_tconstha1
{
  unsigned long LONG;
  struct st_glcdc_tconstha1_bit BIT;
};

struct st_glcdc_tconstha2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SEL : 3;
  unsigned long  : 1;
  unsigned long INV : 1;
  unsigned long  : 3;
  unsigned long HSSEL : 1;
  unsigned long  : 23;
#else
  unsigned long  : 23;
  unsigned long HSSEL : 1;
  unsigned long  : 3;
  unsigned long INV : 1;
  unsigned long  : 1;
  unsigned long SEL : 3;
#endif
};

union un_glcdc_tconstha2
{
  unsigned long LONG;
  struct st_glcdc_tconstha2_bit BIT;
};

struct st_glcdc_tconsthb1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long HW : 11;
  unsigned long  : 5;
  unsigned long HS : 11;
  unsigned long  : 5;
#else
  unsigned long  : 5;
  unsigned long HS : 11;
  unsigned long  : 5;
  unsigned long HW : 11;
#endif
};

union un_glcdc_tconsthb1
{
  unsigned long LONG;
  struct st_glcdc_tconsthb1_bit BIT;
};

struct st_glcdc_tconsthb2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SEL : 3;
  unsigned long  : 1;
  unsigned long INV : 1;
  unsigned long  : 3;
  unsigned long HSSEL : 1;
  unsigned long  : 23;
#else
  unsigned long  : 23;
  unsigned long HSSEL : 1;
  unsigned long  : 3;
  unsigned long INV : 1;
  unsigned long  : 1;
  unsigned long SEL : 3;
#endif
};

union un_glcdc_tconsthb2
{
  unsigned long LONG;
  struct st_glcdc_tconsthb2_bit BIT;
};

struct st_glcdc_tconde_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long INV : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long INV : 1;
#endif
};

union un_glcdc_tconde
{
  unsigned long LONG;
  struct st_glcdc_tconde_bit BIT;
};

struct st_glcdc_dtcten_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VPOSDTC : 1;
  unsigned long GR1UFDTC : 1;
  unsigned long GR2UFDTC : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long GR2UFDTC : 1;
  unsigned long GR1UFDTC : 1;
  unsigned long VPOSDTC : 1;
#endif
};

union un_glcdc_dtcten
{
  unsigned long LONG;
  struct st_glcdc_dtcten_bit BIT;
};

struct st_glcdc_inten_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VPOSINTEN : 1;
  unsigned long GR1UFINTEN : 1;
  unsigned long GR2UFINTEN : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long GR2UFINTEN : 1;
  unsigned long GR1UFINTEN : 1;
  unsigned long VPOSINTEN : 1;
#endif
};

union un_glcdc_inten
{
  unsigned long LONG;
  struct st_glcdc_inten_bit BIT;
};

struct st_glcdc_stclr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VPOSCLR : 1;
  unsigned long GR1UFCLR : 1;
  unsigned long GR2UFCLR : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long GR2UFCLR : 1;
  unsigned long GR1UFCLR : 1;
  unsigned long VPOSCLR : 1;
#endif
};

union un_glcdc_stclr
{
  unsigned long LONG;
  struct st_glcdc_stclr_bit BIT;
};

struct st_glcdc_stmon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VPOS : 1;
  unsigned long GR1UF : 1;
  unsigned long GR2UF : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long GR2UF : 1;
  unsigned long GR1UF : 1;
  unsigned long VPOS : 1;
#endif
};

union un_glcdc_stmon
{
  unsigned long LONG;
  struct st_glcdc_stmon_bit BIT;
};

struct st_glcdc_panelclk_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DCDR : 6;
  unsigned long CLKEN : 1;
  unsigned long  : 1;
  unsigned long CLKSEL : 1;
  unsigned long  : 3;
  unsigned long PIXSEL : 1;
  unsigned long  : 19;
#else
  unsigned long  : 19;
  unsigned long PIXSEL : 1;
  unsigned long  : 3;
  unsigned long CLKSEL : 1;
  unsigned long  : 1;
  unsigned long CLKEN : 1;
  unsigned long DCDR : 6;
#endif
};

union un_glcdc_panelclk
{
  unsigned long LONG;
  struct st_glcdc_panelclk_bit BIT;
};

struct st_mtu4_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu4_tcr
{
  unsigned char BYTE;
  struct st_mtu4_tcr_bit BIT;
};

struct st_mtu4_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu4_tmdr1
{
  unsigned char BYTE;
  struct st_mtu4_tmdr1_bit BIT;
};

struct st_mtu4_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu4_tiorh
{
  unsigned char BYTE;
  struct st_mtu4_tiorh_bit BIT;
};

struct st_mtu4_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu4_tiorl
{
  unsigned char BYTE;
  struct st_mtu4_tiorl_bit BIT;
};

struct st_mtu4_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 1;
  unsigned char TTGE2 : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char TTGE2 : 1;
  unsigned char  : 1;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu4_tier
{
  unsigned char BYTE;
  struct st_mtu4_tier_bit BIT;
};

struct st_mtu4_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};

union un_mtu4_tsr
{
  unsigned char BYTE;
  struct st_mtu4_tsr_bit BIT;
};

struct st_mtu4_tbtm_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TTSA : 1;
  unsigned char TTSB : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TTSB : 1;
  unsigned char TTSA : 1;
#endif
};

union un_mtu4_tbtm
{
  unsigned char BYTE;
  struct st_mtu4_tbtm_bit BIT;
};

struct st_mtu4_tadcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ITB4VE : 1;
  unsigned short ITB3AE : 1;
  unsigned short ITA4VE : 1;
  unsigned short ITA3AE : 1;
  unsigned short DT4BE : 1;
  unsigned short UT4BE : 1;
  unsigned short DT4AE : 1;
  unsigned short UT4AE : 1;
  unsigned short  : 6;
  unsigned short BF : 2;
#else
  unsigned short BF : 2;
  unsigned short  : 6;
  unsigned short UT4AE : 1;
  unsigned short DT4AE : 1;
  unsigned short UT4BE : 1;
  unsigned short DT4BE : 1;
  unsigned short ITA3AE : 1;
  unsigned short ITA4VE : 1;
  unsigned short ITB3AE : 1;
  unsigned short ITB4VE : 1;
#endif
};

union un_mtu4_tadcr
{
  unsigned char BYTE;
  struct st_mtu4_tadcr_bit BIT;
};

struct st_mtu4_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu4_tcr2
{
  unsigned char BYTE;
  struct st_mtu4_tcr2_bit BIT;
};

struct st_mtu4_nfcr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu4_nfcr4
{
  unsigned char BYTE;
  struct st_mtu4_nfcr4_bit BIT;
};

struct st_mtu5_nfcr5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFUEN : 1;
  unsigned char NFVEN : 1;
  unsigned char NFWEN : 1;
  unsigned char  : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char  : 1;
  unsigned char NFWEN : 1;
  unsigned char NFVEN : 1;
  unsigned char NFUEN : 1;
#endif
};

union un_mtu5_nfcr5
{
  unsigned char BYTE;
  struct st_mtu5_nfcr5_bit BIT;
};

struct st_mtu5_tcru_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TPSC : 2;
#endif
};

union un_mtu5_tcru
{
  unsigned char BYTE;
  struct st_mtu5_tcru_bit BIT;
};

struct st_mtu5_tcr2u_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char CKEG : 2;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu5_tcr2u
{
  unsigned char BYTE;
  struct st_mtu5_tcr2u_bit BIT;
};

struct st_mtu5_tioru_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char IOC : 5;
#endif
};

union un_mtu5_tioru
{
  unsigned char BYTE;
  struct st_mtu5_tioru_bit BIT;
};

struct st_mtu5_tcrv_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TPSC : 2;
#endif
};

union un_mtu5_tcrv
{
  unsigned char BYTE;
  struct st_mtu5_tcrv_bit BIT;
};

struct st_mtu5_tcr2v_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char CKEG : 2;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu5_tcr2v
{
  unsigned char BYTE;
  struct st_mtu5_tcr2v_bit BIT;
};

struct st_mtu5_tiorv_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char IOC : 5;
#endif
};

union un_mtu5_tiorv
{
  unsigned char BYTE;
  struct st_mtu5_tiorv_bit BIT;
};

struct st_mtu5_tcrw_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TPSC : 2;
#endif
};

union un_mtu5_tcrw
{
  unsigned char BYTE;
  struct st_mtu5_tcrw_bit BIT;
};

struct st_mtu5_tcr2w_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char CKEG : 2;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu5_tcr2w
{
  unsigned char BYTE;
  struct st_mtu5_tcr2w_bit BIT;
};

struct st_mtu5_tiorw_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char IOC : 5;
#endif
};

union un_mtu5_tiorw
{
  unsigned char BYTE;
  struct st_mtu5_tiorw_bit BIT;
};

struct st_mtu5_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIE5W : 1;
  unsigned char TGIE5V : 1;
  unsigned char TGIE5U : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TGIE5U : 1;
  unsigned char TGIE5V : 1;
  unsigned char TGIE5W : 1;
#endif
};

union un_mtu5_tier
{
  unsigned char BYTE;
  struct st_mtu5_tier_bit BIT;
};

struct st_mtu5_tstr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CSTW5 : 1;
  unsigned char CSTV5 : 1;
  unsigned char CSTU5 : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char CSTU5 : 1;
  unsigned char CSTV5 : 1;
  unsigned char CSTW5 : 1;
#endif
};

union un_mtu5_tstr
{
  unsigned char BYTE;
  struct st_mtu5_tstr_bit BIT;
};

struct st_mtu5_tcntcmpclr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPCLR5W : 1;
  unsigned char CMPCLR5V : 1;
  unsigned char CMPCLR5U : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char CMPCLR5U : 1;
  unsigned char CMPCLR5V : 1;
  unsigned char CMPCLR5W : 1;
#endif
};

union un_mtu5_tcntcmpclr
{
  unsigned char BYTE;
  struct st_mtu5_tcntcmpclr_bit BIT;
};

struct st_smci0_smr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 2;
  unsigned char BCP : 2;
  unsigned char PM : 1;
  unsigned char PE : 1;
  unsigned char BLK : 1;
  unsigned char GM : 1;
#else
  unsigned char GM : 1;
  unsigned char BLK : 1;
  unsigned char PE : 1;
  unsigned char PM : 1;
  unsigned char BCP : 2;
  unsigned char CKS : 2;
#endif
};

union un_smcio_smr
{
  unsigned char BYTE;
  struct st_smci0_smr_bit BIT;
};

struct st_smci0_scr_bit
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

union un_smcio_scr
{
  unsigned char BYTE;
  struct st_smci0_scr_bit BIT;
};
struct st_smci0_ssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MPBT : 1;
  unsigned char MPB : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char ERS : 1;
  unsigned char ORER : 1;
  unsigned char RDRF : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char RDRF : 1;
  unsigned char ORER : 1;
  unsigned char ERS : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char MPB : 1;
  unsigned char MPBT : 1;
#endif
};

union un_smcio_ssr
{
  unsigned char BYTE;
  struct st_smci0_ssr_bit BIT;
};

struct st_smci0_smcr_bit
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

union un_smcio_smcr
{
  unsigned char BYTE;
  struct st_smci0_smcr_bit BIT;
};

struct st_riic_iccr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDAI : 1;
  unsigned char SCLI : 1;
  unsigned char SDAO : 1;
  unsigned char SCLO : 1;
  unsigned char SOWP : 1;
  unsigned char CLO : 1;
  unsigned char IICRST : 1;
  unsigned char ICE : 1;
#else
  unsigned char ICE : 1;
  unsigned char IICRST : 1;
  unsigned char CLO : 1;
  unsigned char SOWP : 1;
  unsigned char SCLO : 1;
  unsigned char SDAO : 1;
  unsigned char SCLI : 1;
  unsigned char SDAI : 1;
#endif
};

union un_riic_iccr1
{
  unsigned char BYTE;
  struct st_riic_iccr1_bit BIT;
};

struct st_riic_iccr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char ST : 1;
  unsigned char RS : 1;
  unsigned char SP : 1;
  unsigned char  : 1;
  unsigned char TRS : 1;
  unsigned char MST : 1;
  unsigned char BBSY : 1;
#else
  unsigned char BBSY : 1;
  unsigned char MST : 1;
  unsigned char TRS : 1;
  unsigned char  : 1;
  unsigned char SP : 1;
  unsigned char RS : 1;
  unsigned char ST : 1;
  unsigned char  : 1;
#endif
};

union un_riic_iccr2
{
  unsigned char BYTE;
  struct st_riic_iccr2_bit BIT;
};

struct st_riic_icmr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BC : 3;
  unsigned char BCWP : 1;
  unsigned char CKS : 3;
  unsigned char MTWP : 1;
#else
  unsigned char MTWP : 1;
  unsigned char CKS : 3;
  unsigned char BCWP : 1;
  unsigned char BC : 3;
#endif
};

union un_riic_icmr1
{
  unsigned char BYTE;
  struct st_riic_icmr1_bit BIT;
};

struct st_riic_icmr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TMOS : 1;
  unsigned char TMOL : 1;
  unsigned char TMOH : 1;
  unsigned char  : 1;
  unsigned char SDDL : 3;
  unsigned char DLCS : 1;
#else
  unsigned char DLCS : 1;
  unsigned char SDDL : 3;
  unsigned char  : 1;
  unsigned char TMOH : 1;
  unsigned char TMOL : 1;
  unsigned char TMOS : 1;
#endif
};

union un_riic_icmr2
{
  unsigned char BYTE;
  struct st_riic_icmr2_bit BIT;
};

struct st_riic_icmr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NF : 2;
  unsigned char ACKBR : 1;
  unsigned char ACKBT : 1;
  unsigned char ACKWP : 1;
  unsigned char RDRFS : 1;
  unsigned char WAIT : 1;
  unsigned char SMBS : 1;
#else
  unsigned char SMBS : 1;
  unsigned char WAIT : 1;
  unsigned char RDRFS : 1;
  unsigned char ACKWP : 1;
  unsigned char ACKBT : 1;
  unsigned char ACKBR : 1;
  unsigned char NF : 2;
#endif
};

union un_riic_icmr3
{
  unsigned char BYTE;
  struct st_riic_icmr3_bit BIT;
};

struct st_riic_icfer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TMOE : 1;
  unsigned char MALE : 1;
  unsigned char NALE : 1;
  unsigned char SALE : 1;
  unsigned char NACKE : 1;
  unsigned char NFE : 1;
  unsigned char SCLE : 1;
  unsigned char FMPE : 1;
#else
  unsigned char FMPE : 1;
  unsigned char SCLE : 1;
  unsigned char NFE : 1;
  unsigned char NACKE : 1;
  unsigned char SALE : 1;
  unsigned char NALE : 1;
  unsigned char MALE : 1;
  unsigned char TMOE : 1;
#endif
};

union un_riic_icfer
{
  unsigned char BYTE;
  struct st_riic_icfer_bit BIT;
};

struct st_riic_icser_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SAR0E : 1;
  unsigned char SAR1E : 1;
  unsigned char SAR2E : 1;
  unsigned char GCAE : 1;
  unsigned char  : 1;
  unsigned char DIDE : 1;
  unsigned char  : 1;
  unsigned char HOAE : 1;
#else
  unsigned char HOAE : 1;
  unsigned char  : 1;
  unsigned char DIDE : 1;
  unsigned char  : 1;
  unsigned char GCAE : 1;
  unsigned char SAR2E : 1;
  unsigned char SAR1E : 1;
  unsigned char SAR0E : 1;
#endif
};

union un_riic_icser
{
  unsigned char BYTE;
  struct st_riic_icser_bit BIT;
};

struct st_riic_icier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TMOIE : 1;
  unsigned char ALIE : 1;
  unsigned char STIE : 1;
  unsigned char SPIE : 1;
  unsigned char NAKIE : 1;
  unsigned char RIE : 1;
  unsigned char TEIE : 1;
  unsigned char TIE : 1;
#else
  unsigned char TIE : 1;
  unsigned char TEIE : 1;
  unsigned char RIE : 1;
  unsigned char NAKIE : 1;
  unsigned char SPIE : 1;
  unsigned char STIE : 1;
  unsigned char ALIE : 1;
  unsigned char TMOIE : 1;
#endif
};

union un_riic_icier
{
  unsigned char BYTE;
  struct st_riic_icier_bit BIT;
};

struct st_riic_icsr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char AAS0 : 1;
  unsigned char AAS1 : 1;
  unsigned char AAS2 : 1;
  unsigned char GCA : 1;
  unsigned char  : 1;
  unsigned char DID : 1;
  unsigned char  : 1;
  unsigned char HOA : 1;
#else
  unsigned char HOA : 1;
  unsigned char  : 1;
  unsigned char DID : 1;
  unsigned char  : 1;
  unsigned char GCA : 1;
  unsigned char AAS2 : 1;
  unsigned char AAS1 : 1;
  unsigned char AAS0 : 1;
#endif
};

union un_riic_icsr1
{
  unsigned char BYTE;
  struct st_riic_icsr1_bit BIT;
};

struct st_riic_icsr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TMOF : 1;
  unsigned char AL : 1;
  unsigned char START : 1;
  unsigned char STOP : 1;
  unsigned char NACKF : 1;
  unsigned char RDRF : 1;
  unsigned char TEND : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char TEND : 1;
  unsigned char RDRF : 1;
  unsigned char NACKF : 1;
  unsigned char STOP : 1;
  unsigned char START : 1;
  unsigned char AL : 1;
  unsigned char TMOF : 1;
#endif
};

union un_riic_icsr2
{
  unsigned char BYTE;
  struct st_riic_icsr2_bit BIT;
};

struct st_riic_sarl0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SVA0 : 1;
  unsigned char SVA : 7;
#else
  unsigned char SVA : 7;
  unsigned char SVA0 : 1;
#endif
};

union un_riic_sarl0
{
  unsigned char BYTE;
  struct st_riic_sarl0_bit BIT;
};

struct st_riic_saru0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FS : 1;
  unsigned char SVA : 2;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SVA : 2;
  unsigned char FS : 1;
#endif
};

union un_riic_saru0
{
  unsigned char BYTE;
  struct st_riic_saru0_bit BIT;
};

struct st_riic_sarl1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SVA0 : 1;
  unsigned char SVA : 7;
#else
  unsigned char SVA : 7;
  unsigned char SVA0 : 1;
#endif
};

union un_riic_sarl1
{
  unsigned char BYTE;
  struct st_riic_sarl1_bit BIT;
};

struct st_riic_saru1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FS : 1;
  unsigned char SVA : 2;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SVA : 2;
  unsigned char FS : 1;
#endif
};

union un_riic_saru1
{
  unsigned char BYTE;
  struct st_riic_saru1_bit BIT;
};

struct st_riic_sarl2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SVA0 : 1;
  unsigned char SVA : 7;
#else
  unsigned char SVA : 7;
  unsigned char SVA0 : 1;
#endif
};

union un_riic_sarl2
{
  unsigned char BYTE;
  struct st_riic_sarl2_bit BIT;
};

struct st_riic_saru2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FS : 1;
  unsigned char SVA : 2;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SVA : 2;
  unsigned char FS : 1;
#endif
};

union un_riic_saru2
{
  unsigned char BYTE;
  struct st_riic_saru2_bit BIT;
};

struct st_riic_icbrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BRL : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char BRL : 5;
#endif
};

union un_riic_icbrl
{
  unsigned char BYTE;
  struct st_riic_icbrl_bit BIT;
};

struct st_riic_icbrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BRH : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char BRH : 5;
#endif
};

union un_riic_icbrh
{
  unsigned char BYTE;
  struct st_riic_icbrh_bit BIT;
};

struct st_rspi_spcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPMS : 1;
  unsigned char TXMD : 1;
  unsigned char MODFEN : 1;
  unsigned char MSTR : 1;
  unsigned char SPEIE : 1;
  unsigned char SPTIE : 1;
  unsigned char SPE : 1;
  unsigned char SPRIE : 1;
#else
  unsigned char SPRIE : 1;
  unsigned char SPE : 1;
  unsigned char SPTIE : 1;
  unsigned char SPEIE : 1;
  unsigned char MSTR : 1;
  unsigned char MODFEN : 1;
  unsigned char TXMD : 1;
  unsigned char SPMS : 1;
#endif
};

union un_rspi_spcr
{
  unsigned char BYTE;
  struct st_rspi_spcr_bit BIT;
};

struct st_rspi_sslp_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSL0P : 1;
  unsigned char SSL1P : 1;
  unsigned char SSL2P : 1;
  unsigned char SSL3P : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char SSL3P : 1;
  unsigned char SSL2P : 1;
  unsigned char SSL1P : 1;
  unsigned char SSL0P : 1;
#endif
};

union un_rspi_sslp
{
  unsigned char BYTE;
  struct st_rspi_sslp_bit BIT;
};

struct st_rspi_sppcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPLP : 1;
  unsigned char SPLP2 : 1;
  unsigned char  : 2;
  unsigned char MOIFV : 1;
  unsigned char MOIFE : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char MOIFE : 1;
  unsigned char MOIFV : 1;
  unsigned char  : 2;
  unsigned char SPLP2 : 1;
  unsigned char SPLP : 1;
#endif
};

union un_rspi_sppcr
{
  unsigned char BYTE;
  struct st_rspi_sppcr_bit BIT;
};

struct st_rspi_spsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char OVRF : 1;
  unsigned char IDLNF : 1;
  unsigned char MODF : 1;
  unsigned char PERF : 1;
  unsigned char UDRF : 1;
  unsigned char SPTEF : 1;
  unsigned char  : 1;
  unsigned char SPRF : 1;
#else
  unsigned char SPRF : 1;
  unsigned char  : 1;
  unsigned char SPTEF : 1;
  unsigned char UDRF : 1;
  unsigned char PERF : 1;
  unsigned char MODF : 1;
  unsigned char IDLNF : 1;
  unsigned char OVRF : 1;
#endif
};

union un_rspi_spsr
{
  unsigned char BYTE;
  struct st_rspi_spsr_bit BIT;
};

struct st_rspi_spdr_word
{
  unsigned short H;
};

struct st_rspi_spdr_byte
{
  unsigned char HH;
};

union un_rspi_spdr
{
  unsigned long LONG;
  struct st_rspi_spdr_word WORD;
};

struct st_rspi_spscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPSLN : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SPSLN : 3;
#endif
};

union un_rspi_spscr
{
  unsigned char BYTE;
  struct st_rspi_spscr_bit BIT;
};

struct st_rspi_spssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPCP : 3;
  unsigned char  : 1;
  unsigned char SPECM : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SPECM : 3;
  unsigned char  : 1;
  unsigned char SPCP : 3;
#endif
};

union un_rspi_spssr
{
  unsigned char BYTE;
  struct st_rspi_spssr_bit BIT;
};

struct st_rspi_spdcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPFC : 2;
  unsigned char  : 2;
  unsigned char SPRDTD : 1;
  unsigned char SPLW : 1;
  unsigned char SPBYT : 1;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char SPBYT : 1;
  unsigned char SPLW : 1;
  unsigned char SPRDTD : 1;
  unsigned char  : 2;
  unsigned char SPFC : 2;
#endif
};

union un_rspi_spdcr
{
  unsigned char BYTE;
  struct st_rspi_spdcr_bit BIT;
};

struct st_rspi_spckd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SCKDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SCKDL : 3;
#endif
};

union un_rspi_spckd
{
  unsigned char BYTE;
  struct st_rspi_spckd_bit BIT;
};

struct st_rspi_sslnd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLNDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SLNDL : 3;
#endif
};

union un_rspi_sslnd
{
  unsigned char BYTE;
  struct st_rspi_sslnd_bit BIT;
};

struct st_rspi_spnd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPNDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SPNDL : 3;
#endif
};

union un_rspi_spnd
{
  unsigned char BYTE;
  struct st_rspi_spnd_bit BIT;
};

struct st_rspi_spcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPPE : 1;
  unsigned char SPOE : 1;
  unsigned char SPIIE : 1;
  unsigned char PTE : 1;
  unsigned char SCKASE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char SCKASE : 1;
  unsigned char PTE : 1;
  unsigned char SPIIE : 1;
  unsigned char SPOE : 1;
  unsigned char SPPE : 1;
#endif
};

union un_rspi_spcr2
{
  unsigned char BYTE;
  struct st_rspi_spcr2_bit BIT;
};

struct st_rspi_spcmd0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd0
{
  unsigned char BYTE;
  struct st_rspi_spcmd0_bit BIT;
};

struct st_rspi_spcmd1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd1
{
  unsigned char BYTE;
  struct st_rspi_spcmd1_bit BIT;
};

struct st_rspi_spcmd2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd2
{
  unsigned char BYTE;
  struct st_rspi_spcmd2_bit BIT;
};

struct st_rspi_spcmd3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd3
{
  unsigned char BYTE;
  struct st_rspi_spcmd3_bit BIT;
};

struct st_rspi_spcmd4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd4
{
  unsigned char BYTE;
  struct st_rspi_spcmd4_bit BIT;
};

struct st_rspi_spcmd5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd5
{
  unsigned char BYTE;
  struct st_rspi_spcmd5_bit BIT;
};

struct st_rspi_spcmd6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd6
{
  unsigned char BYTE;
  struct st_rspi_spcmd6_bit BIT;
};

struct st_rspi_spcmd7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SSLA : 3;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SSLA : 3;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_rspi_spcmd7
{
  unsigned char BYTE;
  struct st_rspi_spcmd7_bit BIT;
};

struct st_rspi_spdcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BYSW : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char BYSW : 1;
#endif
};

union un_rspi_spdcr2
{
  unsigned char BYTE;
  struct st_rspi_spdcr2_bit BIT;
};

struct st_sdhi_spcmd_bit
{
  unsigned long :16;
  unsigned long CMD12AT:2;
  unsigned long TRSTP:1;
  unsigned long CMDRW:1;
  unsigned long CMDTP:1;
  unsigned long RSPTP:3;
  unsigned long ACMD:2;
  unsigned long CMDIDX:6;
};

union un_sdhi_spcmd
{
  unsigned long LONG;
  struct st_sdhi_spcmd_bit BIT;
};

struct st_sdhi_sdstop_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long STP : 1;
  unsigned long  : 7;
  unsigned long SDBLKCNTEN : 1;
  unsigned long  : 23;
#else
  unsigned long  : 23;
  unsigned long SDBLKCNTEN : 1;
  unsigned long  : 7;
  unsigned long STP : 1;
#endif
};

union un_sdhi_sdstop
{
  unsigned long LONG;
  struct st_sdhi_sdstop_bit BIT;
};

struct st_sdhi_sdsts1_bit
{
  unsigned long :21;
  unsigned long SDD3MON:1;
  unsigned long SDD3IN:1;
  unsigned long SDD3RM:1;
  unsigned long SDWPMON:1;
  unsigned long :1;
  unsigned long SDCDMON:1;
  unsigned long SDCDIN:1;
  unsigned long SDCDRM:1;
  unsigned long ACEND:1;
  unsigned long :1;
  unsigned long RSPEND:1;
};

union un_sdhi_sdsts1
{
  unsigned long LONG;
  struct st_sdhi_sdsts1_bit BIT;
};

struct st_sdhi_sdsts2_bit
{
  unsigned long :16;
  unsigned long ILA:1;
  unsigned long CBSY:1;
  unsigned long SDCLKCREN:1;
  unsigned long :3;
  unsigned long BWE:1;
  unsigned long BRE:1;
  unsigned long SDD0MON:1;
  unsigned long RSPTO:1;
  unsigned long ILR:1;
  unsigned long ILW:1;
  unsigned long DTO:1;
  unsigned long ENDE:1;
  unsigned long CRCE:1;
  unsigned long CMDE:1;
};

union un_sdhi_sdsts2
{
  unsigned long LONG;
  struct st_sdhi_sdsts2_bit BIT;
};

struct st_sdhi_sdimsk1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RSPENDM : 1;
  unsigned long  : 1;
  unsigned long ACENDM : 1;
  unsigned long SDCDRMM : 1;
  unsigned long SDCDINM : 1;
  unsigned long  : 3;
  unsigned long SDD3RMM : 1;
  unsigned long SDD3INM : 1;
  unsigned long  : 22;
#else
  unsigned long  : 22;
  unsigned long SDD3INM : 1;
  unsigned long SDD3RMM : 1;
  unsigned long  : 3;
  unsigned long SDCDINM : 1;
  unsigned long SDCDRMM : 1;
  unsigned long ACENDM : 1;
  unsigned long  : 1;
  unsigned long RSPENDM : 1;
#endif
};

union un_sdhi_sdimsk1
{
  unsigned long LONG;
  struct st_sdhi_sdimsk1_bit BIT;
};

struct st_sdhi_sdimsk2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CMDEM : 1;
  unsigned long CRCEM : 1;
  unsigned long ENDEM : 1;
  unsigned long DTTOM : 1;
  unsigned long ILWM : 1;
  unsigned long ILRM : 1;
  unsigned long RSPTOM : 1;
  unsigned long  : 1;
  unsigned long BREM : 1;
  unsigned long BWEM : 1;
  unsigned long  : 5;
  unsigned long ILAM : 1;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long ILAM : 1;
  unsigned long  : 5;
  unsigned long BWEM : 1;
  unsigned long BREM : 1;
  unsigned long  : 1;
  unsigned long RSPTOM : 1;
  unsigned long ILRM : 1;
  unsigned long ILWM : 1;
  unsigned long DTTOM : 1;
  unsigned long ENDEM : 1;
  unsigned long CRCEM : 1;
  unsigned long CMDEM : 1;
#endif
};

union un_sdhi_sdimsk2
{
  unsigned long LONG;
  struct st_sdhi_sdimsk2_bit BIT;
};

struct st_sdhi_sdclkcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CLKSEL : 8;
  unsigned long CLKEN : 1;
  unsigned long CLKCTRLEN : 1;
  unsigned long  : 22;
#else
  unsigned long  : 22;
  unsigned long CLKCTRLEN : 1;
  unsigned long CLKEN : 1;
  unsigned long CLKSEL : 8;
#endif
};

union un_sdhi_sdclkcr
{
  unsigned long LONG;
  struct st_sdhi_sdclkcr_bit BIT;
};

struct st_sdhi_sdsize_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LEN : 10;
  unsigned long  : 22;
#else
  unsigned long  : 22;
  unsigned long LEN : 10;
#endif
};

union un_sdhi_sdsize
{
  unsigned long LONG;
  struct st_sdhi_sdsize_bit BIT;
};

struct st_sdhi_sdopt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CTOP : 4;
  unsigned long TOP : 4;
  unsigned long  : 7;
  unsigned long WIDTH : 1;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long WIDTH : 1;
  unsigned long  : 7;
  unsigned long TOP : 4;
  unsigned long CTOP : 4;
#endif
};

union un_sdhi_sdopt
{
  unsigned long LONG;
  struct st_sdhi_sdopt_bit BIT;
};

struct st_sdhi_sdersts1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CMDE0 : 1;
  unsigned long CMDE1 : 1;
  unsigned long RSPLENE0 : 1;
  unsigned long RSPLENE1 : 1;
  unsigned long RDLENE : 1;
  unsigned long CRCLENE : 1;
  unsigned long  : 2;
  unsigned long RSPCRCE0 : 1;
  unsigned long RSPCRCE1 : 1;
  unsigned long RDCRCE : 1;
  unsigned long CRCTKE : 1;
  unsigned long CRCTK : 3;
  unsigned long  : 17;
#else
  unsigned long  : 17;
  unsigned long CRCTK : 3;
  unsigned long CRCTKE : 1;
  unsigned long RDCRCE : 1;
  unsigned long RSPCRCE1 : 1;
  unsigned long RSPCRCE0 : 1;
  unsigned long  : 2;
  unsigned long CRCLENE : 1;
  unsigned long RDLENE : 1;
  unsigned long RSPLENE1 : 1;
  unsigned long RSPLENE0 : 1;
  unsigned long CMDE1 : 1;
  unsigned long CMDE0 : 1;
#endif
};

union un_sdhi_sdersts1
{
  unsigned long LONG;
  struct st_sdhi_sdersts1_bit BIT;
};

struct st_sdhi_sdersts2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RSPTO0 : 1;
  unsigned long RSPTO1 : 1;
  unsigned long BSYTO0 : 1;
  unsigned long BSYTO1 : 1;
  unsigned long RDTO : 1;
  unsigned long CRCTO : 1;
  unsigned long CRCBSYTO : 1;
  unsigned long  : 25;
#else
  unsigned long  : 25;
  unsigned long CRCBSYTO : 1;
  unsigned long CRCTO : 1;
  unsigned long RDTO : 1;
  unsigned long BSYTO1 : 1;
  unsigned long BSYTO0 : 1;
  unsigned long RSPTO1 : 1;
  unsigned long RSPTO0 : 1;
#endif
};

union un_sdhi_sdersts2
{
  unsigned long LONG;
  struct st_sdhi_sdersts2_bit BIT;
};

struct st_sdhi_sdiomd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long INTEN : 1;
  unsigned long  : 1;
  unsigned long RWREQ : 1;
  unsigned long  : 5;
  unsigned long IOABT : 1;
  unsigned long C52PUB : 1;
  unsigned long  : 22;
#else
  unsigned long  : 22;
  unsigned long C52PUB : 1;
  unsigned long IOABT : 1;
  unsigned long  : 5;
  unsigned long RWREQ : 1;
  unsigned long  : 1;
  unsigned long INTEN : 1;
#endif
};

union un_sdhi_sdiomd
{
  unsigned long LONG;
  struct st_sdhi_sdiomd_bit BIT;
};

struct st_sdhi_sdiosts_bit
{
  unsigned long :16;
  unsigned long EXWT:1;
  unsigned long EXPUB52:1;
  unsigned long :13;
  unsigned long IOIRQ:1;
};

union un_sdhi_sdiosts
{
  unsigned long LONG;
  struct st_sdhi_sdiosts_bit BIT;
};

struct st_sdhi_sdioimsk_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IOIRQM : 1;
  unsigned long  : 13;
  unsigned long EXPUB52M : 1;
  unsigned long EXWTM : 1;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long EXWTM : 1;
  unsigned long EXPUB52M : 1;
  unsigned long  : 13;
  unsigned long IOIRQM : 1;
#endif
};

union un_sdhi_sdioimsk
{
  unsigned long LONG;
  struct st_sdhi_sdioimsk_bit BIT;
};

struct st_sdhi_sddmaen_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 1;
  unsigned long DMAEN : 1;
  unsigned long  : 30;
#else
  unsigned long  : 30;
  unsigned long DMAEN : 1;
  unsigned long  : 1;
#endif
};

union un_sdhi_sddmaen
{
  unsigned long LONG;
  struct st_sdhi_sddmaen_bit BIT;
};

struct st_sdhi_sdrst_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SDRST : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long SDRST : 1;
#endif
};

union un_sdhi_sdrst
{
  unsigned long LONG;
  struct st_sdhi_sdrst_bit BIT;
};

struct st_sdhi_sdver_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IP1 : 8;
  unsigned long IP2 : 4;
  unsigned long  : 2;
  unsigned long CLKRAT : 1;
  unsigned long CPRM : 1;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long CPRM : 1;
  unsigned long CLKRAT : 1;
  unsigned long  : 2;
  unsigned long IP2 : 4;
  unsigned long IP1 : 8;
#endif
};

union un_sdhi_sdver
{
  unsigned long LONG;
  struct st_sdhi_sdver_bit BIT;
};

struct st_sdhi_sdswap_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 6;
  unsigned long BWSWP : 1;
  unsigned long BRSWP : 1;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long BRSWP : 1;
  unsigned long BWSWP : 1;
  unsigned long  : 6;
#endif
};

union un_sdhi_sdswap
{
  unsigned long LONG;
  struct st_sdhi_sdswap_bit BIT;
};

struct st_sdsi_fn1accr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 11;
  unsigned long FN1ACC : 1;
  unsigned long  : 20;
#else
  unsigned long  : 20;
  unsigned long FN1ACC : 1;
  unsigned long  : 11;
#endif
};

union un_sdsi_fn1accr
{
  unsigned long LONG;
  struct st_sdsi_fn1accr_bit BIT;
};

struct st_sdsi_intencr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMD52WIREN : 1;
  unsigned char CMD53WIREN : 1;
  unsigned char CMD53RIREN : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char CMD53RIREN : 1;
  unsigned char CMD53WIREN : 1;
  unsigned char CMD52WIREN : 1;
#endif
};

union un_sdsi_intencr1
{
  unsigned char BYTE;
  struct st_sdsi_intencr1_bit BIT;
};

struct st_sdsi_intsr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMD52W : 1;
  unsigned char CMD53W : 1;
  unsigned char CMD53R : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char CMD53R : 1;
  unsigned char CMD53W : 1;
  unsigned char CMD52W : 1;
#endif
};

union un_sdsi_intsr1
{
  unsigned char BYTE;
  struct st_sdsi_intsr1_bit BIT;
};

struct st_sdsi_sdcmdcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDCMDINDEX : 1;
  unsigned char SDWNRFLG : 1;
  unsigned char SDRAWFLG : 1;
  unsigned char SDBMODE : 1;
  unsigned char SDOPCODE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char SDOPCODE : 1;
  unsigned char SDBMODE : 1;
  unsigned char SDRAWFLG : 1;
  unsigned char SDWNRFLG : 1;
  unsigned char SDCMDINDEX : 1;
#endif
};

union un_sdsi_sdcmdcr
{
  unsigned char BYTE;
  struct st_sdsi_sdcmdcr_bit BIT;
};

struct st_sdsi_sdcadd0r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDCMDACCADD : 8;
#else
  unsigned char SDCMDACCADD : 8;
#endif
};

union un_sdsi_sdcadd0r
{
  unsigned char BYTE;
  struct st_sdsi_sdcadd0r_bit BIT;
};

struct st_sdsi_sdcadd1r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDCMDACCADD : 8;
#else
  unsigned char SDCMDACCADD : 8;
#endif
};

union un_sdsi_sdcadd1r
{
  unsigned char BYTE;
  struct st_sdsi_sdcadd1r_bit BIT;
};

struct st_sdsi_sdcadd2r_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SDCMDACCADD : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SDCMDACCADD : 1;
#endif
};

union un_sdsi_sdcadd2r
{
  unsigned char BYTE;
  struct st_sdsi_sdcadd2r_bit BIT;
};

struct st_sdsi_sdsicr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOE1IOR1 : 1;
  unsigned char EPS : 1;
  unsigned char EMPC : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char EMPC : 1;
  unsigned char EPS : 1;
  unsigned char IOE1IOR1 : 1;
#endif
};

union un_sdsi_sdsicr1
{
  unsigned char BYTE;
  struct st_sdsi_sdsicr1_bit BIT;
};

struct st_sdsi_dmacr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DMAEN : 1;
  unsigned char DMALOCKEN : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char DMALOCKEN : 1;
  unsigned char DMAEN : 1;
#endif
};

union un_sdsi_dmacr1
{
  unsigned char BYTE;
  struct st_sdsi_dmacr1_bit BIT;
};

struct st_sdsi_blkcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMD53BLK : 9;
  unsigned short  : 7;
#else
  unsigned short  : 7;
  unsigned short CMD53BLK : 9;
#endif
};

union un_sdsi_blkcnt
{
  unsigned short WORD;
  struct st_sdsi_blkcnt_bit BIT;
};

struct st_sdsi_bytcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMD53BYT : 12;
  unsigned short  : 4;
#else
  unsigned short  : 4;
  unsigned short CMD53BYT : 12;
#endif
};

union un_sdsi_bytcnt
{
  unsigned short WORD;
  struct st_sdsi_bytcnt_bit BIT;
};

struct st_sdsi_dmatraddr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DMAADD : 32;
#else
  unsigned long DMAADD : 32;
#endif
};

union un_sdsi_dmatraddr
{
  unsigned long LONG;
  struct st_sdsi_dmatraddr_bit BIT;
};

struct st_sdsi_sdsicr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RSWAP : 1;
  unsigned long  : 1;
  unsigned long WSWAP : 1;
  unsigned long  : 1;
  unsigned long REG5EN : 1;
  unsigned long  : 27;
#else
  unsigned long  : 27;
  unsigned long REG5EN : 1;
  unsigned long  : 1;
  unsigned long WSWAP : 1;
  unsigned long  : 1;
  unsigned long RSWAP : 1;
#endif
};

union un_sdsi_sdsicr2
{
  unsigned long LONG;
  struct st_sdsi_sdsicr2_bit BIT;
};

struct st_sdsi_sdsicr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SRST : 1;
  unsigned long  : 1;
  unsigned long IOR0 : 1;
  unsigned long CEN : 1;
  unsigned long  : 14;
  unsigned long SPS : 1;
  unsigned long SMPC : 1;
  unsigned long  : 12;
#else
  unsigned long  : 12;
  unsigned long SMPC : 1;
  unsigned long SPS : 1;
  unsigned long  : 14;
  unsigned long CEN : 1;
  unsigned long IOR0 : 1;
  unsigned long  : 1;
  unsigned long SRST : 1;
#endif
};

union un_sdsi_sdsicr3
{
  unsigned long LONG;
  struct st_sdsi_sdsicr3_bit BIT;
};

struct st_sdsi_intencr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CDFEN : 1;
  unsigned long CDREN : 1;
  unsigned long DTEEN : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long DTEEN : 1;
  unsigned long CDREN : 1;
  unsigned long CDFEN : 1;
#endif
};

union un_sdsi_intencr2
{
  unsigned long LONG;
  struct st_sdsi_intencr2_bit BIT;
};

struct st_sdsi_intsr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CDF : 1;
  unsigned long CDR : 1;
  unsigned long DTE : 1;
  unsigned long  : 29;
#else
  unsigned long  : 29;
  unsigned long DTE : 1;
  unsigned long CDR : 1;
  unsigned long CDF : 1;
#endif
};

union un_sdsi_intsr2
{
  unsigned long LONG;
  struct st_sdsi_intsr2_bit BIT;
};

struct st_sdsi_dmacr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DMARSWAP : 2;
  unsigned long DMAWSWAP : 2;
  unsigned long  : 4;
  unsigned long DMASDSEL : 1;
  unsigned long  : 23;
#else
  unsigned long  : 23;
  unsigned long DMASDSEL : 1;
  unsigned long  : 4;
  unsigned long DMAWSWAP : 2;
  unsigned long DMARSWAP : 2;
#endif
};

union un_sdsi_dmacr2
{
  unsigned long LONG;
  struct st_sdsi_dmacr2_bit BIT;
};

struct st_sdsi_fbr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBR1L : 4;
  unsigned long  : 4;
  unsigned long FBR1U : 8;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long FBR1U : 8;
  unsigned long  : 4;
  unsigned long FBR1L : 4;
#endif
};

union un_sdsi_fbr1
{
  unsigned long LONG;
  struct st_sdsi_fbr1_bit BIT;
};

struct st_sdsi_fbr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBR2 : 8;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long FBR2 : 8;
#endif
};

union un_sdsi_fbr2
{
  unsigned long LONG;
  struct st_sdsi_fbr2_bit BIT;
};

struct st_sdsi_fbr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBR3 : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long FBR3 : 16;
#endif
};

union un_sdsi_fbr3
{
  unsigned long LONG;
  struct st_sdsi_fbr3_bit BIT;
};

struct st_sdsi_fbr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBR4 : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long FBR4 : 16;
#endif
};

union un_sdsi_fbr4
{
  unsigned long LONG;
  struct st_sdsi_fbr4_bit BIT;
};

struct st_sdsi_fbr5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBR5 : 8;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long FBR5 : 8;
#endif
};

union un_sdsi_fbr5
{
  unsigned long LONG;
  struct st_sdsi_fbr5_bit BIT;
};

struct st_sdsi_fn1datar1_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

union un_sdsi_fn1datar1
{
  unsigned long LONG;
  struct st_sdsi_fn1datar1_byte BYTE;
};

struct st_sdsi_fn1datar2_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

union un_sdsi_fn1datar2
{
  unsigned long LONG;
  struct st_sdsi_fn1datar2_byte BYTE;
};

struct st_sdsi_fn1datar3_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

union un_sdsi_fn1datar3
{
  unsigned long LONG;
  struct st_sdsi_fn1datar3_byte BYTE;
};

struct st_sdsi_fn1intvecr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char INTVEC : 8;
#else
  unsigned char INTVEC : 8;
#endif
};

union un_sdsi_fn1intvecr
{
  unsigned char BYTE;
  struct st_sdsi_fn1intvecr_bit BIT;
};

struct st_sdsi_fn1intclrr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char INTCTR : 8;
#else
  unsigned char INTCTR : 8;
#endif
};

union un_sdsi_fn1intclrr
{
  unsigned char BYTE;
  struct st_sdsi_fn1intclrr_bit BIT;
};

struct st_sdsi_fn1datar5_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

union un_sdsi_fn1datar5
{
  unsigned long LONG;
  struct st_sdsi_fn1datar5_byte BYTE;
};

struct st_mtu6_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu6_tcr
{
  unsigned char BYTE;
  struct st_mtu6_tcr_bit BIT;
};

struct st_mtu6_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu6_tmdr1
{
  unsigned char BYTE;
  struct st_mtu6_tmdr1_bit BIT;
};

struct st_mtu6_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu6_tiorh
{
  unsigned char BYTE;
  struct st_mtu6_tiorh_bit BIT;
};

struct st_mtu6_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu6_tiorl
{
  unsigned char BYTE;
  struct st_mtu6_tiorl_bit BIT;
};

struct st_mtu6_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 2;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char  : 2;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu6_tier
{
  unsigned char BYTE;
  struct st_mtu6_tier_bit BIT;
};

struct st_mtu6_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};

union un_mtu6_tsr
{
  unsigned char BYTE;
  struct st_mtu6_tsr_bit BIT;
};

struct st_mtu6_tbtm_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TTSA : 1;
  unsigned char TTSB : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TTSB : 1;
  unsigned char TTSA : 1;
#endif
};

union un_mtu6_tbtm
{
  unsigned char BYTE;
  struct st_mtu6_tbtm_bit BIT;
};

struct st_mtu6_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu6_tcr2
{
  unsigned char BYTE;
  struct st_mtu6_tcr2_bit BIT;
};

struct st_mtu6_tsycr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CE2B : 1;
  unsigned char CE2A : 1;
  unsigned char CE1B : 1;
  unsigned char CE1A : 1;
  unsigned char CE0D : 1;
  unsigned char CE0C : 1;
  unsigned char CE0B : 1;
  unsigned char CE0A : 1;
#else
  unsigned char CE0A : 1;
  unsigned char CE0B : 1;
  unsigned char CE0C : 1;
  unsigned char CE0D : 1;
  unsigned char CE1A : 1;
  unsigned char CE1B : 1;
  unsigned char CE2A : 1;
  unsigned char CE2B : 1;
#endif
};

union un_mtu6_tsycr
{
  unsigned char BYTE;
  struct st_mtu6_tsycr_bit BIT;
};

struct st_mtu7_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu7_tcr
{
  unsigned char BYTE;
  struct st_mtu7_tcr_bit BIT;
};

struct st_mtu7_tmdr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu7_tmdr1
{
  unsigned char BYTE;
  struct st_mtu7_tmdr1_bit BIT;
};

struct st_mtu7_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu7_tiorh
{
  unsigned char BYTE;
  struct st_mtu7_tiorh_bit BIT;
};

struct st_mtu7_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu7_tiorl
{
  unsigned char BYTE;
  struct st_mtu7_tiorl_bit BIT;
};

struct st_mtu7_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 1;
  unsigned char TTGE2 : 1;
  unsigned char TTGE : 1;
#else
  unsigned char TTGE : 1;
  unsigned char TTGE2 : 1;
  unsigned char  : 1;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu7_tier
{
  unsigned char BYTE;
  struct st_mtu7_tier_bit BIT;
};

struct st_mtu7_tsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TCFD : 1;
#else
  unsigned char TCFD : 1;
  unsigned char  : 7;
#endif
};

union un_mtu7_tsr
{
  unsigned char BYTE;
  struct st_mtu7_tsr_bit BIT;
};

struct st_mtu7_tbtm_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TTSA : 1;
  unsigned char TTSB : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char TTSB : 1;
  unsigned char TTSA : 1;
#endif
};

union un_mtu7_tbtm
{
  unsigned char BYTE;
  struct st_mtu7_tbtm_bit BIT;
};

struct st_mtu7_tadcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ITB7VE : 1;
  unsigned short ITB6AE : 1;
  unsigned short ITA7VE : 1;
  unsigned short ITA6AE : 1;
  unsigned short DT7BE : 1;
  unsigned short UT7BE : 1;
  unsigned short DT7AE : 1;
  unsigned short UT7AE : 1;
  unsigned short  : 6;
  unsigned short BF : 2;
#else
  unsigned short BF : 2;
  unsigned short  : 6;
  unsigned short UT7AE : 1;
  unsigned short DT7AE : 1;
  unsigned short UT7BE : 1;
  unsigned short DT7BE : 1;
  unsigned short ITA6AE : 1;
  unsigned short ITA7VE : 1;
  unsigned short ITB6AE : 1;
  unsigned short ITB7VE : 1;
#endif
};

union un_mtu7_tadcr
{
  unsigned char BYTE;
  struct st_mtu7_tadcr_bit BIT;
};
struct st_mtu7_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu7_tcr2
{
  unsigned char BYTE;
  struct st_mtu7_tcr2_bit BIT;
};

struct st_mtu7_nfcr7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu7_nfcr7
{
  unsigned char BYTE;
  struct st_mtu7_nfcr7_bit BIT;
};

struct st_mtu8_nfcr8_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NFAEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFDEN : 1;
  unsigned char NFCS : 2;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char NFCS : 2;
  unsigned char NFDEN : 1;
  unsigned char NFCEN : 1;
  unsigned char NFBEN : 1;
  unsigned char NFAEN : 1;
#endif
};

union un_mtu8_nfcr8
{
  unsigned char BYTE;
  struct st_mtu8_nfcr8_bit BIT;
};
struct st_mtu8_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC : 3;
  unsigned char CKEG : 2;
  unsigned char CCLR : 3;
#else
  unsigned char CCLR : 3;
  unsigned char CKEG : 2;
  unsigned char TPSC : 3;
#endif
};

union un_mtu8_tcr
{
  unsigned char BYTE;
  struct st_mtu8_tcr_bit BIT;
};

struct st_mtu8_tmdr1_bit
{
  #ifdef __RX_LITTLE_ENDIAN__
  unsigned char MD : 4;
  unsigned char BFA : 1;
  unsigned char BFB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char BFB : 1;
  unsigned char BFA : 1;
  unsigned char MD : 4;
#endif
};

union un_mtu8_tmdr1
{
  unsigned char BYTE;
  struct st_mtu8_tmdr1_bit BIT;
};

struct st_mtu8_tiorh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOA : 4;
  unsigned char IOB : 4;
#else
  unsigned char IOB : 4;
  unsigned char IOA : 4;
#endif
};

union un_mtu8_tiorh
{
  unsigned char BYTE;
  struct st_mtu8_tiorh_bit BIT;
};

struct st_mtu8_tiorl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char IOC : 4;
  unsigned char IOD : 4;
#else
  unsigned char IOD : 4;
  unsigned char IOC : 4;
#endif
};

union un_mtu8_tiorl
{
  unsigned char BYTE;
  struct st_mtu8_tiorl_bit BIT;
};

struct st_mtu8_tier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TGIEA : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIED : 1;
  unsigned char TCIEV : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char TCIEV : 1;
  unsigned char TGIED : 1;
  unsigned char TGIEC : 1;
  unsigned char TGIEB : 1;
  unsigned char TGIEA : 1;
#endif
};

union un_mtu8_tier
{
  unsigned char BYTE;
  struct st_mtu8_tier_bit BIT;
};

struct st_mtu8_tcr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPSC2 : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TPSC2 : 3;
#endif
};

union un_mtu8_tcr2
{
  unsigned char BYTE;
  struct st_mtu8_tcr2_bit BIT;
};

struct st_port6_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_pdr
{
  unsigned char BYTE;
  struct st_port6_pdr_bit BIT;
};

struct st_port6_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_podr
{
  unsigned char BYTE;
  struct st_port6_podr_bit BIT;
};

struct st_port6_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_pidr
{
  unsigned char BYTE;
  struct st_port6_pidr_bit BIT;
};

struct st_port6_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_pmr
{
  unsigned char BYTE;
  struct st_port6_pmr_bit BIT;
};

struct st_port6_ord0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_ord0
{
  unsigned char BYTE;
  struct st_port6_ord0_bit BIT;
};

struct st_port6_ord1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_ord1
{
  unsigned char BYTE;
  struct st_port6_ord1_bit BIT;
};

struct st_port6_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_port6_pcr
{
  unsigned char BYTE;
  struct st_port6_pcr_bit BIT;
};

struct st_can_mb_id_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_mb_id_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_mb_id_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EID : 18;
  unsigned long SID : 11;
  unsigned long  : 1;
  unsigned long RTR : 1;
  unsigned long IDE : 1;
#else
  unsigned long IDE : 1;
  unsigned long RTR : 1;
  unsigned long  : 1;
  unsigned long SID : 11;
  unsigned long EID : 18;
#endif
};

union un_can_id
{
  unsigned long LONG;
  struct st_can_mb_id_word WORD;
  struct st_can_mb_id_byte BYTE;
  struct st_can_mb_id_bit BIT;
};

struct st_can_mb
{
  union un_can_id ID;
  unsigned short DLC;
  unsigned char  DATA[8];
  unsigned short TS;
};

struct st_can_mkr_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_mkr_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_mkr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EID : 18;
  unsigned long SID : 11;
  unsigned long  : 3;
#else
  unsigned long  : 3;
  unsigned long SID : 11;
  unsigned long EID : 18;
#endif
};

union un_can_mkr
{
  unsigned long LONG;
  struct st_can_mkr_word WORD;
  struct st_can_mkr_byte BYTE;
  struct st_can_mkr_bit BIT;
};

struct st_can_fidcr0_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_fidcr0_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_fidcr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EID : 18;
  unsigned long SID : 11;
  unsigned long  : 1;
  unsigned long RTR : 1;
  unsigned long IDE : 1;
#else
  unsigned long IDE : 1;
  unsigned long RTR : 1;
  unsigned long  : 1;
  unsigned long SID : 11;
  unsigned long EID : 18;
#endif
};

union un_can_fidcr0
{
  unsigned long LONG;
  struct st_can_fidcr0_word WORD;
  struct st_can_fidcr0_byte BYTE;
  struct st_can_fidcr0_bit BIT;
};

struct st_can_fidcr1_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_fidcr1_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_fidcr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long EID : 18;
  unsigned long SID : 11;
  unsigned long  : 1;
  unsigned long RTR : 1;
  unsigned long IDE : 1;
#else
  unsigned long IDE : 1;
  unsigned long RTR : 1;
  unsigned long  : 1;
  unsigned long SID : 11;
  unsigned long EID : 18;
#endif
};

union un_can_fidcr1
{
  unsigned long LONG;
  struct st_can_fidcr1_word WORD;
  struct st_can_fidcr1_byte BYTE;
  struct st_can_fidcr1_bit BIT;
};

struct st_can_mkivlr_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_mkivlr_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_mkivlr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MB0 : 1;
  unsigned long MB1 : 1;
  unsigned long MB2 : 1;
  unsigned long MB3 : 1;
  unsigned long MB4 : 1;
  unsigned long MB5 : 1;
  unsigned long MB6 : 1;
  unsigned long MB7 : 1;
  unsigned long MB8 : 1;
  unsigned long MB9 : 1;
  unsigned long MB10 : 1;
  unsigned long MB11 : 1;
  unsigned long MB12 : 1;
  unsigned long MB13 : 1;
  unsigned long MB14 : 1;
  unsigned long MB15 : 1;
  unsigned long MB16 : 1;
  unsigned long MB17 : 1;
  unsigned long MB18 : 1;
  unsigned long MB19 : 1;
  unsigned long MB20 : 1;
  unsigned long MB21 : 1;
  unsigned long MB22 : 1;
  unsigned long MB23 : 1;
  unsigned long MB24 : 1;
  unsigned long MB25 : 1;
  unsigned long MB26 : 1;
  unsigned long MB27 : 1;
  unsigned long MB28 : 1;
  unsigned long MB29 : 1;
  unsigned long MB30 : 1;
  unsigned long MB31 : 1;
#else
  unsigned long MB31 : 1;
  unsigned long MB30 : 1;
  unsigned long MB29 : 1;
  unsigned long MB28 : 1;
  unsigned long MB27 : 1;
  unsigned long MB26 : 1;
  unsigned long MB25 : 1;
  unsigned long MB24 : 1;
  unsigned long MB23 : 1;
  unsigned long MB22 : 1;
  unsigned long MB21 : 1;
  unsigned long MB20 : 1;
  unsigned long MB19 : 1;
  unsigned long MB18 : 1;
  unsigned long MB17 : 1;
  unsigned long MB16 : 1;
  unsigned long MB15 : 1;
  unsigned long MB14 : 1;
  unsigned long MB13 : 1;
  unsigned long MB12 : 1;
  unsigned long MB11 : 1;
  unsigned long MB10 : 1;
  unsigned long MB9 : 1;
  unsigned long MB8 : 1;
  unsigned long MB7 : 1;
  unsigned long MB6 : 1;
  unsigned long MB5 : 1;
  unsigned long MB4 : 1;
  unsigned long MB3 : 1;
  unsigned long MB2 : 1;
  unsigned long MB1 : 1;
  unsigned long MB0 : 1;
#endif
};

union un_can_mkivlr
{
  unsigned long LONG;
  struct st_can_mkivlr_word WORD;
  struct st_can_mkivlr_byte BYTE;
  struct st_can_mkivlr_bit BIT;
};

struct st_can_mier_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_mier_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_mier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MB0 : 1;
  unsigned long MB1 : 1;
  unsigned long MB2 : 1;
  unsigned long MB3 : 1;
  unsigned long MB4 : 1;
  unsigned long MB5 : 1;
  unsigned long MB6 : 1;
  unsigned long MB7 : 1;
  unsigned long MB8 : 1;
  unsigned long MB9 : 1;
  unsigned long MB10 : 1;
  unsigned long MB11 : 1;
  unsigned long MB12 : 1;
  unsigned long MB13 : 1;
  unsigned long MB14 : 1;
  unsigned long MB15 : 1;
  unsigned long MB16 : 1;
  unsigned long MB17 : 1;
  unsigned long MB18 : 1;
  unsigned long MB19 : 1;
  unsigned long MB20 : 1;
  unsigned long MB21 : 1;
  unsigned long MB22 : 1;
  unsigned long MB23 : 1;
  unsigned long MB24 : 1;
  unsigned long MB25 : 1;
  unsigned long MB26 : 1;
  unsigned long MB27 : 1;
  unsigned long MB28 : 1;
  unsigned long MB29 : 1;
  unsigned long MB30 : 1;
  unsigned long MB31 : 1;
#else
  unsigned long MB31 : 1;
  unsigned long MB30 : 1;
  unsigned long MB29 : 1;
  unsigned long MB28 : 1;
  unsigned long MB27 : 1;
  unsigned long MB26 : 1;
  unsigned long MB25 : 1;
  unsigned long MB24 : 1;
  unsigned long MB23 : 1;
  unsigned long MB22 : 1;
  unsigned long MB21 : 1;
  unsigned long MB20 : 1;
  unsigned long MB19 : 1;
  unsigned long MB18 : 1;
  unsigned long MB17 : 1;
  unsigned long MB16 : 1;
  unsigned long MB15 : 1;
  unsigned long MB14 : 1;
  unsigned long MB13 : 1;
  unsigned long MB12 : 1;
  unsigned long MB11 : 1;
  unsigned long MB10 : 1;
  unsigned long MB9 : 1;
  unsigned long MB8 : 1;
  unsigned long MB7 : 1;
  unsigned long MB6 : 1;
  unsigned long MB5 : 1;
  unsigned long MB4 : 1;
  unsigned long MB3 : 1;
  unsigned long MB2 : 1;
  unsigned long MB1 : 1;
  unsigned long MB0 : 1;
#endif
};

union un_can_mier
{
  unsigned long LONG;
  struct st_can_mier_word WORD;
  struct st_can_mier_byte BYTE;
  struct st_can_mier_bit BIT;
};

struct st_can_mctl_bit_tx
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SENTDATA : 1;
  unsigned char TRMACTIVE : 1;
  unsigned char TRMABT : 1;
  unsigned char  : 1;
  unsigned char ONESHOT : 1;
  unsigned char  : 1;
  unsigned char RECREQ : 1;
  unsigned char TRMREQ : 1;
#else
  unsigned char TRMREQ : 1;
  unsigned char RECREQ : 1;
  unsigned char  : 1;
  unsigned char ONESHOT : 1;
  unsigned char  : 1;
  unsigned char TRMABT : 1;
  unsigned char TRMACTIVE : 1;
  unsigned char SENTDATA : 1;
#endif
};

struct st_can_mctl_bit_rx
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NEWDATA : 1;
  unsigned char INVALDATA : 1;
  unsigned char MSGLOST : 1;
  unsigned char  : 1;
  unsigned char ONESHOT : 1;
  unsigned char  : 1;
  unsigned char RECREQ : 1;
  unsigned char TRMREQ : 1;
#else
  unsigned char TRMREQ : 1;
  unsigned char RECREQ : 1;
  unsigned char  : 1;
  unsigned char ONESHOT : 1;
  unsigned char  : 1;
  unsigned char MSGLOST : 1;
  unsigned char INVALDATA : 1;
  unsigned char NEWDATA : 1;
#endif
};

union un_can_mctl_bit
{
        struct st_can_mctl_bit_tx TX;
        struct st_can_mctl_bit_rx RX;
};

union un_can_mctl
{
  unsigned char BYTE;
};

struct st_can_ctlr_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_can_ctlr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short MBM : 1;
  unsigned short IDFM : 2;
  unsigned short MLM : 1;
  unsigned short TPM : 1;
  unsigned short TSRC : 1;
  unsigned short TSPS : 2;
  unsigned short CANM : 2;
  unsigned short SLPM : 1;
  unsigned short BOM : 2;
  unsigned short RBOC : 1;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short RBOC : 1;
  unsigned short BOM : 2;
  unsigned short SLPM : 1;
  unsigned short CANM : 2;
  unsigned short TSPS : 2;
  unsigned short TSRC : 1;
  unsigned short TPM : 1;
  unsigned short MLM : 1;
  unsigned short IDFM : 2;
  unsigned short MBM : 1;
#endif
};

union un_can_ctlr
{
  unsigned short WORD;
  struct st_can_ctlr_byte BYTE;
  struct st_can_ctlr_bit BIT;
};

struct st_can_str_byte
{
  unsigned char H;
  unsigned char L;
};

struct st_can_str_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short NDST : 1;
  unsigned short SDST : 1;
  unsigned short RFST : 1;
  unsigned short TFST : 1;
  unsigned short NMLST : 1;
  unsigned short FMLST : 1;
  unsigned short TABST : 1;
  unsigned short EST : 1;
  unsigned short RSTST : 1;
  unsigned short HLTST : 1;
  unsigned short SLPST : 1;
  unsigned short EPST : 1;
  unsigned short BOST : 1;
  unsigned short TRMST : 1;
  unsigned short RECST : 1;
  unsigned short  : 1;
#else
  unsigned short  : 1;
  unsigned short RECST : 1;
  unsigned short TRMST : 1;
  unsigned short BOST : 1;
  unsigned short EPST : 1;
  unsigned short SLPST : 1;
  unsigned short HLTST : 1;
  unsigned short RSTST : 1;
  unsigned short EST : 1;
  unsigned short TABST : 1;
  unsigned short FMLST : 1;
  unsigned short NMLST : 1;
  unsigned short TFST : 1;
  unsigned short RFST : 1;
  unsigned short SDST : 1;
  unsigned short NDST : 1;
#endif
};

union un_can_str
{
  unsigned short WORD;
  struct st_can_str_byte BYTE;
  struct st_can_str_bit BIT;
};

struct st_can_bcr_word
{
  unsigned short H;
  unsigned short L;
};

struct st_can_bcr_byte
{
  unsigned char HH;
  unsigned char HL;
  unsigned char LH;
  unsigned char LL;
};

struct st_can_bcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CCLKS : 1;
  unsigned long  : 7;
  unsigned long TSEG2 : 3;
  unsigned long  : 1;
  unsigned long SJW : 2;
  unsigned long  : 2;
  unsigned long BRP : 10;
  unsigned long  : 2;
  unsigned long TSEG1 : 4;
#else
  unsigned long TSEG1 : 4;
  unsigned long  : 2;
  unsigned long BRP : 10;
  unsigned long  : 2;
  unsigned long SJW : 2;
  unsigned long  : 1;
  unsigned long TSEG2 : 3;
  unsigned long  : 7;
  unsigned long CCLKS : 1;
#endif
};

union un_can_bcr
{
  unsigned long LONG;
  struct st_can_bcr_word WORD;
  struct st_can_bcr_byte BYTE;
  struct st_can_bcr_bit BIT;
};

struct st_can_rfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RFE : 1;
  unsigned char RFUST : 3;
  unsigned char RFMLF : 1;
  unsigned char RFFST : 1;
  unsigned char RFWST : 1;
  unsigned char RFEST : 1;
#else
  unsigned char RFEST : 1;
  unsigned char RFWST : 1;
  unsigned char RFFST : 1;
  unsigned char RFMLF : 1;
  unsigned char RFUST : 3;
  unsigned char RFE : 1;
#endif
};

union un_can_rfcr
{
  unsigned char BYTE;
  struct st_can_rfcr_bit BIT;
};

struct st_can_tfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TFE : 1;
  unsigned char TFUST : 3;
  unsigned char  : 2;
  unsigned char TFFST : 1;
  unsigned char TFEST : 1;
#else
  unsigned char TFEST : 1;
  unsigned char TFFST : 1;
  unsigned char  : 2;
  unsigned char TFUST : 3;
  unsigned char TFE : 1;
#endif
};

union un_can_tfcr
{
  unsigned char BYTE;
  struct st_can_tfcr_bit BIT;
};

struct st_can_eier_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BEIE : 1;
  unsigned char EWIE : 1;
  unsigned char EPIE : 1;
  unsigned char BOEIE : 1;
  unsigned char BORIE : 1;
  unsigned char ORIE : 1;
  unsigned char OLIE : 1;
  unsigned char BLIE : 1;
#else
  unsigned char BLIE : 1;
  unsigned char OLIE : 1;
  unsigned char ORIE : 1;
  unsigned char BORIE : 1;
  unsigned char BOEIE : 1;
  unsigned char EPIE : 1;
  unsigned char EWIE : 1;
  unsigned char BEIE : 1;
#endif
};

union un_can_eier
{
  unsigned char BYTE;
  struct st_can_eier_bit BIT;
};

struct st_can_eifr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BEIF : 1;
  unsigned char EWIF : 1;
  unsigned char EPIF : 1;
  unsigned char BOEIF : 1;
  unsigned char BORIF : 1;
  unsigned char ORIF : 1;
  unsigned char OLIF : 1;
  unsigned char BLIF : 1;
#else
  unsigned char BLIF : 1;
  unsigned char OLIF : 1;
  unsigned char ORIF : 1;
  unsigned char BORIF : 1;
  unsigned char BOEIF : 1;
  unsigned char EPIF : 1;
  unsigned char EWIF : 1;
  unsigned char BEIF : 1;
#endif
};

union un_can_eifr
{
  unsigned char BYTE;
  struct st_can_eifr_bit BIT;
};

struct st_can_ecsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SEF : 1;
  unsigned char FEF : 1;
  unsigned char AEF : 1;
  unsigned char CEF : 1;
  unsigned char BE1F : 1;
  unsigned char BE0F : 1;
  unsigned char ADEF : 1;
  unsigned char EDPM : 1;
#else
  unsigned char EDPM : 1;
  unsigned char ADEF : 1;
  unsigned char BE0F : 1;
  unsigned char BE1F : 1;
  unsigned char CEF : 1;
  unsigned char AEF : 1;
  unsigned char FEF : 1;
  unsigned char SEF : 1;
#endif
};

union un_can_ecsr
{
  unsigned char BYTE;
  struct st_can_ecsr_bit BIT;
};

struct st_can_mssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MBNST : 5;
  unsigned char  : 2;
  unsigned char SEST : 1;
#else
  unsigned char SEST : 1;
  unsigned char  : 2;
  unsigned char MBNST : 5;
#endif
};

union un_can_mssr
{
  unsigned char BYTE;
  struct st_can_mssr_bit BIT;
};

struct st_can_msmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MBSM : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char MBSM : 2;
#endif
};

union un_can_msmr
{
  unsigned char BYTE;
  struct st_can_msmr_bit BIT;
};

struct st_can_tcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TSTE : 1;
  unsigned char TSTM : 2;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char TSTM : 2;
  unsigned char TSTE : 1;
#endif
};

union un_can_tcr
{
  unsigned char BYTE;
  struct st_can_tcr_bit BIT;
};

struct st_dmac_dmast_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DMST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DMST : 1;
#endif
};

union un_dmac_dmast
{
  unsigned char BYTE;
  struct st_dmac_dmast_bit BIT;
};

struct st_dmac_dmist_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 4;
  unsigned char DMIS4 : 1;
  unsigned char DMIS5 : 1;
  unsigned char DMIS6 : 1;
  unsigned char DMIS7 : 1;
#else
  unsigned char DMIS7 : 1;
  unsigned char DMIS6 : 1;
  unsigned char DMIS5 : 1;
  unsigned char DMIS4 : 1;
  unsigned char  : 4;
#endif
};

union un_dmac_dmist
{
  unsigned char BYTE;
  struct st_dmac_dmist_bit BIT;
};

struct st_dmac0_dmtmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DCTG : 2;
  unsigned short  : 6;
  unsigned short SZ : 2;
  unsigned short  : 2;
  unsigned short DTS : 2;
  unsigned short MD : 2;
#else
  unsigned short MD : 2;
  unsigned short DTS : 2;
  unsigned short  : 2;
  unsigned short SZ : 2;
  unsigned short  : 6;
  unsigned short DCTG : 2;
#endif
};

union un_dmac0_dmtmd
{
  unsigned short WORD;
  struct st_dmac0_dmtmd_bit BIT;
};

struct st_dmac0_dmint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DARIE : 1;
  unsigned char SARIE : 1;
  unsigned char RPTIE : 1;
  unsigned char ESIE : 1;
  unsigned char DTIE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char DTIE : 1;
  unsigned char ESIE : 1;
  unsigned char RPTIE : 1;
  unsigned char SARIE : 1;
  unsigned char DARIE : 1;
#endif
};

union un_dmac0_dmint
{
  unsigned char BYTE;
  struct st_dmac0_dmint_bit BIT;
};

struct un_dmac0_dmamd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DARA : 5;
  unsigned short  : 1;
  unsigned short DM : 2;
  unsigned short SARA : 5;
  unsigned short  : 1;
  unsigned short SM : 2;
#else
  unsigned short SM : 2;
  unsigned short  : 1;
  unsigned short SARA : 5;
  unsigned short DM : 2;
  unsigned short  : 1;
  unsigned short DARA : 5;
#endif
};

union un_dmac0_dmamd
{
  unsigned short WORD;
  struct un_dmac0_dmamd_bit BIT;
};

struct st_dmac0_dmcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTE : 1;
#endif
};

union un_dmac0_dmcnt
{
  unsigned char BYTE;
  struct st_dmac0_dmcnt_bit BIT;
};

struct st_dmac0_dmreq_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWREQ : 1;
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
  unsigned char SWREQ : 1;
#endif
};

union un_dmac0_dmreq
{
  unsigned char BYTE;
  struct st_dmac0_dmreq_bit BIT;
};

struct st_dmac0_dmsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ESIF : 1;
  unsigned char  : 3;
  unsigned char DTIF : 1;
  unsigned char  : 2;
  unsigned char ACT : 1;
#else
  unsigned char ACT : 1;
  unsigned char  : 2;
  unsigned char DTIF : 1;
  unsigned char  : 3;
  unsigned char ESIF : 1;
#endif
};

union un_dmac0_dmsts
{
  unsigned char BYTE;
  struct st_dmac0_dmsts_bit BIT;
};

struct st_dmac0_dmcsl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DISEL : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DISEL : 1;
#endif
};

union un_dmac0_dmcsl
{
  unsigned char BYTE;
  struct st_dmac0_dmcsl_bit BIT;
};

struct st_dmac1_dmtmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DCTG : 2;
  unsigned short  : 6;
  unsigned short SZ : 2;
  unsigned short  : 2;
  unsigned short DTS : 2;
  unsigned short MD : 2;
#else
  unsigned short MD : 2;
  unsigned short DTS : 2;
  unsigned short  : 2;
  unsigned short SZ : 2;
  unsigned short  : 6;
  unsigned short DCTG : 2;
#endif
};

union un_dmac1_dmtmd
{
  unsigned short WORD;
  struct st_dmac1_dmtmd_bit BIT;
};

struct st_dmac1_dmint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DARIE : 1;
  unsigned char SARIE : 1;
  unsigned char RPTIE : 1;
  unsigned char ESIE : 1;
  unsigned char DTIE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char DTIE : 1;
  unsigned char ESIE : 1;
  unsigned char RPTIE : 1;
  unsigned char SARIE : 1;
  unsigned char DARIE : 1;
#endif
};

union un_dmac1_dmint
{
  unsigned char BYTE;
  struct st_dmac1_dmint_bit BIT;
};

struct st_dmac1_dmamd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DARA : 5;
  unsigned short  : 1;
  unsigned short DM : 2;
  unsigned short SARA : 5;
  unsigned short  : 1;
  unsigned short SM : 2;
#else
  unsigned short SM : 2;
  unsigned short  : 1;
  unsigned short SARA : 5;
  unsigned short DM : 2;
  unsigned short  : 1;
  unsigned short DARA : 5;
#endif
};

union un_dmac1_dmamd
{
  unsigned short WORD;
  struct st_dmac1_dmamd_bit BIT;
};

struct st_dmac1_dmcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTE : 1;
#endif
};

union un_dmac1_dmcnt
{
  unsigned char BYTE;
  struct st_dmac1_dmcnt_bit BIT;
};

struct st_dmac1_dmreq_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWREQ : 1;
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
  unsigned char SWREQ : 1;
#endif
};

union un_dmac1_dmreq
{
  unsigned char BYTE;
  struct st_dmac1_dmreq_bit BIT;
};

struct st_dmac1_dmsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ESIF : 1;
  unsigned char  : 3;
  unsigned char DTIF : 1;
  unsigned char  : 2;
  unsigned char ACT : 1;
#else
  unsigned char ACT : 1;
  unsigned char  : 2;
  unsigned char DTIF : 1;
  unsigned char  : 3;
  unsigned char ESIF : 1;
#endif
};

union un_dmac1_dmsts
{
  unsigned char BYTE;
  struct st_dmac1_dmsts_bit BIT;
};

struct st_dmac1_dmcsl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DISEL : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DISEL : 1;
#endif
};

union un_dmac1_dmcsl
{
  unsigned char BYTE;
  struct st_dmac1_dmcsl_bit BIT;
};

struct st_drw2d_control_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LIM1EN : 1;
  unsigned long LIM2EN : 1;
  unsigned long LIM3EN : 1;
  unsigned long LIM4EN : 1;
  unsigned long LIM5EN : 1;
  unsigned long LIM6EN : 1;
  unsigned long QUAD1EN : 1;
  unsigned long QUAD2EN : 1;
  unsigned long QUAD3EN : 1;
  unsigned long LIM1TH : 1;
  unsigned long LIM2TH : 1;
  unsigned long LIM3TH : 1;
  unsigned long LIM4TH : 1;
  unsigned long LIM5TH : 1;
  unsigned long LIM6TH : 1;
  unsigned long BAND1EN : 1;
  unsigned long BAND2EN : 1;
  unsigned long UNION12 : 1;
  unsigned long UNION34 : 1;
  unsigned long UNION56 : 1;
  unsigned long UNIONAB : 1;
  unsigned long UNIONCD : 1;
  unsigned long SPANABT : 1;
  unsigned long SPANSTR : 1;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long SPANSTR : 1;
  unsigned long SPANABT : 1;
  unsigned long UNIONCD : 1;
  unsigned long UNIONAB : 1;
  unsigned long UNION56 : 1;
  unsigned long UNION34 : 1;
  unsigned long UNION12 : 1;
  unsigned long BAND2EN : 1;
  unsigned long BAND1EN : 1;
  unsigned long LIM6TH : 1;
  unsigned long LIM5TH : 1;
  unsigned long LIM4TH : 1;
  unsigned long LIM3TH : 1;
  unsigned long LIM2TH : 1;
  unsigned long LIM1TH : 1;
  unsigned long QUAD3EN : 1;
  unsigned long QUAD2EN : 1;
  unsigned long QUAD1EN : 1;
  unsigned long LIM6EN : 1;
  unsigned long LIM5EN : 1;
  unsigned long LIM4EN : 1;
  unsigned long LIM3EN : 1;
  unsigned long LIM2EN : 1;
  unsigned long LIM1EN : 1;
#endif
};

union un_drw2d_control
{
  unsigned long LONG;
  struct st_drw2d_control_bit BIT;
};

struct st_drw2d_status_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BSYENUM : 1;
  unsigned long BSYWR : 1;
  unsigned long CACHEDTY : 1;
  unsigned long DLSTACT : 1;
  unsigned long ENUIR : 1;
  unsigned long DLIR : 1;
  unsigned long  : 26;
#else
  unsigned long  : 26;
  unsigned long DLIR : 1;
  unsigned long ENUIR : 1;
  unsigned long DLSTACT : 1;
  unsigned long CACHEDTY : 1;
  unsigned long BSYWR : 1;
  unsigned long BSYENUM : 1;
#endif
};

union un_drw2d_status
{
  unsigned long LONG;
  struct st_drw2d_status_bit BIT;
};

struct st_drw2d_control2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PTNEN : 1;
  unsigned long TEXENA : 1;
  unsigned long PTNSRCL5 : 1;
  unsigned long USEACB : 1;
  unsigned long RDFMT2 : 2;
  unsigned long BSFA : 1;
  unsigned long BDFA : 1;
  unsigned long WRFMT2 : 1;
  unsigned long BSF : 1;
  unsigned long BDF : 1;
  unsigned long BSI : 1;
  unsigned long BDI : 1;
  unsigned long BC2 : 1;
  unsigned long TEXCLPX : 1;
  unsigned long TEXCLPY : 1;
  unsigned long TEXFILTX : 1;
  unsigned long TEXFILTY : 1;
  unsigned long RDFMT : 2;
  unsigned long WRFMT : 2;
  unsigned long WRALPHA : 2;
  unsigned long RLEEN : 1;
  unsigned long CLUTEN : 1;
  unsigned long COLKEYEN : 1;
  unsigned long CLUTFORM : 1;
  unsigned long BSIA : 1;
  unsigned long BDIA : 1;
  unsigned long RLEPIXW : 2;
#else
  unsigned long RLEPIXW : 2;
  unsigned long BDIA : 1;
  unsigned long BSIA : 1;
  unsigned long CLUTFORM : 1;
  unsigned long COLKEYEN : 1;
  unsigned long CLUTEN : 1;
  unsigned long RLEEN : 1;
  unsigned long WRALPHA : 2;
  unsigned long WRFMT : 2;
  unsigned long RDFMT : 2;
  unsigned long TEXFILTY : 1;
  unsigned long TEXFILTX : 1;
  unsigned long TEXCLPY : 1;
  unsigned long TEXCLPX : 1;
  unsigned long BC2 : 1;
  unsigned long BDI : 1;
  unsigned long BSI : 1;
  unsigned long BDF : 1;
  unsigned long BSF : 1;
  unsigned long WRFMT2 : 1;
  unsigned long BDFA : 1;
  unsigned long BSFA : 1;
  unsigned long RDFMT2 : 2;
  unsigned long USEACB : 1;
  unsigned long PTNSRCL5 : 1;
  unsigned long TEXENA : 1;
  unsigned long PTNEN : 1;
#endif
};

union un_drw2d_control2
{
  unsigned long LONG;
  struct st_drw2d_control2_bit BIT;
};

struct st_drw2d_hwver_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long REV : 12;
  unsigned long  : 5;
  unsigned long DLR : 1;
  unsigned long FBCACHE : 1;
  unsigned long TXCACHE : 1;
  unsigned long PERFCNT : 1;
  unsigned long TEXCLUT : 1;
  unsigned long  : 1;
  unsigned long RLEUNIT : 1;
  unsigned long TEXCLUT256 : 1;
  unsigned long COLKEY : 1;
  unsigned long  : 1;
  unsigned long ACBLD : 1;
  unsigned long  : 4;
#else
  unsigned long  : 4;
  unsigned long ACBLD : 1;
  unsigned long  : 1;
  unsigned long COLKEY : 1;
  unsigned long TEXCLUT256 : 1;
  unsigned long RLEUNIT : 1;
  unsigned long  : 1;
  unsigned long TEXCLUT : 1;
  unsigned long PERFCNT : 1;
  unsigned long TXCACHE : 1;
  unsigned long FBCACHE : 1;
  unsigned long DLR : 1;
  unsigned long  : 5;
  unsigned long REV : 12;
#endif
};

union un_drw2d_hwver
{
  unsigned long LONG;
  struct st_drw2d_hwver_bit BIT;
};

struct st_drw2d_color1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long COL1B : 8;
  unsigned long COL1G : 8;
  unsigned long COL1R : 8;
  unsigned long COL1A : 8;
#else
  unsigned long COL1A : 8;
  unsigned long COL1R : 8;
  unsigned long COL1G : 8;
  unsigned long COL1B : 8;
#endif
};

union un_drw2d_color1
{
  unsigned long LONG;
  struct st_drw2d_color1_bit BIT;
};

struct st_drw2d_color2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long COL2B : 8;
  unsigned long COL2G : 8;
  unsigned long COL2R : 8;
  unsigned long COL2A : 8;
#else
  unsigned long COL2A : 8;
  unsigned long COL2R : 8;
  unsigned long COL2G : 8;
  unsigned long COL2B : 8;
#endif
};

union un_drw2d_color2
{
  unsigned long LONG;
  struct st_drw2d_color2_bit BIT;
};

struct st_drw2d_size_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long X : 16;
  unsigned long Y : 16;
#else
  unsigned long Y : 16;
  unsigned long X : 16;
#endif
};

union un_drw2d_size
{
  unsigned long LONG;
  struct st_drw2d_size_bit BIT;
};

struct st_drw2d_pitch_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PITCH : 16;
  unsigned long SSD : 16;
#else
  unsigned long SSD : 16;
  unsigned long PITCH : 16;
#endif
};

union un_drw2d_pitch
{
  unsigned long LONG;
  struct st_drw2d_pitch_bit BIT;
};

struct st_drw2d_lvyxaddf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LVXADDF : 16;
  unsigned long LVYADDF : 16;
#else
  unsigned long LVYADDF : 16;
  unsigned long LVXADDF : 16;
#endif
};

union un_drw2d_lvyxaddf
{
  unsigned long LONG;
  struct st_drw2d_lvyxaddf_bit BIT;
};

struct st_drw2d_texmsk_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TEXUMSK : 11;
  unsigned long TEXVMSK : 21;
#else
  unsigned long TEXVMSK : 21;
  unsigned long TEXUMSK : 11;
#endif
};

union un_drw2d_texmsk
{
  unsigned long LONG;
  struct st_drw2d_texmsk_bit BIT;
};

struct st_drw2d_irqctl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ENUIREN : 1;
  unsigned long DLIREN : 1;
  unsigned long ENUIRCLR : 1;
  unsigned long DLIRCLR : 1;
  unsigned long  : 28;
#else
  unsigned long  : 28;
  unsigned long DLIRCLR : 1;
  unsigned long ENUIRCLR : 1;
  unsigned long DLIREN : 1;
  unsigned long ENUIREN : 1;
#endif
};

union un_drw2d_irqctl
{
  unsigned long LONG;
  struct st_drw2d_irqctl_bit BIT;
};

struct st_drw2d_cachectl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CENFX : 1;
  unsigned long CFLUFX : 1;
  unsigned long CENTX : 1;
  unsigned long CFLUTX : 1;
  unsigned long  : 28;
#else
  unsigned long  : 28;
  unsigned long CFLUTX : 1;
  unsigned long CENTX : 1;
  unsigned long CFLUFX : 1;
  unsigned long CENFX : 1;
#endif
};

union un_drw2d_cachectl
{
  unsigned long LONG;
  struct st_drw2d_cachectl_bit BIT;
};

struct st_drw2d_perftrg_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TRG1 : 16;
  unsigned long TRG2 : 16;
#else
  unsigned long TRG2 : 16;
  unsigned long TRG1 : 16;
#endif
};

union un_drw2d_perftrg
{
  unsigned long LONG;
  struct st_drw2d_perftrg_bit BIT;
};

struct st_drw2d_colkey_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long B : 8;
  unsigned long G : 8;
  unsigned long R : 8;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long R : 8;
  unsigned long G : 8;
  unsigned long B : 8;
#endif
};

union un_drw2d_colkey
{
  unsigned long LONG;
  struct st_drw2d_colkey_bit BIT;
};

struct st_dtc_dtccr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 4;
  unsigned char RRS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char RRS : 1;
  unsigned char  : 4;
#endif
};

union un_dtc_dtccr
{
  unsigned char BYTE;
  struct st_dtc_dtccr_bit BIT;
};

struct st_dtc_dtcadmod_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SHORT : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SHORT : 1;
#endif
};

union un_dtc_dtcadmod
{
  unsigned char BYTE;
  struct st_dtc_dtcadmod_bit BIT;
};

struct st_dtc_dtcst_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTCST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTCST : 1;
#endif
};

union un_dtc_dtcst
{
  unsigned char BYTE;
  struct st_dtc_dtcst_bit BIT;
};

struct st_dtc_dtcsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short VECN : 8;
  unsigned short  : 7;
  unsigned short ACT : 1;
#else
  unsigned short ACT : 1;
  unsigned short  : 7;
  unsigned short VECN : 8;
#endif
};

union un_dtc_dtcsts
{
  unsigned short WORD;
  struct st_dtc_dtcsts_bit BIT;
};

struct st_dtc_dtcor_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SQTFRL : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SQTFRL : 1;
#endif
};

union un_dtc_dtcor
{
  unsigned char BYTE;
  struct st_dtc_dtcor_bit BIT;
};

struct st_dtc_dtcsqe_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short VECN : 8;
  unsigned short  : 7;
  unsigned short ESPSEL : 1;
#else
  unsigned short ESPSEL : 1;
  unsigned short  : 7;
  unsigned short VECN : 8;
#endif
};

union un_dtc_dtcsqe
{
  unsigned short WORD;
  struct st_dtc_dtcsqe_bit BIT;
};

struct st_edmac_edmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long SWR : 1;
  unsigned long  : 3;
  unsigned long DL : 2;
  unsigned long DE : 1;
  unsigned long  : 25;
#else
  unsigned long  : 25;
  unsigned long DE : 1;
  unsigned long DL : 2;
  unsigned long  : 3;
  unsigned long SWR : 1;
#endif
};

union un_edmac_edmr
{
  unsigned long LONG;
  struct st_edmac_edmr_bit BIT;
};

struct st_edmac_edtrr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TR : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long TR : 1;
#endif
};

union un_edmac_edtrr
{
  unsigned long LONG;
  struct st_edmac_edtrr_bit BIT;
};

struct st_edmac_edrrr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RR : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long RR : 1;
#endif
};

union un_edmac_edrrr
{
  unsigned long LONG;
  struct st_edmac_edrrr_bit BIT;
};

struct st_edmac_eesr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CERF : 1;
  unsigned long PRE : 1;
  unsigned long RTSF : 1;
  unsigned long RTLF : 1;
  unsigned long RRF : 1;
  unsigned long  : 2;
  unsigned long RMAF : 1;
  unsigned long TRO : 1;
  unsigned long CD : 1;
  unsigned long DLC : 1;
  unsigned long CND : 1;
  unsigned long  : 4;
  unsigned long RFOF : 1;
  unsigned long RDE : 1;
  unsigned long FR : 1;
  unsigned long TFUF : 1;
  unsigned long TDE : 1;
  unsigned long TC : 1;
  unsigned long ECI : 1;
  unsigned long  : 1;
  unsigned long RFCOF : 1;
  unsigned long RABT : 1;
  unsigned long TABT : 1;
  unsigned long  : 3;
  unsigned long TWB : 1;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long TWB : 1;
  unsigned long  : 3;
  unsigned long TABT : 1;
  unsigned long RABT : 1;
  unsigned long RFCOF : 1;
  unsigned long  : 1;
  unsigned long ECI : 1;
  unsigned long TC : 1;
  unsigned long TDE : 1;
  unsigned long TFUF : 1;
  unsigned long FR : 1;
  unsigned long RDE : 1;
  unsigned long RFOF : 1;
  unsigned long  : 4;
  unsigned long CND : 1;
  unsigned long DLC : 1;
  unsigned long CD : 1;
  unsigned long TRO : 1;
  unsigned long RMAF : 1;
  unsigned long  : 2;
  unsigned long RRF : 1;
  unsigned long RTLF : 1;
  unsigned long RTSF : 1;
  unsigned long PRE : 1;
  unsigned long CERF : 1;
#endif
};

union un_edmac_eesr
{
  unsigned long LONG;
  struct st_edmac_eesr_bit BIT;
};

struct st_edmac_eesipr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long CERFIP : 1;
  unsigned long PREIP : 1;
  unsigned long RTSFIP : 1;
  unsigned long RTLFIP : 1;
  unsigned long RRFIP : 1;
  unsigned long  : 2;
  unsigned long RMAFIP : 1;
  unsigned long TROIP : 1;
  unsigned long CDIP : 1;
  unsigned long DLCIP : 1;
  unsigned long CNDIP : 1;
  unsigned long  : 4;
  unsigned long RFOFIP : 1;
  unsigned long RDEIP : 1;
  unsigned long FRIP : 1;
  unsigned long TFUFIP : 1;
  unsigned long TDEIP : 1;
  unsigned long TCIP : 1;
  unsigned long ECIIP : 1;
  unsigned long  : 1;
  unsigned long RFCOFIP : 1;
  unsigned long RABTIP : 1;
  unsigned long TABTIP : 1;
  unsigned long  : 3;
  unsigned long TWBIP : 1;
  unsigned long  : 1;
#else
  unsigned long  : 1;
  unsigned long TWBIP : 1;
  unsigned long  : 3;
  unsigned long TABTIP : 1;
  unsigned long RABTIP : 1;
  unsigned long RFCOFIP : 1;
  unsigned long  : 1;
  unsigned long ECIIP : 1;
  unsigned long TCIP : 1;
  unsigned long TDEIP : 1;
  unsigned long TFUFIP : 1;
  unsigned long FRIP : 1;
  unsigned long RDEIP : 1;
  unsigned long RFOFIP : 1;
  unsigned long  : 4;
  unsigned long CNDIP : 1;
  unsigned long DLCIP : 1;
  unsigned long CDIP : 1;
  unsigned long TROIP : 1;
  unsigned long RMAFIP : 1;
  unsigned long  : 2;
  unsigned long RRFIP : 1;
  unsigned long RTLFIP : 1;
  unsigned long RTSFIP : 1;
  unsigned long PREIP : 1;
  unsigned long CERFIP : 1;
#endif
};

union un_edmac_eesipr
{
  unsigned long LONG;
  struct st_edmac_eesipr_bit BIT;
};

struct st_edmac_trscer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 4;
  unsigned long RRFCE : 1;
  unsigned long  : 2;
  unsigned long RMAFCE : 1;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long RMAFCE : 1;
  unsigned long  : 2;
  unsigned long RRFCE : 1;
  unsigned long  : 4;
#endif
};

union un_edmac_trscer
{
  unsigned long LONG;
  struct st_edmac_trscer_bit BIT;
};

struct st_edmac_rmfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MFC : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long MFC : 16;
#endif
};

union un_edmac_rmfcr
{
  unsigned long LONG;
  struct st_edmac_rmfcr_bit BIT;
};

struct st_edmac_tftr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TFT : 11;
  unsigned long  : 21;
#else
  unsigned long  : 21;
  unsigned long TFT : 11;
#endif
};

union un_edmac_tftr
{
  unsigned long LONG;
  struct st_edmac_tftr_bit BIT;
};

struct st_edamc_fdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RFD : 5;
  unsigned long  : 3;
  unsigned long TFD : 5;
  unsigned long  : 19;
#else
  unsigned long  : 19;
  unsigned long TFD : 5;
  unsigned long  : 3;
  unsigned long RFD : 5;
#endif
};

union un_edmac_fdr
{
  unsigned long LONG;
  struct st_edamc_fdr_bit BIT;
};

struct st_edmac_rmcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RNR : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long RNR : 1;
#endif
};

union un_edmac_rmcr
{
  unsigned long LONG;
  struct st_edmac_rmcr_bit BIT;
};

struct st_edmac_tfucr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long UNDER : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long UNDER : 16;
#endif
};

union un_edmac_tfucr
{
  unsigned long LONG;
  struct st_edmac_tfucr_bit BIT;
};

struct st_edmac_rfocr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long OVER : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long OVER : 16;
#endif
};

union un_edmac_rfocr
{
  unsigned long LONG;
  struct st_edmac_rfocr_bit BIT;
};

struct st_edmac_iosr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ELB : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long ELB : 1;
#endif
};

union un_edmac_iosr
{
  unsigned long LONG;
  struct st_edmac_iosr_bit BIT;
};

struct st_edmac_fcftr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RFDO : 3;
  unsigned long  : 13;
  unsigned long RFFO : 3;
  unsigned long  : 13;
#else
  unsigned long  : 13;
  unsigned long RFFO : 3;
  unsigned long  : 13;
  unsigned long RFDO : 3;
#endif
};

union un_edmac_fcftr
{
  unsigned long LONG;
  struct st_edmac_fcftr_bit BIT;
};

struct st_edmac_rpadir_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PADR : 6;
  unsigned long  : 10;
  unsigned long PADS : 2;
  unsigned long  : 14;
#else
  unsigned long  : 14;
  unsigned long PADS : 2;
  unsigned long  : 10;
  unsigned long PADR : 6;
#endif
};

union un_edmac_rpadir
{
  unsigned long LONG;
  struct st_edmac_rpadir_bit BIT;
};

struct st_edmac_trmid_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TIS : 1;
  unsigned long  : 3;
  unsigned long TIM : 1;
  unsigned long  : 27;
#else
  unsigned long  : 27;
  unsigned long TIM : 1;
  unsigned long  : 3;
  unsigned long TIS : 1;
#endif
};

union un_edmac_trimd
{
  unsigned long LONG;
  struct st_edmac_trmid_bit BIT;
};

struct st_elc_elcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char ELCON : 1;
#else
  unsigned char ELCON : 1;
  unsigned char  : 7;
#endif
};

union un_elc_elcr
{
  unsigned char BYTE;
  struct st_elc_elcr_bit BIT;
};

struct st_elc_elsr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr0
{
  unsigned char BYTE;
  struct st_elc_elsr0_bit BIT;
};

struct st_elc_elsr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr3
{
  unsigned char BYTE;
  struct st_elc_elsr3_bit BIT;
};

struct st_elc_elsr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr4
{
  unsigned char BYTE;
  struct st_elc_elsr4_bit BIT;
};

struct st_elc_elsr7_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr7
{
  unsigned char BYTE;
  struct st_elc_elsr7_bit BIT;
};

struct st_elc_elsr10_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr10
{
  unsigned char BYTE;
  struct st_elc_elsr10_bit BIT;
};

struct st_elc_elsr11_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr11
{
  unsigned char BYTE;
  struct st_elc_elsr11_bit BIT;
};

struct st_elc_elsr12_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr12
{
  unsigned char BYTE;
  struct st_elc_elsr12_bit BIT;
};

struct st_elc_elsr13_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr13
{
  unsigned char BYTE;
  struct st_elc_elsr13_bit BIT;
};

struct st_elc_elsr15_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr15
{
  unsigned char BYTE;
  struct st_elc_elsr15_bit BIT;
};

struct st_elc_elsr16_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr16
{
  unsigned char BYTE;
  struct st_elc_elsr16_bit BIT;
};

struct st_elc_elsr18_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr18
{
  unsigned char BYTE;
  struct st_elc_elsr18_bit BIT;
};

struct st_elc_elsr19_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr19
{
  unsigned char BYTE;
  struct st_elc_elsr19_bit BIT;
};

struct st_elc_elsr20_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr20
{
  unsigned char BYTE;
  struct st_elc_elsr20_bit BIT;
};

struct st_elc_elsr21_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr21
{
  unsigned char BYTE;
  struct st_elc_elsr21_bit BIT;
};

struct st_elc_elsr22_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr22
{
  unsigned char BYTE;
  struct st_elc_elsr22_bit BIT;
};

struct st_elc_elsr23_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr23
{
  unsigned char BYTE;
  struct st_elc_elsr23_bit BIT;
};

struct st_elc_elsr24_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr24
{
  unsigned char BYTE;
  struct st_elc_elsr24_bit BIT;
};

struct st_elc_elsr25_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr25
{
  unsigned char BYTE;
  struct st_elc_elsr25_bit BIT;
};

struct st_elc_elsr26_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr26
{
  unsigned char BYTE;
  struct st_elc_elsr26_bit BIT;
};

struct st_elc_elsr27_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr27
{
  unsigned char BYTE;
  struct st_elc_elsr27_bit BIT;
};

struct st_elc_elsr28_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr28
{
  unsigned char BYTE;
  struct st_elc_elsr28_bit BIT;
};

struct st_elc_elopa_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MTU0MD : 2;
  unsigned char  : 4;
  unsigned char MTU3MD : 2;
#else
  unsigned char MTU3MD : 2;
  unsigned char  : 4;
  unsigned char MTU0MD : 2;
#endif
};

union un_elc_elopa
{
  unsigned char BYTE;
  struct st_elc_elopa_bit BIT;
};

struct st_elc_elopb_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MTU4MD : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char MTU4MD : 2;
#endif
};

union un_elc_elopb
{
  unsigned char BYTE;
  struct st_elc_elopb_bit BIT;
};

struct st_elc_elopc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 2;
  unsigned char CMT1MD : 2;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char CMT1MD : 2;
  unsigned char  : 2;
#endif
};

union un_elc_elopc
{
  unsigned char BYTE;
  struct st_elc_elopc_bit BIT;
};

struct st_elc_elopd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TMR0MD : 2;
  unsigned char TMR1MD : 2;
  unsigned char TMR2MD : 2;
  unsigned char TMR3MD : 2;
#else
  unsigned char TMR3MD : 2;
  unsigned char TMR2MD : 2;
  unsigned char TMR1MD : 2;
  unsigned char TMR0MD : 2;
#endif
};

union un_elc_elopd
{
  unsigned char BYTE;
  struct st_elc_elopd_bit BIT;
};

struct st_elc_pgr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PGR0 : 1;
  unsigned char PGR1 : 1;
  unsigned char PGR2 : 1;
  unsigned char PGR3 : 1;
  unsigned char PGR4 : 1;
  unsigned char PGR5 : 1;
  unsigned char PGR6 : 1;
  unsigned char PGR7 : 1;
#else
  unsigned char PGR7 : 1;
  unsigned char PGR6 : 1;
  unsigned char PGR5 : 1;
  unsigned char PGR4 : 1;
  unsigned char PGR3 : 1;
  unsigned char PGR2 : 1;
  unsigned char PGR1 : 1;
  unsigned char PGR0 : 1;
#endif
};

union un_elc_pgr1
{
  unsigned char BYTE;
  struct st_elc_pgr1_bit BIT;
};

struct st_elc_pgr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PGR0 : 1;
  unsigned char PGR1 : 1;
  unsigned char PGR2 : 1;
  unsigned char PGR3 : 1;
  unsigned char PGR4 : 1;
  unsigned char PGR5 : 1;
  unsigned char PGR6 : 1;
  unsigned char PGR7 : 1;
#else
  unsigned char PGR7 : 1;
  unsigned char PGR6 : 1;
  unsigned char PGR5 : 1;
  unsigned char PGR4 : 1;
  unsigned char PGR3 : 1;
  unsigned char PGR2 : 1;
  unsigned char PGR1 : 1;
  unsigned char PGR0 : 1;
#endif
};

union un_elc_pgr2
{
  unsigned char BYTE;
  struct st_elc_pgr2_bit BIT;
};

struct st_elc_pgc1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PGCI : 2;
  unsigned char PGCOVE : 1;
  unsigned char  : 1;
  unsigned char PGCO : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PGCO : 3;
  unsigned char  : 1;
  unsigned char PGCOVE : 1;
  unsigned char PGCI : 2;
#endif
};

union un_elc_pgc1
{
  unsigned char BYTE;
  struct st_elc_pgc1_bit BIT;
};

struct st_elc_pgc2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PGCI : 2;
  unsigned char PGCOVE : 1;
  unsigned char  : 1;
  unsigned char PGCO : 3;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PGCO : 3;
  unsigned char  : 1;
  unsigned char PGCOVE : 1;
  unsigned char PGCI : 2;
#endif
};

union un_elc_pgc2
{
  unsigned char BYTE;
  struct st_elc_pgc2_bit BIT;
};

struct st_elc_pdbf1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PDBF0 : 1;
  unsigned char PDBF1 : 1;
  unsigned char PDBF2 : 1;
  unsigned char PDBF3 : 1;
  unsigned char PDBF4 : 1;
  unsigned char PDBF5 : 1;
  unsigned char PDBF6 : 1;
  unsigned char PDBF7 : 1;
#else
  unsigned char PDBF7 : 1;
  unsigned char PDBF6 : 1;
  unsigned char PDBF5 : 1;
  unsigned char PDBF4 : 1;
  unsigned char PDBF3 : 1;
  unsigned char PDBF2 : 1;
  unsigned char PDBF1 : 1;
  unsigned char PDBF0 : 1;
#endif
};

union un_elc_pdbf1
{
  unsigned char BYTE;
  struct st_elc_pdbf1_bit BIT;
};

struct st_elc_pdbf2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PDBF0 : 1;
  unsigned char PDBF1 : 1;
  unsigned char PDBF2 : 1;
  unsigned char PDBF3 : 1;
  unsigned char PDBF4 : 1;
  unsigned char PDBF5 : 1;
  unsigned char PDBF6 : 1;
  unsigned char PDBF7 : 1;
#else
  unsigned char PDBF7 : 1;
  unsigned char PDBF6 : 1;
  unsigned char PDBF5 : 1;
  unsigned char PDBF4 : 1;
  unsigned char PDBF3 : 1;
  unsigned char PDBF2 : 1;
  unsigned char PDBF1 : 1;
  unsigned char PDBF0 : 1;
#endif
};

union un_elc_pdbf2
{
  unsigned char BYTE;
  struct st_elc_pdbf2_bit BIT;
};

struct st_elc_pel0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSB : 3;
  unsigned char PSP : 2;
  unsigned char PSM : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSM : 2;
  unsigned char PSP : 2;
  unsigned char PSB : 3;
#endif
};

union un_elc_pel0
{
  unsigned char BYTE;
  struct st_elc_pel0_bit BIT;
};

struct st_elc_pel1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSB : 3;
  unsigned char PSP : 2;
  unsigned char PSM : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSM : 2;
  unsigned char PSP : 2;
  unsigned char PSB : 3;
#endif
};

union un_elc_pel1
{
  unsigned char BYTE;
  struct st_elc_pel1_bit BIT;
};

struct st_elc_pel2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSB : 3;
  unsigned char PSP : 2;
  unsigned char PSM : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSM : 2;
  unsigned char PSP : 2;
  unsigned char PSB : 3;
#endif
};

union un_elc_pel2
{
  unsigned char BYTE;
  struct st_elc_pel2_bit BIT;
};

struct un_elc_pel3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PSB : 3;
  unsigned char PSP : 2;
  unsigned char PSM : 2;
  unsigned char  : 1;
#else
  unsigned char  : 1;
  unsigned char PSM : 2;
  unsigned char PSP : 2;
  unsigned char PSB : 3;
#endif
};

union un_elc_pel3
{
  unsigned char BYTE;
  struct un_elc_pel3_bit BIT;
};

struct st_elc_elsegr_bit
{
  unsigned char WI:1;
  unsigned char WE:1;
  unsigned char :5;
  unsigned char SEG:1;
};

union un_elc_elsegr
{
  unsigned char BYTE;
#ifdef IODEFINE_H_HISTORY
  struct st_elc_elsegr_bit BIT;
#endif
};

struct st_elc_elsr33_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr33
{
  unsigned char BYTE;
  struct st_elc_elsr33_bit BIT;
};

struct st_elc_elsr35_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr35
{
  unsigned char BYTE;
  struct st_elc_elsr35_bit BIT;
};

struct st_elc_elsr36_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr36
{
  unsigned char BYTE;
  struct st_elc_elsr36_bit BIT;
};

struct st_elc_elsr37_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr37
{
  unsigned char BYTE;
  struct st_elc_elsr37_bit BIT;
};

struct st_elc_elsr38_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr38
{
  unsigned char BYTE;
  struct st_elc_elsr38_bit BIT;
};

struct st_elc_elsr45_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ELS : 8;
#else
  unsigned char ELS : 8;
#endif
};

union un_elc_elsr45
{
  unsigned char BYTE;
  struct st_elc_elsr45_bit BIT;
};

struct st_elc_elopf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TPU0MD : 2;
  unsigned char TPU1MD : 2;
  unsigned char TPU2MD : 2;
  unsigned char TPU3MD : 2;
#else
  unsigned char TPU3MD : 2;
  unsigned char TPU2MD : 2;
  unsigned char TPU1MD : 2;
  unsigned char TPU0MD : 2;
#endif
};

union un_elc_elopf
{
  unsigned char BYTE;
  struct st_elc_elopf_bit BIT;
};

struct st_elc_eloph_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMTW0MD : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char CMTW0MD : 2;
#endif
};

union un_elc_eloph
{
  unsigned char BYTE;
  struct st_elc_eloph_bit BIT;
};

struct st_etherc_ecmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PRM : 1;
  unsigned long DM : 1;
  unsigned long RTM : 1;
  unsigned long ILB : 1;
  unsigned long  : 1;
  unsigned long TE : 1;
  unsigned long RE : 1;
  unsigned long  : 2;
  unsigned long MPDE : 1;
  unsigned long  : 2;
  unsigned long PRCEF : 1;
  unsigned long  : 3;
  unsigned long TXF : 1;
  unsigned long RXF : 1;
  unsigned long PFR : 1;
  unsigned long ZPF : 1;
  unsigned long TPC : 1;
  unsigned long  : 11;
#else
  unsigned long  : 11;
  unsigned long TPC : 1;
  unsigned long ZPF : 1;
  unsigned long PFR : 1;
  unsigned long RXF : 1;
  unsigned long TXF : 1;
  unsigned long  : 3;
  unsigned long PRCEF : 1;
  unsigned long  : 2;
  unsigned long MPDE : 1;
  unsigned long  : 2;
  unsigned long RE : 1;
  unsigned long TE : 1;
  unsigned long  : 1;
  unsigned long ILB : 1;
  unsigned long RTM : 1;
  unsigned long DM : 1;
  unsigned long PRM : 1;
#endif
};

union un_etherc_ecmr
{
  unsigned long LONG;
  struct st_etherc_ecmr_bit BIT;
};

struct st_etherc_rflr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RFL : 12;
  unsigned long  : 20;
#else
  unsigned long  : 20;
  unsigned long RFL : 12;
#endif
};

union un_etherc_rflr
{
  unsigned long LONG;
  struct st_etherc_rflr_bit BIT;
};

struct st_etherc_ecsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ICD : 1;
  unsigned long MPD : 1;
  unsigned long LCHNG : 1;
  unsigned long  : 1;
  unsigned long PSRTO : 1;
  unsigned long BFR : 1;
  unsigned long  : 26;
#else
  unsigned long  : 26;
  unsigned long BFR : 1;
  unsigned long PSRTO : 1;
  unsigned long  : 1;
  unsigned long LCHNG : 1;
  unsigned long MPD : 1;
  unsigned long ICD : 1;
#endif
};

union un_etherc_ecsr
{
  unsigned long LONG;
  struct st_etherc_ecsr_bit BIT;
};

struct st_etherc_ecsipr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long ICDIP : 1;
  unsigned long MPDIP : 1;
  unsigned long LCHNGIP : 1;
  unsigned long  : 1;
  unsigned long PSRTOIP : 1;
  unsigned long BFSIPR : 1;
  unsigned long  : 26;
#else
  unsigned long  : 26;
  unsigned long BFSIPR : 1;
  unsigned long PSRTOIP : 1;
  unsigned long  : 1;
  unsigned long LCHNGIP : 1;
  unsigned long MPDIP : 1;
  unsigned long ICDIP : 1;
#endif
};

union un_etherc_ecsipr
{
  unsigned long LONG;
  struct st_etherc_ecsipr_bit BIT;
};

struct st_etherc_pir_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MDC : 1;
  unsigned long MMD : 1;
  unsigned long MDO : 1;
  unsigned long MDI : 1;
  unsigned long  : 28;
#else
  unsigned long  : 28;
  unsigned long MDI : 1;
  unsigned long MDO : 1;
  unsigned long MMD : 1;
  unsigned long MDC : 1;
#endif
};

union un_etherc_pir
{
  unsigned long LONG;
  struct st_etherc_pir_bit BIT;
};

struct st_etherc_psr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long LMON : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long LMON : 1;
#endif
};

union un_etherc_psr
{
  unsigned long LONG;
  struct st_etherc_psr_bit BIT;
};

struct st_etherc_rdmlr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RMD : 20;
  unsigned long  : 12;
#else
  unsigned long  : 12;
  unsigned long RMD : 20;
#endif
};

union un_etherc_rdmlr
{
  unsigned long LONG;
  struct st_etherc_rdmlr_bit BIT;
};

struct st_etherc_ipgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long IPG : 5;
  unsigned long  : 27;
#else
  unsigned long  : 27;
  unsigned long IPG : 5;
#endif
};

union un_etherc_ipgr
{
  unsigned long LONG;
  struct st_etherc_ipgr_bit BIT;
};

struct st_etherc_apr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long AP : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long AP : 16;
#endif
};

union un_etherc_apr
{
  unsigned long LONG;
  struct st_etherc_apr_bit BIT;
};

struct st_etherc_mpr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MP : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long MP : 16;
#endif
};

union un_etherc_mpr
{
  unsigned long LONG;
  struct st_etherc_mpr_bit BIT;
};

struct st_etherc_rfcf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long RPAUSE : 8;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long RPAUSE : 8;
#endif
};

union un_etherc_rfcf
{
  unsigned long LONG;
  struct st_etherc_rfcf_bit BIT;
};

struct st_etherc_tpauser_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TPAUSE : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long TPAUSE : 16;
#endif
};

union un_etherc_tpauser
{
  unsigned long LONG;
  struct st_etherc_tpauser_bit BIT;
};

struct st_etherc_tpausecr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long TXP : 8;
  unsigned long  : 24;
#else
  unsigned long  : 24;
  unsigned long TXP : 8;
#endif
};

union un_etherc_tpausecr
{
  unsigned long LONG;
  struct st_etherc_tpausecr_bit BIT;
};

struct st_etherc_bcfrr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long BCF : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long BCF : 16;
#endif
};

union un_etherc_bcfrr
{
  unsigned long LONG;
  struct st_etherc_bcfrr_bit BIT;
};

struct st_etherc_malr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long MA : 16;
  unsigned long  : 16;
#else
  unsigned long  : 16;
  unsigned long MA : 16;
#endif
};

union un_etherc_malr
{
  unsigned long LONG;
  struct st_etherc_malr_bit BIT;
};

struct st_exdmac_edmast_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DMST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DMST : 1;
#endif
};

union un_exdmac_edmast
{
  unsigned char BYTE;
  struct st_exdmac_edmast_bit BIT;
};

struct st_exdmac0_edmtmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DCTG : 2;
  unsigned short  : 6;
  unsigned short SZ : 2;
  unsigned short  : 2;
  unsigned short DTS : 2;
  unsigned short MD : 2;
#else
  unsigned short MD : 2;
  unsigned short DTS : 2;
  unsigned short  : 2;
  unsigned short SZ : 2;
  unsigned short  : 6;
  unsigned short DCTG : 2;
#endif
};

union un_exdmac0_edmtmd
{
  unsigned short WORD;
  struct st_exdmac0_edmtmd_bit BIT;
};

struct st_exdamc0_edmomd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DACKSEL : 1;
  unsigned char DACKW : 1;
  unsigned char DACKE : 1;
  unsigned char DACKS : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char DACKS : 1;
  unsigned char DACKE : 1;
  unsigned char DACKW : 1;
  unsigned char DACKSEL : 1;
#endif
};

union un_exdmac0_edmomd
{
  unsigned char BYTE;
  struct st_exdamc0_edmomd_bit BIT;
};

struct st_exdmac0_edmint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DARIE : 1;
  unsigned char SARIE : 1;
  unsigned char RPTIE : 1;
  unsigned char ESIE : 1;
  unsigned char DTIE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char DTIE : 1;
  unsigned char ESIE : 1;
  unsigned char RPTIE : 1;
  unsigned char SARIE : 1;
  unsigned char DARIE : 1;
#endif
};

union un_exdmac0_edmint
{
  unsigned char BYTE;
  struct st_exdmac0_edmint_bit BIT;
};

struct st_exdmac0_edmamd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DARA : 5;
  unsigned long  : 1;
  unsigned long DM : 2;
  unsigned long SARA : 5;
  unsigned long  : 1;
  unsigned long SM : 2;
  unsigned long DIR : 1;
  unsigned long AMS : 1;
  unsigned long  : 14;
#else
  unsigned long  : 14;
  unsigned long AMS : 1;
  unsigned long DIR : 1;
  unsigned long SM : 2;
  unsigned long  : 1;
  unsigned long SARA : 5;
  unsigned long DM : 2;
  unsigned long  : 1;
  unsigned long DARA : 5;
#endif
};

union un_exdmac0_edmamd
{
  unsigned long LONG;
  struct st_exdmac0_edmamd_bit BIT;
};

struct st_exdmac0_edmcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTE : 1;
#endif
};

union un_exdmac0_edmcnt
{
  unsigned char BYTE;
  struct st_exdmac0_edmcnt_bit BIT;
};

struct st_exdmac0_edmreq_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWREQ : 1;
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
  unsigned char SWREQ : 1;
#endif
};

union un_exdmac0_edmreq
{
  unsigned char BYTE;
  struct st_exdmac0_edmreq_bit BIT;
};

struct st_exdmac0_edmsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ESIF : 1;
  unsigned char  : 3;
  unsigned char DTIF : 1;
  unsigned char  : 2;
  unsigned char ACT : 1;
#else
  unsigned char ACT : 1;
  unsigned char  : 2;
  unsigned char DTIF : 1;
  unsigned char  : 3;
  unsigned char ESIF : 1;
#endif
};

union un_exdmac0_edmsts
{
  unsigned char BYTE;
  struct st_exdmac0_edmsts_bit BIT;
};

struct st_exdmac0_edmrmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DREQS : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char DREQS : 2;
#endif
};

union un_exdmac0_edmrmd
{
  unsigned char BYTE;
  struct st_exdmac0_edmrmd_bit BIT;
};

struct st_exdmac0_edmerf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EREQ : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char EREQ : 1;
#endif
};

union un_exdmac0_edmerf
{
  unsigned char BYTE;
  struct st_exdmac0_edmerf_bit BIT;
};

struct st_exdmac0_edmprf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PREQ : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char PREQ : 1;
#endif
};

union un_exdmac0_edmprf
{
  unsigned char BYTE;
  struct st_exdmac0_edmprf_bit BIT;
};

struct st_exdmac1_edmtmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DCTG : 2;
  unsigned short  : 6;
  unsigned short SZ : 2;
  unsigned short  : 2;
  unsigned short DTS : 2;
  unsigned short MD : 2;
#else
  unsigned short MD : 2;
  unsigned short DTS : 2;
  unsigned short  : 2;
  unsigned short SZ : 2;
  unsigned short  : 6;
  unsigned short DCTG : 2;
#endif
};

union un_exdmac1_edmtmd
{
  unsigned short WORD;
  struct st_exdmac1_edmtmd_bit BIT;
};

struct st_exdmac1_edmomd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DACKSEL : 1;
  unsigned char DACKW : 1;
  unsigned char DACKE : 1;
  unsigned char DACKS : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char DACKS : 1;
  unsigned char DACKE : 1;
  unsigned char DACKW : 1;
  unsigned char DACKSEL : 1;
#endif
};

union un_exdmac1_edmomd
{
  unsigned char BYTE;
  struct st_exdmac1_edmomd_bit BIT;
};

struct st_exdmac1_edmint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DARIE : 1;
  unsigned char SARIE : 1;
  unsigned char RPTIE : 1;
  unsigned char ESIE : 1;
  unsigned char DTIE : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char DTIE : 1;
  unsigned char ESIE : 1;
  unsigned char RPTIE : 1;
  unsigned char SARIE : 1;
  unsigned char DARIE : 1;
#endif
};

union un_exdmac1_edmint
{
  unsigned char BYTE;
  struct st_exdmac1_edmint_bit BIT;
};

struct st_exdmac1_edmamd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long DARA : 5;
  unsigned long  : 1;
  unsigned long DM : 2;
  unsigned long SARA : 5;
  unsigned long  : 1;
  unsigned long SM : 2;
  unsigned long DIR : 1;
  unsigned long AMS : 1;
  unsigned long  : 14;
#else
  unsigned long  : 14;
  unsigned long AMS : 1;
  unsigned long DIR : 1;
  unsigned long SM : 2;
  unsigned long  : 1;
  unsigned long SARA : 5;
  unsigned long DM : 2;
  unsigned long  : 1;
  unsigned long DARA : 5;
#endif
};

union un_exdmac1_edmamd
{
  unsigned long LONG;
  struct st_exdmac1_edmamd_bit BIT;
};

struct st_exdmac1_edmcnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DTE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char DTE : 1;
#endif
};

union un_exdmac1_edmcnt
{
  unsigned char BYTE;
  struct st_exdmac1_edmcnt_bit BIT;
};

struct st_exdmac1_edmreq_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SWREQ : 1;
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char CLRS : 1;
  unsigned char  : 3;
  unsigned char SWREQ : 1;
#endif
};

union un_exdmac1_edmreq
{
  unsigned char BYTE;
  struct st_exdmac1_edmreq_bit BIT;
};

struct st_exdmac1_edmsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ESIF : 1;
  unsigned char  : 3;
  unsigned char DTIF : 1;
  unsigned char  : 2;
  unsigned char ACT : 1;
#else
  unsigned char ACT : 1;
  unsigned char  : 2;
  unsigned char DTIF : 1;
  unsigned char  : 3;
  unsigned char ESIF : 1;
#endif
};

union un_exdmac1_edmsts
{
  unsigned char BYTE;
  struct st_exdmac1_edmsts_bit BIT;
};

struct st_exdmac1_edmrmd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char DREQS : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char DREQS : 2;
#endif
};

union un_exdmac1_edmrmd
{
  unsigned char BYTE;
  struct st_exdmac1_edmrmd_bit BIT;
};

struct st_exdmac1_edmerf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EREQ : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char EREQ : 1;
#endif
};

union un_exdmac1_edmerf
{
  unsigned char BYTE;
  struct st_exdmac1_edmerf_bit BIT;
};

struct st_exdmac1_edmprf_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PREQ : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char PREQ : 1;
#endif
};

union un_exdmac1_edmprf
{
  unsigned char BYTE;
  struct st_exdmac1_edmprf_bit BIT;
};

struct st_flash_romce_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ROMCEN : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short ROMCEN : 1;
#endif
};

union un_flash_romce
{
  unsigned short WORD;
  struct st_flash_romce_bit BIT;
};

struct st_flash_romciv_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ROMCIV : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short ROMCIV : 1;
#endif
};

union un_flash_romciv
{
  unsigned short WORD;
  struct st_flash_romciv_bit BIT;
};

struct st_flash_fwepror_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FLWE : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char FLWE : 2;
#endif
};

union un_flash_fwepror
{
  unsigned char BYTE;
  struct st_flash_fwepror_bit BIT;
};

struct st_flash_fastat_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 3;
  unsigned char DFAE : 1;
  unsigned char CMDLK : 1;
  unsigned char  : 2;
  unsigned char CFAE : 1;
#else
  unsigned char CFAE : 1;
  unsigned char  : 2;
  unsigned char CMDLK : 1;
  unsigned char DFAE : 1;
  unsigned char  : 3;
#endif
};

union un_flash_fastat
{
  unsigned char BYTE;
  struct st_flash_fastat_bit BIT;
};

struct st_flash_faeint_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 3;
  unsigned char DFAEIE : 1;
  unsigned char CMDLKIE : 1;
  unsigned char  : 2;
  unsigned char CFAEIE : 1;
#else
  unsigned char CFAEIE : 1;
  unsigned char  : 2;
  unsigned char CMDLKIE : 1;
  unsigned char DFAEIE : 1;
  unsigned char  : 3;
#endif
};

union un_flash_faeint
{
  unsigned char BYTE;
  struct st_flash_faeint_bit BIT;
};

struct st_flash_frdyie_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char FRDYIE : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char FRDYIE : 1;
#endif
};

union un_flash_frdyie
{
  unsigned char BYTE;
  struct st_flash_frdyie_bit BIT;
};

struct st_flash_fsaddr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FSADDR : 32;
#else
  unsigned long FSADDR : 32;
#endif
};

union un_flash_fsaddr
{
  unsigned long LONG;
  struct st_flash_fsaddr_bit BIT;
};

struct st_flash_feaddr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FEADDR : 32;
#else
  unsigned long FEADDR : 32;
#endif
};

union un_flash_feaddr
{
  unsigned long LONG;
  struct st_flash_feaddr_bit BIT;
};

struct st_flash_fstsatr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 6;
  unsigned long FLWEERR : 1;
  unsigned long  : 1;
  unsigned long PRGSPD : 1;
  unsigned long ERSSPD : 1;
  unsigned long DBFULL : 1;
  unsigned long SUSRDY : 1;
  unsigned long PRGERR : 1;
  unsigned long ERSERR : 1;
  unsigned long ILGLERR : 1;
  unsigned long FRDY : 1;
  unsigned long  : 4;
  unsigned long OTERR : 1;
  unsigned long SECERR : 1;
  unsigned long FESETERR : 1;
  unsigned long ILGCOMERR : 1;
  unsigned long  : 8;
#else
  unsigned long  : 8;
  unsigned long ILGCOMERR : 1;
  unsigned long FESETERR : 1;
  unsigned long SECERR : 1;
  unsigned long OTERR : 1;
  unsigned long  : 4;
  unsigned long FRDY : 1;
  unsigned long ILGLERR : 1;
  unsigned long ERSERR : 1;
  unsigned long PRGERR : 1;
  unsigned long SUSRDY : 1;
  unsigned long DBFULL : 1;
  unsigned long ERSSPD : 1;
  unsigned long PRGSPD : 1;
  unsigned long  : 1;
  unsigned long FLWEERR : 1;
  unsigned long  : 6;
#endif
};

union un_flash_fstatr
{
  unsigned long LONG;
  struct st_flash_fstsatr_bit BIT;
};

struct st_flash_fentryr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short FENTRYC : 1;
  unsigned short  : 6;
  unsigned short FENTRYD : 1;
  unsigned short KEY : 8;
#else
  unsigned short KEY : 8;
  unsigned short FENTRYD : 1;
  unsigned short  : 6;
  unsigned short FENTRYC : 1;
#endif
};

union un_flash_fentryr
{
  unsigned short WORD;
  struct st_flash_fentryr_bit BIT;
};

struct st_flash_fsunitr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short SUINIT : 1;
  unsigned short  : 7;
  unsigned short KEY : 8;
#else
  unsigned short KEY : 8;
  unsigned short  : 7;
  unsigned short SUINIT : 1;
#endif
};

union un_flash_fsunitr
{
  unsigned short WORD;
  struct st_flash_fsunitr_bit BIT;
};

struct st_flash_fcmdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PCMDR : 8;
  unsigned short CMDR : 8;
#else
  unsigned short CMDR : 8;
  unsigned short PCMDR : 8;
#endif
};

union un_flash_fcmdr
{
  unsigned short WORD;
  struct st_flash_fcmdr_bit BIT;
};

struct st_flash_fbccnt_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCDIR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char BCDIR : 1;
#endif
};

union un_flash_fbccnt
{
  unsigned char BYTE;
  struct st_flash_fbccnt_bit BIT;
};

struct st_flash_fbcstat_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char BCST : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char BCST : 1;
#endif
};

union un_flash_fbcstat
{
  unsigned char BYTE;
  struct st_flash_fbcstat_bit BIT;
};

struct st_flash_fpsaddr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PSADR : 19;
  unsigned long  : 13;
#else
  unsigned long  : 13;
  unsigned long PSADR : 19;
#endif
};

union un_flash_fpsaddr
{
  unsigned long LONG;
  struct st_flash_fpsaddr_bit BIT;
};

struct st_flash_fawmon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FAWS : 12;
  unsigned long  : 3;
  unsigned long FSPR : 1;
  unsigned long FAWE : 12;
  unsigned long  : 3;
  unsigned long BTFLG : 1;
#else
  unsigned long BTFLG : 1;
  unsigned long  : 3;
  unsigned long FAWE : 12;
  unsigned long FSPR : 1;
  unsigned long  : 3;
  unsigned long FAWS : 12;
#endif
};

union un_flash_fawmon
{
  unsigned long LONG;
  struct st_flash_fawmon_bit BIT;
};

struct st_flash_fcpsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ESUSPMD : 1;
  unsigned short  : 15;
#else
  unsigned short  : 15;
  unsigned short ESUSPMD : 1;
#endif
};

union un_flash_fcpsr
{
  unsigned short WORD;
  struct st_flash_fcpsr_bit BIT;
};

struct st_flash_fpckar_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PCKA : 8;
  unsigned short KEY : 8;
#else
  unsigned short KEY : 8;
  unsigned short PCKA : 8;
#endif
};

union un_flash_fpckar
{
  unsigned short WORD;
  struct st_flash_fpckar_bit BIT;
};

struct st_flash_fsuacr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short SAS : 2;
  unsigned short  : 6;
  unsigned short KEY : 8;
#else
  unsigned short KEY : 8;
  unsigned short  : 6;
  unsigned short SAS : 2;
#endif
        };

union un_flash_fsuacr
{
  unsigned short WORD;
  struct st_flash_fsuacr_bit BIT;
};

struct st_pdc_pccr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PCKE : 1;
  unsigned long VPS : 1;
  unsigned long HPS : 1;
  unsigned long PRST : 1;
  unsigned long DFIE : 1;
  unsigned long FEIE : 1;
  unsigned long OVIE : 1;
  unsigned long UDRIE : 1;
  unsigned long VERIE : 1;
  unsigned long HERIE : 1;
  unsigned long PCKOE : 1;
  unsigned long PCKDIV : 3;
  unsigned long EDS : 1;
  unsigned long  : 17;
#else
  unsigned long  : 17;
  unsigned long EDS : 1;
  unsigned long PCKDIV : 3;
  unsigned long PCKOE : 1;
  unsigned long HERIE : 1;
  unsigned long VERIE : 1;
  unsigned long UDRIE : 1;
  unsigned long OVIE : 1;
  unsigned long FEIE : 1;
  unsigned long DFIE : 1;
  unsigned long PRST : 1;
  unsigned long HPS : 1;
  unsigned long VPS : 1;
  unsigned long PCKE : 1;
#endif
};

union un_pdc_pccr0
{
  unsigned long LONG;
  struct st_pdc_pccr0_bit BIT;
};

struct st_pdc_pccr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long PCE : 1;
  unsigned long  : 31;
#else
  unsigned long  : 31;
  unsigned long PCE : 1;
#endif
};

union un_pdc_pccr1
{
  unsigned long LONG;
  struct st_pdc_pccr1_bit BIT;
};

struct st_pdc_pcsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long FBSY : 1;
  unsigned long FEMPF : 1;
  unsigned long FEF : 1;
  unsigned long OVRF : 1;
  unsigned long UDRF : 1;
  unsigned long VERF : 1;
  unsigned long HERF : 1;
  unsigned long  : 25;
#else
  unsigned long  : 25;
  unsigned long HERF : 1;
  unsigned long VERF : 1;
  unsigned long UDRF : 1;
  unsigned long OVRF : 1;
  unsigned long FEF : 1;
  unsigned long FEMPF : 1;
  unsigned long FBSY : 1;
#endif
};

union un_pdc_pcsr
{
  unsigned long LONG;
  struct st_pdc_pcsr_bit BIT;
};

struct st_pdc_pcmonr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VSYNC : 1;
  unsigned long HSYNC : 1;
  unsigned long  : 30;
#else
  unsigned long  : 30;
  unsigned long HSYNC : 1;
  unsigned long VSYNC : 1;
#endif
};

union un_pdc_pcmonr
{
  unsigned long LONG;
  struct st_pdc_pcmonr_bit BIT;
};

union un_pdc_pcdr
{
  unsigned long LONG;
};

struct st_pdc_vcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long VST : 12;
  unsigned long  : 4;
  unsigned long VSZ : 12;
  unsigned long  : 4;
#else
  unsigned long  : 4;
  unsigned long VSZ : 12;
  unsigned long  : 4;
  unsigned long VST : 12;
#endif
};

union un_pdc_vcr
{
  unsigned long LONG;
  struct st_pdc_vcr_bit BIT;
};

struct st_pdc_hcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long HST : 12;
  unsigned long  : 4;
  unsigned long HSZ : 12;
  unsigned long  : 4;
#else
  unsigned long  : 4;
  unsigned long HSZ : 12;
  unsigned long  : 4;
  unsigned long HST : 12;
#endif
};

union un_pdc_hcr
{
  unsigned long LONG;
  struct st_pdc_hcr_bit BIT;
};

struct st_poe_icsr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short POE0M : 2;
  unsigned short  : 6;
  unsigned short PIE1 : 1;
  unsigned short  : 3;
  unsigned short POE0F : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short POE0F : 1;
  unsigned short  : 3;
  unsigned short PIE1 : 1;
  unsigned short  : 6;
  unsigned short POE0M : 2;
#endif
};

union un_poe_icsr1
{
  unsigned short WORD;
  struct st_poe_icsr1_bit BIT;
};

struct st_poe_ocsr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 8;
  unsigned short OIE1 : 1;
  unsigned short OCE1 : 1;
  unsigned short  : 5;
  unsigned short OSF1 : 1;
#else
  unsigned short OSF1 : 1;
  unsigned short  : 5;
  unsigned short OCE1 : 1;
  unsigned short OIE1 : 1;
  unsigned short  : 8;
#endif
};

union un_poe_ocsr1
{
  unsigned short WORD;
  struct st_poe_ocsr1_bit BIT;
};

struct st_poe_icsr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short POE4M : 2;
  unsigned short  : 6;
  unsigned short PIE2 : 1;
  unsigned short  : 3;
  unsigned short POE4F : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short POE4F : 1;
  unsigned short  : 3;
  unsigned short PIE2 : 1;
  unsigned short  : 6;
  unsigned short POE4M : 2;
#endif
};

union un_poe_icsr2
{
  unsigned short WORD;
  struct st_poe_icsr2_bit BIT;
};

struct st_poe_ocsr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 8;
  unsigned short OIE2 : 1;
  unsigned short OCE2 : 1;
  unsigned short  : 5;
  unsigned short OSF2 : 1;
#else
  unsigned short OSF2 : 1;
  unsigned short  : 5;
  unsigned short OCE2 : 1;
  unsigned short OIE2 : 1;
  unsigned short  : 8;
#endif
};

union un_poe_ocsr2
{
  unsigned short WORD;
  struct st_poe_ocsr2_bit BIT;
};

struct st_poe_icsr3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short POE8M : 2;
  unsigned short  : 6;
  unsigned short PIE3 : 1;
  unsigned short POE8E : 1;
  unsigned short  : 2;
  unsigned short POE8F : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short POE8F : 1;
  unsigned short  : 2;
  unsigned short POE8E : 1;
  unsigned short PIE3 : 1;
  unsigned short  : 6;
  unsigned short POE8M : 2;
#endif
};

union un_poe_icsr3
{
  unsigned short WORD;
  struct st_poe_icsr3_bit BIT;
};

struct st_poe_spoer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MTUCH34HIZ : 1;
  unsigned char MTUCH67HIZ : 1;
  unsigned char MTUCH0HIZ : 1;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char MTUCH0HIZ : 1;
  unsigned char MTUCH67HIZ : 1;
  unsigned char MTUCH34HIZ : 1;
#endif
};

union un_poe_spoer
{
  unsigned char BYTE;
  struct st_poe_spoer_bit BIT;
};

struct st_poe_poecr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MTU0AZE : 1;
  unsigned char MTU0BZE : 1;
  unsigned char MTU0CZE : 1;
  unsigned char MTU0DZE : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char MTU0DZE : 1;
  unsigned char MTU0CZE : 1;
  unsigned char MTU0BZE : 1;
  unsigned char MTU0AZE : 1;
#endif
};

union un_poe_poecr1
{
  unsigned char BYTE;
  struct st_poe_poecr1_bit BIT;
};

struct st_poe_poecr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short MTU7BDZE : 1;
  unsigned short MTU7ACZE : 1;
  unsigned short MTU6BDZE : 1;
  unsigned short  : 5;
  unsigned short MTU4BDZE : 1;
  unsigned short MTU4ACZE : 1;
  unsigned short MTU3BDZE : 1;
  unsigned short  : 5;
#else
  unsigned short  : 5;
  unsigned short MTU3BDZE : 1;
  unsigned short MTU4ACZE : 1;
  unsigned short MTU4BDZE : 1;
  unsigned short  : 5;
  unsigned short MTU6BDZE : 1;
  unsigned short MTU7ACZE : 1;
  unsigned short MTU7BDZE : 1;
#endif
};

union un_poe_poecr2
{
  unsigned short WORD;
  struct st_poe_poecr2_bit BIT;
};

struct st_poe_poecr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 2;
  unsigned short IC2ADDMT34ZE : 1;
  unsigned short IC3ADDMT34ZE : 1;
  unsigned short IC4ADDMT34ZE : 1;
  unsigned short IC5ADDMT34ZE : 1;
  unsigned short  : 3;
  unsigned short IC1ADDMT67ZE : 1;
  unsigned short  : 1;
  unsigned short IC3ADDMT67ZE : 1;
  unsigned short IC4ADDMT67ZE : 1;
  unsigned short IC5ADDMT67ZE : 1;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short IC5ADDMT67ZE : 1;
  unsigned short IC4ADDMT67ZE : 1;
  unsigned short IC3ADDMT67ZE : 1;
  unsigned short  : 1;
  unsigned short IC1ADDMT67ZE : 1;
  unsigned short  : 3;
  unsigned short IC5ADDMT34ZE : 1;
  unsigned short IC4ADDMT34ZE : 1;
  unsigned short IC3ADDMT34ZE : 1;
  unsigned short IC2ADDMT34ZE : 1;
  unsigned short  : 2;
#endif
};

union un_poe_poecr4
{
  unsigned short WORD;
  struct st_poe_poecr4_bit BIT;
};

struct st_poe_poecr5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 1;
  unsigned short IC1ADDMT0ZE : 1;
  unsigned short IC2ADDMT0ZE : 1;
  unsigned short  : 1;
  unsigned short IC4ADDMT0ZE : 1;
  unsigned short IC5ADDMT0ZE : 1;
  unsigned short  : 10;
#else
  unsigned short  : 10;
  unsigned short IC5ADDMT0ZE : 1;
  unsigned short IC4ADDMT0ZE : 1;
  unsigned short  : 1;
  unsigned short IC2ADDMT0ZE : 1;
  unsigned short IC1ADDMT0ZE : 1;
  unsigned short  : 1;
#endif
};

union un_poe_poecr5
{
  unsigned short WORD;
  struct st_poe_poecr5_bit BIT;
};

struct st_poe_icsr4_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short POE10M : 2;
  unsigned short  : 6;
  unsigned short PIE4 : 1;
  unsigned short POE10E : 1;
  unsigned short  : 2;
  unsigned short POE10F : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short POE10F : 1;
  unsigned short  : 2;
  unsigned short POE10E : 1;
  unsigned short PIE4 : 1;
  unsigned short  : 6;
  unsigned short POE10M : 2;
#endif
};

union un_poe_icsr4
{
  unsigned short WORD;
  struct st_poe_icsr4_bit BIT;
};

struct st_poe_icsr5_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short POE11M : 2;
  unsigned short  : 6;
  unsigned short PIE5 : 1;
  unsigned short POE11E : 1;
  unsigned short  : 2;
  unsigned short POE11F : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short POE11F : 1;
  unsigned short  : 2;
  unsigned short POE11E : 1;
  unsigned short PIE5 : 1;
  unsigned short  : 6;
  unsigned short POE11M : 2;
#endif
};

union un_poe_icsr5
{
  unsigned short WORD;
  struct st_poe_icsr5_bit BIT;
};

struct st_poe_alr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short OLSG0A : 1;
  unsigned short OLSG0B : 1;
  unsigned short OLSG1A : 1;
  unsigned short OLSG1B : 1;
  unsigned short OLSG2A : 1;
  unsigned short OLSG2B : 1;
  unsigned short  : 1;
  unsigned short OLSEN : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short OLSEN : 1;
  unsigned short  : 1;
  unsigned short OLSG2B : 1;
  unsigned short OLSG2A : 1;
  unsigned short OLSG1B : 1;
  unsigned short OLSG1A : 1;
  unsigned short OLSG0B : 1;
  unsigned short OLSG0A : 1;
#endif
};

union un_poe_alr1
{
  unsigned short WORD;
  struct st_poe_alr1_bit BIT;
};

struct st_poe_icsr6_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 9;
  unsigned short OSTSTE : 1;
  unsigned short  : 2;
  unsigned short OSTSTF : 1;
  unsigned short  : 3;
#else
  unsigned short  : 3;
  unsigned short OSTSTF : 1;
  unsigned short  : 2;
  unsigned short OSTSTE : 1;
  unsigned short  : 9;
#endif
};

union un_poe_icsr6
{
  unsigned short WORD;
  struct st_poe_icsr6_bit BIT;
};

struct st_poe_m0selr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M0ASEL : 4;
  unsigned char M0BSEL : 4;
#else
  unsigned char M0BSEL : 4;
  unsigned char M0ASEL : 4;
#endif
};

union un_poe_m0selr1
{
  unsigned char BYTE;
  struct st_poe_m0selr1_bit BIT;
};

struct st_poe_m0selr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M0CSEL : 4;
  unsigned char M0DSEL : 4;
#else
  unsigned char M0DSEL : 4;
  unsigned char M0CSEL : 4;
#endif
};

union un_poe_m0selr2
{
  unsigned char BYTE;
  struct st_poe_m0selr2_bit BIT;
};

struct st_poe_m3selr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M3BSEL : 4;
  unsigned char M3DSEL : 4;
#else
  unsigned char M3DSEL : 4;
  unsigned char M3BSEL : 4;
#endif
};

union un_poe_m3selr
{
  unsigned char BYTE;
  struct st_poe_m3selr_bit BIT;
};

struct st_poe_m4selr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M4ASEL : 4;
  unsigned char M4CSEL : 4;
#else
  unsigned char M4CSEL : 4;
  unsigned char M4ASEL : 4;
#endif
};

union un_poe_m4selr1
{
  unsigned char BYTE;
  struct st_poe_m4selr1_bit BIT;
};

struct un_poe_m4selr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M4BSEL : 4;
  unsigned char M4DSEL : 4;
#else
  unsigned char M4DSEL : 4;
  unsigned char M4BSEL : 4;
#endif
};

union un_poe_m4selr2
{
  unsigned char BYTE;
  struct un_poe_m4selr2_bit BIT;
};

struct st_poe_m6selr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char M6BSEL : 4;
  unsigned char M6DSEL : 4;
#else
  unsigned char M6DSEL : 4;
  unsigned char M6BSEL : 4;
#endif
};

union un_poe_m6selr
{
  unsigned char BYTE;
  struct st_poe_m6selr_bit BIT;
};

struct st_portd_pdr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_pdr
{
  unsigned char BYTE;
  struct st_portd_pdr_bit BIT;
};

struct st_portd_podr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_podr
{
  unsigned char BYTE;
  struct st_portd_podr_bit BIT;
};

struct st_portd_pidr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_pidr
{
  unsigned char BYTE;
  struct st_portd_pidr_bit BIT;
};

struct st_portd_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_pmr
{
  unsigned char BYTE;
  struct st_portd_pmr_bit BIT;
};

struct st_portd_ord0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_ord0
{
  unsigned char BYTE;
  struct st_portd_ord0_bit BIT;
};

struct st_portd_ord1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_ord1
{
  unsigned char BYTE;
  struct st_portd_ord1_bit BIT;
};

struct st_portd_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_pcr
{
  unsigned char BYTE;
  struct st_portd_pcr_bit BIT;
};

struct st_portd_dscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_dscr
{
  unsigned char BYTE;
  struct st_portd_dscr_bit BIT;
};

struct st_portd_dscr2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char B0 : 1;
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
  unsigned char B0 : 1;
#endif
};

union un_portd_dscr2
{
  unsigned char BYTE;
  struct st_portd_dscr2_bit BIT;
};

struct st_ppg0_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char G0CMS : 2;
  unsigned char G1CMS : 2;
  unsigned char G2CMS : 2;
  unsigned char G3CMS : 2;
#else
  unsigned char G3CMS : 2;
  unsigned char G2CMS : 2;
  unsigned char G1CMS : 2;
  unsigned char G0CMS : 2;
#endif
};

union un_ppg0_pcr
{
  unsigned char BYTE;
  struct st_ppg0_pcr_bit BIT;
};

struct st_ppg0_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char G0NOV : 1;
  unsigned char G1NOV : 1;
  unsigned char G2NOV : 1;
  unsigned char G3NOV : 1;
  unsigned char G0INV : 1;
  unsigned char G1INV : 1;
  unsigned char G2INV : 1;
  unsigned char G3INV : 1;
#else
  unsigned char G3INV : 1;
  unsigned char G2INV : 1;
  unsigned char G1INV : 1;
  unsigned char G0INV : 1;
  unsigned char G3NOV : 1;
  unsigned char G2NOV : 1;
  unsigned char G1NOV : 1;
  unsigned char G0NOV : 1;
#endif
};

union un_ppg0_pmr
{
  unsigned char BYTE;
  struct st_ppg0_pmr_bit BIT;
};

struct st_ppg0_nderh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDER8 : 1;
  unsigned char NDER9 : 1;
  unsigned char NDER10 : 1;
  unsigned char NDER11 : 1;
  unsigned char NDER12 : 1;
  unsigned char NDER13 : 1;
  unsigned char NDER14 : 1;
  unsigned char NDER15 : 1;
#else
  unsigned char NDER15 : 1;
  unsigned char NDER14 : 1;
  unsigned char NDER13 : 1;
  unsigned char NDER12 : 1;
  unsigned char NDER11 : 1;
  unsigned char NDER10 : 1;
  unsigned char NDER9 : 1;
  unsigned char NDER8 : 1;
#endif
};

union un_ppg0_nderh
{
  unsigned char BYTE;
  struct st_ppg0_nderh_bit BIT;
};

struct st_ppg0_nderl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDER0 : 1;
  unsigned char NDER1 : 1;
  unsigned char NDER2 : 1;
  unsigned char NDER3 : 1;
  unsigned char NDER4 : 1;
  unsigned char NDER5 : 1;
  unsigned char NDER6 : 1;
  unsigned char NDER7 : 1;
#else
  unsigned char NDER7 : 1;
  unsigned char NDER6 : 1;
  unsigned char NDER5 : 1;
  unsigned char NDER4 : 1;
  unsigned char NDER3 : 1;
  unsigned char NDER2 : 1;
  unsigned char NDER1 : 1;
  unsigned char NDER0 : 1;
#endif
};

union un_ppg0_nderl
{
  unsigned char BYTE;
  struct st_ppg0_nderl_bit BIT;
};

struct st_ppg0_podrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char POD8 : 1;
  unsigned char POD9 : 1;
  unsigned char POD10 : 1;
  unsigned char POD11 : 1;
  unsigned char POD12 : 1;
  unsigned char POD13 : 1;
  unsigned char POD14 : 1;
  unsigned char POD15 : 1;
#else
  unsigned char POD15 : 1;
  unsigned char POD14 : 1;
  unsigned char POD13 : 1;
  unsigned char POD12 : 1;
  unsigned char POD11 : 1;
  unsigned char POD10 : 1;
  unsigned char POD9 : 1;
  unsigned char POD8 : 1;
#endif
};

union un_ppg0_podrh
{
  unsigned char BYTE;
  struct st_ppg0_podrh_bit BIT;
};

struct st_ppg0_podrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char POD0 : 1;
  unsigned char POD1 : 1;
  unsigned char POD2 : 1;
  unsigned char POD3 : 1;
  unsigned char POD4 : 1;
  unsigned char POD5 : 1;
  unsigned char POD6 : 1;
  unsigned char POD7 : 1;
#else
  unsigned char POD7 : 1;
  unsigned char POD6 : 1;
  unsigned char POD5 : 1;
  unsigned char POD4 : 1;
  unsigned char POD3 : 1;
  unsigned char POD2 : 1;
  unsigned char POD1 : 1;
  unsigned char POD0 : 1;
#endif
};

union un_ppg0_podrl
{
  unsigned char BYTE;
  struct st_ppg0_podrl_bit BIT;
};

struct st_ppg0_ndrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR8 : 1;
  unsigned char NDR9 : 1;
  unsigned char NDR10 : 1;
  unsigned char NDR11 : 1;
  unsigned char NDR12 : 1;
  unsigned char NDR13 : 1;
  unsigned char NDR14 : 1;
  unsigned char NDR15 : 1;
#else
  unsigned char NDR15 : 1;
  unsigned char NDR14 : 1;
  unsigned char NDR13 : 1;
  unsigned char NDR12 : 1;
  unsigned char NDR11 : 1;
  unsigned char NDR10 : 1;
  unsigned char NDR9 : 1;
  unsigned char NDR8 : 1;
#endif
};

union un_ppg0_ndrh
{
  unsigned char BYTE;
  struct st_ppg0_ndrh_bit BIT;
};

struct st_ppg0_ndrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR0 : 1;
  unsigned char NDR1 : 1;
  unsigned char NDR2 : 1;
  unsigned char NDR3 : 1;
  unsigned char NDR4 : 1;
  unsigned char NDR5 : 1;
  unsigned char NDR6 : 1;
  unsigned char NDR7 : 1;
#else
  unsigned char NDR7 : 1;
  unsigned char NDR6 : 1;
  unsigned char NDR5 : 1;
  unsigned char NDR4 : 1;
  unsigned char NDR3 : 1;
  unsigned char NDR2 : 1;
  unsigned char NDR1 : 1;
  unsigned char NDR0 : 1;
#endif
};

union un_ppg0_ndrl
{
  unsigned char BYTE;
  struct st_ppg0_ndrl_bit BIT;
};

struct st_ppg0_ndrh2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR8 : 1;
  unsigned char NDR9 : 1;
  unsigned char NDR10 : 1;
  unsigned char NDR11 : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char NDR11 : 1;
  unsigned char NDR10 : 1;
  unsigned char NDR9 : 1;
  unsigned char NDR8 : 1;
#endif
};

union un_ppg0_ndrh2
{
  unsigned char BYTE;
  struct st_ppg0_ndrh2_bit BIT;
};

struct st_ppg0_ndrl2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR0 : 1;
  unsigned char NDR1 : 1;
  unsigned char NDR2 : 1;
  unsigned char NDR3 : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char NDR3 : 1;
  unsigned char NDR2 : 1;
  unsigned char NDR1 : 1;
  unsigned char NDR0 : 1;
#endif
};

union un_ppg0_ndrl2
{
  unsigned char BYTE;
  struct st_ppg0_ndrl2_bit BIT;
};

struct st_ppg1_ptrslr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PTRSL : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char PTRSL : 1;
#endif
};

union un_ppg1_ptrslr
{
  unsigned char BYTE;
  struct st_ppg1_ptrslr_bit BIT;
};

struct st_ppg1_pcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char G0CMS : 2;
  unsigned char G1CMS : 2;
  unsigned char G2CMS : 2;
  unsigned char G3CMS : 2;
#else
  unsigned char G3CMS : 2;
  unsigned char G2CMS : 2;
  unsigned char G1CMS : 2;
  unsigned char G0CMS : 2;
#endif
};

union un_ppg1_pcr
{
  unsigned char BYTE;
  struct st_ppg1_pcr_bit BIT;
};

struct st_ppg1_pmr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char G0NOV : 1;
  unsigned char G1NOV : 1;
  unsigned char G2NOV : 1;
  unsigned char G3NOV : 1;
  unsigned char G0INV : 1;
  unsigned char G1INV : 1;
  unsigned char G2INV : 1;
  unsigned char G3INV : 1;
#else
  unsigned char G3INV : 1;
  unsigned char G2INV : 1;
  unsigned char G1INV : 1;
  unsigned char G0INV : 1;
  unsigned char G3NOV : 1;
  unsigned char G2NOV : 1;
  unsigned char G1NOV : 1;
  unsigned char G0NOV : 1;
#endif
};

union un_ppg1_pmr
{
  unsigned char BYTE;
  struct st_ppg1_pmr_bit BIT;
};

struct st_ppg1_nderh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDER24 : 1;
  unsigned char NDER25 : 1;
  unsigned char NDER26 : 1;
  unsigned char NDER27 : 1;
  unsigned char NDER28 : 1;
  unsigned char NDER29 : 1;
  unsigned char NDER30 : 1;
  unsigned char NDER31 : 1;
#else
  unsigned char NDER31 : 1;
  unsigned char NDER30 : 1;
  unsigned char NDER29 : 1;
  unsigned char NDER28 : 1;
  unsigned char NDER27 : 1;
  unsigned char NDER26 : 1;
  unsigned char NDER25 : 1;
  unsigned char NDER24 : 1;
#endif
};

union un_ppg1_nderh
{
  unsigned char BYTE;
  struct st_ppg1_nderh_bit BIT;
};

struct st_ppg1_nerl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDER16 : 1;
  unsigned char NDER17 : 1;
  unsigned char NDER18 : 1;
  unsigned char NDER19 : 1;
  unsigned char NDER20 : 1;
  unsigned char NDER21 : 1;
  unsigned char NDER22 : 1;
  unsigned char NDER23 : 1;
#else
  unsigned char NDER23 : 1;
  unsigned char NDER22 : 1;
  unsigned char NDER21 : 1;
  unsigned char NDER20 : 1;
  unsigned char NDER19 : 1;
  unsigned char NDER18 : 1;
  unsigned char NDER17 : 1;
  unsigned char NDER16 : 1;
#endif
};

union un_ppg1_nderl
{
  unsigned char BYTE;
  struct st_ppg1_nerl_bit BIT;
};

struct st_ppg1_podrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char POD24 : 1;
  unsigned char POD25 : 1;
  unsigned char POD26 : 1;
  unsigned char POD27 : 1;
  unsigned char POD28 : 1;
  unsigned char POD29 : 1;
  unsigned char POD30 : 1;
  unsigned char POD31 : 1;
#else
  unsigned char POD31 : 1;
  unsigned char POD30 : 1;
  unsigned char POD29 : 1;
  unsigned char POD28 : 1;
  unsigned char POD27 : 1;
  unsigned char POD26 : 1;
  unsigned char POD25 : 1;
  unsigned char POD24 : 1;
#endif
};

union un_ppg1_podrh
{
  unsigned char BYTE;
  struct st_ppg1_podrh_bit BIT;
};

struct st_ppg1_podrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char POD16 : 1;
  unsigned char POD17 : 1;
  unsigned char POD18 : 1;
  unsigned char POD19 : 1;
  unsigned char POD20 : 1;
  unsigned char POD21 : 1;
  unsigned char POD22 : 1;
  unsigned char POD23 : 1;
#else
  unsigned char POD23 : 1;
  unsigned char POD22 : 1;
  unsigned char POD21 : 1;
  unsigned char POD20 : 1;
  unsigned char POD19 : 1;
  unsigned char POD18 : 1;
  unsigned char POD17 : 1;
  unsigned char POD16 : 1;
#endif
};

union un_ppg1_podrl
{
  unsigned char BYTE;
  struct st_ppg1_podrl_bit BIT;
};

struct st_ppg1_ndrh_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR24 : 1;
  unsigned char NDR25 : 1;
  unsigned char NDR26 : 1;
  unsigned char NDR27 : 1;
  unsigned char NDR28 : 1;
  unsigned char NDR29 : 1;
  unsigned char NDR30 : 1;
  unsigned char NDR31 : 1;
#else
  unsigned char NDR31 : 1;
  unsigned char NDR30 : 1;
  unsigned char NDR29 : 1;
  unsigned char NDR28 : 1;
  unsigned char NDR27 : 1;
  unsigned char NDR26 : 1;
  unsigned char NDR25 : 1;
  unsigned char NDR24 : 1;
#endif
};

union un_ppg1_ndrh
{
  unsigned char BYTE;
  struct st_ppg1_ndrh_bit BIT;
};

struct st_ppg1_ndrl_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR16 : 1;
  unsigned char NDR17 : 1;
  unsigned char NDR18 : 1;
  unsigned char NDR19 : 1;
  unsigned char NDR20 : 1;
  unsigned char NDR21 : 1;
  unsigned char NDR22 : 1;
  unsigned char NDR23 : 1;
#else
  unsigned char NDR23 : 1;
  unsigned char NDR22 : 1;
  unsigned char NDR21 : 1;
  unsigned char NDR20 : 1;
  unsigned char NDR19 : 1;
  unsigned char NDR18 : 1;
  unsigned char NDR17 : 1;
  unsigned char NDR16 : 1;
#endif
};

union un_ppg1_ndrl
{
  unsigned char BYTE;
  struct st_ppg1_ndrl_bit BIT;
};

struct st_ppg1_ndrh2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR24 : 1;
  unsigned char NDR25 : 1;
  unsigned char NDR26 : 1;
  unsigned char NDR27 : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char NDR27 : 1;
  unsigned char NDR26 : 1;
  unsigned char NDR25 : 1;
  unsigned char NDR24 : 1;
#endif
};

union un_ppg1_ndrh2
{
  unsigned char BYTE;
  struct st_ppg1_ndrh2_bit BIT;
};

struct st_ppg1_ndrl2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char NDR16 : 1;
  unsigned char NDR17 : 1;
  unsigned char NDR18 : 1;
  unsigned char NDR19 : 1;
  unsigned char  : 4;
#else
  unsigned char  : 4;
  unsigned char NDR19 : 1;
  unsigned char NDR18 : 1;
  unsigned char NDR17 : 1;
  unsigned char NDR16 : 1;
#endif
};

union un_ppg1_ndrl2
{
  unsigned char BYTE;
  struct st_ppg1_ndrl2_bit BIT;
};

struct st_qspi_spcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 1;
  unsigned char SPSSLIE : 1;
  unsigned char  : 1;
  unsigned char MSTR : 1;
  unsigned char  : 1;
  unsigned char SPTIE : 1;
  unsigned char SPE : 1;
  unsigned char SPRIE : 1;
#else
  unsigned char SPRIE : 1;
  unsigned char SPE : 1;
  unsigned char SPTIE : 1;
  unsigned char  : 1;
  unsigned char MSTR : 1;
  unsigned char  : 1;
  unsigned char SPSSLIE : 1;
  unsigned char  : 1;
#endif
};

union un_qspi_spcr
{
  unsigned char BYTE;
  struct st_qspi_spcr_bit BIT;
};

struct st_qspi_sslp_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SSLP : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SSLP : 1;
#endif
};

union un_qspi_sslp
{
  unsigned char BYTE;
  struct st_qspi_sslp_bit BIT;
};

struct st_qspi_sppcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPLP : 1;
  unsigned char IO2FV : 1;
  unsigned char IO3FV : 1;
  unsigned char  : 1;
  unsigned char MOIFV : 1;
  unsigned char MOIFE : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char MOIFE : 1;
  unsigned char MOIFV : 1;
  unsigned char  : 1;
  unsigned char IO3FV : 1;
  unsigned char IO2FV : 1;
  unsigned char SPLP : 1;
#endif
};

union un_qspi_sppcr
{
  unsigned char BYTE;
  struct st_qspi_sppcr_bit BIT;
};

struct st_qspi_spsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 4;
  unsigned char SPSSLF : 1;
  unsigned char SPTEF : 1;
  unsigned char TREND : 1;
  unsigned char SPRFF : 1;
#else
  unsigned char SPRFF : 1;
  unsigned char TREND : 1;
  unsigned char SPTEF : 1;
  unsigned char SPSSLF : 1;
  unsigned char  : 4;
#endif
};

union un_qspi_spsr
{
  unsigned char BYTE;
  struct st_qspi_spsr_bit BIT;
};

struct st_qspi_spdr_word
{
  unsigned short H;
};

struct st_qspi_spsr_byte
{
  unsigned char HH;
};

union un_qspi_spdr
{
  unsigned long LONG;
  struct st_qspi_spdr_word WOED;
  struct st_qspi_spsr_byte BYTE;
};

struct st_qspi_spscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPSC : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char SPSC : 2;
#endif
};

union un_qspi_spscr
{
  unsigned char BYTE;
  struct st_qspi_spscr_bit BIT;
};

struct st_qspi_spssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPSS : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char SPSS : 2;
#endif
};

union un_qspi_spssr
{
  unsigned char BYTE;
  struct st_qspi_spssr_bit BIT;
};

struct st_qspi_spbr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPBR0 : 1;
  unsigned char SPBR1 : 1;
  unsigned char SPBR2 : 1;
  unsigned char SPBR3 : 1;
  unsigned char SPBR4 : 1;
  unsigned char SPBR5 : 1;
  unsigned char SPBR6 : 1;
  unsigned char SPBR7 : 1;
#else
  unsigned char SPBR7 : 1;
  unsigned char SPBR6 : 1;
  unsigned char SPBR5 : 1;
  unsigned char SPBR4 : 1;
  unsigned char SPBR3 : 1;
  unsigned char SPBR2 : 1;
  unsigned char SPBR1 : 1;
  unsigned char SPBR0 : 1;
#endif
};

union un_qspi_spbr
{
  unsigned char BYTE;
  struct st_qspi_spbr_bit BIT;
};

struct st_qspi_spdcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char  : 7;
  unsigned char TXDMY : 1;
#else
  unsigned char TXDMY : 1;
  unsigned char  : 7;
#endif
};

union un_qspi_spdcr
{
  unsigned char BYTE;
  struct st_qspi_spdcr_bit BIT;
};

struct st_qspi_spckd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SCKDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SCKDL : 3;
#endif
};

union un_qspi_spckd
{
  unsigned char BYTE;
  struct st_qspi_spckd_bit BIT;
};

struct st_qspi_sslnd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SLNDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SLNDL : 3;
#endif
};

union un_qspi_sslnd
{
  unsigned char BYTE;
  struct st_qspi_sslnd_bit BIT;
};

struct st_qspi_spnd_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SPNDL : 3;
  unsigned char  : 5;
#else
  unsigned char  : 5;
  unsigned char SPNDL : 3;
#endif
};

union un_qspi_spnd
{
  unsigned char BYTE;
  struct st_qspi_spnd_bit BIT;
};

struct st_qspi_spcmd0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SPRW : 1;
  unsigned short SPIMOD : 2;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SPIMOD : 2;
  unsigned short SPRW : 1;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_qspi_spcmd0
{
  unsigned short WORD;
  struct st_qspi_spcmd0_bit BIT;
};

struct st_qspi_spcmd1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SPRW : 1;
  unsigned short SPIMOD : 2;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SPIMOD : 2;
  unsigned short SPRW : 1;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_qspi_spcmd1
{
  unsigned short WORD;
  struct st_qspi_spcmd1_bit BIT;
};

struct st_qspi_spcmd2_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SPRW : 1;
  unsigned short SPIMOD : 2;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SPIMOD : 2;
  unsigned short SPRW : 1;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_qspi_spcmd2
{
  unsigned short WORD;
  struct st_qspi_spcmd2_bit BIT;
};

struct st_qspi_spcmd3_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CPHA : 1;
  unsigned short CPOL : 1;
  unsigned short BRDV : 2;
  unsigned short SPRW : 1;
  unsigned short SPIMOD : 2;
  unsigned short SSLKP : 1;
  unsigned short SPB : 4;
  unsigned short LSBF : 1;
  unsigned short SPNDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SCKDEN : 1;
#else
  unsigned short SCKDEN : 1;
  unsigned short SLNDEN : 1;
  unsigned short SPNDEN : 1;
  unsigned short LSBF : 1;
  unsigned short SPB : 4;
  unsigned short SSLKP : 1;
  unsigned short SPIMOD : 2;
  unsigned short SPRW : 1;
  unsigned short BRDV : 2;
  unsigned short CPOL : 1;
  unsigned short CPHA : 1;
#endif
};

union un_qspi_spcmd3
{
  unsigned short WORD;
  struct st_qspi_spcmd3_bit BIT;
};

struct st_qspi_spbfcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RXTRG : 3;
  unsigned char TXTRGEX : 1;
  unsigned char TXTRG : 2;
  unsigned char RXRST : 1;
  unsigned char TXRST : 1;
#else
  unsigned char TXRST : 1;
  unsigned char RXRST : 1;
  unsigned char TXTRG : 2;
  unsigned char TXTRGEX : 1;
  unsigned char RXTRG : 3;
#endif
};

union un_qspi_spbfcr
{
  unsigned char BYTE;
  struct st_qspi_spbfcr_bit BIT;
};

struct st_qspi_spbdcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short RXBC : 6;
  unsigned short  : 2;
  unsigned short TXBC : 6;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short TXBC : 6;
  unsigned short  : 2;
  unsigned short RXBC : 6;
#endif
};

union un_qspi_spbdcr
{
  unsigned short WORD;
  struct st_qspi_spbdcr_bit BIT;
};

struct st_ram_rammode_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RAMMODE : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char RAMMODE : 2;
#endif
};

union un_ram_rammode
{
  unsigned char BYTE;
  struct st_ram_rammode_bit BIT;
};

struct st_ram_ramsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RAMERR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char RAMERR : 1;
#endif
};

union un_ram_ramsts
{
  unsigned char BYTE;
  struct st_ram_ramsts_bit BIT;
};

struct st_ram_ramprcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char RAMPRCR : 1;
  unsigned char KW : 7;
#else
  unsigned char KW : 7;
  unsigned char RAMPRCR : 1;
#endif
};

union un_ram_ramprcr
{
  unsigned char BYTE;
  struct st_ram_ramprcr_bit BIT;
};

struct st_ram_ramecad_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 3;
  unsigned long READ : 16;
  unsigned long  : 13;
#else
  unsigned long  : 13;
  unsigned long READ : 16;
  unsigned long  : 3;
#endif
};

union un_ram_ramecad
{
  unsigned long LONG;
  struct st_ram_ramecad_bit BIT;
};

struct st_ram_exrammode_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EXRAMMODE : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char EXRAMMODE : 2;
#endif
};

union un_ram_exrammode
{
  unsigned char BYTE;
  struct st_ram_exrammode_bit BIT;
};

struct srt_ram_exramsts_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EXRAMERR : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char EXRAMERR : 1;
#endif
};

union un_ram_exramsts
{
  unsigned char BYTE;
  struct srt_ram_exramsts_bit BIT;
};

struct st_ram_exramprcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char EXRAMPRCR1 : 1;
  unsigned char KW : 7;
#else
  unsigned char KW : 7;
  unsigned char EXRAMPRCR2 : 1;
#endif
};

union un_ram_exramprcr
{
  unsigned char BYTE;
  struct st_ram_exramprcr_bit BIT;
};

struct st_ram_exramecad_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned long  : 3;
  unsigned long READ : 16;
  unsigned long  : 13;
#else
  unsigned long  : 13;
  unsigned long READ : 16;
  unsigned long  : 3;
#endif
};

union un_ram_exramecad
{
  unsigned long LONG;
  struct st_ram_exramecad_bit BIT;
};

struct st_s12ad_adcsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DBLANS : 5;
  unsigned short  : 1;
  unsigned short GBADIE : 1;
  unsigned short DBLE : 1;
  unsigned short EXTRG : 1;
  unsigned short TRGE : 1;
  unsigned short  : 2;
  unsigned short ADIE : 1;
  unsigned short ADCS : 2;
  unsigned short ADST : 1;
#else
  unsigned short ADST : 1;
  unsigned short ADCS : 2;
  unsigned short ADIE : 1;
  unsigned short  : 2;
  unsigned short TRGE : 1;
  unsigned short EXTRG : 1;
  unsigned short DBLE : 1;
  unsigned short GBADIE : 1;
  unsigned short  : 1;
  unsigned short DBLANS : 5;
#endif
};

union un_s12ad_adcsr
{
  unsigned short WORD;
  struct st_s12ad_adcsr_bit BIT;
};

struct st_s12ad_adansa0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSA000 : 1;
  unsigned short ANSA001 : 1;
  unsigned short ANSA002 : 1;
  unsigned short ANSA003 : 1;
  unsigned short ANSA004 : 1;
  unsigned short ANSA005 : 1;
  unsigned short ANSA006 : 1;
  unsigned short ANSA007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short ANSA007 : 1;
  unsigned short ANSA006 : 1;
  unsigned short ANSA005 : 1;
  unsigned short ANSA004 : 1;
  unsigned short ANSA003 : 1;
  unsigned short ANSA002 : 1;
  unsigned short ANSA001 : 1;
  unsigned short ANSA000 : 1;
#endif
};

union un_s12ad_adansa0
{
  unsigned short WORD;
  struct st_s12ad_adansa0_bit BIT;
};

struct st_s12ad_adads0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ADS000 : 1;
  unsigned short ADS001 : 1;
  unsigned short ADS002 : 1;
  unsigned short ADS003 : 1;
  unsigned short ADS004 : 1;
  unsigned short ADS005 : 1;
  unsigned short ADS006 : 1;
  unsigned short ADS007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short ADS007 : 1;
  unsigned short ADS006 : 1;
  unsigned short ADS005 : 1;
  unsigned short ADS004 : 1;
  unsigned short ADS003 : 1;
  unsigned short ADS002 : 1;
  unsigned short ADS001 : 1;
  unsigned short ADS000 : 1;
#endif
};

union un_s12ad_adads0
{
  unsigned short WORD;
  struct st_s12ad_adads0_bit BIT;
};

struct st_s12ad_adadc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADC : 3;
  unsigned char  : 4;
  unsigned char AVEE : 1;
#else
  unsigned char AVEE : 1;
  unsigned char  : 4;
  unsigned char ADC : 3;
#endif
};

union un_s12ad_adadc
{
  unsigned char BYTE;
  struct st_s12ad_adadc_bit BIT;
};

struct st_s12ad_adcer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 1;
  unsigned short ADPRC : 2;
  unsigned short  : 2;
  unsigned short ACE : 1;
  unsigned short  : 2;
  unsigned short DIAGVAL : 2;
  unsigned short DIAGLD : 1;
  unsigned short DIAGM : 1;
  unsigned short  : 3;
  unsigned short ADRFMT : 1;
#else
  unsigned short ADRFMT : 1;
  unsigned short  : 3;
  unsigned short DIAGM : 1;
  unsigned short DIAGLD : 1;
  unsigned short DIAGVAL : 2;
  unsigned short  : 2;
  unsigned short ACE : 1;
  unsigned short  : 2;
  unsigned short ADPRC : 2;
  unsigned short  : 1;
#endif
};

union un_s12ad_adcer
{
  unsigned short WORD;
  struct st_s12ad_adcer_bit BIT;
};

struct st_s12ad_adstrgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TRSB : 6;
  unsigned short  : 2;
  unsigned short TRSA : 6;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short TRSA : 6;
  unsigned short  : 2;
  unsigned short TRSB : 6;
#endif
};

union un_s12ad_adstrgr
{
  unsigned short WORD;
  struct st_s12ad_adstrgr_bit BIT;
};

struct st_s12ad_adansb0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSB000 : 1;
  unsigned short ANSB001 : 1;
  unsigned short ANSB002 : 1;
  unsigned short ANSB003 : 1;
  unsigned short ANSB004 : 1;
  unsigned short ANSB005 : 1;
  unsigned short ANSB006 : 1;
  unsigned short ANSB007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short ANSB007 : 1;
  unsigned short ANSB006 : 1;
  unsigned short ANSB005 : 1;
  unsigned short ANSB004 : 1;
  unsigned short ANSB003 : 1;
  unsigned short ANSB002 : 1;
  unsigned short ANSB001 : 1;
  unsigned short ANSB000 : 1;
#endif
};

union un_s12ad_adansb0
{
  unsigned short WORD;
  struct st_s12ad_adansb0_bit BIT;
};

union un_s12ad_addbldr
{
  unsigned short WORD;
};

struct st_s12ad_adrd_bit_right
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short AD : 12;
  unsigned short  : 2;
  unsigned short DIAGST : 2;
#else
  unsigned short DIAGST : 2;
  unsigned short  : 2;
  unsigned short AD : 12;
#endif
};

struct st_s12ad_adrd_bit_left
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DIAGST : 2;
  unsigned short  : 2;
  unsigned short AD : 12;
#else
  unsigned short AD : 12;
  unsigned short  : 2;
  unsigned short DIAGST : 2;
#endif
};

union un_s12ad_adrd_bit
{
  struct st_s12ad_adrd_bit_right RIGHT;
  struct st_s12ad_adrd_bit_left LEFT;
};

union un_s12ad_adrd
{
  unsigned short WORD;
};

struct st_s12ad_adsampr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PRO : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char PRO : 2;
#endif
};

union un_s12ad_adsampr
{
  unsigned char BYTE;
  struct st_s12ad_adsampr_bit BIT;
};

struct st_s12ad_adshcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short SSTSH : 8;
  unsigned short SHANS : 3;
  unsigned short  : 5;
#else
  unsigned short  : 5;
  unsigned short SHANS : 3;
  unsigned short SSTSH : 8;
#endif
};

union un_s12ad_adshcr
{
  unsigned short WORD;
  struct st_s12ad_adshcr_bit BIT;
};

struct st_s12ad_adsam_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 5;
  unsigned short SAM : 1;
  unsigned short  : 10;
#else
  unsigned short  : 10;
  unsigned short SAM : 1;
  unsigned short  : 5;
#endif
};

union un_s12ad_adsam
{
  unsigned short WORD;
  struct st_s12ad_adsam_bit BIT;
};

struct st_s12ad_addiscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADNDIS : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char ADNDIS : 5;
#endif
};

union un_s12ad_addiscr
{
  unsigned char BYTE;
  struct st_s12ad_addiscr_bit BIT;
};

struct st_s12ad_adshmsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char SHMD : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char SHMD : 1;
#endif
};

union un_s12ad_adshmsr
{
  unsigned char BYTE;
  struct st_s12ad_adshmsr_bit BIT;
};

struct st_s12ad_adgspcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PGS : 1;
  unsigned short GBRSCN : 1;
  unsigned short  : 12;
  unsigned short LGRRS : 1;
  unsigned short GBRP : 1;
#else
  unsigned short GBRP : 1;
  unsigned short LGRRS : 1;
  unsigned short  : 12;
  unsigned short GBRSCN : 1;
  unsigned short PGS : 1;
#endif
};

union un_s12ad_adgspcr
{
  unsigned short WORD;
  struct st_s12ad_adgspcr_bit BIT;
};

struct st_s12ad_adwinmon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MONCOMB : 1;
  unsigned char  : 3;
  unsigned char MONCMPA : 1;
  unsigned char MONCMPB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char MONCMPB : 1;
  unsigned char MONCMPA : 1;
  unsigned char  : 3;
  unsigned char MONCOMB : 1;
#endif
};

union un_s12ad_adwinmon
{
  unsigned char BYTE;
  struct st_s12ad_adwinmon_bit BIT;
};

struct st_s12ad_adcmpcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPAB : 2;
  unsigned short  : 7;
  unsigned short CMPBE : 1;
  unsigned short  : 1;
  unsigned short CMPAE : 1;
  unsigned short  : 1;
  unsigned short CMPBIE : 1;
  unsigned short WCMPE : 1;
  unsigned short CMPAIE : 1;
#else
  unsigned short CMPAIE : 1;
  unsigned short WCMPE : 1;
  unsigned short CMPBIE : 1;
  unsigned short  : 1;
  unsigned short CMPAE : 1;
  unsigned short  : 1;
  unsigned short CMPBE : 1;
  unsigned short  : 7;
  unsigned short CMPAB : 2;
#endif
};

union un_s12ad_adcmpcr
{
  unsigned short WORD;
  struct st_s12ad_adcmpcr_bit BIT;
};

struct st_s12ad_adcmpansr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPCHA000 : 1;
  unsigned short CMPCHA001 : 1;
  unsigned short CMPCHA002 : 1;
  unsigned short CMPCHA003 : 1;
  unsigned short CMPCHA004 : 1;
  unsigned short CMPCHA005 : 1;
  unsigned short CMPCHA006 : 1;
  unsigned short CMPCHA007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short CMPCHA007 : 1;
  unsigned short CMPCHA006 : 1;
  unsigned short CMPCHA005 : 1;
  unsigned short CMPCHA004 : 1;
  unsigned short CMPCHA003 : 1;
  unsigned short CMPCHA002 : 1;
  unsigned short CMPCHA001 : 1;
  unsigned short CMPCHA000 : 1;
#endif
};

union un_s12ad_adcmpansr0
{
  unsigned short WORD;
  struct st_s12ad_adcmpansr0_bit BIT;
};

struct st_s12ad_adcmplr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPLCHA000 : 1;
  unsigned short CMPLCHA001 : 1;
  unsigned short CMPLCHA002 : 1;
  unsigned short CMPLCHA003 : 1;
  unsigned short CMPLCHA004 : 1;
  unsigned short CMPLCHA005 : 1;
  unsigned short CMPLCHA006 : 1;
  unsigned short CMPLCHA007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short CMPLCHA007 : 1;
  unsigned short CMPLCHA006 : 1;
  unsigned short CMPLCHA005 : 1;
  unsigned short CMPLCHA004 : 1;
  unsigned short CMPLCHA003 : 1;
  unsigned short CMPLCHA002 : 1;
  unsigned short CMPLCHA001 : 1;
  unsigned short CMPLCHA000 : 1;
#endif
};

union un_s12ad_adcmplr0
{
  unsigned short WORD;
  struct st_s12ad_adcmplr0_bit BIT;
};

struct st_s12ad_adcmpsr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPSTCHA000 : 1;
  unsigned short CMPSTCHA001 : 1;
  unsigned short CMPSTCHA002 : 1;
  unsigned short CMPSTCHA003 : 1;
  unsigned short CMPSTCHA004 : 1;
  unsigned short CMPSTCHA005 : 1;
  unsigned short CMPSTCHA006 : 1;
  unsigned short CMPSTCHA007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short CMPSTCHA007 : 1;
  unsigned short CMPSTCHA006 : 1;
  unsigned short CMPSTCHA005 : 1;
  unsigned short CMPSTCHA004 : 1;
  unsigned short CMPSTCHA003 : 1;
  unsigned short CMPSTCHA002 : 1;
  unsigned short CMPSTCHA001 : 1;
  unsigned short CMPSTCHA000 : 1;
#endif
};

union un_s12ad_adcmpsr0
{
  unsigned short WORD;
  struct st_s12ad_adcmpsr0_bit BIT;
};

struct st_s12ad_adcmpbnsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPCHB : 6;
  unsigned char  : 1;
  unsigned char CMPLB : 1;
#else
  unsigned char CMPLB : 1;
  unsigned char  : 1;
  unsigned char CMPCHB : 6;
#endif
};

union un_s12ad_adcmpbnsr
{
  unsigned char BYTE;
  struct st_s12ad_adcmpbnsr_bit BIT;
};

struct st_s12ad_adcmpbsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPSTB : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char CMPSTB : 1;
#endif
};

union un_s12ad_adcmpbsr
{
  unsigned char BYTE;
  struct st_s12ad_adcmpbsr_bit BIT;
};

struct st_s12ad_adansc0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSC000 : 1;
  unsigned short ANSC001 : 1;
  unsigned short ANSC002 : 1;
  unsigned short ANSC003 : 1;
  unsigned short ANSC004 : 1;
  unsigned short ANSC005 : 1;
  unsigned short ANSC006 : 1;
  unsigned short ANSC007 : 1;
  unsigned short  : 8;
#else
  unsigned short  : 8;
  unsigned short ANSC007 : 1;
  unsigned short ANSC006 : 1;
  unsigned short ANSC005 : 1;
  unsigned short ANSC004 : 1;
  unsigned short ANSC003 : 1;
  unsigned short ANSC002 : 1;
  unsigned short ANSC001 : 1;
  unsigned short ANSC000 : 1;
#endif
};

union un_s12ad_adansc0
{
  unsigned short WORD;
  struct st_s12ad_adansc0_bit BIT;
};

struct st_s12ad_adgctrgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRSC : 6;
  unsigned char GCADIE : 1;
  unsigned char GRCE : 1;
#else
  unsigned char GRCE : 1;
  unsigned char GCADIE : 1;
  unsigned char TRSC : 6;
#endif
};

union un_s12ad_adgctrgr
{
  unsigned char BYTE;
  struct st_s12ad_adgctrgr_bit BIT;
};

struct st_s12ad1_adcsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DBLANS : 5;
  unsigned short  : 1;
  unsigned short GBADIE : 1;
  unsigned short DBLE : 1;
  unsigned short EXTRG : 1;
  unsigned short TRGE : 1;
  unsigned short  : 2;
  unsigned short ADIE : 1;
  unsigned short ADCS : 2;
  unsigned short ADST : 1;
#else
  unsigned short ADST : 1;
  unsigned short ADCS : 2;
  unsigned short ADIE : 1;
  unsigned short  : 2;
  unsigned short TRGE : 1;
  unsigned short EXTRG : 1;
  unsigned short DBLE : 1;
  unsigned short GBADIE : 1;
  unsigned short  : 1;
  unsigned short DBLANS : 5;
#endif
};

union un_s12ad1_adcsr
{
  unsigned short WORD;
  struct st_s12ad1_adcsr_bit BIT;
};

struct st_s12ad1_adansa0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSA000 : 1;
  unsigned short ANSA001 : 1;
  unsigned short ANSA002 : 1;
  unsigned short ANSA003 : 1;
  unsigned short ANSA004 : 1;
  unsigned short ANSA005 : 1;
  unsigned short ANSA006 : 1;
  unsigned short ANSA007 : 1;
  unsigned short ANSA008 : 1;
  unsigned short ANSA009 : 1;
  unsigned short ANSA010 : 1;
  unsigned short ANSA011 : 1;
  unsigned short ANSA012 : 1;
  unsigned short ANSA013 : 1;
  unsigned short ANSA014 : 1;
  unsigned short ANSA015 : 1;
#else
  unsigned short ANSA015 : 1;
  unsigned short ANSA014 : 1;
  unsigned short ANSA013 : 1;
  unsigned short ANSA012 : 1;
  unsigned short ANSA011 : 1;
  unsigned short ANSA010 : 1;
  unsigned short ANSA009 : 1;
  unsigned short ANSA008 : 1;
  unsigned short ANSA007 : 1;
  unsigned short ANSA006 : 1;
  unsigned short ANSA005 : 1;
  unsigned short ANSA004 : 1;
  unsigned short ANSA003 : 1;
  unsigned short ANSA002 : 1;
  unsigned short ANSA001 : 1;
  unsigned short ANSA000 : 1;
#endif
};

union un_s12ad1_adansa0
{
  unsigned short WORD;
  struct st_s12ad1_adansa0_bit BIT;
};

struct st_s12ad1_adansa1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSA100 : 1;
  unsigned short ANSA101 : 1;
  unsigned short ANSA102 : 1;
  unsigned short ANSA103 : 1;
  unsigned short ANSA104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short ANSA104 : 1;
  unsigned short ANSA103 : 1;
  unsigned short ANSA102 : 1;
  unsigned short ANSA101 : 1;
  unsigned short ANSA100 : 1;
#endif
};

union un_s12ad1_adansa1
{
  unsigned short WORD;
  struct st_s12ad1_adansa1_bit BIT;
};

struct st_s12ad1_adads0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ADS000 : 1;
  unsigned short ADS001 : 1;
  unsigned short ADS002 : 1;
  unsigned short ADS003 : 1;
  unsigned short ADS004 : 1;
  unsigned short ADS005 : 1;
  unsigned short ADS006 : 1;
  unsigned short ADS007 : 1;
  unsigned short ADS008 : 1;
  unsigned short ADS009 : 1;
  unsigned short ADS010 : 1;
  unsigned short ADS011 : 1;
  unsigned short ADS012 : 1;
  unsigned short ADS013 : 1;
  unsigned short ADS014 : 1;
  unsigned short ADS015 : 1;
#else
  unsigned short ADS015 : 1;
  unsigned short ADS014 : 1;
  unsigned short ADS013 : 1;
  unsigned short ADS012 : 1;
  unsigned short ADS011 : 1;
  unsigned short ADS010 : 1;
  unsigned short ADS009 : 1;
  unsigned short ADS008 : 1;
  unsigned short ADS007 : 1;
  unsigned short ADS006 : 1;
  unsigned short ADS005 : 1;
  unsigned short ADS004 : 1;
  unsigned short ADS003 : 1;
  unsigned short ADS002 : 1;
  unsigned short ADS001 : 1;
  unsigned short ADS000 : 1;
#endif
};

union un_s12ad1_adads0
{
  unsigned short WORD;
  struct st_s12ad1_adads0_bit BIT;
};

struct st_s12ad1_adads1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ADS100 : 1;
  unsigned short ADS101 : 1;
  unsigned short ADS102 : 1;
  unsigned short ADS103 : 1;
  unsigned short ADS104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short ADS104 : 1;
  unsigned short ADS103 : 1;
  unsigned short ADS102 : 1;
  unsigned short ADS101 : 1;
  unsigned short ADS100 : 1;
#endif
};

union un_s12ad1_adads1
{
  unsigned short WORD;
  struct st_s12ad1_adads1_bit BIT;
};

struct st_s12ad1_adadc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADC : 3;
  unsigned char  : 4;
  unsigned char AVEE : 1;
#else
  unsigned char AVEE : 1;
  unsigned char  : 4;
  unsigned char ADC : 3;
#endif
};

union un_s12ad1_adadc
{
  unsigned char BYTE;
  struct st_s12ad1_adadc_bit BIT;
};

struct st_s12ad1_adcer_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 1;
  unsigned short ADPRC : 2;
  unsigned short  : 2;
  unsigned short ACE : 1;
  unsigned short  : 2;
  unsigned short DIAGVAL : 2;
  unsigned short DIAGLD : 1;
  unsigned short DIAGM : 1;
  unsigned short  : 3;
  unsigned short ADRFMT : 1;
#else
  unsigned short ADRFMT : 1;
  unsigned short  : 3;
  unsigned short DIAGM : 1;
  unsigned short DIAGLD : 1;
  unsigned short DIAGVAL : 2;
  unsigned short  : 2;
  unsigned short ACE : 1;
  unsigned short  : 2;
  unsigned short ADPRC : 2;
  unsigned short  : 1;
#endif
};

union un_s12ad1_adcer
{
  unsigned short WORD;
  struct st_s12ad1_adcer_bit BIT;
};

struct st_s12ad1_adstrgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TRSB : 6;
  unsigned short  : 2;
  unsigned short TRSA : 6;
  unsigned short  : 2;
#else
  unsigned short  : 2;
  unsigned short TRSA : 6;
  unsigned short  : 2;
  unsigned short TRSB : 6;
#endif
};

union un_s12ad1_adstrgr
{
  unsigned short WORD;
  struct st_s12ad1_adstrgr_bit BIT;
};

struct st_s12ad1_adexicr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short TSSAD : 1;
  unsigned short OCSAD : 1;
  unsigned short  : 6;
  unsigned short TSSA : 1;
  unsigned short OCSA : 1;
  unsigned short TSSB : 1;
  unsigned short OCSB : 1;
  unsigned short  : 1;
  unsigned short EXSEL : 2;
  unsigned short EXOEN : 1;
#else
  unsigned short EXOEN : 1;
  unsigned short EXSEL : 2;
  unsigned short  : 1;
  unsigned short OCSB : 1;
  unsigned short TSSB : 1;
  unsigned short OCSA : 1;
  unsigned short TSSA : 1;
  unsigned short  : 6;
  unsigned short OCSAD : 1;
  unsigned short TSSAD : 1;
#endif
};

union un_s12ad1_adexicr
{
  unsigned short WORD;
  struct st_s12ad1_adexicr_bit BIT;
};

struct st_s12ad1_adansb0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSB000 : 1;
  unsigned short ANSB001 : 1;
  unsigned short ANSB002 : 1;
  unsigned short ANSB003 : 1;
  unsigned short ANSB004 : 1;
  unsigned short ANSB005 : 1;
  unsigned short ANSB006 : 1;
  unsigned short ANSB007 : 1;
  unsigned short ANSB008 : 1;
  unsigned short ANSB009 : 1;
  unsigned short ANSB010 : 1;
  unsigned short ANSB011 : 1;
  unsigned short ANSB012 : 1;
  unsigned short ANSB013 : 1;
  unsigned short ANSB014 : 1;
  unsigned short ANSB015 : 1;
#else
  unsigned short ANSB015 : 1;
  unsigned short ANSB014 : 1;
  unsigned short ANSB013 : 1;
  unsigned short ANSB012 : 1;
  unsigned short ANSB011 : 1;
  unsigned short ANSB010 : 1;
  unsigned short ANSB009 : 1;
  unsigned short ANSB008 : 1;
  unsigned short ANSB007 : 1;
  unsigned short ANSB006 : 1;
  unsigned short ANSB005 : 1;
  unsigned short ANSB004 : 1;
  unsigned short ANSB003 : 1;
  unsigned short ANSB002 : 1;
  unsigned short ANSB001 : 1;
  unsigned short ANSB000 : 1;
#endif
};

union un_s12ad1_adansb0
{
  unsigned short WORD;
  struct st_s12ad1_adansb0_bit BIT;
};

struct st_s12ad1_adansb1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSB100 : 1;
  unsigned short ANSB101 : 1;
  unsigned short ANSB102 : 1;
  unsigned short ANSB103 : 1;
  unsigned short ANSB104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short ANSB104 : 1;
  unsigned short ANSB103 : 1;
  unsigned short ANSB102 : 1;
  unsigned short ANSB101 : 1;
  unsigned short ANSB100 : 1;
#endif
};

union un_s12ad1_adansb1
{
  unsigned short WORD;
  struct st_s12ad1_adansb1_bit BIT;
};

struct st_s12ad1_adrd_bit_right
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short AD : 12;
  unsigned short  : 2;
  unsigned short DIAGST : 2;
#else
  unsigned short DIAGST : 2;
  unsigned short  : 2;
  unsigned short AD : 12;
#endif
};

struct st_s12ad1_adrd_bit_left
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short DIAGST : 2;
  unsigned short  : 2;
  unsigned short AD : 12;
#else
  unsigned short AD : 12;
  unsigned short  : 2;
  unsigned short DIAGST : 2;
#endif
};

union un_s12ad1_adrd_bit
{
  struct st_s12ad1_adrd_bit_right RIGHT;
  struct st_s12ad1_adrd_bit_left LEFT;
};

union un_s12ad1_adrd
{
  unsigned short WORD;
};

struct st_s12ad1_adsampr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char PRO : 2;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char PRO : 2;
#endif
};

union un_s12ad1_adsampr
{
  unsigned char BYTE;
  struct st_s12ad1_adsampr_bit BIT;
};

struct st_s12ad1_adsam_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short  : 5;
  unsigned short SAM : 1;
  unsigned short  : 10;
#else
  unsigned short  : 10;
  unsigned short SAM : 1;
  unsigned short  : 5;
#endif
};

union un_s12ad1_adsam
{
  unsigned short WORD;
  struct st_s12ad1_adsam_bit BIT;
};

struct st_s12ad1_addiscr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char ADNDIS : 5;
  unsigned char  : 3;
#else
  unsigned char  : 3;
  unsigned char ADNDIS : 5;
#endif
};

union un_s12ad1_addiscr
{
  unsigned char BYTE;
  struct st_s12ad1_addiscr_bit BIT;
};

struct st_s12ad1_adgspcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short PGS : 1;
  unsigned short GBRSCN : 1;
  unsigned short  : 12;
  unsigned short LGRRS : 1;
  unsigned short GBRP : 1;
#else
  unsigned short GBRP : 1;
  unsigned short LGRRS : 1;
  unsigned short  : 12;
  unsigned short GBRSCN : 1;
  unsigned short PGS : 1;
#endif
};

union un_s12ad1_adgspcr
{
  unsigned short WORD;
  struct st_s12ad1_adgspcr_bit BIT;
};

struct st_s12ad1_adwinmon_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MONCOMB : 1;
  unsigned char  : 3;
  unsigned char MONCMPA : 1;
  unsigned char MONCMPB : 1;
  unsigned char  : 2;
#else
  unsigned char  : 2;
  unsigned char MONCMPB : 1;
  unsigned char MONCMPA : 1;
  unsigned char  : 3;
  unsigned char MONCOMB : 1;
#endif
};

union un_s12ad1_adwinmon
{
  unsigned char BYTE;
  struct st_s12ad1_adwinmon_bit BIT;
};

struct st_s12ad1_adcmpcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPAB : 2;
  unsigned short  : 7;
  unsigned short CMPBE : 1;
  unsigned short  : 1;
  unsigned short CMPAE : 1;
  unsigned short  : 1;
  unsigned short CMPBIE : 1;
  unsigned short WCMPE : 1;
  unsigned short CMPAIE : 1;
#else
  unsigned short CMPAIE : 1;
  unsigned short WCMPE : 1;
  unsigned short CMPBIE : 1;
  unsigned short  : 1;
  unsigned short CMPAE : 1;
  unsigned short  : 1;
  unsigned short CMPBE : 1;
  unsigned short  : 7;
  unsigned short CMPAB : 2;
#endif
};

union un_s12ad1_adcmpcr
{
  unsigned short WORD;
  struct st_s12ad1_adcmpcr_bit BIT;
};

struct st_s12ad1_adcmpanser_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPSTS : 1;
  unsigned char CMPSOC : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char CMPSOC : 1;
  unsigned char CMPSTS : 1;
#endif
};

union un_s12ad1_adcmpanser
{
  unsigned char BYTE;
  struct st_s12ad1_adcmpanser_bit BIT;
};

struct st_s12ad1_adcmpler_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPLTS : 1;
  unsigned char CMPLOC : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char CMPLOC : 1;
  unsigned char CMPLTS : 1;
#endif
};

union un_s12ad1_adcmpler
{
  unsigned char BYTE;
  struct st_s12ad1_adcmpler_bit BIT;
};

struct st_s12ad1_adcmpansr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPCHA000 : 1;
  unsigned short CMPCHA001 : 1;
  unsigned short CMPCHA002 : 1;
  unsigned short CMPCHA003 : 1;
  unsigned short CMPCHA004 : 1;
  unsigned short CMPCHA005 : 1;
  unsigned short CMPCHA006 : 1;
  unsigned short CMPCHA007 : 1;
  unsigned short CMPCHA008 : 1;
  unsigned short CMPCHA009 : 1;
  unsigned short CMPCHA010 : 1;
  unsigned short CMPCHA011 : 1;
  unsigned short CMPCHA012 : 1;
  unsigned short CMPCHA013 : 1;
  unsigned short CMPCHA014 : 1;
  unsigned short CMPCHA015 : 1;
#else
  unsigned short CMPCHA015 : 1;
  unsigned short CMPCHA014 : 1;
  unsigned short CMPCHA013 : 1;
  unsigned short CMPCHA012 : 1;
  unsigned short CMPCHA011 : 1;
  unsigned short CMPCHA010 : 1;
  unsigned short CMPCHA009 : 1;
  unsigned short CMPCHA008 : 1;
  unsigned short CMPCHA007 : 1;
  unsigned short CMPCHA006 : 1;
  unsigned short CMPCHA005 : 1;
  unsigned short CMPCHA004 : 1;
  unsigned short CMPCHA003 : 1;
  unsigned short CMPCHA002 : 1;
  unsigned short CMPCHA001 : 1;
  unsigned short CMPCHA000 : 1;
#endif
};

union un_s12ad1_adcmpansr0
{
  unsigned short WORD;
  struct st_s12ad1_adcmpansr0_bit BIT;
};

struct st_s12ad1_adcmpansr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPCHA100 : 1;
  unsigned short CMPCHA101 : 1;
  unsigned short CMPCHA102 : 1;
  unsigned short CMPCHA103 : 1;
  unsigned short CMPCHA104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short CMPCHA104 : 1;
  unsigned short CMPCHA103 : 1;
  unsigned short CMPCHA102 : 1;
  unsigned short CMPCHA101 : 1;
  unsigned short CMPCHA100 : 1;
#endif
};

union un_s12ad1_adcmpansr1
{
  unsigned short WORD;
  struct st_s12ad1_adcmpansr1_bit BIT;
};

struct st_s12ad1_adcmplr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPLCHA000 : 1;
  unsigned short CMPLCHA001 : 1;
  unsigned short CMPLCHA002 : 1;
  unsigned short CMPLCHA003 : 1;
  unsigned short CMPLCHA004 : 1;
  unsigned short CMPLCHA005 : 1;
  unsigned short CMPLCHA006 : 1;
  unsigned short CMPLCHA007 : 1;
  unsigned short CMPLCHA008 : 1;
  unsigned short CMPLCHA009 : 1;
  unsigned short CMPLCHA010 : 1;
  unsigned short CMPLCHA011 : 1;
  unsigned short CMPLCHA012 : 1;
  unsigned short CMPLCHA013 : 1;
  unsigned short CMPLCHA014 : 1;
  unsigned short CMPLCHA015 : 1;
#else
  unsigned short CMPLCHA015 : 1;
  unsigned short CMPLCHA014 : 1;
  unsigned short CMPLCHA013 : 1;
  unsigned short CMPLCHA012 : 1;
  unsigned short CMPLCHA011 : 1;
  unsigned short CMPLCHA010 : 1;
  unsigned short CMPLCHA009 : 1;
  unsigned short CMPLCHA008 : 1;
  unsigned short CMPLCHA007 : 1;
  unsigned short CMPLCHA006 : 1;
  unsigned short CMPLCHA005 : 1;
  unsigned short CMPLCHA004 : 1;
  unsigned short CMPLCHA003 : 1;
  unsigned short CMPLCHA002 : 1;
  unsigned short CMPLCHA001 : 1;
  unsigned short CMPLCHA000 : 1;
#endif
};

union un_s12ad1_adcmplr0
{
  unsigned short WORD;
  struct st_s12ad1_adcmplr0_bit BIT;
};

struct st_s12ad1_adcmplr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPLCHA100 : 1;
  unsigned short CMPLCHA101 : 1;
  unsigned short CMPLCHA102 : 1;
  unsigned short CMPLCHA103 : 1;
  unsigned short CMPLCHA104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short CMPLCHA104 : 1;
  unsigned short CMPLCHA103 : 1;
  unsigned short CMPLCHA102 : 1;
  unsigned short CMPLCHA101 : 1;
  unsigned short CMPLCHA100 : 1;
#endif
};

union un_s12ad1_adcmplr1
{
  unsigned short WORD;
  struct st_s12ad1_adcmplr1_bit BIT;
};

struct st_s12ad1_adcmpsr0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPSTCHA000 : 1;
  unsigned short CMPSTCHA001 : 1;
  unsigned short CMPSTCHA002 : 1;
  unsigned short CMPSTCHA003 : 1;
  unsigned short CMPSTCHA004 : 1;
  unsigned short CMPSTCHA005 : 1;
  unsigned short CMPSTCHA006 : 1;
  unsigned short CMPSTCHA007 : 1;
  unsigned short CMPSTCHA008 : 1;
  unsigned short CMPSTCHA009 : 1;
  unsigned short CMPSTCHA010 : 1;
  unsigned short CMPSTCHA011 : 1;
  unsigned short CMPSTCHA012 : 1;
  unsigned short CMPSTCHA013 : 1;
  unsigned short CMPSTCHA014 : 1;
  unsigned short CMPSTCHA015 : 1;
#else
  unsigned short CMPSTCHA015 : 1;
  unsigned short CMPSTCHA014 : 1;
  unsigned short CMPSTCHA013 : 1;
  unsigned short CMPSTCHA012 : 1;
  unsigned short CMPSTCHA011 : 1;
  unsigned short CMPSTCHA010 : 1;
  unsigned short CMPSTCHA009 : 1;
  unsigned short CMPSTCHA008 : 1;
  unsigned short CMPSTCHA007 : 1;
  unsigned short CMPSTCHA006 : 1;
  unsigned short CMPSTCHA005 : 1;
  unsigned short CMPSTCHA004 : 1;
  unsigned short CMPSTCHA003 : 1;
  unsigned short CMPSTCHA002 : 1;
  unsigned short CMPSTCHA001 : 1;
  unsigned short CMPSTCHA000 : 1;
#endif
};

union un_s12ad1_adcmpsr0
{
  unsigned short WORD;
  struct st_s12ad1_adcmpsr0_bit BIT;
};

struct st_s12ad1_adcmpsr1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short CMPSTCHA100 : 1;
  unsigned short CMPSTCHA101 : 1;
  unsigned short CMPSTCHA102 : 1;
  unsigned short CMPSTCHA103 : 1;
  unsigned short CMPSTCHA104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short CMPSTCHA104 : 1;
  unsigned short CMPSTCHA103 : 1;
  unsigned short CMPSTCHA102 : 1;
  unsigned short CMPSTCHA101 : 1;
  unsigned short CMPSTCHA100 : 1;
#endif
};

union un_s12ad1_adcmpsr1
{
  unsigned short WORD;
  struct st_s12ad1_adcmpsr1_bit BIT;
};

struct st_s12ad1_adcmpser_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPFTS : 1;
  unsigned char CMPFOC : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char CMPFOC : 1;
  unsigned char CMPFTS : 1;
#endif
};

union un_s12ad1_adcmpser
{
  unsigned char BYTE;
  struct st_s12ad1_adcmpser_bit BIT;
};

struct st_s12ad1_adcmpbnsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPCHB : 6;
  unsigned char  : 1;
  unsigned char CMPLB : 1;
#else
  unsigned char CMPLB : 1;
  unsigned char  : 1;
  unsigned char CMPCHB : 6;
#endif
};

union un_s12ad1_adcmpbnsr
{
  unsigned char BYTE;
  struct st_s12ad1_adcmpbnsr_bit BIT;
};

struct st_s12ad1_adcmpbsr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CMPSTB : 1;
  unsigned char  : 7;
#else
  unsigned char  : 7;
  unsigned char CMPSTB : 1;
#endif
};

union un_s12ad1_adcmpbsr
{
  unsigned char BYTE;
  struct st_s12ad1_adcmpbsr_bit BIT;
};

struct st_s12ad1_adansc0_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSC000 : 1;
  unsigned short ANSC001 : 1;
  unsigned short ANSC002 : 1;
  unsigned short ANSC003 : 1;
  unsigned short ANSC004 : 1;
  unsigned short ANSC005 : 1;
  unsigned short ANSC006 : 1;
  unsigned short ANSC007 : 1;
  unsigned short ANSC008 : 1;
  unsigned short ANSC009 : 1;
  unsigned short ANSC010 : 1;
  unsigned short ANSC011 : 1;
  unsigned short ANSC012 : 1;
  unsigned short ANSC013 : 1;
  unsigned short ANSC014 : 1;
  unsigned short ANSC015 : 1;
#else
  unsigned short ANSC015 : 1;
  unsigned short ANSC014 : 1;
  unsigned short ANSC013 : 1;
  unsigned short ANSC012 : 1;
  unsigned short ANSC011 : 1;
  unsigned short ANSC010 : 1;
  unsigned short ANSC009 : 1;
  unsigned short ANSC008 : 1;
  unsigned short ANSC007 : 1;
  unsigned short ANSC006 : 1;
  unsigned short ANSC005 : 1;
  unsigned short ANSC004 : 1;
  unsigned short ANSC003 : 1;
  unsigned short ANSC002 : 1;
  unsigned short ANSC001 : 1;
  unsigned short ANSC000 : 1;
#endif
};

union un_s12ad1_adansc0
{
  unsigned short WORD;
  struct st_s12ad1_adansc0_bit BIT;
};

struct st_s12ad1_adansc1_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned short ANSC100 : 1;
  unsigned short ANSC101 : 1;
  unsigned short ANSC102 : 1;
  unsigned short ANSC103 : 1;
  unsigned short ANSC104 : 1;
  unsigned short  : 11;
#else
  unsigned short  : 11;
  unsigned short ANSC104 : 1;
  unsigned short ANSC103 : 1;
  unsigned short ANSC102 : 1;
  unsigned short ANSC101 : 1;
  unsigned short ANSC100 : 1;
#endif
};

union un_s12ad1_adansc1
{
  unsigned short WORD;
  struct st_s12ad1_adansc1_bit BIT;
};

struct st_s12ad1_adgcexcr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TSSC : 1;
  unsigned char OCSC : 1;
  unsigned char  : 6;
#else
  unsigned char  : 6;
  unsigned char OCSC : 1;
  unsigned char TSSC : 1;
#endif
};

union un_s12ad1_adgcexcr
{
  unsigned char BYTE;
  struct st_s12ad1_adgcexcr_bit BIT;
};

struct st_s12ad1_adgctrgr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char TRSC : 6;
  unsigned char GCADIE : 1;
  unsigned char GRCE : 1;
#else
  unsigned char GRCE : 1;
  unsigned char GCADIE : 1;
  unsigned char TRSC : 6;
#endif
};

union un_s12ad1_adgctrgr
{
  unsigned char BYTE;
  struct st_s12ad1_adgctrgr_bit BIT;
};

struct st_smci10_smr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char CKS : 2;
  unsigned char BCP : 2;
  unsigned char PM : 1;
  unsigned char PE : 1;
  unsigned char BLK : 1;
  unsigned char GM : 1;
#else
  unsigned char GM : 1;
  unsigned char BLK : 1;
  unsigned char PE : 1;
  unsigned char PM : 1;
  unsigned char BCP : 2;
  unsigned char CKS : 2;
#endif
};

union un_smci10_smr
{
  unsigned char BYTE;
  struct st_smci10_smr_bit BIT;
};

struct st_smci_scr_bit
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

union un_smci10_scr
{
  unsigned char BYTE;
  struct st_smci_scr_bit BIT;
};

struct st_smci10_ssr_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  unsigned char MPBT : 1;
  unsigned char MPB : 1;
  unsigned char TEND : 1;
  unsigned char PER : 1;
  unsigned char ERS : 1;
  unsigned char ORER : 1;
  unsigned char RDRF : 1;
  unsigned char TDRE : 1;
#else
  unsigned char TDRE : 1;
  unsigned char RDRF : 1;
  unsigned char ORER : 1;
  unsigned char ERS : 1;
  unsigned char PER : 1;
  unsigned char TEND : 1;
  unsigned char MPB : 1;
  unsigned char MPBT : 1;
#endif
};

union un_smci10_ssr
{
  unsigned char BYTE;
  struct st_smci10_ssr_bit BIT;
};

struct st_smci10_scmr_bit
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

union un_smci10_scmr
{
  unsigned char BYTE;
  struct st_smci10_scmr_bit BIT;
};

typedef struct st_smci10
{
  union un_smci10_smr SMR;
  char           wk0[1];
  union un_smci10_scr SCR;
  char           wk1[1];
  union un_smci10_ssr SSR;
  char           wk2[1];
  union un_smci10_scmr SCMR;
} st_smci10_t;

typedef struct st_bsc
{
  union un_bsc_berclr BERCLR;
  char  wk0[3];
  union un_bsc_beren BEREN;
  char  wk1[3];
  union un_bsc_bersr1 BERSR1;
  char  wk2[1];
  union un_bsc_bersr2 BERSR2;
  char  wk3[4];
  union un_bsc_buspri BUSPRI;
  char  wk4[7408];
  union un_bsc_cs0mod CS0MOD;
  union un_bsc_cs0wcr1 CS0WCR1;
  union un_bsc_cs0wcr2 CS0WCR2;
  char  wk5[6];
  union un_bsc_cs1mod CS1MOD;
  union un_bsc_cs1wcr1 CS1WCR1;
  union un_bsc_cs1wcr2 CS1WCR2;
  char  wk6[6];
  union un_bsc_cs2mod CS2MOD;
  union un_bsc_cs2wcr1 CS2WCR1;
  union un_bsc_cs2wcr2 CS2WCR2;
  char  wk7[6];
  union un_bsc_cs3mod CS3MOD;
  union un_bsc_cs3wcr1 CS3WCR1;
  union un_bsc_cs3wcr2 CS3WCR2;
  char  wk8[6];
  union un_bsc_cs4mod CS4MOD;
  union un_bsc_cs4wcr1 CS4WCR1;
  union un_bsc_cs4wcr2 CS4WCR2;
  char  wk9[6];
  union un_bsc_cs5mod CS5MOD;
  union un_bsc_cs5wcr1 CS5WCR1;
  union un_bsc_cs5wcr2 CS5WCR2;
  char  wk10[6];
  union un_bsc_cs6mod CS6MOD;
  union un_bsc_cs6wcr1 CS6WCR1;
  union un_bsc_cs6wcr2 CS6WCR2;
  char  wk11[6];
  union un_bsc_cs7mod CS7MOD;
  union un_bsc_cs7wcr1 CS7WCR1;
  union un_bsc_cs7wcr2 CS7WCR2;
  char  wk12[1926];
  union un_bsc_cs0cr CS0CR;
  char  wk13[6];
  union un_bsc_cs0rec CS0REC;
  char  wk14[6];
  union un_bsc_cs1cr CS1CR;
  char  wk15[6];
  union un_bsc_cs1rec CS1REC;
  char  wk16[6];
  union un_bsc_cs2cr CS2CR;
  char  wk17[6];
  union un_bsc_cs2rec CS2REC;
  char  wk18[6];
  union un_bsc_cs3cr CS3CR;
  char  wk19[6];
  union un_bsc_cs3rec CS3REC;
  char  wk20[6];
  union un_bsc_cs4cr CS4CR;
  char  wk21[6];
  union un_bsc_cs4rec CS4REC;
  char  wk22[6];
  union un_bsc_cs5cr CS5CR;
  char  wk23[6];
  union un_bsc_cs5rec CS5REC;
  char  wk24[6];
  union un_bsc_cs6cr CS6CR;
  char  wk25[6];
  union un_bsc_cs6rec CS6REC;
  char  wk26[6];
  union un_bsc_cs7cr CS7CR;
  char  wk27[6];
  union un_bsc_cs7rec CS7REC;
  char  wk28[4];
  union un_bsc_csrecen CSRECEN;
  char  wk29[894];
  union un_bsc_sdccr SDCCR;
  union un_bsc_sdcmod SDCMOD;
  union un_bsc_sdamod SDAMOD;
  char  wk30[13];
  union un_bsc_sdself SDSELF;
  char  wk31[3];
  union un_bsc_sdrfcr SDRFCR;
  union un_bsc_sdrfen SDRFEN;
  char  wk32[9];
  union un_bsc_sdicr SDICR;
  char  wk33[3];
  union un_bsc_sdir SDIR;
  char  wk34[26];
  union un_bsc_sdadr SDADR;
  char  wk35[3];
  union un_bsc_sdtr SDTR;
  union un_bsc_sdmod SDMOD;
  char  wk36[6];
  union un_bsc_sdsr SDSR;
  char  wk37[269231];
  union un_bsc_ebmapcr EBMAPCR;
} st_bsc_t;

typedef struct st_cac
{
  union un_cac_cacr0 CACR0;
  union un_cac_cacr1 CACR1;
  union un_cac_cacr2 CACR2;
  union un_cac_caicr CAICR;
  union un_cac_castr CASTR;
  char  wk0[1];
  unsigned short CAULVR;
  unsigned short CALLVR;
  unsigned short CACNTBR;
} st_cac_t;

typedef struct st_can
{
  struct st_can_mb MB[32];
  union  un_can_mkr MKR[8];
  union  un_can_fidcr0 FIDCR0;
  union  un_can_fidcr1 FIDCR1;
  union  un_can_mkivlr MKIVLR;
  union  un_can_mier MIER;
  char   wk0[1008];
  union  un_can_mctl MCTL[32];
  union  un_can_ctlr CTLR;
  union  un_can_str STR;
  union  un_can_bcr BCR;
  union  un_can_rfcr RFCR;
  unsigned char  RFPCR;
  union  un_can_tfcr TFCR;
  unsigned char  TFPCR;
  union  un_can_eier EIER;
  union  un_can_eifr EIFR;
  unsigned char  RECR;
  unsigned char  TECR;
  union  un_can_ecsr ECSR;
  unsigned char  CSSR;
  union  un_can_mssr MSSR;
  union  un_can_msmr MSMR;
  unsigned short TSR;
  unsigned short AFSR;
  union  un_can_tcr TCR;
} st_can_t;

typedef struct st_cmt
{
  union un_cmt_cmstr0 CMSTR0;
  char  wk0[14];
  union un_cmt_cmstr1 CMSTR1;
} st_cmt_t;

typedef struct st_cmt0
{
  union un_cmt0_cmcr CMCR;
  unsigned short CMCNT;
  unsigned short CMCOR;
} st_cmt0_t;

typedef struct st_icu
{
  union un_icu_ir256 IR[256];
  union un_icu_dtcer256 DTCER[256];
  union un_icu_ier32 IER[32];
  char  wk0[192];
  union un_icu_swintr SWINTR;
  union un_icu_swint2r SWINT2R;
  char  wk1[14];
  union un_icu_fir FIR;
  char  wk2[14];
  union un_icu_ipr256 IPR[256];
  unsigned char  DMRSR0;
  char  wk3[3];
  unsigned char  DMRSR1;
  char  wk4[3];
  unsigned char  DMRSR2;
  char  wk5[3];
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
  union un_icu_irqcr16 IRQCR[16];
  char  wk11[16];
  union un_icu_irqflte0 IRQFLTE0;
  union un_icu_irqflte1 IRQFLTE1;
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
  union un_icu_grpbl0 GRPBL0;
  union un_icu_grpbl1 GRPBL1;
  union un_icu_grpbl2 GRPBL2;
  char  wk18[4];
  union un_icu_genbe0 GENBE0;
  char  wk19[44];
  union un_icu_genbl0 GENBL0;
  union un_icu_genbl1 GENBL1;
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
  union un_icu_grpal0 GRPAL0;
  union un_icu_grpal1 GRPAL1;
  char  wk24[56];
  union un_icu_genal0 GENAL0;
  union un_icu_genal1 GENAL1;
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
  union un_icu_sliar236 SLIAR237;
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
  char  wk0[1];
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
  union un_mpc_pwpr PWPR;
  char  wk3[32];
  union un_mpc_p00pfs P00PFS;
  union un_mpc_p01pfs P01PFS;
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
  union un_port0_pdr PDR;
  char  wk0[31];
  union un_port0_podr PODR;
  char  wk1[31];
  union un_port0_pidr PIDR;
  char  wk2[31];
  union un_port0_pmr PMR;
  char  wk3[31];
  union un_port0_odr0 ODR0;
  union un_port0_odr1 ODR1;
  char  wk4[62];
  union un_port0_pcr PCR;
  char  wk5[31];
  union un_port0_dscr DSCR;
  char  wk6[71];
  union un_port0_dscr2 DSCR2;
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
  char  wk8[1];
  union
  {
    union un_rtc_rhrar RHRAR;
    union un_rtc_bcnt2ar BCNT2AR;
  };
  char  wk9[1];
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
  char  wk12[1];
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
  char  wk13[3];
  union un_rtc_rcr1 RCR1;
  char  wk14[1];
  union un_rtc_rcr2 RCR2;
  char  wk15[1];
  union un_rtc_rcr3 RCR3;
  char  wk16[1];
  union un_rtc_rcr4 RCR4;
  char  wk17[1];
  union un_rtc_rfrh RFRH;
  union un_rtc_rfrl RFRL;
  union un_rtc_radj RADJ;
  char  wk18[17];
  union un_rtc_rtccr0 RTCCR0;
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
  char  wk22[1];
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
  char  wk25[1];
  union un_rtc_rmoncp0 RMONCP0;
  char  wk26[5];
  union
  {
    union un_rtc_rseccp1 RSECCP1;
    union un_rtc_bcnt0cp1 BCNT0CP1;
  };
  char  wk27[1];
  union
  {
    union un_rtc_rmincp1 RMINCP1;
    union un_rtc_bcnt1cp1 BCNT1CP1;
  };
  char  wk28[1];
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
  char  wk35[1];
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
  union un_system_mdmonr MDMONR;
  char  wk0[4];
  union un_system_syscr0 SYSCR0;
  union un_system_syscr1 SYSCR1;
  char  wk1[2];
  union un_system_sbycr SBYCR;
  char  wk2[2];
  union un_system_mstpcra MSTPCRA;
  union un_system_mstpcrb MSTPCRB;
  union un_system_mstpcrc MSTPCRC;
  union un_system_mstpcrd MSTPCRD;
  union un_system_sckcr SCKCR;
  union un_system_sckcr2 SCKCR2;
  union un_system_sckcr3 SCKCR3;
  union un_system_pllcr PLLCR;
  union un_system_pllcr2 PLLCR2;
  char  wk3[5];
  union un_system_bckcr BCKCR;
  char  wk4[1];
  union un_system_mosccr MOSCCR;
  union un_system_sosccr SOSCCR;
  union un_system_lococr LOCOCR;
  union un_system_ilococr ILOCOCR;
  union un_system_hococr HOCOCR;
  union un_system_hococr2 HOCOCR2;
  char  wk5[4];
  union un_system_oscovfsr OSCOVFSR;
  char  wk6[3];
  union un_system_ostdcr OSTDCR;
  union un_system_ostdsr OSTDSR;
  char  wk7[94];
  union un_system_opccr OPCCR;
  union un_system_rstckcr RSTCKCR;
  union un_system_moscwtcr MOSCWTCR;
  union un_system_soscwtcr SOSCWTCR;
  char  wk8[28];
  union un_system_rstsr2 RSTSR2;
  char  wk9[1];
  unsigned short SWRR;
  char  wk10[28];
  union un_system_lvd1cr1 LVD1CR1;
  union un_system_lvd1sr LVD1SR;
  union un_system_lvd2cr1 LVD2CR1;
  union un_system_lvd2sr LVD2SR;
  char  wk11[794];
  union un_system_prcr PRCR;
  char  wk12[3100];
  union un_system_romwt ROMWT;
  char  wk13[45667];
  union un_system_dpsbycr DPSBYCR;
  char  wk14[1];
  union un_system_dpsier0 DPSIER0;
  union un_system_dpsier1 DPSIER1;
  union un_system_dpsier2 DPSIER2;
  union un_system_dpsier3 DPSIER3;
  union un_system_dpsifr0 DPSIFR0;
  union un_system_dpsifr1 DPSIFR1;
  union un_system_dpsifr2 DPSIFR2;
  union un_system_dpsifr3 DPSIFR3;
  union un_system_dpsiegr0 DPSIEGR0;
  union un_system_dpsiegr1 DPSIEGR1;
  union un_system_dpsiegr2 DPSIEGR2;
  union un_system_dpsiegr3 DPSIEGR3;
  char  wk15[2];
  union un_system_rstsr0 RSTSR0;
  union un_system_rstsr1 RSTSR1;
  char  wk16[1];
  union un_system_mofcr MOFCR;
  union un_system_hocopcr HOCOPCR;
  char  wk17[2];
  union un_system_lvcmpcr LVCMPCR;
  union un_system_lvdlvlr LVDLVLR;
  char  wk18[1];
  union un_system_lvd1cr0 LVD1CR0;
  union un_system_lvd2cr0 LVD2CR0;
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
  char  wk0[13];
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
  char  wk0[2];
  union un_cmtw_cmwcr CMWCR;
  char  wk1[2];
  union un_cmtw_cmwior CMWIOR;
  char  wk2[6];
  unsigned long  CMWCNT;
  unsigned long  CMWCOR;
  unsigned long  CMWICR0;
  unsigned long  CMWICR1;
  unsigned long  CMWOCR0;
  unsigned long  CMWOCR1;
} st_cmtw_t;

typedef struct st_temps
{
  union un_temps_tscr TSCR;
} st_temps_t;

typedef struct st_tmr0
{
  union un_tmr0_tcr TCR;
  char  wk0[1];
  union un_tmr0_tcsr TCSR;
  char  wk1[1];
  unsigned char  TCORA;
  char  wk2[1];
  unsigned char  TCORB;
  char  wk3[1];
  unsigned char  TCNT;
  char  wk4[1];
  union un_tmr0_tccr TCCR;
  char  wk5[1];
  union un_tmr0_tcstr TCSTR;
} st_tmr0_t;

typedef struct st_tmr1
{
  union un_tmr1_tcr TCR;
  char  wk0[1];
  union un_tmr1_tcsr TCSR;
  char  wk1[1];
  unsigned char  TCORA;
  char  wk2[1];
  unsigned char  TCORB;
  char  wk3[1];
  unsigned char  TCNT;
  char  wk4[1];
  union un_tmr1_tccr TCCR;
  char  wk5[1];
  union un_tmr1_tcstr TCSTR;
} st_tmr1_t;

typedef struct st_tmr01
{
  unsigned short TCORA;
  unsigned short TCORB;
  unsigned short TCNT;
  unsigned short TCCR;
} st_tmr01_t;

typedef struct st_tpu0
{
  union un_tpu0_nfcr NFCR;
  char  wk0[7];
  union un_tpu0_tcr TCR;
  union un_tpu0_tmdr TMDR;
  union un_tpu0_tiorh TIORH;
  union un_tpu0_tiorl TIORL;
  union un_tpu0_tier TIER;
  union un_tpu0_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
  unsigned short TGRC;
  unsigned short TGRD;
} st_tpu0_t;

typedef struct st_tpu1
{
  char  wk0[1];
  union un_tpu1_nfcr NFCR;
  char  wk1[22];
  union un_tpu1_tcr TCR;
  union un_tpu1_tmdr TMDR;
  union un_tpu1_tior TIOR;
  char  wk2[1];
  union un_tpu1_tier TIER;
  union un_tpu1_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
} st_tpu1_t;

typedef struct st_tpu2
{
  union un_tpu2_nfcr NFCR;
  char  wk0[37];
  union un_tpu2_tcr TCR;
  union un_tpu2_tmdr TMDR;
  union un_tpu2_tior TIOR;
  char  wk1[1];
  union un_tpu2_tier TIER;
  union un_tpu2_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
} st_tpu2_t;

typedef struct st_tpu3
{
  char  wk0[1];
  union un_tpu3_nfcr NFCR;
  char  wk1[52];
  union un_tpu3_tcr TCR;
  union un_tpu3_tmdr TMDR;
  union un_tpu3_tiorh TIORH;
  union un_tpu3_tiorl TIORL;
  union un_tpu3_tier TIER;
  union un_tpu3_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
  unsigned short TGRC;
  unsigned short TGRD;
} st_tpu3_t;

typedef struct st_tpu4
{
  union un_tpu4_nfcr NFCR;
  char  wk0[67];
  union un_tpu4_tcr TCR;
  union un_tpu4_tmdr TMDR;
  union un_tpu4_tior TIOR;
  char  wk1[1];
  union un_tpu4_tier TIER;
  union un_tpu4_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
} st_tpu4_t;

typedef struct st_tpu5
{
  char  wk0[1];
  union un_tpu5_nfcr NFCR;
  char  wk1[82];
  union un_tpu5_tcr TCR;
  union un_tpu5_tmdr TMDR;
  union un_tpu5_tior TIOR;
  char  wk2[1];
  union un_tpu5_tier TIER;
  union un_tpu5_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
} st_tpu5_t;

typedef struct st_tpua
{
  union un_tpua_tstr TSTR;
  union un_tpua_tsyr TSYR;
} st_tpua_t;

typedef struct st_usb
{
  union un_usb_dpusr0r DPUSR0R;
  union un_usb_dpusr1r DPUSR1R;
} st_usb_t;

typedef struct st_usb0
{
  union un_usb0_syscfg SYSCFG;
  char  wk0[2];
  union un_usb0_syssts0 SYSSTS0;
  char  wk1[2];
  union un_usb0_dvstctr0 DVSTCTR0;
  char  wk2[10];
  union un_usb0_cfifo CFIFO;
  char  wk3[2];
  union un_usb0_d0fifo D0FIFO;
  char  wk4[2];
  union un_usb0_d1fifo D1FIFO;
  char  wk5[2];
  union un_usb0_cfifosel CFIFOSEL;
  union un_usb0_cfifoctr CFIFOCTR;
  char  wk6[4];
  union un_usb0_d0fifosel D0FIFOSEL;
  union un_usb0_d0fifoctr D0FIFOCTR;
  union un_usb0_d1fifosel D1FIFOSEL;
  union un_usb0_d1fifoctr D1FIFOCTR;
  union un_usb0_intenb0 INTENB0;
  union un_usb0_intenb1 INTENB1;
  char  wk7[2];
  union un_usb0_brdyenb BRDYENB;
  union un_usb0_nrdyenb NRDYENB;
  union un_usb0_bempenb BEMPENB;
  union un_usb0_sofcfg SOFCFG;
  char  wk8[2];
  union un_usb0_intsts0 INTSTS0;
  union un_usb0_intsts1 INTSTS1;
  char  wk9[2];
  union un_usb0_brdysts BRDYSTS;
  union un_usb0_nrdysts NRDYSTS;
  union un_usb0_bempsts BEMPSTS;
  union un_usb0_frmnum FRMNUM;
  union un_usb0_dvchgr DVCHGR;
  union un_usb0_usbaddr USBADDR;
  char  wk10[2];
  union un_usb0_usbreq USBREQ;
  unsigned short USBVAL;
  unsigned short USBINDX;
  unsigned short USBLENG;
  union un_usb0_dcpcfg DCPCFG;
  union un_usb0_dcpmaxp DCPMAXP;
  union un_usb0_dcpctr DCPCTR;
  char  wk11[2];
  union un_usb0_pipesel PIPESEL;
  char  wk12[2];
  union un_usb0_pipecfg PIPECFG;
  char  wk13[2];
  union un_usb0_pipemaxp PIPEMAXP;
  union un_usb0_pipeperi PIPEPERI;
  union un_usb0_pipe1ctr PIPE1CTR;
  union un_usb0_pipe2ctr PIPE2CTR;
  union un_usb0_pipe3ctr PIPE3CTR;
  union un_usb0_pipe4ctr PIPE4CTR;
  union un_usb0_pipe5ctr PIPE5CTR;
  union un_usb0_pipe6ctr PIPE6CTR;
  union un_usb0_pipe7ctr PIPE7CTR;
  union un_usb0_pipe8ctr PIPE8CTR;
  union un_usb0_pipe9ctr PIPE9CTR;
  char  wk14[14];
  union un_usb0_pipe1tre PIPE1TRE;
  unsigned short PIPE1TRN;
  union un_usb0_pipe2tre PIPE2TRE;
  unsigned short PIPE2TRN;
  union un_usb0_pipe3tre PIPE3TRE;
  unsigned short PIPE3TRN;
  union un_usb0_pipe4tre PIPE4TRE;
  unsigned short PIPE4TRN;
  union un_usb0_pipe5tre PIPE5TRE;
  unsigned short PIPE5TRN;
  char  wk15[44];
  union un_usb0_devadd0 DEVADD0;
  union un_usb0_devadd1 DEVADD1;
  union un_usb0_devadd2 DEVADD2;
  union un_usb0_devadd3 DEVADD3;
  union un_usb0_devadd4 DEVADD4;
  union un_usb0_devadd5 DEVADD5;
  char  wk16[20];
  union un_usb0_physlew PHYSLEW;
} st_usb0_t;

typedef struct st_wdt
{
  unsigned char  WDTRR;
  char  wk0[1];
  union un_wdt_wdtcr WDTCR;
  union un_wdt_wdtsr WDTSR;
  union un_wdt_wdtrcr WDTRCR;
} st_wdt_t;

typedef struct st_flashconst
{
  unsigned long  UIDR0;
  unsigned long  UIDR1;
  unsigned long  UIDR2;
  unsigned long  UIDR3;
} st_flashconst_t;

typedef struct st_tempsconst
{
  unsigned long  TSCDR;
} st_tempsconst_t;

typedef struct st_crc
{
  union un_crc_crccr CRCCR;
  char  wk0[3];
  union un_crc_crcdir CRCDIR;
  union un_crc_crcdor CRCDOR;
} st_crc_t;

typedef struct st_da
{
  unsigned short DADR0;
  unsigned short DADR1;
  union un_da_dacr DACR;
  union un_da_dadpr DADPR;
  union un_da_daadscr DAADSCR;
  char  wk0[1];
  union un_da_daampcr DAAMPCR;
  char  wk1[19];
  union un_da_daaswcr DAASWCR;
  char  wk2[17763];
  union un_da_daadusr DAADUSR;
} st_da_t;

typedef struct st_doc
{
  union un_doc_docr DOCR;
  char  wk0[1];
  unsigned short DODIR;
  unsigned short DODSR;
} st_doc_t;

typedef struct st_mtu
{
  union un_mtu_toera TOERA;
  char  wk0[2];
  union un_mtu_tgcra TGCRA;
  union un_mtu_tocr1a TOCR1A;
  union un_mtu_tocr2a TOCR2A;
  char  wk1[4];
  unsigned short TCDRA;
  unsigned short TDDRA;
  char  wk2[8];
  unsigned short TCNTSA;
  unsigned short TCBRA;
  char  wk3[12];
  union un_mtu_titcr1a TITCR1A;
  union un_mtu_titcnt1a TITCNT1A;
  union un_mtu_tbtera TBTERA;
  char  wk4[1];
  union un_mtu_tdera TDERA;
  char  wk5[1];
  union un_mtu_tolbra TOLBRA;
  char  wk6[3];
  union un_mtu_titmra TITMRA;
  union un_mtu_titcr2a TITCR2A;
  union un_mtu_titcnt2a TITCNT2A;
  char  wk7[35];
  union un_mtu_twcra TWCRA;
  char  wk8[15];
  union un_mtu_tmdr2a TMDR2A;
  char  wk9[15];
  union un_mtu_tstra TSTRA;
  union un_mtu_tsyra TSYRA;
  union un_mtu_tcsystr TCSYSTR;
  char  wk10[1];
  union un_mtu_trwera TRWERA;
  char  wk11[1925];
  union un_mtu_toerb TOERB;
  char  wk12[3];
  union un_mtu_tocr1b TOCR1B;
  union un_mtu_tocr2b TOCR2B;
  char  wk13[4];
  unsigned short TCDRB;
  unsigned short TDDRB;
  char  wk14[8];
  unsigned short TCNTSB;
  unsigned short TCBRB;
  char  wk15[12];
  union un_mtu_titcr1b TITCR1B;
  union un_mtu_titcnt1b TITCNT1B;
  union un_mtu_tbterb TBTERB;
  char  wk16[1];
  union un_mtu_tderb TDERB;
  char  wk17[1];
  union un_mtu_tolbrb TOLBRB;
  char  wk18[3];
  union un_mtu_titmrb TITMRB;
  union un_mtu_titcr2b TITCR2B;
  union un_mtu_titcnt2b TITCNT2B;
  char  wk19[35];
  union un_mtu_twcrb TWCRB;
  char  wk20[15];
  union un_mtu_twdr2b TMDR2B;
  char  wk21[15];
  union un_mtu_tstrb TSTRB;
  union un_mtu_tsyrb TSYRB;
  char  wk22[2];
  union un_mtu_trwerb TRWERB;
} st_mtu_t;

typedef struct st_mtu0
{
  union un_mtu0_nfcro NFCRO;
  char  wk0[8];
  union un_mtu0_nfcrc NFCRC;
  char  wk1[102];
  union un_mtu0_tcr TCR;
  union un_mtu0_tmdr1 TMDR1;
  union un_mtu0_tiorh TIORH;
  union un_mtu0_tiorl TIOR1;
  union un_mtu0_tier TIER;
  char  wk2[1];
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
  unsigned short TGRC;
  unsigned short TGRD;
  char  wk3[16];
  unsigned short TGRE;
  unsigned short TGRF;
  union un_mtu0_tier2 TIER2;
  char  wk4[1];
  union un_mtu0_tbtm TBTM;
  char  wk5[1];
  union un_mtu0_tcr2 TCR2;
} st_mtu0_t;

typedef struct st_mtu1
{
  char  wk0[1];
  union un_mtu1_nfcr1 NFCR1;
  char  wk1[238];
  union un_mtu1_tcr TCR;
  union un_mtu1_tmdr1 TMDR1;
  union un_mtu1_tior TIOR;
  char  wk2[1];
  union un_mtu1_tier TIER;
  union un_mtu1_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
  char  wk3[4];
  union un_mtu1_ticcr TICCR;
  union un_mtu1_tmdr3 TMDR3;
  char  wk4[2];
  union un_mtu1_tcr2 TCR2;
  char  wk5[11];
  unsigned long  TCNTLW;
  unsigned long  TGRALW;
  unsigned long  TGRBLW;
} st_mtu1_t;

typedef struct st_mtu2
{
  union un_mtu2_nfcr2 NFCR2;
  char  wk0[365];
  union un_mtu2_tcr TCR;
  union un_mtu2_tmdr1 TMDR1;
  union un_mtu2_tior TIOR;
  char  wk1[1];
  union un_mtu2_tier TIER;
  union un_mtu2_tsr TSR;
  unsigned short TCNT;
  unsigned short TGRA;
  unsigned short TGRB;
  union un_mtu2_tcr2 TCR2;
} st_mtu2_t;

typedef struct st_mtu3
{
  union un_mtu3_tcr TCR;
  char  wk0[1];
  union un_mtu3_tmdr1 TMDR1;
  char  wk1[1];
  union un_mtu3_tiorh TIORH;
  union un_mtu3_tiorl TIORL;
  char  wk2[2];
  union un_mtu3_tier TIER;
  char  wk3[7];
  unsigned short TCNT;
  char  wk4[6];
  unsigned short TGRA;
  unsigned short TGRB;
  char  wk5[8];
  unsigned short TGRC;
  unsigned short TGRD;
  char  wk6[4];
  union un_mtu3_tsr TSR;
  char  wk7[11];
  union un_mtu3_tbtm TBTM;
  char  wk8[19];
  union un_mtu3_tcr2 TCR2;
  char  wk9[37];
  unsigned short TGRE;
  char  wk10[31];
  union un_mtu3_nfcr3 NFCR3;
} st_mtu3_t;

typedef struct st_iwdt
{
  unsigned char  IWDTRR;
  char  wk0[1];
  union un_iwdt_iwdtcr IWDTCR;
  union un_iwdt_iwdtsr IWDTSR;
  union un_iwdt_iwdtrcr IWDTRCR;
  char  wk1[1];
  union un_iwdt_iwdtcstpr IWDTCSTPR;
} st_iwdt_t;

typedef struct st_mpu
{
  union un_mpu_rspage0 RSPAGE0;
  union un_mpu_repage0 REPAGE0;
  union un_mpu_rspage1 RSPAGE1;
  union un_mpu_repage1 REPAGE1;
  union un_mpu_rspage2 RSPAGE2;
  union un_mpu_repage2 REPAGE2;
  union un_mpu_rspage3 RSPAGE3;
  union un_mpu_repage3 REPAGE3;
  union un_mpu_rspage4 RSPAGE4;
  union un_mpu_repage4 REPAGE4;
  union un_mpu_rspage5 RSPAGE5;
  union un_mpu_repage5 REPAGE5;
  union un_mpu_rspage6 RSPAGE6;
  union un_mpu_repage6 REPAGE6;
  union un_mpu_rspage7 RSPAGE7;
  union un_mpu_repage7 REPAGE7;
  char  wk0[192];
  union un_mpu_mpen MPEN;
  union un_mpu_mpbac MPBAC;
  union un_mpu_mpeclr MPECLR;
  union un_mpu_mpests MPESTS;
  char  wk1[4];
  union un_mpu_mpdea MPDEA;
  char  wk2[8];
  union un_mpu_mpsa MPSA;
  union un_mpu_mpops MPOPS;
  union un_mpu_mpopi MPOPI;
  union un_mpu_mhiti MHITI;
  union un_mpu_mhitd MHITD;
} st_mpu_t;

typedef struct st_mmcif
{
  union un_mmcif_cecmdset CECMDSET;
  char  wk0[4];
  union un_mmcif_cearg CEARG;
  union un_mmcif_ceargcmd12 CEARGCMD12;
  union un_mmcif_cecmdctrl CECMDCTR1;
  union un_mmcif_ceblockset CEBLOCKSET;
  union un_mmcif_ceclkctrl CECLKCTRL;
  union un_mmcif_cebufacc CEBUFACC;
  unsigned long  CERESP3;
  unsigned long  CERESP2;
  unsigned long  CERESP1;
  unsigned long  CERESP0;
  union un_mmcif_cerespcmd12 CERESPCMD12;
  union un_mmcif_cedata CEDATA;
  char  wk1[4];
  union un_mmcif_ceboot CEBOOT;
  union un_mmcif_ceint CEINT;
  union un_mmcif_ceinten CEINTEN;
  union un_mmcif_cehoststs1 CEHOSTSTS1;
  union un_mmcif_cehoststs2 CEHOSTSTS2;
  char  wk2[32];
  union un_mmcif_cedetect CEDETECT;
  union un_mmcif_ceaddmode CEADDMODE;
  char  wk3[4];
  union un_mmcif_ceversion CEVERSION;
} st_mmcif_t;

typedef struct st_glcdc
{
  union un_glcdc_gr1clut0 GR1CLUT0[256];
  union un_glcdc_gr1clut1 GR1CLUT1[256];
  union un_glcdc_gr2clut0 GR2CLUT0[256];
  union un_glcdc_gr2clut1 GR2CLUT1[256];
  union un_glcdc_bgen BGEN;
  union un_glcdc_bgperi BGPERI;
  union un_glcdc_bgsync BGSYNC;
  union un_glcdc_bgvsize BGVSIZE;
  union un_glcdc_bghsize BGHSIZE;
  union un_glcdc_bgcolor BGCOLOR;
  union un_glcdc_bgmon BGMON;
  char  wk0[228];
  union un_glcdc_gr1ven GR1VEN;
  union un_glcdc_grlflmrd GR1FLMRD;
  char  wk1[4];
  unsigned long  GR1FLM2;
  union un_glcdc_gr1flm3 GR1FLM3;
  char  wk2[4];
  union un_glcdc_gr1flm5 GR1FLM5;
  union un_glcdc_gr1flm6 GR1FLM6;
  union un_glcdc_gr1ab1 GR1AB1;
  union un_glcdc_gr1ab2 GR1AB2;
  union un_glcdc_gr1ab3 GR1AB3;
  union un_glcdc_gr1ab4 GR1AB4;
  union un_glcdc_gr1ab5 GR1AB5;
  union un_glcdc_gr1ab6 GR1AB6;
  union un_glcdc_gr1ab7 GR1AB7;
  union un_glcdc_gr1ab8 GR1AB8;
  union un_glcdc_gr1ab9 GR1AB9;
  char  wk3[8];
  union un_glcdc_gr1base GRBASE;
  union un_glcdc_gr1clutint GR1CLUTINT;
  union un_glcdc_gr1mon GR1MON;
  char  wk4[168];
  union un_glcdc_gr2ven GR2VEN;
  union un_glcdc_gr2flmrd GR2FLMRD;
  char  wk5[4];
  unsigned long  GR2FLM2;
  union un_glcdc_gr2flm3 GR2FLM3;
  char  wk6[4];
  union un_glcdc_gr2flm5 GR2FLM5;
  union un_glcdc_gr2flm6 GR2FLM6;
  union un_glcdc_gr2ab1 GR2AB1;
  union un_glcdc_gr2ab2 GR2AB2;
  union un_glcdc_gr2ab3 GR2AB3;
  union un_glcdc_gr2ab4 GR2AB4;
  union un_glcdc_gr2ab5 GR2AB5;
  union un_glcdc_gr2ab6 GR2AB6;
  union un_glcdc_gr2ab7 GR2AB7;
  union un_glcdc_gr2ab8 GR2AB8;
  union un_glcdc_gr2ab9 GR2AB9;
  char  wk7[8];
  union un_glcdc_gr2base GR2BASE;
  union un_glcdc_gr2clutint GR2CLUTINT;
  union un_glcdc_gr2mon GR2MON;
  char  wk8[168];
  union un_glcdc_gamgven GAMGVEN;
  union un_glcdc_gamsw GAMSW;
  union un_glcdc_gamglut1 GAMGLUT1;
  union un_glcdc_gamglut2 GAMGLUT2;
  union un_glcdc_gamglut3 GAMGLUT3;
  union un_glcdc_gamglut4 GAMGLUT4;
  union un_glcdc_gamglut5 GAMGLUT5;
  union un_glcdc_gamglut6 GAMGLUT6;
  union un_glcdc_gamglut7 GAMGLUT7;
  union un_glcdc_gamglut8 GAMGLUT8;
  union un_glcdc_gamgarea1 GAMGAREA1;
  union un_glcdc_gamgarea2 GAMGAREA2;
  union un_glcdc_gamgarea3 GAMGAREA3;
  union un_glcdc_gamgarea4 GAMGAREA4;
  union un_glcdc_gamgarea5 GAMGAREA5;
  char  wk9[4];
  union un_glcdc_gambven GAMBVEN;
  char  wk10[4];
  union un_glcdc_gamblut1 GAMBLUT1;
  union un_glcdc_gamblut2 GAMBLUT2;
  union un_glcdc_gamblut3 GAMBLUT3;
  union un_glcdc_gamblut4 GAMBLUT4;
  union un_glcdc_gamblut5 GAMBLUT5;
  union un_glcdc_gamblut6 GAMBLUT6;
  union un_glcdc_gamblut7 GAMBLUT7;
  union un_glcdc_gamblut8 GAMBLUT8;
  union un_glcdc_gambarea1 GAMBAREA1;
  union un_glcdc_gambarea2 GAMBAREA2;
  union un_glcdc_gambarea3 GAMBAREA3;
  union un_glcdc_gambarea4 GAMBAREA4;
  union un_glcdc_gambarea5 GAMBAREA5;
  char  wk11[4];
  union un_glcdc_gamrven GAMRVEN;
  char  wk12[4];
  union un_glcdc_gamrlut1 GAMRLUT1;
  union un_glcdc_gamrlut2 GAMRLUT2;
  union un_glcdc_gamrlut3 GAMRLUT3;
  union un_glcdc_gamrlut4 GAMRLUT4;
  union un_glcdc_gamrlut5 GAMRLUT5;
  union un_glcdc_gamrlut6 GAMRLUT6;
  union un_glcdc_gamrlut7 GAMRLUT7;
  union un_glcdc_gamrlut8 GAMRLUT8;
  union un_glcdc_gamrarea1 GAMRAREA1;
  union un_glcdc_gamrarea2 GAMRAREA2;
  union un_glcdc_gamrarea3 GAMRAREA3;
  union un_glcdc_gamrarea4 GAMRAREA4;
  union un_glcdc_gamrarea5 GAMRAREA5;
  char  wk13[4];
  union un_glcdc_outven OUTVEN;
  union un_glcdc_outset OUTSET;
  union un_glcdc_bright1 BRIGHT1;
  union un_glcdc_bright2 BRIGHT2;
  union un_glcdc_contrast CONTRAST;
  union un_glcdc_paneldtha PANELDTHA;
  char  wk14[12];
  union un_glcdc_clkphase CLKPHASE;
  char  wk15[28];
  union un_glcdc_tcontim TCONTIM;
  union un_glcdc_tconstva1 TCONSTVA1;
  union un_glcdc_tconstvat2 TCONSTVAT2;
  union un_glcdc_tconstvb1 TCONSTVB1;
  union un_glcdc_tconstvb2 TCONSTVB2;
  union un_glcdc_tconstha1 TCONSTHA1;
  union un_glcdc_tconstha2 TCONSTHA2;
  union un_glcdc_tconsthb1 TCONSTHB1;
  union un_glcdc_tconsthb2 TCONSTHB2;
  union un_glcdc_tconde TCONDE;
  char  wk16[20];
  union un_glcdc_dtcten DTCTEN;
  union un_glcdc_inten INTEN;
  union un_glcdc_stclr STCLR;
  union un_glcdc_stmon STMON;
  union un_glcdc_panelclk PANELCLK;
} st_glcdc_t;

typedef struct st_mtu4
{
  char  wk0[1];
  union un_mtu4_tcr TCR;
  char  wk1[1];
  union un_mtu4_tmdr1 TMDR1;
  char  wk2[2];
  union un_mtu4_tiorh TIORH;
  union un_mtu4_tiorl TIORL;
  char  wk3[1];
  union un_mtu4_tier TIER;
  char  wk4[8];
  unsigned short TCNT;
  char  wk5[8];
  unsigned short TGRA;
  unsigned short TGRB;
  char  wk6[8];
  unsigned short TGRC;
  unsigned short TGRD;
  char  wk7[1];
  union un_mtu4_tsr TSR;
  char  wk8[11];
  union un_mtu4_tbtm TBTM;
  char  wk9[6];
  union un_mtu4_tadcr TADCR;
  char  wk10[2];
  unsigned short TADCORA;
  unsigned short TADCORB;
  unsigned short TADCOBRA;
  unsigned short TADCOBRB;
  char wk11[1];
  union un_mtu4_tcr2 TCR2;
  char  wk12[38];
  unsigned short TGRE;
  unsigned short TGRF;
  char  wk13[28];
  union un_mtu4_nfcr4 NFCR4;
} st_mtu4_t;

typedef struct st_mtu5
{
  char  wk0[1];
  union un_mtu5_nfcr5 NFCR5;
  char  wk1[490];
  unsigned short TCNTU;
  unsigned short TGRU;
  union un_mtu5_tcru TCRU;
  union un_mtu5_tcr2u TCR2U;
  union un_mtu5_tioru TIORU;
  char  wk2[9];
  unsigned short TCNTV;
  unsigned short TGRV;
  union un_mtu5_tcrv TCRV;
  union un_mtu5_tcr2v TCR2V;
  union un_mtu5_tiorv TIORV;
  char  wk3[9];
  unsigned short TCNTW;
  unsigned short TGRW;
  union un_mtu5_tcrw TCRW;
  union un_mtu5_tcr2w TCR2W;
  union un_mtu5_tiorw TIORW;
  char  wk4[11];
  union un_mtu5_tier TIER;
  char  wk5[1];
  union un_mtu5_tstr TSTR;
  char  wk6[1];
  union un_mtu5_tcntcmpclr TCNTCMPCLR;
} st_mtu5_t;

typedef struct st_smci0
{
  union un_smcio_smr SMR;
  unsigned char  BRR;
  union un_smcio_scr SCR;
  unsigned char  TDR;
  union un_smcio_ssr SSR;
  unsigned char  RDR;
  union un_smcio_smcr SMCR;
} st_smci0_t;

typedef struct st_riic
{
  union un_riic_iccr1 ICCR1;
  union un_riic_iccr2 ICCR2;
  union un_riic_icmr1 ICMR1;
  union un_riic_icmr2 ICMR2;
  union un_riic_icmr3 ICMR3;
  union un_riic_icfer ICFER;
  union un_riic_icser ICSER;
  union un_riic_icier ICIER;
  union un_riic_icsr1 ICSR1;
  union un_riic_icsr2 ICSR2;
  union un_riic_sarl0 SARL0;
  union un_riic_saru0 SARU0;
  union un_riic_sarl1 SARL1;
  union un_riic_saru1 SARU1;
  union un_riic_sarl2 SARL2;
  union un_riic_saru2 SARU2;
  union un_riic_icbrl ICBRL;
  union un_riic_icbrh ICBRH;
  unsigned char  ICDRT;
  unsigned char  ICDRR;
} st_riic_t;

typedef struct st_rspi
{
  union un_rspi_spcr SPCR;
  union un_rspi_sslp SSLP;
  union un_rspi_sppcr SPPCR;
  union un_rspi_spsr SPSR;
  union un_rspi_spdr SPDR;
  union un_rspi_spscr SPSCR;
  union un_rspi_spssr SPSSR;
  unsigned char  SPBR;
  union un_rspi_spdcr SPDCR;
  union un_rspi_spckd SPCKD;
  union un_rspi_sslnd SSLND;
  union un_rspi_spnd SPND;
  union un_rspi_spcr2 SPCR2;
  union un_rspi_spcmd0 SPCMD0;
  union un_rspi_spcmd1 SPCMD1;
  union un_rspi_spcmd2 SPCMD2;
  union un_rspi_spcmd3 SPCMD3;
  union un_rspi_spcmd4 SPCMD4;
  union un_rspi_spcmd5 SPCMD5;
  union un_rspi_spcmd6 SPCMD6;
  union un_rspi_spcmd7 SPCMD7;
  union un_rspi_spdcr2 SPDCR2;
} st_rspi_t;

typedef struct st_sdhi
{
  union un_sdhi_spcmd SPCMD;
  char  wk0[4];
  unsigned long  SDARG;
  char  wk1[4];
  union un_sdhi_sdstop SDSTOP;
  unsigned long  SDBLKCNT;
  unsigned long  SDRSP10;
  char  wk2[4];
  unsigned long  SDRSP32;
  char  wk3[4];
  unsigned long  SDRSP54;
  char  wk4[4];
  unsigned long  SDRSP76;
  char  wk5[4];
  union un_sdhi_sdsts1 SDSTS1;
  union un_sdhi_sdsts2 SDSTS2;
  union un_sdhi_sdimsk1 SDIMSK1;
  union un_sdhi_sdimsk2 SDIMSK2;
  union un_sdhi_sdclkcr SDCLKCR;
  union un_sdhi_sdsize SDSIZE;
  union un_sdhi_sdopt SDOPT;
  char  wk6[4];
  union un_sdhi_sdersts1 SDERSTS1;
  union un_sdhi_sdersts2 SDERSTS2;
  unsigned long  SDBUFR;
  char  wk7[4];
  union un_sdhi_sdiomd SDIOMD;
  union un_sdhi_sdiosts SDIOSTS;
  union un_sdhi_sdioimsk SDIOIMSK;
  char  wk8[316];
  union un_sdhi_sddmaen SDDMAEN;
  char  wk9[12];
  union un_sdhi_sdrst SDRST;
  union un_sdhi_sdver SDVER;
  char  wk10[24];
  union un_sdhi_sdswap SDSWAP;
} st_sdhi_t;

typedef struct st_sdsi
{
  union un_sdsi_fn1accr FN1ACCR;
  union un_sdsi_intencr1 INTENCR1;
  union un_sdsi_intsr1 INTSR1;
  union un_sdsi_sdcmdcr SDCMDCR;
  union un_sdsi_sdcadd0r SDCADD0R;
  union un_sdsi_sdcadd1r SDCADD1R;
  union un_sdsi_sdcadd2r SDCADD2R;
  union un_sdsi_sdsicr1 SDSICR1;
  union un_sdsi_dmacr1 DMACR1;
  union un_sdsi_blkcnt BLKCNT;
  union un_sdsi_bytcnt BYTCNT;
  union un_sdsi_dmatraddr DMATRADDR;
  char  wk0[236];
  union un_sdsi_sdsicr2 SDICR2;
  union un_sdsi_sdsicr3 SDICR3;
  union un_sdsi_intencr2 INTENCR2;
  union un_sdsi_intsr2 INTSR2;
  union un_sdsi_dmacr2 DMACR2;
  char  wk1[236];
  unsigned long  CISDATAR[27];
  char  wk2[4];
  union un_sdsi_fbr1 FBR1;
  union un_sdsi_fbr2 FBR2;
  union un_sdsi_fbr3 FBR3;
  union un_sdsi_fbr4 FBR4;
  union un_sdsi_fbr5 FBR5;
  char  wk3[1404];
  union un_sdsi_fn1datar1 FN1DATAR1[64];
  union un_sdsi_fn1datar2 FN1DATAR2[64];
  union un_sdsi_fn1datar3 FN1DATAR3[64];
  union un_sdsi_fn1intvecr FN1INTVECR;
  union un_sdsi_fn1intclrr FN1INTCLRR;
  char  wk4[254];
  union un_sdsi_fn1datar5 FN1DATAR5[256];
} st_sdsi_t;

typedef struct st_mtu6
{
  union un_mtu6_tcr TCR;
  char  wk0[1];
  union un_mtu6_tmdr1 TMDR1;
  char  wk1[1];
  union un_mtu6_tiorh TIORH;
  union un_mtu6_tiorl TIORL;
  char  wk2[2];
  union un_mtu6_tier TIER;
  char  wk3[7];
  unsigned short TCNT;
  char  wk4[6];
  unsigned short TGRA;
  unsigned short TGRB;
  char  wk5[8];
  unsigned short TGRC;
  unsigned short TGRD;
  char  wk6[4];
  union un_mtu6_tsr TSR;
  char  wk7[11];
  union un_mtu6_tbtm TBTM;
  char  wk8[19];
  union un_mtu6_tcr2 TCR2;
  char  wk9[3];
  union un_mtu6_tsycr TSYCR;
  char  wk10[33];
  unsigned short TGRE;
  char  wk11[31];
} st_mtu6_t;

typedef struct st_mtu7
{
  char  wk0[1];
  union un_mtu7_tcr TCR;
  char  wk1[1];
  union un_mtu7_tmdr1 TMDR1;
  char  wk2[2];
  union un_mtu7_tiorh TIORH;
  union un_mtu7_tiorl TIORL;
  char  wk3[1];
  union un_mtu7_tier TIER;
  char  wk4[8];
  unsigned short TCNT;
  char  wk5[8];
  unsigned short TGRA;
  unsigned short TGRB;
  char  wk6[8];
  unsigned short TGRC;
  unsigned short TGRD;
  char  wk7[1];
  union un_mtu7_tsr TSR;
  char  wk8[11];
  union un_mtu7_tbtm TBTM;
  char  wk9[6];
  union un_mtu7_tadcr TADCR;
  char  wk10[2];
  unsigned short TADCORA;
  unsigned short TADCORB;
  unsigned short TADCOBRA;
  unsigned short TADCOBRB;
  char  wk11[1];
  union un_mtu7_tcr2 TCR2;
  char  wk12[38];
  unsigned short TGRE;
  unsigned short TGRF;
  char  wk13[28];
  union un_mtu7_nfcr7 NFCR7;
} st_mtu7_t;

typedef struct st_mtu8
{
  union un_mtu8_nfcr8 NFCR8;
  char  wk0[871];
  union un_mtu8_tcr TCR;
  union un_mtu8_tmdr1 TMDR1;
  union un_mtu8_tiorh TIORH;
  union un_mtu8_tiorl TIORL;
  union un_mtu8_tier TIER;
  char  wk1[1];
  union un_mtu8_tcr2 TCR2;
  char  wk2[1];
  unsigned long  TCNT;
  unsigned long  TGRA;
  unsigned long  TGRB;
  unsigned long  TGRC;
  unsigned long  TGRD;
} st_mtu8_t;

typedef struct st_port6
{
  union un_port6_pdr PDR;
  char  wk0[31];
  union un_port6_podr PODR;
  char  wk1[31];
  union un_port6_pidr PIDR;
  char  wk2[31];
  union un_port6_pmr PMR;
  char  wk3[37];
  union un_port6_ord0 ORD0;
  union un_port6_ord1 ORD1;
  char  wk4[56];
  union un_port6_pcr PCR;
} st_port6_t;

typedef struct st_dmac
{
  union un_dmac_dmast DMAST;
  char  wk0[3];
  union un_dmac_dmist DMIST;
} st_dmac_t;

typedef struct st_dmac0
{
  void  *DMSAR;
  void  *DMDAR;
  unsigned long  DMCRA;
  unsigned short DMCRB;
  char  wk0[2];
  union un_dmac0_dmtmd DMTMD;
  char  wk1[1];
  union un_dmac0_dmint DMINT;
  union un_dmac0_dmamd DMAMD;
  char  wk2[2];
  unsigned long  DMOFR;
  union un_dmac0_dmcnt DMCNT;
  union un_dmac0_dmreq DMREQ;
  union un_dmac0_dmsts DMSTS;
  union un_dmac0_dmcsl DMCSL;
} st_dmac0_t;

typedef struct st_dmac1
{
  void  *DMSAR;
  void  *DMDAR;
  unsigned long  DMCRA;
  unsigned short DMCRB;
  char  wk0[2];
  union un_dmac1_dmtmd DMTMD;
  char  wk1[1];
  union un_dmac1_dmint DMINT;
  union un_dmac1_dmamd DMAMD;
  char  wk2[6];
  union un_dmac1_dmcnt DMCNT;
  union un_dmac1_dmreq DMREQ;
  union un_dmac1_dmsts DMSTS;
  union un_dmac1_dmcsl DMCSL;
} st_dmac1_t;

typedef struct st_drw2d
{
  union
  {
    union un_drw2d_control CONTROL;
    union un_drw2d_status STATUS;
  };
  union
  {
    union un_drw2d_control2 CONTROL2;
    union un_drw2d_hwver HWVER;
  };
  char  wk0[8];
  unsigned long  L1START;
  unsigned long  L2START;
  unsigned long  L3START;
  unsigned long  L4START;
  unsigned long  L5START;
  unsigned long  L6START;
  unsigned long  L1XADD;
  unsigned long  L2XADD;
  unsigned long  L3XADD;
  unsigned long  L4XADD;
  unsigned long  L5XADD;
  unsigned long  L6XADD;
  unsigned long  L1YADD;
  unsigned long  L2YADD;
  unsigned long  L3YADD;
  unsigned long  L4YADD;
  unsigned long  L5YADD;
  unsigned long  L6YADD;
  unsigned long  L1BAND;
  unsigned long  L2BAND;
  char  wk1[4];
  union un_drw2d_color1 COLOR1;
  union un_drw2d_color2 COLOR2;
  char  wk2[8];
  unsigned long  PATTERN;
  union un_drw2d_size SIZE;
  union un_drw2d_pitch PITCH;
  unsigned long  ORIGIN;
  char  wk3[12];
  unsigned long  LUST;
  unsigned long  LUXADD;
  unsigned long  LUYADD;
  unsigned long  LVSTI;
  unsigned long  LVSTF;
  unsigned long  LVXADDI;
  unsigned long  LVYADDI;
  union un_drw2d_lvyxaddf LVYXADDF;
  char  wk4[4];
  unsigned long  TEXPITCH;
  union un_drw2d_texmsk TEXMSK;
  unsigned long  TEXORG;
  union un_drw2d_irqctl IRQCTL;
  union un_drw2d_cachectl CACHECTL;
  unsigned long  DLISTST;
  unsigned long  PERFCNT1;
  unsigned long  PERFCNT2;
  union un_drw2d_perftrg PERFTRG;
  char  wk5[4];
  unsigned long  TEXCLADDR;
  unsigned long  TEXCLDATA;
  unsigned long  TEXCLOFST;
  union un_drw2d_colkey COLKEY;
} st_drw2d_t;

typedef struct st_dtc
{
  union un_dtc_dtccr DTCCR;
  char  wk0[3];
  void  *DTCVBR;
  union un_dtc_dtcadmod DTCADMOD;
  char  wk1[3];
  union un_dtc_dtcst DTCST;
  char  wk2[1];
  union un_dtc_dtcsts DTCSTS;
  void  *DTCIBR;
  union un_dtc_dtcor DTCOR;
  char  wk3[1];
  union un_dtc_dtcsqe DTCSQE;
  unsigned long  DTCDISP;
} st_dtc_t;

typedef struct st_edmac
{
  union un_edmac_edmr EMDR;
  char  wk0[4];
  union un_edmac_edtrr EDTRR;
  char  wk1[4];
  union un_edmac_edrrr EDRRR;
  char  wk2[4];
  void  *TDLAR;
  char  wk3[4];
  void  *RDLAR;
  char  wk4[4];
  union un_edmac_eesr EESR;
  char  wk5[4];
  union un_edmac_eesipr EESIPR;
  char  wk6[4];
  union un_edmac_trscer TRSCER;
  char  wk7[4];
  union un_edmac_rmfcr RMFCR;
  char  wk8[4];
  union un_edmac_tftr TFTR;
  char  wk9[4];
  union un_edmac_fdr FDR;
  char  wk10[4];
  union un_edmac_rmcr RMCR;
  char  wk11[8];
  union un_edmac_tfucr TFUCR;
  union un_edmac_rfocr RFOCR;
  union un_edmac_iosr IOSR;
  union un_edmac_fcftr FCFTR;
  char  wk12[4];
  union un_edmac_rpadir RPADIR;
  union un_edmac_trimd TRIMD;
  char  wk13[72];
  void  *RBWAR;
  void  *RDFAR;
  char  wk14[4];
  void  *TBRAR;
  void  *TDFAR;
} st_edmac_t;

typedef struct st_elc
{
  union un_elc_elcr ELCR;
  union un_elc_elsr0 ELSR0;
  char  wk0[2];
  union un_elc_elsr3 ELSR3;
  union un_elc_elsr4 ELSR4;
  char  wk1[2];
  union un_elc_elsr7 ELSR7;
  char  wk2[2];
  union un_elc_elsr10 ELSR10;
  union un_elc_elsr11 ELSR11;
  union un_elc_elsr12 ELSR12;
  union un_elc_elsr13 ELSR13;
  char  wk3[1];
  union un_elc_elsr15 ELSR15;
  union un_elc_elsr16 ELSR16;
  char  wk4[1];
  union un_elc_elsr18 ELSR18;
  union un_elc_elsr19 ELSR19;
  union un_elc_elsr20 ELSR20;
  union un_elc_elsr21 ELSR21;
  union un_elc_elsr22 ELSR22;
  union un_elc_elsr23 ELSR23;
  union un_elc_elsr24 ELSR24;
  union un_elc_elsr25 ELSR25;
  union un_elc_elsr26 ELSR26;
  union un_elc_elsr27 ELSR27;
  union un_elc_elsr28 ELSR28;
  char  wk5[1];
  union un_elc_elopa ELOPA;
  union un_elc_elopb ELOPB;
  union un_elc_elopc ELOPC;
  union un_elc_elopd ELOPD;
  union un_elc_pgr1 PGR1;
  union un_elc_pgr2 PGR2;
  union un_elc_pgc1 PGC1;
  union un_elc_pgc2 PGC2;
  union un_elc_pdbf1 PDBF1;
  union un_elc_pdbf2 PDBF2;
  union un_elc_pel0 PEL0;
  union un_elc_pel1 PEL1;
  union un_elc_pel2 PEL2;
  union un_elc_pel3 PEL3;
  union un_elc_elsegr ELSEGR;
  char  wk6[3];
  union un_elc_elsr33 ELSR33;
  char  wk7[1];
  union un_elc_elsr35 ELSR35;
  union un_elc_elsr36 ELSR36;
  union un_elc_elsr37 ELSR37;
  union un_elc_elsr38 ELSR38;
  char  wk8[6];
  union un_elc_elsr45 ELSR45;
  char  wk9[1];
  union un_elc_elopf ELOPF;
  char  wk10[1];
  union un_elc_eloph ELOPH;
} st_elc_t;

typedef struct st_etherc
{
  union un_etherc_ecmr EMCR;
  char  wk0[4];
  union un_etherc_rflr RFLR;
  char  wk1[4];
  union un_etherc_ecsr ECSR;
  char  wk2[4];
  union un_etherc_ecsipr ECSIPR;
  char  wk3[4];
  union un_etherc_pir PIR;
  char  wk4[4];
  union un_etherc_psr PSR;
  char  wk5[20];
  union un_etherc_rdmlr RDMLR;
  char  wk6[12];
  union un_etherc_ipgr IPGR;
  union un_etherc_apr APR;
  union un_etherc_mpr MPR;
  char  wk7[4];
  union un_etherc_rfcf RFCF;
  union un_etherc_tpauser TPAUSER;
  union un_etherc_tpausecr TPAUSECR;
  union un_etherc_bcfrr BCFRR;
  char  wk8[80];
  unsigned long  MAHR;
  char  wk9[4];
  union un_etherc_malr MALR;
  char  wk10[4];
  unsigned long  TROCR;
  unsigned long  CDCR;
  unsigned long  LCCR;
  unsigned long  CNDCR;
  char  wk11[4];
  unsigned long  CEFCR;
  unsigned long  FRECR;
  unsigned long  TSFRCR;
  unsigned long  TLFRCR;
  unsigned long  RFCR;
  unsigned long  MAFCR;
} st_etherc_t;

typedef struct st_exdmac
{
  union un_exdmac_edmast EDMAST;
  char  wk0[479];
  unsigned long  CLSBR0;
  unsigned long  CLSBR1;
  unsigned long  CLSBR2;
  unsigned long  CLSBR3;
  unsigned long  CLSBR4;
  unsigned long  CLSBR5;
  unsigned long  CLSBR6;
  unsigned long  CLSBR7;
} st_exdmac_t;

typedef struct st_exdmac0
{
  void  *EDMSAR;
  void  *EDMDAR;
  unsigned long  EDMCRA;
  unsigned short EDMCRB;
  char  wk0[2];
  union un_exdmac0_edmtmd EDMTMD;
  union un_exdmac0_edmomd EDMOMD;
  union un_exdmac0_edmint EDMINT;
  union un_exdmac0_edmamd EDMAMD;
  unsigned long  EDMOFR;
  union un_exdmac0_edmcnt EDMCNT;
  union un_exdmac0_edmreq EDMREQ;
  union un_exdmac0_edmsts EDMSTS;
  char  wk1[1];
  union un_exdmac0_edmrmd EDMRMD;
  union un_exdmac0_edmerf EDMERF;
  union un_exdmac0_edmprf EDMPRF;
} st_exdmac0_t;

typedef struct st_exdmac1
{
  void  *EDMSAR;
  void  *EDMDAR;
  unsigned long  EDMCRA;
  unsigned short EDMCRB;
  char  wk0[2];
  union un_exdmac1_edmtmd EDMTMD;
  union un_exdmac1_edmomd EDMOMD;
  union un_exdmac1_edmint EDMINT;
  union un_exdmac1_edmamd EDMAMD;
  char  wk1[4];
  union un_exdmac1_edmcnt EDMCNT;
  union un_exdmac1_edmreq EDMREQ;
  union un_exdmac1_edmsts EDMSTS;
  char  wk2[1];
  union un_exdmac1_edmrmd EDMRMD;
  union un_exdmac1_edmerf EDMERF;
  union un_exdmac1_edmprf EDMPRF;
} st_exdmac1_t;

typedef struct st_flash
{
  union un_flash_romce ROMCE;
  char  wk0[2];
  union un_flash_romciv ROMCIV;
  char  wk1[45712];
  union un_flash_fwepror FWEPROR;
  char  wk2[7798185];
  unsigned char  EEPFCLK;
  char  wk3[8143];
  union un_flash_fastat FASTAT;
  char  wk4[3];
  union un_flash_faeint FAEINT;
  char  wk5[3];
  union un_flash_frdyie FRDYIE;
  char  wk6[23];
  union un_flash_fsaddr FSADDR;
  union un_flash_feaddr FEADDR;
  char  wk7[72];
  union un_flash_fstatr FSTATR;
  union un_flash_fentryr FENTRYR;
  char  wk8[6];
  union un_flash_fsunitr FSUNITR;
  char  wk9[18];
  union un_flash_fcmdr FCMDR;
  char  wk10[46];
  union un_flash_fbccnt FBCCNT;
  char  wk11[3];
  union un_flash_fbcstat FBCSTAT;
  char  wk12[3];
  union un_flash_fpsaddr FPSADDR;
  union un_flash_fawmon FAWMON;
  union un_flash_fcpsr FCPSR;
  char  wk13[2];
  union un_flash_fpckar FPCKAR;
  char  wk14[2];
  union un_flash_fsuacr FSUACR;
} st_flash_t;

typedef struct st_pdc
{
  union un_pdc_pccr0 PCCR0;
  union un_pdc_pccr1 PCCR1;
  union un_pdc_pcsr PCSR;
  union un_pdc_pcmonr PCMONR;
  union un_pdc_pcdr PCDR;
  union un_pdc_vcr VCR;
  union un_pdc_hcr HCR;
} st_pdc_t;

typedef struct st_poe
{
  union un_poe_icsr1 ICSR1;
  union un_poe_ocsr1 OCSR1;
  union un_poe_icsr2 ICSR2;
  union un_poe_ocsr2 OCSR2;
  union un_poe_icsr3 ICSR3;
  union un_poe_spoer SPOER;
  union un_poe_poecr1 POECR1;
  union un_poe_poecr2 POECR2;
  char  wk0[2];
  union un_poe_poecr4 POECR4;
  union un_poe_poecr5 POECR5;
  char  wk1[2];
  union un_poe_icsr4 ICSR4;
  union un_poe_icsr5 ICSR5;
  union un_poe_alr1 ALR1;
  union un_poe_icsr6 ICSR6;
  char  wk2[6];
  union un_poe_m0selr1 M0SELR1;
  union un_poe_m0selr2 M0SELR2;
  union un_poe_m3selr M3SELR;
  union un_poe_m4selr1 M4SELR1;
  union un_poe_m4selr2 M4SELR2;
  char  wk3[1];
  union un_poe_m6selr M6SELR;
} st_poe_t;

typedef struct st_portd
{
  union un_portd_pdr PDR;
  union un_portd_podr PODR;
  union un_portd_pidr PIDR;
  union un_portd_pmr PMR;
  union un_portd_ord0 ORD0;
  union un_portd_ord1 ORD1;
  union un_portd_pcr PCR;
  union un_portd_dscr DSCR;
  union un_portd_dscr2 DSCR2;
  char wk0[31];
  char wk1[31];
  char wk2[31];
  char wk3[44];
  char wk4[49];
  char wk5[31];
  char wk6[71];
} st_portd_t;

typedef struct st_ppg0
{
  union un_ppg0_pcr PCR;
  union un_ppg0_pmr OMR;
  union un_ppg0_nderh NDERH;
  union un_ppg0_nderl NDERL;
  union un_ppg0_podrh PODRH;
  union un_ppg0_podrl PODRL;
  union un_ppg0_ndrh NDRH;
  union un_ppg0_ndrl NDRL;
  union un_ppg0_ndrh2 NDRH2;
  union un_ppg0_ndrl2 NDRL2;
} st_ppg0_t;

typedef struct st_ppg1
{
  union un_ppg1_ptrslr PTRSLR;
  char  wk0[5];
  union un_ppg1_pcr PCR;
  union un_ppg1_pmr PMR;
  union un_ppg1_nderh NDERH;
  union un_ppg1_nderl NDERL;
  union un_ppg1_podrh PODRH;
  union un_ppg1_podrl PODRL;
  union un_ppg1_ndrh NDRH;
  union un_ppg1_ndrl NDRL;
  union un_ppg1_ndrh2 NDRH2;
  union un_ppg1_ndrl2 NDRL2;
} st_ppg1_t;

typedef struct st_qspi
{
  union un_qspi_spcr SPCR;
  union un_qspi_sslp SSLP;
  union un_qspi_sppcr SPPCR;
  union un_qspi_spsr SPSR;
  union un_qspi_spdr SPDR;
  union un_qspi_spscr SPSCR;
  union un_qspi_spssr SPSSR;
  union un_qspi_spbr SPBR;
  union un_qspi_spdcr SPDCR;
  union un_qspi_spckd SPCKD;
  union un_qspi_sslnd SSLND;
  union un_qspi_spnd SPND;
  char  wk0[1];
  union un_qspi_spcmd0 SPCMD0;
  union un_qspi_spcmd1 SPCMD1;
  union un_qspi_spcmd2 SPCMD2;
  union un_qspi_spcmd3 SPCMD3;
  union un_qspi_spbfcr SPBFCR;
  char  wk1[1];
  union un_qspi_spbdcr SPBDCR;
  unsigned long  SPBMUL0;
  unsigned long  SPBMUL1;
  unsigned long  SPBMUL2;
  unsigned long  SPBMUL3;
} st_qspi_t;

typedef struct st_ram
{
  union un_ram_rammode RAMMODE;
  union un_ram_ramsts RAMSTS;
  char  wk0[2];
  union un_ram_ramprcr RAMPRCR;
  char  wk1[3];
  union un_ram_ramecad RAMECAD;
  char  wk2[52];
  union un_ram_exrammode EXRAMMODE;
  union un_ram_exramsts EXRAMSTS;
  char  wk3[2];
  union un_ram_exramprcr EXRAMPRCR;
  char  wk4[3];
  union un_ram_exramecad EXRAMECAD;
} st_ram_t;

typedef struct st_s12ad
{
  union un_s12ad_adcsr ADCSR;
  char  wk0[2];
  union un_s12ad_adansa0 ADANSA0;
  char  wk1[2];
  union un_s12ad_adads0 ADADS0;
  char  wk2[2];
  union un_s12ad_adadc ADADC;
  char  wk3[1];
  union un_s12ad_adcer ADCER;
  union un_s12ad_adstrgr ADSTRGR;
  char  wk4[2];
  union un_s12ad_adansb0 ADANSB0;
  char  wk5[2];
  union un_s12ad_addbldr ADDBLDR;
  char  wk6[4];
  union un_s12ad_adrd ADRD;
  unsigned short ADDR0;
  unsigned short ADDR1;
  unsigned short ADDR2;
  unsigned short ADDR3;
  unsigned short ADDR4;
  unsigned short ADDR5;
  unsigned short ADDR6;
  unsigned short ADDR7;
  char  wk7[51];
  union un_s12ad_adsampr ADSAMPR;
  char  wk8[2];
  union un_s12ad_adshcr ADSHCR;
  char  wk9[6];
  union un_s12ad_adsam ADSAM;
  char  wk10[10];
  union un_s12ad_addiscr ADDISCR;
  char  wk11[1];
  union un_s12ad_adshmsr ADSHMSR;
  char  wk12[3];
  union un_s12ad_adgspcr ADGSPCR;
  char  wk13[2];
  unsigned short ADDBLDRA;
  unsigned short ADDBLDRB;
  char  wk14[4];
  union un_s12ad_adwinmon ADWINMON;
  char  wk15[3];
  union un_s12ad_adcmpcr ADCMPCR;
  char  wk16[2];
  union un_s12ad_adcmpansr0 ADCMPANSR0;
  char  wk17[2];
  union un_s12ad_adcmplr0 ADCMPLR0;
  char  wk18[2];
  unsigned short ADCMPDR0;
  unsigned short ADCMPDR1;
  union un_s12ad_adcmpsr0 ADCMPSR0;
  char  wk19[4];
  union un_s12ad_adcmpbnsr ADCMPBNSR;
  char  wk20[1];
  unsigned short ADWINLLB;
  unsigned short ADWINULB;
  union un_s12ad_adcmpbsr ADCMPBSR;
  char  wk21[39];
  union un_s12ad_adansc0 ADANSC0;
  char  wk22[3];
  union un_s12ad_adgctrgr ADGCTRGR;
  char  wk23[6];
  unsigned char  ADSSTR0;
  unsigned char  ADSSTR1;
  unsigned char  ADSSTR2;
  unsigned char  ADSSTR3;
  unsigned char  ADSSTR4;
  unsigned char  ADSSTR5;
  unsigned char  ADSSTR6;
  unsigned char  ADSSTR7;
} st_s12ad_t;

typedef struct st_s12ad1
{
  union un_s12ad1_adcsr ADCSR;
  char  wk0[2];
  union un_s12ad1_adansa0 ADANSA0;
  union un_s12ad1_adansa1 ADANSA1;
  union un_s12ad1_adads0 ADADSO;
  union un_s12ad1_adads1 ADADS1;
  union un_s12ad1_adadc ADADC;
  char  wk1[1];
  union un_s12ad1_adcer ADCER;
  union un_s12ad1_adstrgr ADSTRGR;
  union un_s12ad1_adexicr ADEXICR;
  union un_s12ad1_adansb0 ADANSB0;
  union un_s12ad1_adansb1 ADANSB1;
  unsigned short ADDBLDR;
  unsigned short ADTSDR;
  unsigned short ADOCDR;
  union un_s12ad1_adrd ADRD;
  unsigned short ADDR0;
  unsigned short ADDR1;
  unsigned short ADDR2;
  unsigned short ADDR3;
  unsigned short ADDR4;
  unsigned short ADDR5;
  unsigned short ADDR6;
  unsigned short ADDR7;
  unsigned short ADDR8;
  unsigned short ADDR9;
  unsigned short ADDR10;
  unsigned short ADDR11;
  unsigned short ADDR12;
  unsigned short ADDR13;
  unsigned short ADDR14;
  unsigned short ADDR15;
  unsigned short ADDR16;
  unsigned short ADDR17;
  unsigned short ADDR18;
  unsigned short ADDR19;
  unsigned short ADDR20;
  char  wk2[25];
  union un_s12ad1_adsampr ADSAMPR;
  char  wk3[10];
  union un_s12ad1_adsam ADSAM;
  char  wk4[10];
  union un_s12ad1_addiscr ADDISCR;
  char  wk5[5];
  union un_s12ad1_adgspcr ADGSPCR;
  char  wk6[2];
  unsigned short ADDBLDRA;
  unsigned short ADDBLDRB;
  char  wk7[4];
  union un_s12ad1_adwinmon ADWINMON;
  char  wk8[3];
  union un_s12ad1_adcmpcr ADCMPCR;
  union un_s12ad1_adcmpanser ADCMPANSER;
  union un_s12ad1_adcmpler ADCMPLER;
  union un_s12ad1_adcmpansr0 ADCMPANSR0;
  union un_s12ad1_adcmpansr1 ADCMPANSR1;
  union un_s12ad1_adcmplr0 ADCMPLR0;
  union un_s12ad1_adcmplr1 ADCMPLR1;
  unsigned short ADCMPDR0;
  unsigned short ADCMPDR1;
  union un_s12ad1_adcmpsr0 ADCMPSR0;
  union un_s12ad1_adcmpsr1 ADCMPSR1;
  union un_s12ad1_adcmpser ADCMPSER;
  char  wk9[1];
  union un_s12ad1_adcmpbnsr ADCMPBNSR;
  char  wk10[1];
  unsigned short ADWINLLB;
  unsigned short ADWINULB;
  union un_s12ad1_adcmpbsr ADCMPBSR;
  char  wk11[39];
  union un_s12ad1_adansc0 ADANSC0;
  union un_s12ad1_adansc1 ADANSC1;
  union un_s12ad1_adgcexcr ADGCEXCR;
  union un_s12ad1_adgctrgr ADGCTRGR;
  char  wk12[3];
  unsigned char  ADSSTRL;
  unsigned char  ADSSTRT;
  unsigned char  ADSSTRO;
  unsigned char  ADSSTR0;
  unsigned char  ADSSTR1;
  unsigned char  ADSSTR2;
  unsigned char  ADSSTR3;
  unsigned char  ADSSTR4;
  unsigned char  ADSSTR5;
  unsigned char  ADSSTR6;
  unsigned char  ADSSTR7;
  unsigned char  ADSSTR8;
  unsigned char  ADSSTR9;
  unsigned char  ADSSTR10;
  unsigned char  ADSSTR11;
  unsigned char  ADSSTR12;
  unsigned char  ADSSTR13;
  unsigned char  ADSSTR14;
  unsigned char  ADSSTR15;
} st_s12ad1_t;

#pragma pack()

#endif  /* __ASSEMBLER__ */
#endif

